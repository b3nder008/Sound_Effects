# LED Matrix Visualizer — Firmware Architecture
**Version 2.0 | Hardware: XIAO ESP32-S3 + INMP441 + WS2812B 8×8**

---

## 1. Purpose

This document describes the firmware architecture for a sound-reactive 8×8 WS2812B
NeoPixel matrix driven by a Seeed XIAO ESP32-S3. The system captures audio via an
INMP441 I2S MEMS microphone, runs a real-time FFT pipeline, and renders one of 30
animated visual effects synchronized to the sound. BLE GATT allows an iOS companion
app to control power, mode, and brightness, and to calibrate the FFT noise floor and
per-band sensitivity remotely.

---

## 2. System Overview

```
┌────────────────────────────────────────────────────────────────────────┐
│                         XIAO ESP32-S3 Firmware                         │
│                                                                        │
│  INMP441 I2S ──► audioUpdate() ──► fftProcess() ──► visualizerUpdate()│
│  Microphone        (audio.cpp)       (fft.cpp)        (visualizer.cpp) │
│                                          │                             │
│                                   bleCalOnFftFrame()                   │
│                                          │                             │
│  Physical        ┌─────────────────────────────────────────┐          │
│  Buttons ───────►│  buttonsUpdate()  bleUpdate()           │          │
│  (buttons.cpp)   │  (shared state: power / mode / bri)     │          │
│                  └───────────────────┬─────────────────────┘          │
│                                      │ BLE GATT                        │
└──────────────────────────────────────┼────────────────────────────────┘
                                       │
                              ┌────────▼────────┐
                              │  iPhone App      │
                              │  (CoreBluetooth) │
                              └─────────────────┘
                                       │
                               WS2812B 8×8 Matrix
```

---

## 3. Module Breakdown

### 3.1 main.cpp — Orchestrator

**Setup order** (sequential; order matters):

1. `audioInit()` — ESP-IDF I2S driver, DMA ring buffers
2. `fftInit()` — seeds `runtimeFloor[]` and `runtimeSensitivity[]` from factory constants
3. `fftCalibrate()` — boot-time noise floor measurement (no-op when `FFT_AUTO_CALIBRATE 0`)
4. `visualizerInit()` — NeoMatrix init, starts in `ACTIVE_MODE`
5. `buttonsInit()` — GPIO `INPUT_PULLUP`, debounce object init
6. `bleInit()` — BLE device, GATT service + all characteristics, starts advertising

**Loop order** (every iteration):

1. `buttonsUpdate()` — debounced button reads; applies mode/brightness/power changes
2. `bleUpdate()` — applies pending BLE writes; detects button-driven changes; sends
   Status notify; calls `bleCalUpdate()` for pending calibration commands
3. **Power-off path:** `audioUpdate()` + `fftProcess()` (keep pipeline warm), then return
4. `audioUpdate()` — drain DMA queue, flip double buffer
5. If `audioBufferReady`: `fftProcess()` → `visualizerUpdate()`
   (`bleCalOnFftFrame()` is called at the end of `fftProcess()`)

### 3.2 audio.cpp — I2S Audio Capture

- Configures ESP-IDF I2S in master RX mode at 16 kHz, 32-bit frames (left channel)
- **Double buffer:** DMA fills `dmaFillBuf`, FFT reads from `audioProcessBuffer`
- `audioUpdate()` drains the I2S DMA queue in 256-sample chunks; when 512 samples
  accumulate it flips the buffers and sets `audioBufferReady = true`
- Declared constants (`I2S_WS_GPIO`, `I2S_SCK_GPIO`, `I2S_SD_GPIO`) use raw integer
  GPIO numbers, not D-prefix aliases — required for ESP-IDF `i2s_set_pin()`

### 3.3 fft.cpp — FFT Pipeline

Pipeline per call to `fftProcess()`:

```
audioProcessBuffer[512]  (int16 PCM, Hann-windowed → double vReal[512])
        │
        ▼
  ArduinoFFT<double> forward FFT → complexToMagnitude → vReal[0..255]
        │
        ├── 8 perceptual bands (logarithmic bin groups, ~1/3 octave):
        │
        │   FFT_AUTO_CALIBRATE 0 (static floor — current default):
        │     Only bins > runtimeFloor[b] contribute to band average.
        │     avg = max(0, bandAvg − runtimeFloor[b])
        │
        │   FFT_AUTO_CALIBRATE 1 (adaptive floor):
        │     All bins averaged; asymmetric IIR updates adaptedFloor[b].
        │     avg = max(0, bandAvg − adaptedFloor[b])
        │
        ├── normalised = clamp(avg / BAND_SCALE[b] × runtimeSensitivity[b], 0, 1)
        │
        └── bandMagnitude[b] = IIR smooth(prev, normalised, BAND_SMOOTH[b])
        │
        └── bleCalOnFftFrame()  ← telemetry + peak accumulation
```

**Key static constants** (tuned on XIAO ESP32-S3 + INMP441 + 22 Ω/1 µF RC filter,
4-run calibration sessions):

| Array | Purpose |
|-------|---------|
| `NOISE_FLOOR_STATIC[8]` | Lab-measured ambient noise (p95, RC-filtered, ×2.0) |
| `BAND_SCALE[8]` | Expected above-floor magnitude at "full scale" (loud music) |
| `SENSITIVITY[8]` | Per-band boost after normalization (0.1–4.0×) |
| `BAND_SMOOTH[8]` | Per-band IIR weight on previous output (0.5–0.7) |
| `FLOOR_RISE_COEFF[8]` | Asymmetric IIR rise speed when adaptive floor enabled |
| `FLOOR_MIN[8]` | Minimum adaptive floor (prevents collapse to zero in silence) |

**Runtime arrays** (BLE-writable via CalParams; reset to factory on command):

| Variable | Starts as | Writable via |
|----------|-----------|-------------|
| `runtimeFloor[8]` | `NOISE_FLOOR_STATIC[]` | BLE CalParams (0007), or `CAL_CMD_RESET_FLOORS` |
| `runtimeSensitivity[8]` | `SENSITIVITY[]` | BLE CalParams (0007), or `CAL_CMD_RESET_SENS` |

### 3.4 visualizer.cpp — Rendering Engine

**Mode registration:** all modes are declared in a single X-macro table
`VISUALIZER_MODE_TABLE` in `visualizer.h`. This single table drives:
- Numeric enum IDs (zero-indexed, assigned automatically)
- Button cycling array in `buttons.cpp`
- Debug name strings in `visualizerModeName()`
- iOS mode name list (mirrored manually in `MatrixViewModel.swift`)

**Per-frame flow:**

1. `beatDetect()` — shared beat detector watching `bandMagnitude[2]` (low-mid).
   Outputs `beatFired` (true for exactly one frame) and `beatEnergy` (1.0 on beat,
   decays by `BEAT_DECAY` ≈ 1/6 per frame). All render functions receive these.
2. Runtime `switch(_runtimeMode)` dispatches to the active `renderXxx()`. All 30
   render functions are always compiled; mode switching is runtime-only.
3. `matrix.show()` — flushes pixel buffer to the WS2812B strip via NeoMatrix.

### 3.5 buttons.cpp — Physical Input

Three buttons using the `Button` debounce class (`button.h` / `include/button.h`):

| Constant | Pin | GPIO | Behaviour |
|----------|-----|------|-----------|
| `BTN_POWER_PIN` | D2 | GPIO3 | Short press: toggle display on/off (audio keeps running) |
| `BTN_MODE_PIN` | D8 | GPIO7 | Press: advance through `MODE_TABLE[]` |
| `BTN_BRIGHT_PIN` | D7 | GPIO44 | Press: step brightness [30 → 80 → 150 → 255 → 30] |

`buttonsSetPower()`, `buttonsSetMode()`, `buttonsSetBrightness()` allow BLE writes
to override the same state that buttons control (single source of truth).

> **⚠ GPIO44 / D7:** UART0 RX is held active by USB-CDC. This can cause erratic
> brightness button reads. If brightness is unreliable, move `BTN_BRIGHT_PIN` to D9.

### 3.6 ble.cpp — BLE GATT Server

Uses the ESP32 Arduino BLE library (BLEDevice / NimBLE backend, no extra install
needed with the Espressif Arduino core).

**Thread safety:** BLE write callbacks run in a BLE RTOS task. They set volatile
pending flags + value snapshots only. `bleUpdate()` in `loop()` applies the changes
and sends notifications — never from the callback, to avoid conflicts with I2S DMA.

**State machine:**

```
[Advertising "LedMatrix"]
        │  iOS connects
        ▼
[Connected]  ←──── bleUpdate() applies writes, sends Status notify each frame
        │  disconnect / out of range
        ▼
[Advertising again]  (auto-restarted in onDisconnect callback)
```

### 3.7 ble_cal.cpp — BLE Calibration Extension

Controlled by `BLE_CALIBRATION_ENABLED` in `ble_cal.h`. When `0`, every function
compiles to an empty stub — zero flash/RAM impact, and the GATT service presents
only characteristics 0001–0004.

**Lifecycle:**

- `bleCalInit(svc)` — called from `bleInit()` before `svc->start()`; creates
  Telemetry (0005), Command (0006), CalParams (0007) characteristics
- `bleCalOnFftFrame()` — called at end of `fftProcess()`; accumulates measurement
  peaks; sends throttled Telemetry notification (~15 fps) when connected
- `bleCalUpdate()` — called from `bleUpdate()`; processes pending Command bytes and
  CalParams writes; checks 3-second measurement timer completion

**Measurement flow:**

1. iOS sends `0x01` to Command → `_measuring = true`, peaks reset
2. `bleCalOnFftFrame()` accumulates `max(bandRaw[b])` for 3 seconds
3. At 3 s: `runtimeFloor[b] = max(peak × FFT_CALIBRATE_MARGIN, factory × 0.5)`
4. `_sendMeasResult()` — sends one Telemetry notification with `buf[0] = 0xFF`
   sentinel + new floor values. iOS detects the sentinel and parses as floor data.

---

## 4. GATT Service Layout

**Service UUID:** `4fafc201-1fb5-459e-8fcc-c5c9c331914b`
**Advertised name:** `LedMatrix`

| Characteristic | UUID suffix | Properties | Payload |
|---------------|-------------|------------|---------|
| Power | …-0001 | R/W/Notify | uint8: 0=off, 1=on |
| Mode | …-0002 | R/W/Notify | uint8: 0–29 (see §5) |
| Brightness | …-0003 | R/W/Notify | uint8: 0–255 |
| Status | …-0004 | R/Notify | 3 bytes: [power, mode, bri] — pushed on any state change |
| Telemetry¹ | …-0005 | Notify | 16 bytes: `bandRaw[8]` as uint16 BE, ~15 fps; `buf[0]=0xFF` sentinel signals floor result |
| Command¹ | …-0006 | Write-NR | 1 byte: 0x01=start meas, 0x02=cancel, 0x03=reset floors, 0x04=reset sens, 0x05=reset all |
| CalParams¹ | …-0007 | R/Write-NR | 32 bytes: floors[8] uint16 BE (bytes 0–15) + sens[8]×1000 uint16 BE (bytes 16–31) |

¹ Present only when `BLE_CALIBRATION_ENABLED 1`.

---

## 5. Mode Registry (30 modes, indices 0–29)

Defined by `VISUALIZER_MODE_TABLE` in `visualizer.h`. Index = row order in that table.

| Index | Identifier | Description |
|------:|-----------|-------------|
| 0 | `MODE_SPECTRUM` | Spectrum analyser bars |
| 1 | `MODE_FIRE` | Fire2012 simulation |
| 2 | `MODE_TORCH` | Torch (cool) |
| 3 | `MODE_TORCH2` | Torch (warm) |
| 4 | `MODE_RAIN` | Falling rain + lightning |
| 5 | `MODE_STARFIELD` | Warp-speed starfield |
| 6 | `MODE_WISP` | Will-o'-the-wisp |
| 7 | `MODE_PCBA` | PCB traces |
| 8 | `MODE_ATOM` | Bohr atom model |
| 9 | `MODE_GEOMETRIC` | Geometric rectangles |
| 10 | `MODE_DUNE` | Desert dunes |
| 11 | `MODE_PULSE` | Expanding rings |
| 12 | `MODE_WAVE` | Sine wave sweep |
| 13 | `MODE_RAINBOW_NOISE` | Rainbow Perlin noise |
| 14 | `MODE_RAINBOW_STRIPE_NOISE` | Rainbow stripe noise |
| 15 | `MODE_PARTY_NOISE` | Party noise |
| 16 | `MODE_FOREST_NOISE` | Forest noise |
| 17 | `MODE_CLOUD_NOISE` | Cloud noise |
| 18 | `MODE_FIRE_NOISE` | Fire noise |
| 19 | `MODE_LAVA_NOISE` | Lava noise |
| 20 | `MODE_OCEAN_NOISE` | Ocean noise |
| 21 | `MODE_CONFETTI` | Confetti speckles |
| 22 | `MODE_PRIDE` | Pride shifting rainbow |
| 23 | `MODE_COLOR_WAVES` | Color waves |
| 24 | `MODE_CLOUD_TWINKLES` | Cloud twinkles |
| 25 | `MODE_RAINBOW_TWINKLES` | Rainbow twinkles |
| 26 | `MODE_SINELON` | Sinelon dot |
| 27 | `MODE_JUGGLE` | Juggle dots |
| 28 | `MODE_LIFE` | Conway's Game of Life |
| 29 | `MODE_MICROBE` | Paramecium microbe |

To add a mode: append a row to `VISUALIZER_MODE_TABLE`, implement `renderXxx()` and
`_xxxInit()` in `visualizer.cpp`, and register both in `visualizerSetMode()` and
`visualizerUpdate()`. All index assignments, button cycling, and name lookup update
automatically.

---

## 6. Hardware Pinout

### INMP441 I2S Microphone

| Signal | XIAO Pin | GPIO | Notes |
|--------|----------|-----:|-------|
| WS (LRCK) | D3 | 4 | Word select |
| SCK (BCLK) | D4 | 5 | Bit clock |
| SD (data) | D5 | 6 | Serial data |
| L/R | GND | — | Selects left channel |
| VDD | 3.3V | — | |

Raw integer GPIO numbers used for `i2s_set_pin()` — NOT D-prefix board aliases.

### WS2812B 8×8 NeoMatrix

| Signal | XIAO Pin | GPIO | Notes |
|--------|----------|-----:|-------|
| DIN | D10 | 10 | 300–470 Ω series resistor on data line |
| VCC | 5V | — | 100–1000 µF bulk cap near connector |

### Physical Buttons (INPUT_PULLUP, wire each to GND)

| Button | XIAO Pin | GPIO | Function |
|--------|----------|-----:|----------|
| Power | D2 | 3 | Toggle display on/off (audio keeps running) |
| Mode | D8 | 7 | Cycle effect modes |
| Brightness | D7 | 44 | Step brightness: 30 → 80 → 150 → 255 → 30 |

> **⚠ D7 / GPIO44:** UART0 RX. USB-CDC holds UART0 active, which weakly drives
> GPIO44 and may fight `INPUT_PULLUP`. If the brightness button misbehaves, move
> `BTN_BRIGHT_PIN` to D9 (GPIO8).

---

## 7. File Structure

```
Sound_Effects/
├── platformio.ini              ← board (seeed_xiao_esp32s3), framework, lib_deps
├── src/
│   ├── main.cpp                ← setup() + loop(); full hardware pin-map comment
│   ├── audio.h / audio.cpp     ← INMP441 I2S driver, double-buffered DMA
│   ├── fft.h / fft.cpp         ← FFT pipeline; 8-band extraction; floor + sens
│   ├── visualizer.h            ← X-macro mode registry; hardware constants
│   ├── visualizer.cpp          ← all 30 render functions; beat detector
│   ├── buttons.h / buttons.cpp ← three-button debounce; mode/bri/power cycling
│   ├── ble.h / ble.cpp         ← BLE GATT server; power/mode/brightness/status
│   ├── ble_cal.h / ble_cal.cpp ← optional calibration extension (chars 0005–0007)
│   ├── fastled_compat.h        ← FastLED-style helpers over Adafruit NeoPixel
│   └── debug.h                 ← DEBUG_SERIAL, DEBUG_FFT_BANDS, DEBUG_LIFE flags
├── include/
│   └── button.h                ← Button debounce class
└── docs/
    ├── LedMatrix_Architecture.md  ← this file
    └── LedMatrix_FRD.md
```

---

## 8. Feature Flags — One-Line Reverts

| Flag | File | Default | Effect when 0 |
|------|------|:-------:|---------------|
| `BLE_ENABLED` | `ble.h` | 1 | BLE compiles to empty stubs; zero overhead |
| `BUTTONS_ENABLED` | `buttons.h` | 1 | Buttons disabled; boots into `ACTIVE_MODE` at fixed brightness |
| `BLE_CALIBRATION_ENABLED` | `ble_cal.h` | 1 | Cal characteristics removed; service shows 0001–0004 only |
| `FFT_AUTO_CALIBRATE` | `fft.h` | 0 | Adaptive floor disabled; uses static `NOISE_FLOOR_STATIC[]` |
| `DEBUG_SERIAL` | `debug.h` | 1 | All Serial output removed |
| `DEBUG_FFT_BANDS` | `debug.h` | 1 | Per-frame FFT band readout |
| `DEBUG_LIFE` | `debug.h` | 0 | GoL per-step state + ASCII grid |
| `ACTIVE_MODE` | `visualizer.h` | `MODE_ATOM` | Default mode when buttons disabled |

---

## 9. Threading Model

| Context | What runs there |
|---------|----------------|
| `loop()` (Arduino main task) | `buttonsUpdate`, `bleUpdate`, `audioUpdate`, `fftProcess`, `visualizerUpdate` |
| BLE RTOS task | `BLECharacteristicCallbacks::onWrite` — sets volatile flags only, never calls matrix/audio/visualizer |
| I2S DMA (hardware) | Writes to DMA ring buffer; `audioUpdate()` drains it from `loop()` |

---

## 10. iOS Companion App

A separate Xcode project at `LEDMatrix/` (SwiftUI + CoreBluetooth + MVVM).

| File | Responsibility |
|------|---------------|
| `BLEManager.swift` | CoreBluetooth central: scans, connects, publishes state, writes characteristics |
| `MatrixViewModel.swift` | Bridges BLE bytes to UI — mode name list mirrors `VISUALIZER_MODE_TABLE` |
| `ContentView.swift` | Root view; `TabView` with Control and Calibrate tabs when `CalibrationFeature.enabled` |
| `CalibrationViewModel.swift` | Drives calibration: start/cancel measurement, apply/discard/reset floors and sensitivity |
| `CalibrationView.swift` | Noise floor bar chart (live + floor threshold), sensitivity sliders, result card |
| `CalibrationFeatureFlag.swift` | `CalibrationFeature.enabled` — set `false` to hide Calibration tab entirely |

The iOS app uses `PBXFileSystemSynchronizedRootGroup` (Xcode 15+), so adding files
to the `LEDMatrix/` directory tree is sufficient — no manual `project.pbxproj` edits.
