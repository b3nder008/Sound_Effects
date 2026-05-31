# LED Matrix Visualizer — Functional Requirements Document (FRD)
**Version 2.0 | Hardware: XIAO ESP32-S3 + INMP441 + WS2812B 8×8**

---

## 1. Document Purpose

This FRD defines every feature the firmware and iOS companion app must have, what
"done" looks like for each one, and what is explicitly out of scope.

---

## 2. Stakeholders

| Role | Person |
|------|--------|
| Developer / Owner | Benjamin Rush |
| Target User | Developer + anyone handed the device |
| Hardware | XIAO ESP32-S3 + INMP441 I2S mic + WS2812B 8×8 NeoMatrix |

---

## 3. Scope

The system is a sound-reactive LED matrix controlled over BLE from an iOS app.
It provides real-time audio visualization across 30 effect modes, physical button
control of power/mode/brightness, and a BLE calibration interface for tuning the
FFT noise floor and per-band sensitivity without reflashing.

---

## 4. Definitions

| Term | Meaning |
|------|---------|
| **Device** | The XIAO ESP32-S3 with INMP441 mic and WS2812B 8×8 matrix |
| **Central** | The iPhone (BLE controller) |
| **Peripheral** | The Device (BLE server) |
| **Characteristic** | A single data value inside the BLE GATT service |
| **Notification** | Unsolicited data push from Peripheral → Central |
| **Mode** | One of 30 visual effects (indices 0–29) |
| **Status payload** | 3-byte BLE payload: `[power (uint8), mode (uint8), brightness (uint8)]` |
| **Runtime floor** | Per-band noise floor used by FFT; starts at factory values, BLE-writable |
| **Factory values** | Compile-time constants in `fft.cpp`: `NOISE_FLOOR_STATIC[]` and `SENSITIVITY[]` |

---

## 5. Functional Requirements

### FR-01: Audio Capture

**Description:** The device must continuously capture audio from the INMP441 mic.

**Acceptance criteria:**
- I2S captures at 16 kHz, 16-bit effective resolution (32-bit frames, upper 16 bits used)
- Audio runs even when the display is off — the FFT pipeline stays warm so the
  display is immediately reactive when power is restored
- `audioBufferReady` is set when 512 samples are available for FFT

---

### FR-02: FFT Processing — 8 Perceptual Bands

**Description:** The device must run a real-time FFT and map results to 8
logarithmically-spaced frequency bands.

**Band definitions** (at 16 kHz sample rate, 512-point FFT = 31.25 Hz/bin):

| Band | Freq Range | Description |
|-----:|------------|-------------|
| 0 | 20–150 Hz | Sub-bass |
| 1 | 150–400 Hz | Bass |
| 2 | 400–800 Hz | Low-mid |
| 3 | 800–2000 Hz | Mid |
| 4 | 2–4 kHz | Upper-mid |
| 5 | 4–6 kHz | Presence |
| 6 | 6–7 kHz | Brilliance |
| 7 | 7–8 kHz | Air (Nyquist limited) |

**Acceptance criteria:**
- Hann window applied before FFT to reduce spectral leakage
- Floor subtraction: only energy above `runtimeFloor[b]` contributes to each band
- Normalization: `(above-floor avg) / BAND_SCALE[b] × runtimeSensitivity[b]`, clamped 0–1
- Per-band IIR smoothing applied to `bandMagnitude[]` output
- `bandRaw[b]` exposes pre-floor raw magnitude for calibration telemetry

---

### FR-03: Visual Effect Modes

**Description:** The device must render 30 distinct visual effects synchronized to audio.

**Acceptance criteria:**
- All 30 modes are available and cycle correctly via BTN_MODE and BLE write
- The shared beat detector fires `beatFired` for exactly one frame per detected beat
  and provides a decaying `beatEnergy` float for beat-reactive effects
- Mode switching is instantaneous (no recompile, no reboot)
- Adding a new mode requires only: one row in `VISUALIZER_MODE_TABLE`, a render
  function, and a case in `visualizerUpdate()` — no other file changes

**Mode list** (index = cycling order):

| Index | Name |
|------:|------|
| 0 | Spectrum analyser bars |
| 1 | Fire2012 simulation |
| 2 | Torch (cool) |
| 3 | Torch (warm) |
| 4 | Falling rain + lightning |
| 5 | Warp-speed starfield |
| 6 | Will-o'-the-wisp |
| 7 | PCB traces |
| 8 | Bohr atom model |
| 9 | Geometric rectangles |
| 10 | Desert dunes |
| 11 | Expanding rings |
| 12 | Sine wave sweep |
| 13 | Rainbow Perlin noise |
| 14 | Rainbow stripe noise |
| 15 | Party noise |
| 16 | Forest noise |
| 17 | Cloud noise |
| 18 | Fire noise |
| 19 | Lava noise |
| 20 | Ocean noise |
| 21 | Confetti speckles |
| 22 | Pride shifting rainbow |
| 23 | Color waves |
| 24 | Cloud twinkles |
| 25 | Rainbow twinkles |
| 26 | Sinelon dot |
| 27 | Juggle dots |
| 28 | Conway's Game of Life |
| 29 | Paramecium microbe |

---

### FR-04: Physical Button Control

**Description:** Three physical buttons must control the device without requiring
the iOS app.

**Acceptance criteria:**
- **BTN_POWER (D2 / GPIO3):** Short press toggles display on/off. Audio and beat
  detection continue running while the display is off. Long press is reserved.
- **BTN_MODE (D8 / GPIO7):** Each press advances to the next mode in the table.
  Wraps from last mode back to index 0.
- **BTN_BRIGHT (D7 / GPIO44):** Each press steps through [30, 80, 150, 255],
  wrapping. Value passed directly to `matrix.setBrightness()`.
- All buttons use hardware debounce via the `Button` class. No external resistors
  required (internal INPUT_PULLUP).
- `BUTTONS_ENABLED 0` compiles out all button code; device boots into `ACTIVE_MODE`
  at fixed brightness with display always on.

---

### FR-05: BLE GATT Server — Core Control

**Description:** The device must expose a BLE GATT service allowing an iOS app
to read and control power, mode, and brightness.

**Service UUID:** `4fafc201-1fb5-459e-8fcc-c5c9c331914b`

**Characteristics:**

| Name | UUID suffix | Props | Format |
|------|-------------|-------|--------|
| Power | …-0001 | R/W/Notify | uint8: 0=off, 1=on |
| Mode | …-0002 | R/W/Notify | uint8: 0–29 |
| Brightness | …-0003 | R/W/Notify | uint8: 0–255 |
| Status | …-0004 | R/Notify | 3 bytes: [power, mode, bri] |

**Acceptance criteria:**
- Device advertises as `"LedMatrix"` and accepts one connection at a time
- Writes to 0001–0003 are applied within one `loop()` iteration
- Status (0004) notification fires whenever any of power/mode/brightness changes,
  whether from a BLE write or a physical button press
- BLE write callbacks only set volatile flags — state is applied in `loop()` to
  avoid conflicts with I2S DMA
- `BLE_ENABLED 0` compiles out BLE entirely; zero flash/RAM impact
- After disconnect, device immediately resumes advertising

---

### FR-06: iOS App — Connection Management

**Description:** The iOS app must scan for and connect to the device automatically.

**Acceptance criteria:**
- Scanning starts when Bluetooth is enabled and the app is in the foreground
- App scans for the peripheral named `"LedMatrix"` exactly
- Connection and characteristic discovery happen automatically after discovery
- App subscribes to Status (0004) and reads its current value on connect to
  populate initial UI state
- On disconnect, app auto-reconnects; shows "Disconnected" banner with retry if
  device does not reappear
- Connection state is always visible in a status banner at the top of the screen

---

### FR-07: iOS App — Power / Mode / Brightness Control

**Description:** The iOS app must let the user control the device in real time.

**Acceptance criteria:**
- **Power:** toggle writes 0x00 or 0x01 to characteristic 0001
- **Mode:** picker shows all 30 modes by name in index order; selection writes
  mode index (0–29) to characteristic 0002
- **Brightness:** slider maps 0–255 to 0–100% display; writes raw uint8 to 0003;
  throttled to ≤ 1 write per 50 ms while dragging
- Physical button presses on the device update the iOS UI within ~1 s via Status
  notification (0004)
- All controls are disabled and visually greyed while disconnected

---

### FR-08: BLE Calibration — Telemetry

**Description:** When `BLE_CALIBRATION_ENABLED 1`, the device must stream live FFT
band data to the app for real-time floor visualization.

**Characteristic:** Telemetry (UUID …-0005), Notify only.

**Acceptance criteria:**
- Notification payload: 16 bytes — `bandRaw[8]` as uint16 big-endian
- Rate: every 2nd `fftProcess()` call ≈ 15 fps
- Normal values: `buf[0] ≠ 0xFF` (bandRaw values are well below 0xFF00)
- Measurement result sentinel: `buf[0] = 0xFF` — payload contains new
  `runtimeFloor[8]` values (not live bandRaw). B0 floor packed as `buf[1] × 8`
  since the 0xFF occupies the high byte. All other bands (B1–B7) packed normally.
- Telemetry is only sent when a Central is connected

---

### FR-09: BLE Calibration — Noise Floor Measurement

**Description:** The device must support a 3-second silent noise floor measurement
triggered from the iOS app.

**Characteristic:** Command (UUID …-0006), Write-NR.

**Command bytes:**

| Byte | Action |
|:----:|--------|
| 0x01 | Start 3-second floor measurement |
| 0x02 | Cancel in-progress measurement |
| 0x03 | Reset `runtimeFloor[]` to `NOISE_FLOOR_STATIC[]` factory values |
| 0x04 | Reset `runtimeSensitivity[]` to `SENSITIVITY[]` factory values |
| 0x05 | Reset both floors and sensitivity to factory values |

**Acceptance criteria:**
- On 0x01: device accumulates `max(bandRaw[b])` per band over 3000 ms
- On completion: `runtimeFloor[b] = max(peak × FFT_CALIBRATE_MARGIN, factory × 0.5)` —
  the guard prevents the floor from collapsing below half the factory value
- Result delivered via one Telemetry notification with the 0xFF sentinel
- If 0x02 received during measurement, measurement aborts without changing floors
- Cancellation on disconnect (`bleCalSetConnected(false)`) also aborts measurement

---

### FR-10: BLE Calibration — CalParams Read/Write

**Description:** The device must allow the iOS app to read and write the current
runtime floor and sensitivity values directly.

**Characteristic:** CalParams (UUID …-0007), R/Write-NR.

**32-byte payload encoding:**

```
Bytes  0–15:  floors[8]      as uint16 big-endian (raw FFT units)
Bytes 16–31:  sensitivity[8] as uint16 big-endian (float × 1000)
               e.g. 2.000 → 2000,  0.800 → 800
```

**Acceptance criteria:**
- Read returns current `runtimeFloor[]` and `runtimeSensitivity[]` values
- Write applies all 16 values immediately (no reboot, no recompile)
- `CalParams` characteristic value is updated after measurement completion so
  a subsequent Read returns the newly measured floors

---

### FR-11: iOS App — Calibration UI

**Description:** The iOS app must provide a dedicated Calibration tab when
`CalibrationFeature.enabled` is true.

**Noise Floor tab:**
- Live animated bar chart: 8 bands, bar height = `bandRaw[b]` normalized to `maxRaw=4000`
- Orange threshold line on each bar at the current `runtimeFloor[b]` position
- Live bar turns cyan when signal is above the floor + 5%; grey otherwise
- Floor values readout grid (8 numeric labels)
- "Measure Noise Floor (3 s)" button — disabled while disconnected
- Progress indicator during measurement: countdown from 3 s; Cancel button
- Measurement result card with Apply / Discard buttons; Apply writes CalParams
- "Reset Floors to Factory" button (writes command 0x03)

**Sensitivity tab:**
- Bar chart: 8 indigo bars proportional to current `runtimeSensitivity[b] / 4.0`
- Per-band sliders 0.1×–4.0×, step 0.05
- "Apply Sensitivity" button — enabled only when slider values differ from device state
- "Reset Sensitivity to Factory" button (writes command 0x04)

**Acceptance criteria:**
- Tab only appears when `CalibrationFeature.enabled = true` in `CalibrationFeatureFlag.swift`
- When not connected: placeholder with antenna-slash icon instead of calibration UI
- Setting `CalibrationFeature.enabled = false` hides the tab; no other files change

---

## 6. Non-Functional Requirements

### NFR-01: Real-Time Performance
- FFT pipeline runs at ≈ 31 fps (512 samples ÷ 16 kHz), limited by audio buffer fill rate
- `visualizerUpdate()` must complete in time to avoid visible LED stutter at 30+ fps
- iOS UI must update within 100 ms of a BLE notification

### NFR-02: Audio Continuity
- The I2S pipeline must run continuously, including when the display is off
- DMA buffer overflow (caused by slow loop processing) should not occur under
  normal single-mode operation

### NFR-03: BLE Stability
- BLE write callbacks must not call matrix, audio, or visualizer functions
- All state changes from BLE are applied via volatile pending flags in `loop()` context
- The device must resume advertising immediately after any disconnect

### NFR-04: Feature Flag Safety
- Setting `BLE_ENABLED 0`, `BUTTONS_ENABLED 0`, `BLE_CALIBRATION_ENABLED 0`, or
  `FFT_AUTO_CALIBRATE 0` must produce a clean build with zero warnings
- Each flag is a one-line edit in one file

### NFR-05: iOS Compatibility
- iOS 16.0 minimum deployment target
- Must support iPhone SE (4.7") through iPhone Pro Max (6.7")
- Must support Light Mode and Dark Mode

---

## 7. BLE Reference Summary

**Service UUID:** `4fafc201-1fb5-459e-8fcc-c5c9c331914b`
**Device name:** `LedMatrix`

| Name | UUID | R | W | Notify | Format |
|------|------|:-:|:-:|:------:|--------|
| Power | `…-ea07361b0001` | ✓ | ✓ | ✓ | uint8: 0=off, 1=on |
| Mode | `…-ea07361b0002` | ✓ | ✓ | ✓ | uint8: 0–29 |
| Brightness | `…-ea07361b0003` | ✓ | ✓ | ✓ | uint8: 0–255 |
| Status | `…-ea07361b0004` | ✓ | — | ✓ | 3 bytes: [power, mode, bri] |
| Telemetry¹ | `…-ea07361b0005` | — | — | ✓ | 16 bytes: bandRaw[8] uint16 BE |
| Command¹ | `…-ea07361b0006` | — | NR | — | 1 byte command |
| CalParams¹ | `…-ea07361b0007` | ✓ | NR | — | 32 bytes: floors + sens |

¹ Present only when `BLE_CALIBRATION_ENABLED 1` in `ble_cal.h`.

Full UUID prefix: `beb5483e-36e1-4688-b7f5-`

---

## 8. Acceptance Test Cases

| Test ID | Test | Pass Condition |
|---------|------|----------------|
| T-01 | Power on device | BLE advertises "LedMatrix"; matrix shows `ACTIVE_MODE` |
| T-02 | iOS connects | Banner turns green; all controls interactive |
| T-03 | Tap power OFF in app | Matrix goes dark; app toggle shows Off |
| T-04 | Tap power ON in app | Matrix lights up; app toggle shows On |
| T-05 | Press BTN_POWER on device | App power toggle updates within 1 s |
| T-06 | Select mode in app | Matrix switches within 1 frame; app shows mode name |
| T-07 | Press BTN_MODE on device | App mode selection updates within 1 s |
| T-08 | Drag brightness slider | Matrix dims/brightens in real time |
| T-09 | Press BTN_BRIGHT on device | App brightness slider snaps within 1 s |
| T-10 | Disconnect device while connected | Banner turns red "Disconnected" |
| T-11 | Reconnect device | App reconnects automatically; banner turns green |
| T-12 | All controls while disconnected | Controls greyed out and non-interactive |
| T-13 | Rapid mode taps (5 in 1 s) | No crash; last selected mode active |
| T-14 | Bluetooth off on iPhone | Banner shows "Bluetooth is off" |
| T-15 | `BLE_ENABLED 0`, rebuild | Clean build; no BLE code; buttons still work |
| T-16 | `BLE_CALIBRATION_ENABLED 0`, rebuild | Clean build; service shows 0001–0004 only |
| T-17 | Open Calibration tab | Live bar chart animates; floor lines visible |
| T-18 | Keep room quiet, tap Measure | After 3 s: result card shows new floors |
| T-19 | Tap Apply on result card | FFT immediately uses new floors (quieter spectrum in silence) |
| T-20 | Tap Reset Floors to Factory | `runtimeFloor[]` restored; bar chart reflects change |
| T-21 | Adjust sensitivity slider, Apply | Band heights change immediately |
| T-22 | Reset Sensitivity to Factory | Sliders return to factory values |
| T-23 | `CalibrationFeature.enabled = false` | Calibration tab absent; Control tab unchanged |

---

## 9. Out of Scope

- iPad-specific layout
- macOS or watchOS support
- OTA firmware update over BLE
- Multiple simultaneous BLE connections
- Scheduling (auto on/off timers)
- Custom colour palette editor
- iCloud sync
- Push notifications
