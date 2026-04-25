# LedMatrix iOS App — Architecture Document
**Version 1.0 | Project: LED Matrix BLE Controller**

---

## 1. Purpose

This document describes the software architecture for an iPhone app that connects
to the XIAO ESP32-C3 LED matrix device over Bluetooth Low Energy (BLE) and lets
the user control power, display mode, and brightness in real time.

---

## 2. System Overview

```
┌──────────────────────────────────────────────────────────────────┐
│                          iPhone App                              │
│                                                                  │
│  ┌──────────────┐    ┌─────────────────┐    ┌───────────────┐  │
│  │   UI Layer   │◄──►│  ViewModel      │◄──►│  BLE Manager  │  │
│  │ (SwiftUI)    │    │  (ObservableObj)│    │ (CoreBluetooth│  │
│  └──────────────┘    └─────────────────┘    └───────────────┘  │
│                                                     │            │
└─────────────────────────────────────────────────────┼────────────┘
                                                       │ BLE (GATT)
                                          ┌────────────▼──────────┐
                                          │   XIAO ESP32-C3        │
                                          │   "LedMatrix" device   │
                                          │   WS2812B 8×8 matrix   │
                                          └───────────────────────┘
```

The three layers are clean and separated. The UI never talks directly to BLE,
and BLE never knows anything about the UI.

---

## 3. Technology Choices Explained

| Choice | What it is | Why we use it |
|--------|-----------|---------------|
| **Swift** | Apple's programming language | Modern, safe, required for iOS apps |
| **SwiftUI** | Apple's UI framework | Declarative — you describe what to show, not how to draw it. Much simpler for beginners |
| **CoreBluetooth** | Apple's BLE framework | Built into iOS, no third-party library needed |
| **MVVM pattern** | Model-View-ViewModel | Keeps BLE code out of UI; easy to test and extend |
| **Xcode** | Apple's IDE | Required to build and deploy iOS apps |
| **Combine / @Published** | Apple's reactive system | Automatically updates the UI when BLE state changes |

---

## 4. Layer Descriptions

### 4.1 BLE Manager (the "Model" layer)

**File:** `BLEManager.swift`

**Responsibility:** Everything Bluetooth. Nothing else.

- Scans for the device named `"LedMatrix"`
- Connects and discovers the GATT service and 4 characteristics
- Subscribes to the Status notification characteristic (UUID …-0004)
- Exposes `write()` methods: `writePower()`, `writeMode()`, `writeBrightness()`
- Publishes `@Published` state properties so the ViewModel can observe them

**CoreBluetooth roles:**
- App acts as **Central** (the controller)
- ESP32 acts as **Peripheral** (the device)

**UUIDs it uses (from `ble.h`):**

| Characteristic | UUID | Operation |
|---------------|------|-----------|
| Power | `…-0001` | Read, Write, Subscribe |
| Mode  | `…-0002` | Read, Write, Subscribe |
| Brightness | `…-0003` | Read, Write, Subscribe |
| Status | `…-0004` | Read, Subscribe (3-byte payload: [power, mode, bri]) |

**State machine:**

```
[Idle / Bluetooth off]
        │  User taps "Connect" or app foregrounds
        ▼
[Scanning for "LedMatrix"]
        │  Device found
        ▼
[Connecting]
        │  Connected
        ▼
[Discovering Services]
        │  Service 4fafc201… found
        ▼
[Discovering Characteristics]
        │  All 4 characteristics found; subscribe to Status (0004)
        ▼
[Connected & Ready]  ◄──► [Receiving Notifications / Sending Writes]
        │  Disconnect / device out of range
        ▼
[Disconnected — will retry]
```

### 4.2 ViewModel

**File:** `MatrixViewModel.swift`

**Responsibility:** The bridge between raw BLE bytes and the UI.

- Owns the `BLEManager` instance
- Translates raw `uint8` mode numbers into human-readable names (using the same
  mode list from `visualizer.h`)
- Exposes simple, UI-friendly properties: `isConnected`, `isPowered`, `brightness`,
  `currentModeName`, `availableModes`
- Handles edge cases: e.g. disabling the mode picker while disconnected

**Mode name list** (mirrors `VISUALIZER_MODE_TABLE` in `visualizer.h`, 31 entries):

```swift
let modeNames = [
    "Spectrum analyser bars",    // 0
    "Fire2012 simulation",       // 1
    "Torch (cool)",              // 2
    "Torch (warm)",              // 3
    "Falling rain + lightning",  // 4
    "Warp-speed starfield",      // 5
    "Will-o'-the-wisp",          // 6
    "PCB traces",                // 7
    "Bohr atom model",           // 8
    "Geometric rectangles",      // 9
    "Desert dunes",              // 10
    "Expanding rings",           // 11
    "Sine wave sweep",           // 12
    "Rainbow Perlin noise",      // 13
    "Rainbow stripe noise",      // 14
    "Party noise",               // 15
    "Forest noise",              // 16
    "Cloud noise",               // 17
    "Fire noise",                // 18
    "Lava noise",                // 19
    "Ocean noise",               // 20
    "Confetti speckles",         // 21
    "Pride shifting rainbow",    // 22
    "Color waves",               // 23
    "Rainbow",                   // 24
    "Rainbow + glitter",         // 25
    "Hue cycle solid",           // 26
    "Cloud twinkles",            // 27
    "Rainbow twinkles",          // 28
    "Sinelon dot",               // 29
    "Juggle dots",               // 30
]
```

### 4.3 UI Layer

**Files:** `ContentView.swift`, supporting view files

**Responsibility:** What the user sees and touches.

- Connection status bar at top
- Power toggle (large, obvious)
- Mode picker (scrollable list or Picker wheel)
- Brightness slider (0–255, mapped to a 0–100% display)
- All controls disabled/greyed when disconnected
- Status reflects notifications pushed from the device (e.g. physical button presses
  update the app without the user touching the phone)

---

## 5. Data Flow

### Writing a command (user → device)

```
User drags brightness slider
        │
        ▼
SliderView sends new value to ViewModel
        │
        ▼
ViewModel calls BLEManager.writeBrightness(value)
        │  CoreBluetooth writes 1 byte to characteristic 0003
        ▼
ESP32 receives write, applies immediately
        │  Status notification fires: [power, mode, newBri]
        ▼
BLEManager receives notification, parses 3 bytes
        │
        ▼
ViewModel @Published properties update
        │  SwiftUI re-renders only the changed controls
        ▼
UI reflects confirmed device state
```

### Receiving a state change (physical button → app)

```
User presses MODE button on the device
        │
        ▼
ESP32 changes mode, sends Status notification (0004)
        │
        ▼
BLEManager.peripheral(_:didUpdateValueFor:) fires
        │ Parses [power, mode, bri]
        ▼
BLEManager publishes new values
        │
        ▼
ViewModel observes changes, updates UI-facing properties
        │
        ▼
App UI mode label snaps to new mode name — no user action needed
```

---

## 6. Project File Structure

```
LedMatrix/
├── LedMatrixApp.swift          ← app entry point (@main)
├── BLE/
│   └── BLEManager.swift        ← CoreBluetooth logic
├── ViewModel/
│   └── MatrixViewModel.swift   ← state + translation
├── Views/
│   ├── ContentView.swift       ← root view
│   ├── ConnectionBanner.swift  ← status bar
│   ├── PowerToggleView.swift   ← large on/off button
│   ├── ModePickerView.swift    ← mode list/wheel
│   └── BrightnessSliderView.swift
└── Resources/
    └── Assets.xcassets         ← icons, colors
```

---

## 7. iOS Permissions Required

Add these to `Info.plist`:

| Key | Value | Why |
|-----|-------|-----|
| `NSBluetoothAlwaysUsageDescription` | "Used to control the LED matrix" | iOS 13+ BLE requirement |
| `NSBluetoothPeripheralUsageDescription` | "Used to control the LED matrix" | Older iOS fallback |

The app will crash on launch without the first key.

---

## 8. Threading Model

| Thread | What runs there |
|--------|----------------|
| Main thread | All SwiftUI rendering; all `@Published` property updates (dispatched via `DispatchQueue.main`) |
| CoreBluetooth internal queue | BLE delegate callbacks arrive here first |
| Rule | BLEManager always dispatches state changes to `DispatchQueue.main` before updating `@Published` vars |

Violating this rule causes SwiftUI warnings or crashes.

---

## 9. Error Handling Strategy

| Scenario | Behaviour |
|----------|-----------|
| Bluetooth is off | Banner shows "Bluetooth is off — enable in Settings" |
| Device not found after 10 s | Banner shows "Device not found — is it powered on?" with retry button |
| Connection dropped | Auto-reconnect once; if still failing, show "Disconnected" with manual retry |
| Write fails | Log silently; UI does not roll back (device will notify correct state) |
| Unknown mode index from device | Show mode number as "Mode 12" fallback |

---

## 10. Minimum Deployment Target

- **iOS 16.0** — gives access to modern SwiftUI features (charts, navigation stack)
  while covering virtually all iPhones in active use (iPhone 8 and later)
- Xcode 15 or later required to build

---

## 11. What Is NOT in Scope (v1.0)

- iPad-optimised layout (iPhone only for now)
- macOS / Catalyst build
- Multiple simultaneous device connections
- Firmware OTA update
- Scheduling / timers ("turn off at 11 pm")
- Custom colour palette editing
- watchOS companion
