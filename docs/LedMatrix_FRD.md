# LedMatrix iOS App — Functional Requirements Document (FRD)
**Version 1.0 | Project: LED Matrix BLE Controller**

---

## 1. Document Purpose

This FRD defines every feature the app must have, what "done" looks like for
each one, and what the app must not do. It is the source of truth for
implementation and testing.

---

## 2. Stakeholders

| Role | Person |
|------|--------|
| Developer / Owner | You |
| Target User | You (and anyone you hand the app to) |
| Hardware | XIAO ESP32-C3 + WS2812B 8×8 LED matrix |

---

## 3. Scope

The app controls a single BLE peripheral named `"LedMatrix"` over a GATT
service. It provides real-time control of power, display mode, and brightness.
The app must also reflect state changes initiated by the physical hardware
buttons on the device.

---

## 4. Definitions

| Term | Meaning |
|------|---------|
| **Device** | The XIAO ESP32-C3 with attached LED matrix |
| **Central** | The iPhone (the controller in BLE terminology) |
| **Peripheral** | The Device (the server in BLE terminology) |
| **Characteristic** | A single data value inside the BLE service |
| **Notification** | An unsolicited data push from Peripheral → Central |
| **Mode** | One of 31 visual effects running on the LED matrix |
| **Status byte triplet** | The 3-byte payload `[power, mode, brightness]` from characteristic 0004 |

---

## 5. Functional Requirements

### FR-01: Bluetooth Availability Detection

**Description:** The app must detect whether Bluetooth is enabled on the iPhone.

**Acceptance criteria:**
- On launch, if Bluetooth is off, a non-blocking banner message says:
  "Bluetooth is off. Enable it in Settings to connect."
- The banner disappears automatically when Bluetooth is turned on.
- The app never crashes due to Bluetooth being unavailable.
- No BLE scan is started while Bluetooth is off.

---

### FR-02: Device Discovery (Scanning)

**Description:** The app must automatically scan for the device and connect to it.

**Acceptance criteria:**
- Scanning starts automatically when: (a) the app launches with Bluetooth on,
  or (b) Bluetooth turns on while the app is running.
- The app scans for a peripheral whose local name equals `"LedMatrix"` exactly.
- While scanning, the connection banner displays "Scanning…" with an activity
  indicator.
- If no device is found within 15 seconds, scanning stops and the banner
  displays "Device not found. Is it powered on?" with a "Retry" button.
- Tapping "Retry" restarts the scan.
- Scanning stops as soon as the target device is discovered.

---

### FR-03: BLE Connection and Service Discovery

**Description:** The app must connect to the device and prepare all characteristics.

**Acceptance criteria:**
- After discovery, the app initiates connection automatically (no user action needed).
- The app discovers the service with UUID `4fafc201-1fb5-459e-8fcc-c5c9c331914b`.
- The app discovers all 4 characteristics within that service:
  - Power: `beb5483e-36e1-4688-b7f5-ea07361b0001`
  - Mode: `beb5483e-36e1-4688-b7f5-ea07361b0002`
  - Brightness: `beb5483e-36e1-4688-b7f5-ea07361b0003`
  - Status: `beb5483e-36e1-4688-b7f5-ea07361b0004`
- The app subscribes to notifications on the Status characteristic (0004).
- After subscribing, the app reads the current value of Status to populate the
  initial UI state.
- The connection banner changes to "Connected — LedMatrix" and turns green.
- All controls become interactive once the connection is ready.
- If service/characteristic discovery fails, the app disconnects and retries
  from scanning (with a log message, no crash).

---

### FR-04: Connection Loss and Auto-Reconnect

**Description:** The app must handle drops gracefully.

**Acceptance criteria:**
- If the connection drops (device powered off, out of range), the connection
  banner immediately shows "Disconnected" in red.
- All controls become disabled (non-interactive, visually greyed out).
- The app automatically begins scanning again after a 2-second delay.
- The app reconnects silently if the device reappears within 30 seconds.
- If the device does not reappear, the banner shows "Device not found" with a
  manual Retry button (same as FR-02 timeout behaviour).

---

### FR-05: Power Control

**Description:** The user must be able to turn the LED matrix display on and off.

**Acceptance criteria:**
- A clearly labeled toggle (or large button) is visible on the main screen.
- When the display is ON: the toggle shows "On" state (e.g., green / filled).
- When the display is OFF: the toggle shows "Off" state (e.g., grey / empty).
- Tapping the toggle writes 1 byte to characteristic 0001: `0x01` (on) or `0x00` (off).
- The toggle state updates to reflect the confirmed state received via the
  Status notification — not optimistically on tap.
- If the physical power button on the device is pressed, the toggle in the app
  updates within 1 second without any user action.
- The power control is disabled (non-interactive) while the app is disconnected.

**Data format:** `uint8` — `0x00` = off, `0x01` = on.

---

### FR-06: Mode Selection

**Description:** The user must be able to select any of the 31 visual effect modes.

**Acceptance criteria:**
- All 31 modes are listed by their human-readable names (see mode name table
  in the Architecture document, Section 4.2).
- The currently active mode is highlighted / selected in the list.
- Selecting a mode from the list writes 1 byte to characteristic 0002 containing
  the mode's index (0–30).
- If the physical mode button on the device is pressed, the selected mode in
  the app updates within 1 second.
- The mode list is disabled while disconnected.
- Mode names are displayed in the same order as they appear in the firmware's
  `VISUALIZER_MODE_TABLE` (index 0 = "Spectrum analyser bars", index 30 = "Juggle dots").

**Data format:** `uint8` — value 0 through 30 corresponding to the mode index.

---

### FR-07: Brightness Control

**Description:** The user must be able to adjust the LED matrix brightness.

**Acceptance criteria:**
- A slider spans the full width of the screen.
- The slider range maps 0 (off / minimum) to 255 (maximum brightness).
- The current brightness value is displayed numerically next to the slider
  (show as a 0–100% value: `round(value / 255 * 100)%`).
- Sliding writes 1 byte to characteristic 0003 with the raw 0–255 value.
- Writes are throttled: no more than one write per 50 ms while the user is
  actively dragging (to avoid BLE congestion).
- Releasing the slider sends one final write with the exact position.
- If the physical brightness button on the device is pressed, the slider snaps
  to the new value within 1 second.
- The slider is disabled while disconnected.

**Data format:** `uint8` — raw value 0–255.

---

### FR-08: Live Status Synchronisation

**Description:** The app must stay in sync with the device at all times, including
changes made by the physical buttons.

**Acceptance criteria:**
- The app subscribes to notifications on characteristic 0004 (Status).
- Every notification carries exactly 3 bytes: `[power (uint8), mode (uint8), brightness (uint8)]`.
- On receiving a notification, the app updates all three controls simultaneously.
- No user-visible flicker or lag: updates complete within one UI frame (~16 ms)
  of receiving the notification.
- If the device sends a mode index outside 0–30, the app displays "Unknown Mode (N)"
  rather than crashing.

---

### FR-09: Connection Status Banner

**Description:** A persistent status indicator must always show the connection state.

**Acceptance criteria:**
- A banner is always visible at the top of the screen (or in a status area).
- States and their display:

| State | Colour | Text |
|-------|--------|------|
| Bluetooth off | Grey | "Bluetooth is off" |
| Scanning | Yellow/Amber | "Scanning…" + spinner |
| Connecting | Yellow/Amber | "Connecting…" + spinner |
| Connected | Green | "Connected — LedMatrix" |
| Disconnected | Red | "Disconnected" |
| Device not found | Red | "Device not found" + Retry button |

- Transitions between states are animated (fade or slide, ≤ 0.3 s).

---

### FR-10: Controls Disabled When Disconnected

**Description:** No control should be interactive when not connected.

**Acceptance criteria:**
- The power toggle, mode picker, and brightness slider are visually greyed out
  and do not respond to touch when `connectionState != .connected`.
- An optional short message below the controls reads:
  "Connect to LedMatrix to control the display."

---

### FR-11: App Background Behaviour

**Description:** Define what happens when the app is backgrounded.

**Acceptance criteria:**
- When the app moves to background, active BLE connection is maintained (iOS
  allows this for connected peripherals).
- BLE notifications received in the background do not need to be processed
  (no background processing required for v1.0).
- When the app returns to foreground, it re-reads the Status characteristic
  once to refresh UI state in case it missed notifications.

---

## 6. Non-Functional Requirements

### NFR-01: Performance
- UI must update within 100 ms of a BLE notification being received.
- The app must not drop below 60 fps during normal use.

### NFR-02: Battery
- When connected and idle (no user interaction, no button presses on device),
  the app must not drain more than 2% battery per hour.
- BLE scanning must stop immediately upon connection (no continuous background scan).

### NFR-03: Reliability
- The app must not crash on any of the following: Bluetooth toggle on/off,
  device power off/on, rapid mode switches, extreme (0 and 255) brightness values.

### NFR-04: Usability
- A new user must be able to connect and change a mode within 30 seconds of
  first launching the app, with no instructions.
- All interactive controls must meet Apple's minimum touch target of 44×44 pts.

### NFR-05: Compatibility
- The app must run on iOS 16.0 and later.
- The app must support iPhone SE (2nd gen, 4.7") through iPhone Pro Max (6.7") screen sizes.
- The app must support both Light Mode and Dark Mode.

---

## 7. Out of Scope (v1.0)

The following are explicitly NOT requirements for this version:

- iPad-specific layout
- macOS or Apple Watch support
- Connecting to more than one device simultaneously
- Editing or creating new visual modes
- Scheduling (timers, auto on/off)
- Push notifications
- iCloud sync
- Custom colour palette entry
- Firmware update over BLE

---

## 8. BLE Reference Summary

All values below are taken directly from `ble.h` in the firmware.

**Service UUID:**
```
4fafc201-1fb5-459e-8fcc-c5c9c331914b
```

**Characteristics:**

| Name | UUID | R | W | Notify | Format |
|------|------|---|---|--------|--------|
| Power | `beb5483e-36e1-4688-b7f5-ea07361b0001` | ✓ | ✓ | ✓ | uint8: 0=off, 1=on |
| Mode | `beb5483e-36e1-4688-b7f5-ea07361b0002` | ✓ | ✓ | ✓ | uint8: 0–30 |
| Brightness | `beb5483e-36e1-4688-b7f5-ea07361b0003` | ✓ | ✓ | ✓ | uint8: 0–255 |
| Status | `beb5483e-36e1-4688-b7f5-ea07361b0004` | ✓ | — | ✓ | 3 bytes: [power, mode, bri] |

**Device advertisement name:** `LedMatrix`

**Note on BLE_ENABLED:** The firmware has `#define BLE_ENABLED 0` in `ble.h`
by default. You must change this to `#define BLE_ENABLED 1` and re-flash the
device before the iOS app can connect.

---

## 9. Acceptance Test Cases

| Test ID | Test | Pass Condition |
|---------|------|----------------|
| T-01 | Launch with Bluetooth off | Banner shows "Bluetooth is off" |
| T-02 | Turn on Bluetooth | App begins scanning automatically |
| T-03 | Power on device | App connects within 5 s; banner turns green |
| T-04 | Tap power toggle to OFF | Matrix goes dark; toggle shows Off state |
| T-05 | Tap power toggle to ON | Matrix lights up; toggle shows On state |
| T-06 | Select "Fire2012 simulation" | Matrix shows fire effect within 1 s |
| T-07 | Select "Spectrum analyser bars" | Matrix shows spectrum within 1 s |
| T-08 | Drag brightness to 0% | Matrix dims to minimum |
| T-09 | Drag brightness to 100% | Matrix at maximum brightness |
| T-10 | Press physical MODE button | App mode selection updates within 1 s |
| T-11 | Press physical POWER button | App power toggle updates within 1 s |
| T-12 | Press physical BRIGHT button | App brightness slider updates within 1 s |
| T-13 | Power off device while connected | Banner turns red "Disconnected" |
| T-14 | Power device back on | App reconnects automatically; banner turns green |
| T-15 | All controls while disconnected | Controls are greyed out, non-interactive |
| T-16 | Rapid mode switching (tap 5 modes fast) | No crash; last selected mode wins |
| T-17 | Toggle Bluetooth off while connected | Banner shows "Bluetooth is off" |
| T-18 | Return to foreground after backgrounding | UI refreshes to current device state |
