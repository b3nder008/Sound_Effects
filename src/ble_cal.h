#pragma once
// ─────────────────────────────────────────────────────────────────────────────
// ble_cal.h  —  BLE calibration extension
// ─────────────────────────────────────────────────────────────────────────────
//
// ── FEATURE FLAG ─────────────────────────────────────────────────────────────
//
//   BLE_CALIBRATION_ENABLED  1  →  three extra GATT characteristics added to
//                                  the existing service (0005, 0006, 0007).
//                                  Telemetry streams bandRaw[] at ~15 fps.
//                                  Commands trigger floor measurement / reset.
//                                  CalParams reads/writes runtime floors and
//                                  sensitivity values.  All changes persist in
//                                  NVS (ESP32 flash) across power cycles.
//
//   BLE_CALIBRATION_ENABLED  0  →  this entire file compiles to empty stubs.
//                                  The GATT service is exactly as before
//                                  (0001–0004 only).  Zero flash / RAM impact.
//
// ── REVERT PROCEDURE ─────────────────────────────────────────────────────────
//   Firmware:  set BLE_CALIBRATION_ENABLED 0 below and re-flash.
//   iOS:       set CalibrationEnabled = false in CalibrationFeatureFlag.swift.
//   Both are single-line changes.  No other file needs to change.
//
// ── GATT CHARACTERISTICS  (appended to existing service 4fafc201-…) ──────────
//
//   UUID suffix  Name       Props        Payload
//   ──────────────────────────────────────────────────────────────────────────
//   …-0005       Telemetry  Notify       16 bytes: bandRaw[8] as uint16 BE.
//                                        Streamed ~15 fps while connected.
//                                        Normal packet: raw[0..7].
//                                        Measurement-result sentinel: buf[0]==0xFF,
//                                        buf[1]==floor[0]/8, buf[2..15]==floor[1..7].
//
//   …-0006       Command    Write (NR)   1 byte command (CAL_CMD_* constants below).
//
//   …-0007       CalParams  R/W (NR)     32 bytes:
//                                          bytes  0–15: floors[8]  as uint16 BE
//                                          bytes 16–31: sens[8]    as uint16 BE × 1000
//                                        Read returns current runtimeFloor/Sensitivity.
//                                        Write applies immediately and saves to NVS.
//
// ── COMMAND BYTES ────────────────────────────────────────────────────────────
#define CAL_CMD_START_MEAS      0x01  // start 3-second silent floor measurement
#define CAL_CMD_CANCEL_MEAS     0x02  // cancel in-progress measurement
#define CAL_CMD_RESET_FLOORS    0x03  // restore runtimeFloor[] from NOISE_FLOOR_STATIC[]
#define CAL_CMD_RESET_SENS      0x04  // restore runtimeSensitivity[] from SENSITIVITY[]
#define CAL_CMD_RESET_ALL       0x05  // reset both floors and sensitivity
#define CAL_CMD_RESTORE_FACTORY 0x06  // restore from first-boot NVS snapshot

// ── TUNING ────────────────────────────────────────────────────────────────────
#define CAL_MEAS_MS        3000   // measurement window duration (ms)
#define CAL_SENS_SCALE     1000   // sensitivity fixed-point scale (float × 1000 → uint16)
#define CAL_TELEM_EVERY_N     2   // send telemetry every Nth fftProcess() call (~15 fps)

// ─────────────────────────────────────────────────────────────────────────────

#define BLE_CALIBRATION_ENABLED  1   // ← set 0 to revert to original 4-characteristic BLE

// ── Characteristic UUIDs ──────────────────────────────────────────────────────
#define BLE_CHAR_TELEM_UUID  "beb5483e-36e1-4688-b7f5-ea07361b0005"
#define BLE_CHAR_CMD_UUID    "beb5483e-36e1-4688-b7f5-ea07361b0006"
#define BLE_CHAR_CAL_UUID    "beb5483e-36e1-4688-b7f5-ea07361b0007"

// ── Public API ────────────────────────────────────────────────────────────────
// All four functions compile to empty stubs when BLE_CALIBRATION_ENABLED 0.
// Call sites in ble.cpp and fft.cpp never need #if guards.

#include <Arduino.h>

// Forward-declare BLEService so this header doesn't pull in BLE headers
// in files that don't need them (e.g. fft.cpp).
#if BLE_CALIBRATION_ENABLED
class BLEService;
#endif

void bleCalInit(BLEService* svc);  // register characteristics; call before svc->start()
void bleCalUpdate();               // call from bleUpdate() every loop()
void bleCalOnFftFrame();           // call from fftProcess() after bandRaw[] is populated
void bleCalSetConnected(bool connected); // call from BLE connect/disconnect callbacks