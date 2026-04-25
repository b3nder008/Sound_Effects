#pragma once
/*
 * ble.h
 * ─────────────────────────────────────────────────────────────────────────────
 * BLE GATT server for the LED matrix — mode, brightness, and power control
 * from any BLE Central (iPhone, iPad, nRF Connect app, custom app).
 *
 * ── Enable / disable ───────────────────────────────────────────────────────
 *
 *   #define BLE_ENABLED  1   ← full BLE support (default)
 *   #define BLE_ENABLED  0   ← compile out entirely, zero overhead
 *
 * ── GATT service layout ────────────────────────────────────────────────────
 *
 *   Service UUID:  4FAFC201-1FB5-459E-8FCC-C5C9C331914B
 *
 *   Characteristic   UUID suffix   Props          Format   Range
 *   ───────────────────────────────────────────────────────────────────────
 *   Power            …-0001        R/W/Notify     uint8    0=off  1=on
 *   Mode             …-0002        R/W/Notify     uint8    0–28 (MODE_xxx)
 *   Brightness       …-0003        R/W/Notify     uint8    0–255
 *   Status           …-0004        R/Notify       3 bytes  [power, mode, bri]
 *
 * Write any writable characteristic to change state immediately.
 * All characteristics support Notify — subscribe to Status (0004) to receive
 * the full [power, mode, brightness] state whenever anything changes,
 * including changes made by the physical buttons.
 *
 * ── Device name ────────────────────────────────────────────────────────────
 *
 *   Advertises as "LedMatrix" — change BLE_DEVICE_NAME below if desired.
 *
 * ── iPhone testing (nRF Connect) ───────────────────────────────────────────
 *
 *   See README section at bottom of ble.cpp for step-by-step instructions.
 *
 * ── Thread safety ──────────────────────────────────────────────────────────
 *
 *   BLE write callbacks run in a BLE stack task context, not loop().
 *   State changes from writes are applied via atomic flags and picked up
 *   by bleUpdate() which runs in loop() context.  Notifications are always
 *   sent from loop() — never from the callback — to avoid stack conflicts
 *   with the I2S audio DMA interrupts.
 */

#include <Arduino.h>

// ── Master enable ─────────────────────────────────────────────────────────────
#define BLE_ENABLED  1

// ── Device name (advertised over BLE) ────────────────────────────────────────
#define BLE_DEVICE_NAME  "LedMatrix"

// ── UUIDs ─────────────────────────────────────────────────────────────────────
// Service
#define BLE_SERVICE_UUID     "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
// Characteristics
#define BLE_CHAR_POWER_UUID  "beb5483e-36e1-4688-b7f5-ea07361b0001"
#define BLE_CHAR_MODE_UUID   "beb5483e-36e1-4688-b7f5-ea07361b0002"
#define BLE_CHAR_BRITE_UUID  "beb5483e-36e1-4688-b7f5-ea07361b0003"
#define BLE_CHAR_STATUS_UUID "beb5483e-36e1-4688-b7f5-ea07361b0004"

// ── Public API ────────────────────────────────────────────────────────────────
void bleInit();
void bleUpdate();   // call every loop() — handles pending writes and notifications
