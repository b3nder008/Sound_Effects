#pragma once
/*
 * buttons.h
 * ─────────────────────────────────────────────────────────────────────────────
 * Three physical buttons for the XIAO ESP32-S3 LED matrix.
 *
 * ── GPIO assignments ───────────────────────────────────────────────────────
 *
 *   Button        Pin   GPIO   Notes
 *   ──────────────────────────────────────────────────────────────────────────
 *   BTN_POWER     D2    GPIO3  Toggle display on/off (audio keeps running)
 *   BTN_MODE      D8    GPIO7  Cycle through effect modes
 *   BTN_BRIGHT    D7    GPIO44 Step through 4 brightness levels  ⚠ see note
 *
 * Wire each button between its GPIO pin and GND. INPUT_PULLUP is used
 * internally — no external resistor needed.
 *
 * These pins are free on the XIAO ESP32-S3 given the current wiring:
 *   I2S mic  : D3(WS)  D4(SCK)  D5(SD)       -- D2/D7/D8 clear
 *   Matrix   : D10(DIN)                        -- D2/D7/D8 clear
 *   USB-CDC  : internal GPIO19/20, no conflict with D2/D7/D8
 *
 * ── Pin constant type ──────────────────────────────────────────────────────
 *
 * BTN_xxx_PIN constants are declared as const int, matching the ESP32
 * Arduino API (pinMode / digitalRead both take int). The D3/D4/D5 board
 * constants are uint32_t — assigning them to int is safe for all XIAO
 * GPIO numbers and avoids implicit narrowing to uint8_t.
 *
 * ── Debug bypass ───────────────────────────────────────────────────────────
 *
 * Set BUTTONS_ENABLED 0 to compile out all button hardware and logic.
 * The device boots directly into whatever ACTIVE_MODE is set in
 * visualizer.h, at BRIGHTNESS_DEFAULT, with display always on.
 * Use this when iterating on a specific effect — no button presses needed,
 * no accidental mode switches, no overhead.
 *
 *   #define BUTTONS_ENABLED  0   <- ignore all buttons, direct boot into effect
 *   #define BUTTONS_ENABLED  1   <- full button support
 *
 * ── Mode ordering ──────────────────────────────────────────────────────────
 *
 * BTN_MODE steps through the MODE_TABLE array in buttons.cpp.
 * Edit VISUALIZER_MODE_TABLE in visualizer.h to choose which effects are
 * included and in what order. Comment out a row there to exclude a mode.
 *
 * ── Brightness levels ──────────────────────────────────────────────────────
 *
 * BTN_BRIGHT steps through BRIGHTNESS_LEVELS[] in buttons.cpp.
 * Default steps: 30, 80, 150, 255.
 *
 * ── Power button behaviour ─────────────────────────────────────────────────
 *
 * Short press  : toggle display on/off. Matrix goes dark but audio and beat
 *                detection keep running so the display is instantly reactive
 *                when turned back on.
 *
 * Long press   : hold > BTN_POWER_LONG_MS. Currently reserved — add
 *                deep-sleep or reboot in buttonsUpdate() where marked.
 *                Long press suppresses the short-press toggle on release.
 */

#include "button.h"

// ── Master enable ─────────────────────────────────────────────────────────────
// 0 = compile out all button code — boot straight into ACTIVE_MODE at full bri
// 1 = full button support
#define BUTTONS_ENABLED  1


// ── GPIO pins — const int matches ESP32 Arduino digitalRead/pinMode API ───────
const int BTN_POWER_PIN  = D2;   // GPIO3  — strapping pin, safe after boot
const int BTN_MODE_PIN   = D8;   // GPIO7  — clean general-purpose GPIO
const int BTN_BRIGHT_PIN = D7;   // GPIO44 — UART0 RX, see ⚠ warning above


// ── Timing ────────────────────────────────────────────────────────────────────
const uint32_t BTN_POWER_LONG_MS = 2000;   // hold duration for long-press

// ── Public API ────────────────────────────────────────────────────────────────
void    buttonsInit();
void    buttonsUpdate();          // call every loop() before audioUpdate()

bool    buttonsPowerOn();         // false when display is blanked
uint8_t buttonsBrightness();      // current brightness 0-255
uint8_t buttonsCurrentMode();     // current mode constant (MODE_xxx)

// ── Setters (called by BLE and any other runtime control source) ──────────────
void buttonsSetPower(bool on);
void buttonsSetBrightness(uint8_t bri);
void buttonsSetMode(uint8_t modeId);