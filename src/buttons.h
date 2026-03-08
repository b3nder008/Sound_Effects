#pragma once
/*
 * buttons.h
 * ─────────────────────────────────────────────────────────────────────────────
 * Three physical buttons for the XIAO ESP32-C3 LED matrix.
 *
 * ── GPIO assignments ───────────────────────────────────────────────────────
 *
 *   Button        Pin   GPIO   Notes
 *   ──────────────────────────────────────────────────────────────────────────
 *   BTN_MODE      D3    GPIO5  Cycle through effect modes
 *   BTN_POWER     D4    GPIO6  Toggle display on/off (audio keeps running)
 *   BTN_BRIGHT    D5    GPIO7  Step through 4 brightness levels
 *
 * Wire each button between its GPIO pin and GND. INPUT_PULLUP is used
 * internally — no external resistor needed.
 *
 * These pins are free on the XIAO ESP32-C3 given the current wiring:
 *   I2S mic  : D2(SD)  D6(WS)  D7(SCK)      -- D3/D4/D5 clear
 *   Matrix   : D10(DIN)                       -- D3/D4/D5 clear
 *   USB-CDC  : internal, no GPIO conflict
 *   SPI/I2C  : D4/D5 double as SDA/SCL but are free when I2C unused
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
 * Edit that array to choose which effects are included and in what order.
 * The table is independent of the visualizer's compile-time ACTIVE_MODE —
 * at runtime the current mode index drives visualizerSetMode().
 *
 * ── Brightness levels ──────────────────────────────────────────────────────
 *
 * BTN_BRIGHT steps through BRIGHTNESS_LEVELS[] in buttons.cpp.
 * Default steps: 30, 80, 150, 255.
 *
 * ── Power button ───────────────────────────────────────────────────────────
 *
 * Short press: toggle display on/off. Matrix goes dark but audio and
 * beat detection keep running so the display is instantly reactive
 * when turned back on.
 *
 * Long press (hold > BTN_POWER_LONG_MS): not yet used — reserved for
 * future deep-sleep or reboot.
 */

#include "button.h"

// ── Master enable ─────────────────────────────────────────────────────────────
// 0 = compile out all button code — boot straight into ACTIVE_MODE at full bri
// 1 = full button support
#define BUTTONS_ENABLED  1

// ── GPIO pins ─────────────────────────────────────────────────────────────────
#define BTN_MODE_PIN    D3   // GPIO5
#define BTN_POWER_PIN   D4   // GPIO6
#define BTN_BRIGHT_PIN  D5   // GPIO7

// ── Timing ────────────────────────────────────────────────────────────────────
#define BTN_POWER_LONG_MS  2000   // hold duration for long-press (reserved)

// ── Public API ────────────────────────────────────────────────────────────────
void buttonsInit();
void buttonsUpdate();         // call every loop() before audioUpdate()

bool buttonsPowerOn();        // returns false when display is blanked
uint8_t buttonsBrightness();  // current brightness 0-255
uint8_t buttonsCurrentMode(); // current mode index into MODE_TABLE

// ── Setters (called by BLE and any other runtime control source) ──────────
void buttonsSetPower(bool on);          // set display on/off directly
void buttonsSetBrightness(uint8_t bri); // set brightness 0-255 directly
void buttonsSetMode(uint8_t modeId);    // set mode by MODE_xxx constant