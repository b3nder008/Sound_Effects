/*
 * buttons.cpp
 * ─────────────────────────────────────────────────────────────────────────────
 * Button handling: mode cycling, brightness stepping, power toggle.
 *
 * Depends on button.h for debounce and event detection.
 * All code in this file is compiled out when BUTTONS_ENABLED == 0.
 *
 * ── Update contract ────────────────────────────────────────────────────────
 * buttonsUpdate() calls btn.update() on every Button object first, before
 * reading any event. This guarantees pressed(), released(), isHeld(), and
 * heldDuration() all reflect a consistent snapshot for that loop() frame,
 * identical to the hello-world test pattern that was validated on hardware.
 */
 
#include "buttons.h"
#include "visualizer.h"   // mode constants and visualizerSetMode()
 
#if BUTTONS_ENABLED
 
// ── Mode table ────────────────────────────────────────────────────────────────
// Automatically derived from VISUALIZER_MODE_TABLE in visualizer.h.
// To add, remove, or reorder modes: edit visualizer.h only.
// To temporarily exclude a mode from cycling: comment out its row there.
#define _BTN_MODE_X(id, desc) id,
static const uint8_t MODE_TABLE[] = { VISUALIZER_MODE_TABLE(_BTN_MODE_X) };
#undef _BTN_MODE_X
static const uint8_t MODE_TABLE_LEN =
    (uint8_t)(sizeof(MODE_TABLE) / sizeof(MODE_TABLE[0]));
 
// ── Brightness levels ─────────────────────────────────────────────────────────
// BTN_BRIGHT steps through these in order, wrapping around.
// Values are passed directly to matrix.setBrightness().
static const uint8_t BRIGHTNESS_LEVELS[]    = { 30, 80, 150, 255 };
static const uint8_t BRIGHTNESS_LEVEL_COUNT =
    (uint8_t)(sizeof(BRIGHTNESS_LEVELS) / sizeof(BRIGHTNESS_LEVELS[0]));
 
// ── Button objects ────────────────────────────────────────────────────────────
static Button _btnMode;
static Button _btnPower;
static Button _btnBright;
 
// ── Application state ─────────────────────────────────────────────────────────
static uint8_t _modeIdx   = 0;      // index into MODE_TABLE
static uint8_t _brightIdx = 1;      // index into BRIGHTNESS_LEVELS (default: 80)
static bool    _powerOn   = true;   // display on/off
 
// BLE brightness override — when active, bypasses the step table
static uint8_t _bleRawBrightness       = 80;
static bool    _bleRawBrightnessActive = false;
 
// Long-press gate — prevents short-press toggle firing after a long press
static bool _powerLongFired = false;
 
// ── Public accessors ──────────────────────────────────────────────────────────
 
bool buttonsPowerOn() { return _powerOn; }
 
uint8_t buttonsBrightness() {
    return _bleRawBrightnessActive
        ? _bleRawBrightness
        : BRIGHTNESS_LEVELS[_brightIdx];
}
 
uint8_t buttonsCurrentMode() { return MODE_TABLE[_modeIdx]; }
 
// ── Setters — called by BLE (and optionally other sources) ───────────────────
 
void buttonsSetPower(bool on) {
    if (_powerOn && !on) {
        visualizerBlank();   // blank immediately when BLE powers off
    }
    _powerOn = on;
}
 
void buttonsSetBrightness(uint8_t bri) {
    // Store raw BLE value directly — takes effect next frame via buttonsBrightness().
    // Also snap _brightIdx to the nearest step so the physical button resumes
    // from a sensible position if the user presses it after a BLE set.
    _bleRawBrightness       = bri;
    _bleRawBrightnessActive = true;
 
    uint8_t best     = 0;
    uint8_t bestDiff = 255;
    for (uint8_t i = 0; i < BRIGHTNESS_LEVEL_COUNT; i++) {
        uint8_t d = (uint8_t)abs((int)bri - (int)BRIGHTNESS_LEVELS[i]);
        if (d < bestDiff) { bestDiff = d; best = i; }
    }
    _brightIdx = best;
}
 
void buttonsSetMode(uint8_t modeId) {
    for (uint8_t i = 0; i < MODE_TABLE_LEN; i++) {
        if (MODE_TABLE[i] == modeId) {
            _modeIdx = i;
            visualizerSetMode(modeId);
            return;
        }
    }
    // Mode not in cycling table — set directly anyway
    visualizerSetMode(modeId);
}
 
// ── Init ──────────────────────────────────────────────────────────────────────
 
void buttonsInit() {
    _btnMode.begin(BTN_MODE_PIN);
    _btnPower.begin(BTN_POWER_PIN);
    _btnBright.begin(BTN_BRIGHT_PIN);
 
    // Start at MODE_SPECTRUM regardless of ACTIVE_MODE compile-time constant.
    // ACTIVE_MODE only controls which mode boots when BUTTONS_ENABLED 0.
    _modeIdx = 0;
    for (uint8_t i = 0; i < MODE_TABLE_LEN; i++) {
        if (MODE_TABLE[i] == MODE_SPECTRUM) { _modeIdx = i; break; }
    }
 
    visualizerSetMode(MODE_TABLE[_modeIdx]);
}
 
// ── Update — call every loop() ────────────────────────────────────────────────
 
void buttonsUpdate() {

    // ── 1. Update all buttons ─────────────────────────────────────────────────
    _btnMode.update();
    _btnPower.update();
    _btnBright.update();

    // ── 2. MODE button ────────────────────────────────────────────────────────
    if (_btnMode.released()) {
        if (!_powerOn) {
            // Wake from off — resume without cycling mode
            _powerOn = true;
        } else {
            // Already on — cycle to next mode as normal
            _modeIdx = (_modeIdx + 1) % MODE_TABLE_LEN;
            visualizerSetMode(MODE_TABLE[_modeIdx]);
        }
    }

    // ── 3. BRIGHTNESS button ──────────────────────────────────────────────────
    if (_btnBright.released()) {
        if (!_powerOn) {
            // Wake from off — resume without stepping brightness
            _powerOn = true;
        } else {
            // Already on — step brightness as normal
            _bleRawBrightnessActive = false;
            _brightIdx = (_brightIdx + 1) % BRIGHTNESS_LEVEL_COUNT;
        }
    }

    // ── 4. POWER button ───────────────────────────────────────────────────────
    if (_btnPower.pressed()) {
        _powerLongFired = false;
    }
    if (_btnPower.isHeld()
            && !_powerLongFired
            && _btnPower.heldDuration() >= BTN_POWER_LONG_MS) {
        _powerLongFired = true;
        // reserved for deep sleep / reboot
    }
    if (_btnPower.released() && !_powerLongFired) {
        _powerOn = !_powerOn;
        if (!_powerOn) {
            visualizerBlank();   // blank matrix immediately on power-off
        }
    }
}
 
// ── Stubs when buttons disabled ───────────────────────────────────────────────
// All public functions must still link when BUTTONS_ENABLED 0 because ble.cpp
// calls the setters unconditionally.
#else  // BUTTONS_ENABLED == 0
 
static bool    _powerOn  = true;
static uint8_t _rawBrite = 150;
static uint8_t _curMode  = ACTIVE_MODE;
 
void    buttonsInit()           {}
void    buttonsUpdate()         {}
bool    buttonsPowerOn()        { return _powerOn; }
uint8_t buttonsBrightness()     { return _rawBrite; }
uint8_t buttonsCurrentMode()    { return _curMode; }
 
void buttonsSetPower(bool on)          { _powerOn  = on; }
void buttonsSetBrightness(uint8_t bri) { _rawBrite = bri; }
void buttonsSetMode(uint8_t modeId)    { _curMode  = modeId; visualizerSetMode(modeId); }
 
#endif // BUTTONS_ENABLED