/*
 * buttons.cpp
 * ─────────────────────────────────────────────────────────────────────────────
 * Button handling: mode cycling, brightness stepping, power toggle.
 *
 * All code in this file is compiled out when BUTTONS_ENABLED == 0.
 */

#include "buttons.h"
#include "visualizer.h"   // for mode constants and visualizerSetMode()

#if BUTTONS_ENABLED

// ── Mode table ────────────────────────────────────────────────────────────────
// Edit this list to choose which effects the MODE button cycles through
// and in what order. Use the MODE_xxx constants from visualizer.h.
// The table can be a subset — you don't have to include all 29 modes.
static const uint8_t MODE_TABLE[] = {
    MODE_SPECTRUM,
    MODE_FIRE,
    MODE_TORCH,
    MODE_TORCH2,
    MODE_RAIN,
    MODE_STARFIELD,
    MODE_GEOMETRIC,
    MODE_DUNE,
    MODE_PULSE,
    MODE_WAVE,
    MODE_RAINBOW_NOISE,
    MODE_FIRE_NOISE,
    MODE_OCEAN_NOISE,
    MODE_CONFETTI,
    MODE_PRIDE,
    MODE_COLOR_WAVES,
    MODE_RAINBOW_GLITTER,
    MODE_RAINBOW_TWINKLES,
    MODE_CLOUD_TWINKLES,
    MODE_SINELON,
    MODE_JUGGLE,
    MODE_WISP,
    MODE_PCBA,
};
static const uint8_t MODE_TABLE_LEN =
    (uint8_t)(sizeof(MODE_TABLE) / sizeof(MODE_TABLE[0]));

// ── Brightness levels ─────────────────────────────────────────────────────────
// BTN_BRIGHT steps through these in order, wrapping around.
// Values are passed directly to matrix.setBrightness().
static const uint8_t BRIGHTNESS_LEVELS[] = { 30, 80, 150, 255 };
static const uint8_t BRIGHTNESS_LEVEL_COUNT =
    (uint8_t)(sizeof(BRIGHTNESS_LEVELS) / sizeof(BRIGHTNESS_LEVELS[0]));

// ── State ─────────────────────────────────────────────────────────────────────
static Button _btnMode;
static Button _btnPower;
static Button _btnBright;

static uint8_t _modeIdx      = 0;     // index into MODE_TABLE
static uint8_t _brightIdx    = 1;     // index into BRIGHTNESS_LEVELS (default: 80)
static bool    _powerOn      = true;  // display on/off state
static uint8_t _bleRawBrightness = 80;   // BLE-set raw brightness value
static bool    _bleRawBrightnessActive = false; // true when BLE overrides step table

// Long-press tracking for power button
static bool     _powerHeld    = false;
static uint32_t _powerHeldAt  = 0;
static bool     _longFired    = false;

// ── Public accessors ──────────────────────────────────────────────────────────

bool buttonsPowerOn()        { return _powerOn; }
uint8_t buttonsBrightness()  {
    return _bleRawBrightnessActive ? _bleRawBrightness : BRIGHTNESS_LEVELS[_brightIdx];
}
uint8_t buttonsCurrentMode() { return MODE_TABLE[_modeIdx]; }

// ── Setters — called by BLE (and optionally other sources) ──────────────────

void buttonsSetPower(bool on) {
    _powerOn = on;
}

void buttonsSetBrightness(uint8_t bri) {
    // Clamp to valid range, store directly (bypasses the step table)
    // The BRIGHTNESS macro reads buttonsBrightness() each frame,
    // so this takes effect on the very next rendered frame.
    // Find the closest step index for consistency with physical button.
    uint8_t best = 0;
    uint8_t bestDiff = 255;
    for (uint8_t i = 0; i < BRIGHTNESS_LEVEL_COUNT; i++) {
        uint8_t d = (uint8_t)abs((int)bri - (int)BRIGHTNESS_LEVELS[i]);
        if (d < bestDiff) { bestDiff = d; best = i; }
    }
    _brightIdx = best;
    // Override: store raw value by inserting it as the active level.
    // Since BRIGHTNESS_LEVELS is const, we keep a separate raw override.
    _bleRawBrightness = bri;
    _bleRawBrightnessActive = true;
}

void buttonsSetMode(uint8_t modeId) {
    // Find the mode in the table; if not found, add it at current position
    for (uint8_t i = 0; i < MODE_TABLE_LEN; i++) {
        if (MODE_TABLE[i] == modeId) {
            _modeIdx = i;
            visualizerSetMode(modeId);
            return;
        }
    }
    // Mode not in table — set directly anyway
    visualizerSetMode(modeId);
}

// ── Init ──────────────────────────────────────────────────────────────────────

void buttonsInit() {
    _btnMode.begin(BTN_MODE_PIN);
    _btnPower.begin(BTN_POWER_PIN);
    _btnBright.begin(BTN_BRIGHT_PIN);

    // Start in MODE_SPECTRUM regardless of what ACTIVE_MODE is set to
    // in visualizer.h. The compile-time constant only affects which
    // mode code is compiled; at runtime we drive the mode through
    // visualizerSetMode(). Find MODE_SPECTRUM in the table.
    _modeIdx = 0;
    for (uint8_t i = 0; i < MODE_TABLE_LEN; i++) {
        if (MODE_TABLE[i] == MODE_SPECTRUM) { _modeIdx = i; break; }
    }

    visualizerSetMode(MODE_TABLE[_modeIdx]);
}

// ── Update — call every loop() ────────────────────────────────────────────────

void buttonsUpdate() {
    // ── MODE button: short press cycles to next effect ────────────────────────
    if (_btnMode.debounce()) {
        _modeIdx = (_modeIdx + 1) % MODE_TABLE_LEN;
        visualizerSetMode(MODE_TABLE[_modeIdx]);
    }

    // ── BRIGHTNESS button: short press steps through brightness levels ─────────
    if (_btnBright.debounce()) {
        _bleRawBrightnessActive = false;  // physical button reclaims control
        _brightIdx = (_brightIdx + 1) % BRIGHTNESS_LEVEL_COUNT;
    }

    // ── POWER button: short press toggles display, long press reserved ─────────
    bool powerRaw = (digitalRead(BTN_POWER_PIN) == LOW); // held down

    if (powerRaw && !_powerHeld) {
        // Just pressed
        _powerHeld   = true;
        _powerHeldAt = millis();
        _longFired   = false;
    }
    if (!powerRaw && _powerHeld) {
        // Just released
        _powerHeld = false;
        if (!_longFired) {
            // Short press: toggle display
            _powerOn = !_powerOn;
        }
    }
    if (_powerHeld && !_longFired
            && (millis() - _powerHeldAt) >= BTN_POWER_LONG_MS) {
        _longFired = true;
        // Long press reserved — add deep sleep or reboot here if desired
        // e.g.: esp_deep_sleep_start();
    }
}

// ── Stubs when buttons disabled ───────────────────────────────────────────────
// All public functions must still link when BUTTONS_ENABLED 0 because ble.cpp
// calls the setters unconditionally.
#else  // BUTTONS_ENABLED == 0

static bool    _powerOn   = true;
static uint8_t _rawBrite  = 150;
static uint8_t _curMode   = ACTIVE_MODE;

void buttonsInit()           {}
void buttonsUpdate()         {}
bool buttonsPowerOn()        { return _powerOn; }
uint8_t buttonsBrightness()  { return _rawBrite; }
uint8_t buttonsCurrentMode() { return _curMode; }

void buttonsSetPower(bool on)          { _powerOn  = on; }
void buttonsSetBrightness(uint8_t bri) { _rawBrite = bri; }
void buttonsSetMode(uint8_t modeId)    { _curMode  = modeId; visualizerSetMode(modeId); }

#endif // BUTTONS_ENABLED
