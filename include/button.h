#ifndef button_h
#define button_h

/*
 * button.h
 * ─────────────────────────────────────────────────────────────────────────────
 * Single-button debounce and event detection for ESP32 Arduino.
 *
 * ── Type correctness ───────────────────────────────────────────────────────
 * Pin number is stored and passed as int, matching the ESP32 Arduino API
 * signatures for pinMode() and digitalRead(). The D0/D3/D8 etc. constants
 * on the XIAO ESP32-C3 are uint32_t — storing them in int avoids implicit
 * narrowing and matches what the underlying GPIO driver expects.
 *
 * ── Debounce algorithm ─────────────────────────────────────────────────────
 * 16-bit shift-register. Each call to update() shifts the register left,
 * ORs in the current digitalRead() result, and masks the top 7 bits to 1.
 * A clean press  = 0xFF00 (8 consecutive LOWs  after HIGHs).
 * A clean release = 0xFFFF (8 consecutive HIGHs after LOWs).
 * This requires 8 consecutive identical readings before any state change,
 * filtering out contact bounce regardless of loop() timing.
 *
 * ── Usage ──────────────────────────────────────────────────────────────────
 * Button btn;
 *
 * void setup() { btn.begin(D3); }
 *
 * void loop() {
 *     btn.update();                    // call first, every loop()
 *
 *     if (btn.pressed())   { ... }    // true once on press
 *     if (btn.released())  { ... }    // true once on release
 *     if (btn.isHeld())    { ... }    // true every frame while held
 *     if (btn.heldDuration() > 2000) { ... }  // long press
 * }
 *
 * ── Wiring ─────────────────────────────────────────────────────────────────
 * Wire each button between its GPIO pin and GND.
 * INPUT_PULLUP is configured in begin() — no external resistor needed.
 */

#include "Arduino.h"

class Button {
  private:
    int      _pin;          // GPIO pin number — int matches ESP32 Arduino API
    uint16_t _state;        // shift-register debounce history
    bool     _isPressed;    // true while button is debounced-held
    bool     _wasPressed;   // _isPressed snapshot from previous update() call
    uint32_t _pressedAt;    // millis() when button first debounced-pressed

    void _updateState(int reading) {
        // Shift history left, OR in new reading (HIGH=1, LOW=0), keep top 7 bits set.
        // INPUT_PULLUP: not pressed = HIGH (1), pressed = LOW (0).
        _state = (_state << 1) | (reading == HIGH ? 1 : 0) | 0xFE00;

        // Snapshot previous pressed state before updating
        _wasPressed = _isPressed;

        if (_state == 0xFF00) {
            // 8 consecutive LOWs after HIGHs — clean press detected
            if (!_isPressed) {
                _isPressed = true;
                _pressedAt = millis();
            }
        } else if (_state == 0xFFFF) {
            // 8 consecutive HIGHs — fully released
            _isPressed = false;
        }
    }

  public:
    // ── Setup ────────────────────────────────────────────────────────────────
    void begin(int pin) {
        _pin        = pin;
        _state      = 0xFFFF;   // start fully released (all HIGHs)
        _isPressed  = false;
        _wasPressed = false;
        _pressedAt  = 0;
        pinMode(_pin, INPUT_PULLUP);
    }

    // ── Call once per loop(), before reading any event methods ───────────────
    void update() {
        _updateState(digitalRead(_pin));
    }

    // ── Event queries — read after update() each frame ───────────────────────

    // True for exactly one frame when a clean press is first detected
    bool pressed() const { return _isPressed && !_wasPressed; }

    // True for exactly one frame when button transitions to released
    bool released() const { return !_isPressed && _wasPressed; }

    // True every frame while button is debounced-held down
    bool isHeld() const { return _isPressed; }

    // Milliseconds since press began — returns 0 when not pressed
    uint32_t heldDuration() const {
        return _isPressed ? (millis() - _pressedAt) : 0;
    }
};

#endif