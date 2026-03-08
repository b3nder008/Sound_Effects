/*
 * XIAO ESP32-C3 — Audio-Reactive LED Matrix
 * ─────────────────────────────────────────────────────────────────────────────
 * Hardware
 *   INMP441 I2S mic : WS→D6  SCK→D7  SD→D2  L/R→GND  VDD→3.3V
 *   WS2812B 8×8     : DIN→D10  VCC→5V  GND→GND
 *   MODE button     : D3 → GND   (cycles effects)
 *   POWER button    : D4 → GND   (toggles display on/off)
 *   BRIGHTNESS btn  : D5 → GND   (steps through brightness levels)
 *
 * Libraries (Arduino Library Manager)
 *   Adafruit NeoPixel · Adafruit NeoMatrix · Adafruit GFX · arduinoFFT >= v2.x
 *   ESP32 BLE Arduino (included with esp32 Arduino core — no extra install)
 *
 * ── Quick iteration / debugging ─────────────────────────────────────────────
 * To work on a specific effect without button or BLE interference:
 *   1. buttons.h  →  #define BUTTONS_ENABLED  0
 *   2. ble.h      →  #define BLE_ENABLED      0
 *   3. visualizer.h → #define ACTIVE_MODE  MODE_YOUR_EFFECT
 *   Upload. Device boots straight into that effect at fixed brightness.
 *
 * ── BLE control (iPhone / Android) ─────────────────────────────────────────
 * See ble.cpp for full instructions. Quick start:
 *   • Install "nRF Connect for Mobile" (free, App Store / Play Store)
 *   • Scan → connect to "LedMatrix"
 *   • Write to characteristics to control power / mode / brightness
 *   • Subscribe to Status (0004) for live state updates
 */

#include "audio.h"
#include "fft.h"
#include "visualizer.h"
#include "buttons.h"
#include "ble.h"

#define DEBUG_SERIAL  1

#if DEBUG_SERIAL
  #define DPRINT(x)   do { if (Serial) Serial.print(x);   } while(0)
  #define DPRINTLN(x) do { if (Serial) Serial.println(x); } while(0)
#else
  #define DPRINT(x)
  #define DPRINTLN(x)
#endif

void setup() {
#if DEBUG_SERIAL
    Serial.setTxTimeoutMs(0);
    Serial.begin(115200);
    uint32_t t = millis();
    while (!Serial && (millis() - t) < 1000) {}
#endif

    if (!audioInit()) {
        DPRINTLN("I2S init failed - check wiring.");
        while (true) delay(1000);
    }

    fftInit();
    visualizerInit();
    buttonsInit();
    bleInit();        // starts BLE advertising as "LedMatrix"

    DPRINTLN("Ready. BLE advertising as: " BLE_DEVICE_NAME);
}

void loop() {
    // ── Control inputs (buttons + BLE) — always first ────────────────────────
    buttonsUpdate();
    bleUpdate();      // applies pending BLE writes and sends notifications

    // ── Power-off path: keep audio warm, skip render ──────────────────────────
    if (!buttonsPowerOn()) {
        audioUpdate();
        if (audioBufferReady) {
            fftProcess();
            audioBufferReady = false;
        }
        return;
    }

    // ── Normal render path ────────────────────────────────────────────────────
    audioUpdate();
    if (audioBufferReady) {
        fftProcess();
        visualizerUpdate();

#if DEBUG_SERIAL
        if (Serial) {
            Serial.print("M="); Serial.print(buttonsCurrentMode());
            Serial.print(" B="); Serial.print(buttonsBrightness());
            Serial.print(" P="); Serial.print(buttonsPowerOn() ? "on" : "off");
            Serial.print("  ");
            for (int b = 0; b < NUM_BANDS; b++) {
                Serial.print("B"); Serial.print(b); Serial.print("=");
                Serial.print(bandMagnitude[b] * 100.0f, 1);
                Serial.print("  ");
            }
            Serial.println();
        }
#endif
    }
}
