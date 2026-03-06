/*
 * XIAO ESP32-C3 — Audio-Reactive LED Matrix
 * ─────────────────────────────────────────────────────────────────────────────
 * Hardware
 *   INMP441 I2S mic : WS→D6  SCK→D7  SD→D2  L/R→GND  VDD→3.3V
 *   WS2812B 8×8     : DIN→D10  VCC→5V  GND→GND
 *
 * Libraries (Arduino Library Manager)
 *   Adafruit NeoPixel · Adafruit NeoMatrix · Adafruit GFX · arduinoFFT >= v2.x
 *
 * ── Switching modes ────────────────────────────────────────────────────────
 * Open visualizer.h and change the single #define:
 *   #define ACTIVE_MODE  MODE_SPECTRUM      <- spectrum analyser (default)
 *   #define ACTIVE_MODE  MODE_FIRE          <- fire simulation
 *   ... (see visualizer.h for full list)
 *
 * ── Serial logging ─────────────────────────────────────────────────────────
 * Serial output is optional. The device boots and runs without a serial
 * monitor connected. If DEBUG_SERIAL is 1, band magnitudes are printed
 * whenever a host is connected — safe to open/close at any time.
 */

#include "audio.h"
#include "fft.h"
#include "visualizer.h"

// Set to 1 to print band magnitudes over serial (only when monitor is open).
// Set to 0 to compile out all serial code entirely.
#define DEBUG_SERIAL  1

// Convenience macro — only calls Serial if a host is actually connected.
// Safe to use anywhere; evaluates to nothing when DEBUG_SERIAL is 0.
#if DEBUG_SERIAL
  #define DPRINT(x)   do { if (Serial) Serial.print(x);   } while(0)
  #define DPRINTLN(x) do { if (Serial) Serial.println(x); } while(0)
#else
  #define DPRINT(x)
  #define DPRINTLN(x)
#endif

void setup() {
#if DEBUG_SERIAL
    // setTxTimeoutMs(0): do not block waiting for a host to open the port.
    // Without this, Serial.begin() on the ESP32-C3 USB-CDC stalls until
    // the serial monitor is opened.
    Serial.setTxTimeoutMs(0);
    Serial.begin(115200);
    // Brief optional wait — only up to 1 second, then proceed regardless.
    // Remove the while() entirely if you never want any startup delay.
    uint32_t t = millis();
    while (!Serial && (millis() - t) < 1000) {}
#endif

    if (!audioInit()) {
        // Can't use Serial here safely — flash the matrix red to signal failure
        // instead of hanging on a serial print.
        DPRINTLN("I2S init failed - check wiring.");
        // Visual error indicator: matrix will be initialised but show solid red.
        // visualizerInit() is intentionally skipped so the error is persistent.
        while (true) delay(1000);
    }

    fftInit();
    visualizerInit();

    DPRINTLN("Ready. Active mode: " + String(ACTIVE_MODE));
}

void loop() {
    audioUpdate();

    if (audioBufferReady) {
        fftProcess();
        visualizerUpdate();

#if DEBUG_SERIAL
        // Print band magnitudes only when serial monitor is open.
        // Checking Serial (the bool operator) is non-blocking — if no host
        // is connected this entire block is skipped in microseconds.
        if (Serial) {
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