 
 /* ── Pin map ────────────────────────────────────────────────────────────────
 *
 *   Board   GPIO    Used in          Notes
 *   ──────────────────────────────────────────────────────────────────────
 *   D0      GPIO1   (free)           Available
 *   D1      GPIO2   (free)           Available
 *   D2      GPIO3   MODE_BUTTONS     On/Off button → GND
 *   D3      GPIO4   MODE_MIC         Mic WS (LRCK)  raw GPIO used for IDF
 *   D4      GPIO5   MODE_MIC         Mic SCK (BCLK) raw GPIO used for IDF
 *   D5      GPIO6   MODE_MIC         Mic SD (data)  raw GPIO used for IDF
 *   D6      GPIO43  (avoided)        UART TX — hard-clamped, conflicts Serial
 *   D7      GPIO44  MODE_BUTTONS     Brightness button → GND  ⚠ see note
 *   D8      GPIO7   MODE_BUTTONS     Mode button → GND
 *   D9      GPIO8   (free)           Available
 *   D10     GPIO10  MODE_MATRIX      NeoPixel DIN
 *
 * ── ⚠ D7 / GPIO44 warning ─────────────────────────────────────────────────
 *
 *   D7 is GPIO44, the hardware UART0 RX pin. The ESP32-S3 USB-CDC stack
 *   holds UART0 active at boot and gpio_reset_pin() cannot fully release
 *   it while Serial is running. This means the UART RX driver is weakly
 *   driving GPIO44 and may fight INPUT_PULLUP, causing the Brightness
 *   button to read as unreliably pressed or to miss presses.
 *
 *   If Brightness behaves erratically during button testing:
 *     Option A — move Brightness to D9 (GPIO8) and leave D7 free.
 *     Option B — disable Serial (comment out Serial.begin) which releases
 *                UART0, but you lose Serial Monitor output entirely.
 *
 *   D6 / GPIO43 (UART TX) is fully avoided for the same reason.
 *   D2 / GPIO3 is a strapping pin but is safe as a button input after boot.
 *
 * ── Button wiring ──────────────────────────────────────────────────────────
 *
 *   Button        XIAO    GPIO    Wire to
 *   ────────────────────────────────────────
 *   On/Off        D2      GPIO3   GND
 *   Brightness    D7      GPIO44  GND  ⚠ see D7 warning above
 *   Mode          D8      GPIO7   GND
 *
 *   INPUT_PULLUP set in firmware — no external resistors needed.
 *   D-prefix safe here: Arduino-layer digitalRead uses board aliases.
 *
 * ── Microphone wiring ──────────────────────────────────────────────────────
 *
 *   INMP441       XIAO    GPIO    Notes
 *   ──────────────────────────────────────────────────────────────────────
 *   WS  (LRCK)    D3      GPIO4   Word select
 *   SCK (BCLK)    D4      GPIO5   Bit clock
 *   SD  (data)    D5      GPIO6   Serial data
 *   L/R           GND     —       Selects left channel
 *   VDD           3.3V    —
 *   GND           GND     —
 *
 *   Raw GPIO integers used for i2s_set_pin() — NOT D-prefix.
 *   i2s_set_pin() is an ESP-IDF call below the Arduino layer; D-prefix
 *   aliases are Arduino-layer only and unreliable at IDF level on the S3.
 *
 * ── NeoPixel matrix wiring ─────────────────────────────────────────────────
 *
 *   WS2812B 8×8   XIAO    GPIO    Notes
 *   ──────────────────────────────────────────────────────────────────────
 *   DIN           D10     GPIO10  Data in — D-prefix safe, Arduino layer
 *   VCC           5V      —       Must be 5V, not 3.3V
 *   GND           GND     —
 *
 *   Use a 300-470Ω series resistor on DIN. Add 100-1000µF across VCC/GND
 *   near the matrix connector to absorb inrush current.
 *
 * ── Library required for MODE_MATRIX ───────────────────────────────────────
 *
 *   platformio.ini:  lib_deps = adafruit/Adafruit NeoPixel
 *
 * ── Serial Monitor settings ────────────────────────────────────────────────
 *
 *   MODE_BUTTONS : Serial Monitor at 115200
 *   MODE_MIC     : Serial Plotter at 115200 (waveform) or Serial Monitor
 *   MODE_MATRIX  : Serial Monitor at 115200 (progress log)
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
            Serial.print(visualizerModeName(buttonsCurrentMode()));
            Serial.print(" bri="); Serial.print(buttonsBrightness());
            Serial.print(" pwr="); Serial.print(buttonsPowerOn() ? "on" : "off");
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
