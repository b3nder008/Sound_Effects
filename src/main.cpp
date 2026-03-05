/*
 * XIAO ESP32-C3 — Audio-Reactive LED Matrix
 * ─────────────────────────────────────────────────────────────────────────────
 * Hardware
 *   INMP441 I2S mic : WS→D6  SCK→D7  SD→D2  L/R→GND  VDD→3.3V
 *   WS2812B 8×8     : DIN→D10  VCC→5V  GND→GND
 *
 * Libraries (Arduino Library Manager)
 *   Adafruit NeoPixel · Adafruit NeoMatrix · Adafruit GFX · arduinoFFT ≥ v2.x
 *
 * ══════════════════════════════════════════════════════════════════════════════
 * SWITCHING MODES
 * ══════════════════════════════════════════════════════════════════════════════
 * Open visualizer.h and change the single #define:
 *
 *   #define ACTIVE_MODE  MODE_SPECTRUM      ← the original spectrum analyser
 *   #define ACTIVE_MODE  MODE_FIRE          ← fire simulation
 *   #define ACTIVE_MODE  MODE_TORCH         ← candle/torch flame
 *   #define ACTIVE_MODE  MODE_TORCH2        ← torch, more yellow
 *   #define ACTIVE_MODE  MODE_PULSE         ← expanding rings
 *   #define ACTIVE_MODE  MODE_WAVE          ← sine wave sweep
 *   #define ACTIVE_MODE  MODE_RAINBOW_NOISE ← Perlin noise, rainbow colours
 *   #define ACTIVE_MODE  MODE_PARTY_NOISE   ← Perlin noise, party colours
 *   #define ACTIVE_MODE  MODE_FIRE_NOISE    ← Perlin noise, heat palette
 *   #define ACTIVE_MODE  MODE_LAVA_NOISE    ← Perlin noise, lava palette
 *   #define ACTIVE_MODE  MODE_OCEAN_NOISE   ← Perlin noise, ocean palette
 *   #define ACTIVE_MODE  MODE_CONFETTI      ← random colour speckles
 *   #define ACTIVE_MODE  MODE_JUGGLE        ← weaving colour dots
 *   #define ACTIVE_MODE  MODE_SINELON       ← sweeping dot with trail
 *   #define ACTIVE_MODE  MODE_PRIDE         ← Pride2015 shifting rainbow
 *   #define ACTIVE_MODE  MODE_COLOR_WAVES   ← colour waves, cycling palettes
 *   #define ACTIVE_MODE  MODE_RAINBOW       ← fill_rainbow
 *   #define ACTIVE_MODE  MODE_RAINBOW_GLITTER ← rainbow + white sparkle
 *   #define ACTIVE_MODE  MODE_HUE_CYCLE     ← solid slowly cycling hue
 *   #define ACTIVE_MODE  MODE_CLOUD_TWINKLES  ← cloud-coloured twinkle stars
 *   #define ACTIVE_MODE  MODE_RAINBOW_TWINKLES← rainbow twinkle stars
 *
 * Only the selected mode is compiled. All others cost zero flash or RAM.
 *
 * ══════════════════════════════════════════════════════════════════════════════
 * BEAT DETECTION
 * ══════════════════════════════════════════════════════════════════════════════
 * Beat detection runs automatically for every mode. Each mode's render
 * function uses beatFired and beatEnergy to modulate its own parameters.
 * No extra configuration needed.
 */

#include "audio.h"
#include "fft.h"
#include "visualizer.h"

void setup() {
    Serial.begin(115200);
    delay(200);

    if (!audioInit()) {
        Serial.println("I2S init failed — check wiring.");
        while (true) delay(1000);
    }
    fftInit();
    visualizerInit();

    Serial.println("Ready. Active mode: " + String(ACTIVE_MODE));
}

void loop() {
    audioUpdate();
    if (audioBufferReady) {
        fftProcess();
        visualizerUpdate();
    }
}
