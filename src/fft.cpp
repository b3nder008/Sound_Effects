#include "fft.h"
#include "audio.h"
#include <arduinoFFT.h>

// ─── FFT Buffers ──────────────────────────────────────────────────────────────
// arduinoFFT v2.x: class is now a template named ArduinoFFT<T>.
// Buffers are passed at construction and must match the template type.
static double vReal[FFT_SIZE];
static double vImag[FFT_SIZE];
static ArduinoFFT<double> FFT(vReal, vImag, FFT_SIZE, SAMPLE_RATE);

float bandMagnitude[NUM_BANDS] = {0};

/*
Band	Frequency Range	What Lives Here
1	20 – 60 Hz	Sub-bass rumble, kick thump
2	60 – 120 Hz	Bass guitar body
3	120 – 250 Hz	Low mids, warmth
4	250 – 500 Hz	Vocal fullness
5	500 Hz – 1 kHz	Vocal clarity
6	1 – 2 kHz	Presence, speech
7	2 – 6 kHz	Attack, snare crack
8	6 – 16 kHz	Air, sparkle, hiss
*/


// ─── Perceptual Band Bin Ranges ───────────────────────────────────────────────
// Bin = frequency / (SAMPLE_RATE / FFT_SIZE) = frequency / 31.25
// Limits below are [startBin, endBin) inclusive of start, exclusive of end.
static const uint16_t bandBinStart[NUM_BANDS] = {
     1,   //  20 Hz  → bin   1
     5,   // 150 Hz  → bin   5
    13,   // 400 Hz  → bin  13
    26,   // 800 Hz  → bin  26
    64,   //   2 kHz → bin  64
   128,   //   4 kHz → bin 128
   192,   //   6 kHz → bin 192
   224    //   7 kHz → bin 224
};
static const uint16_t bandBinEnd[NUM_BANDS] = {
     5,   // 150 Hz
    13,   // 400 Hz
    26,   // 800 Hz
    64,   //   2 kHz
   128,   //   4 kHz
   192,   //   6 kHz
   224,   //   7 kHz
   256    //   8 kHz (Nyquist for 16 kHz SR)
};

// ─── Peak tracking for normalisation ─────────────────────────────────────────
static float peakHold[NUM_BANDS];
//#define PEAK_DECAY   0.94f   // Slowly drop peak envelope

// Per-band noise gates — low bands need much higher gates than high bands.
// Tune each value independently until that band goes dark in silence.
static const float noiseFl[NUM_BANDS] = {
    10000.0f,  // Band 0: sub-bass  — nearly unusable, gate hard (was 2400)
    8000.0f,  // Band 1: bass      — marginal, gate hard  (was 2100)
    1500.0f,  // Band 2: low-mid
    580.0f,  // Band 3: mid
    490.0f,  // Band 4: upper-mid
    730.0f,  // Band 5: presence
    810.0f,  // Band 6: brilliance (was 170)
    840.0f,  // Band 7: air (was 160)
};
// Higher value = less sensitive (bars sit lower).
// Lower value = more sensitive (bars reach top more easily).
// Tune by playing music and adjusting each band until the display looks balanced.
static const float sensitivity[NUM_BANDS] = {
    30000.0f,  // Band 0: small range above gate, will rarely trigger
    26500.0f,  // Band 1: B1 peaked at ~3600, gate at 1200, range = 2400
    10500.0f,  // Band 2: B2 peaked at ~3400, gate at 450, range = 3000
    8000.0f,  // Band 3: B3 peaked at 12151, gate at 280, range ~12000
    5100.0f,  // Band 4: B4 peaked at 20364, gate at 160, range ~20000
    4600.0f,  // Band 5: small range, keep sensitive
    900.0f,  // Band 6: very small range
    900.0f,  // Band 7: very small range
};


void fftInit() {
    memset(peakHold, 0, sizeof(peakHold));
}

void fftProcess() {
    if (!audioBufferReady) return;

    // ── 1. Copy int16 PCM into double real buffer, apply Hann window ──────────
    for (int i = 0; i < FFT_SIZE; i++) {
        double window = 0.5 * (1.0 - cos(2.0 * M_PI * i / (FFT_SIZE - 1)));
        vReal[i] = (double)audioProcessBuffer[i] * window;
        vImag[i] = 0.0;
    }

    // ── 2. Signal buffer consumed — release double buffer ─────────────────────
    audioBufferReady = false;

    // ── 3. Forward FFT ────────────────────────────────────────────────────────
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude(); // results in vReal[0..FFT_SIZE/2]

    // ── 4. Accumulate bands ───────────────────────────────────────────────────
    static uint32_t lastPrint = 0;
    bool doPrint = (millis() - lastPrint > 500); //remove comment to serial print values
    //bool doPrint = false; //comment out to unblock serial print values

    if (doPrint) lastPrint = millis();

    for (int b = 0; b < NUM_BANDS; b++) {
        double sum = 0.0;
        int count = 0;
        for (int k = bandBinStart[b]; k < bandBinEnd[b]; k++) {
            double mag = vReal[k];
            if (mag > noiseFl[b]) {
                sum += mag;
                count++;
            }
        }
        float avg = (count > 0) ? (float)(sum / count) : 0.0f;

        // ── Print RAW avg before any normalisation ────────────────────────────
        if (doPrint) {
            Serial.print("B"); Serial.print(b);
            Serial.print("="); Serial.print(avg, 1);
            Serial.print("  ");
        }

        // ── Fixed scale normalisation ─────────────────────────────────────────
        float normalised = avg / sensitivity[b];
        if (normalised > 1.0f) normalised = 1.0f;

        // ── IIR smoothing ─────────────────────────────────────────────────────
        bandMagnitude[b] = bandMagnitude[b] * 0.3f + normalised * 0.7f;
    }
    if (doPrint) Serial.println();
/*
    // ── Diagnostic: print raw avg per band to Serial ──────────────────────
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 500) {   // Print twice per second
        lastPrint = millis();
        for (int b = 0; b < NUM_BANDS; b++) {
            Serial.print("B");
            Serial.print(b);
            Serial.print("=");
            Serial.print(bandMagnitude[b] * sensitivity[b], 1); // Print raw avg
            Serial.print("  ");
        }
        Serial.println();
    }
        */
}