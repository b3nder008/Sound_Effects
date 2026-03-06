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
#define PEAK_DECAY  0.998f   // Slower decay — prevents AGC pumping on transients

// Per-band noise floor: measured in silence, max observed x 2.0 safety margin.
// Any band average below this value is clamped to zero before normalisation.
static const float NOISE_FLOOR[NUM_BANDS] = {
    16800.0f,  // B0  sub-bass   — high EMI / board pickup
    15200.0f,  // B1  bass       — noisy
    22500.0f,  // B2  low-mid    — noisiest, likely 60 Hz harmonic coupling
     7700.0f,  // B3  mid
     2900.0f,  // B4  upper-mid  — clean, drops sharply here
      870.0f,  // B5  presence
      615.0f,  // B6  brilliance
      515.0f,  // B7  air
};

// Per-band sensitivity multiplier applied after AGC normalisation.
// The AGC produces 0.0-1.0 per band. This scales it before the renderer sees it,
// clamped to 1.0 so bars never exceed full height.
//
// Why needed even with AGC:
//   After noise floor subtraction, low bands have compressed headroom and the
//   AGC takes time to re-calibrate after quiet passages. High bands carry less
//   raw energy in typical music and would otherwise stay dim.
//
// 1.0 = neutral. Raise to make a band more responsive, lower to tame it.
// Tune by ear with a track that has clear kick, snare, vocals, and hi-hats.
static const float SENSITIVITY[NUM_BANDS] = {
    1.0f,   // B0  sub-bass   — kick drum fills this naturally
    1.0f,   // B1  bass
    1.0f,   // B2  low-mid    — watch for false triggers if raised
    1.2f,   // B3  mid        — slight boost, snare and vocals
    1.4f,   // B4  upper-mid  — often under-represented
    1.8f,   // B5  presence   — high freqs have less raw energy
    2.0f,   // B6  brilliance
    2.0f,   // B7  air        — nearly always dark without a boost
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
    for (int b = 0; b < NUM_BANDS; b++) {
        double sum = 0.0;
        int count = 0;
        for (int k = bandBinStart[b]; k < bandBinEnd[b]; k++) {
            double mag = vReal[k];
            if (mag > NOISE_FLOOR[b]) {
                sum += mag;
                count++;
            }
        }
        // Subtract noise floor so peakHold tracks only signal, not noise
        float avg = (count > 0) ? fmaxf(0.0f, (float)(sum / count) - NOISE_FLOOR[b]) : 0.0f;

        // ── 5. Normalise against decaying peak ────────────────────────────────
        peakHold[b] *= PEAK_DECAY;
        if (avg > peakHold[b]) peakHold[b] = avg;

        float normalised = (peakHold[b] > 0.0f) ? (avg / peakHold[b]) : 0.0f;

        // 6. Apply per-band sensitivity, clamp to 1.0
        normalised = fminf(1.0f, normalised * SENSITIVITY[b]);

        // ── 6. Smooth output (IIR low-pass) ───────────────────────────────────
        bandMagnitude[b] = bandMagnitude[b] * 0.5f + normalised * 0.5f;
    }
}