#include "fft.h"
#include "audio.h"
#include "ble_cal.h"
#include <arduinoFFT.h>
#include <Preferences.h>
 
// ─── FFT Buffers ──────────────────────────────────────────────────────────────
// arduinoFFT v2.x: class is now a template named ArduinoFFT<T>.
// Buffers are passed at construction and must match the template type.
static double vReal[FFT_SIZE];
static double vImag[FFT_SIZE];
static ArduinoFFT<double> FFT(vReal, vImag, FFT_SIZE, SAMPLE_RATE);
 
float bandMagnitude[NUM_BANDS] = {0};
float bandRaw[NUM_BANDS]       = {0};
 
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
 
// ─── Per-band fixed scale ─────────────────────────────────────────────────────
// Divides the above-floor signal to map to 0–1 before SENSITIVITY is applied.
// Set each value to the above-floor magnitude expected at "full scale" (loud
// music, not clipping). Starting point: ~4× the static noise floor.
// Raise a band's value if it clips; lower it if bars stay near zero.
static const float BAND_SCALE[NUM_BANDS] = {
    7200.0f,  // B0  sub-bass   (~4× floor)
    3500.0f,  // B1  bass
    1250.0f,  // B2  low-mid
     700.0f,  // B3  mid
     500.0f,  // B4  upper-mid
     440.0f,  // B5  presence
     480.0f,  // B6  brilliance
     400.0f,  // B7  air
};

// ─── Noise floor seed / static floor ─────────────────────────────────────────
// Calibrated on ESP32-S3 XIAO + INMP441 + 22Ω + 1µF RC filter.
// Derived from 4 calibration runs (R1/R2 unfiltered, R3/R4 RC-filtered).
// Values = R3+R4 p95 average × 2.0.  Re-measure if hardware changes.
static const float NOISE_FLOOR_STATIC[NUM_BANDS] = {
     1809.6f,  // B0  sub-bass    85 Hz  R3+R4 avg  SNR_min=14.1×
      871.1f,  // B1  bass       275 Hz  R3+R4 avg  SNR_min=37.2×
      313.0f,  // B2  low-mid   600 Hz  R3+R4 avg  SNR_min= 7.2×
      178.8f,  // B3  mid      1400 Hz  R3+R4 avg  SNR_min=34.0×  (reference)
      123.4f,  // B4  upper-mid 3000 Hz  R3+R4 avg  SNR_min= 9.8×
      109.6f,  // B5  presence  5000 Hz  R3+R4 avg  SNR_min= 4.9×
      120.0f,  // B6  brilliance 7000 Hz  floor identical both runs
      100.6f,  // B7  air       7500 Hz  floor identical both runs
};
 
// ─── Per-band sensitivity ─────────────────────────────────────────────────────
// Multiplier applied after AGC normalisation, clamped to 1.0.
// Derived from R3+R4 tone-peak vs floor SNR ratios.
static const float SENSITIVITY[NUM_BANDS] = {
    2.000f,  // B0  sub-bass   — conservative start; raise with kick drum music
    1.080f,  // B1  bass       — R3+R4 confirmed
    1.277f,  // B2  low-mid    — R3+R4 confirmed
    0.800f,  // B3  mid        — reference band, slightly attenuated
    3.305f,  // B4  upper-mid  — lower to 2.0 if presence feels harsh
    4.000f,  // B5  presence   — at cap; mic rolls off here; tune by ear
    4.000f,  // B6  brilliance — at cap; mic rolloff; tune by ear
    4.000f,  // B7  air        — at cap; near Nyquist; tune by ear
};

float runtimeFloor[NUM_BANDS];
float runtimeSensitivity[NUM_BANDS];
// ─── Per-band adaptive floor minimum ─────────────────────────────────────────
// Clamps adaptedFloor[] so it never collapses below measured ambient.
// Values ≈ half of R3 p95 per band.
static const float FLOOR_MIN[NUM_BANDS] = {
      402.1f,  // B0  half of R3 p95 (conservative lower bound)
      196.5f,  // B1
       59.3f,  // B2
       36.1f,  // B3
       30.7f,  // B4
       27.4f,  // B5
       30.0f,  // B6
       25.0f,  // B7
};


// ─── Per-band adaptive floor rise coefficient ─────────────────────────────────
// Applied when raw > adaptedFloor (floor tracking up toward noise/signal).
// Lower = faster rise (more aggressive noise gating).
// Replaces global FFT_FLOOR_RISE_COEFF define.
static const float FLOOR_RISE_COEFF[NUM_BANDS] = {
    0.997f,  // B0  — RC filter handles EMI; slow rise so kick drums aren't eaten
    0.995f,  // B1  — same reasoning; was 0.85 (chased transients too fast)
    0.97f,   // B2
    0.97f,   // B3
    0.95f,   // B4
    0.92f,   // B5
    0.90f,   // B6
    0.88f,   // B7
};

// ─── Per-band output IIR smoothing ───────────────────────────────────────────
// Weight applied to the previous bandMagnitude[] sample.
// Higher = more smoothing (slower response). Lower = faster/noisier.
static const float BAND_SMOOTH[NUM_BANDS] = {
    0.55f,  // B0  — faster response for kick drums
    0.60f,  // B1
    0.50f,  // B2
    0.50f,  // B3
    0.55f,  // B4
    0.60f,  // B5
    0.65f,  // B6
    0.70f,  // B7
};

// ─── Per-band trust flag ──────────────────────────────────────────────────────
// 1 = band data is reliable for visualisation. 0 = too noisy; skip or dim.
// All bands trusted post RC filter. Exposed via fft.h extern declaration.
extern const uint8_t BAND_TRUSTED[NUM_BANDS];
const uint8_t BAND_TRUSTED[NUM_BANDS] = {
    1,  // B0  sub-bass   — confirmed post RC filter
    1,  // B1  bass
    1,  // B2  low-mid
    1,  // B3  mid
    1,  // B4  upper-mid
    1,  // B5  presence
    1,  // B6  brilliance
    1,  // B7  air
};
 
void fftResetFloors() {
    for (int b = 0; b < NUM_BANDS; b++)
        runtimeFloor[b] = NOISE_FLOOR_STATIC[b];
}
 
void fftResetSens() {
    for (int b = 0; b < NUM_BANDS; b++)
        runtimeSensitivity[b] = SENSITIVITY[b];
}
 

float fftFactoryFloor(int b) { return NOISE_FLOOR_STATIC[b]; }

// ─── NVS persistence ──────────────────────────────────────────────────────────
// Namespace "ledcal" stores two blobs: "floors" (8 floats) and "sens" (8 floats).
// Writes are only triggered by explicit user calibration actions, not in the loop.
static const char* _NVS_NS     = "ledcal";
static const char* _NVS_FLOORS = "floors";
static const char* _NVS_SENS   = "sens";

void fftSaveToNVS() {
    Preferences p;
    if (!p.begin(_NVS_NS, /*readOnly=*/false)) return;
    p.putBytes(_NVS_FLOORS, runtimeFloor,       sizeof(runtimeFloor));
    p.putBytes(_NVS_SENS,   runtimeSensitivity, sizeof(runtimeSensitivity));
    p.end();
}

void fftLoadFromNVS() {
    Preferences p;
    if (!p.begin(_NVS_NS, /*readOnly=*/true)) return;
    float tmp[NUM_BANDS];
    if (p.getBytes(_NVS_FLOORS, tmp, sizeof(tmp)) == sizeof(tmp))
        memcpy(runtimeFloor, tmp, sizeof(tmp));
    if (p.getBytes(_NVS_SENS, tmp, sizeof(tmp)) == sizeof(tmp))
        memcpy(runtimeSensitivity, tmp, sizeof(tmp));
    p.end();
}

// ═════════════════════════════════════════════════════════════════════════════
// AUTO-CALIBRATION BLOCK
// Everything inside this #if is compiled out when FFT_AUTO_CALIBRATE 0.
// ═════════════════════════════════════════════════════════════════════════════
#if FFT_AUTO_CALIBRATE
 
// Active noise floor — starts at static values, updated by calibration and
// continuous adaptation. Used in place of NOISE_FLOOR_STATIC[] when enabled.
static float adaptedFloor[NUM_BANDS];
 
// ── Raw FFT pass (no windowing, no floor subtraction) ─────────────────────────
// Used only during fftCalibrate() to measure raw per-bin magnitudes cleanly.
// Returns the per-band raw average into outRaw[NUM_BANDS].
static void _fftRawBands(float outRaw[NUM_BANDS]) {
    // Wait for a fresh audio buffer — spin up to ~200 ms
    uint32_t deadline = millis() + 200;
    while (!audioBufferReady && millis() < deadline) {
        audioUpdate();
    }
    if (!audioBufferReady) {
        // No audio data available; return zeros so calibration can continue
        for (int b = 0; b < NUM_BANDS; b++) outRaw[b] = 0.0f;
        return;
    }
 
    // Copy PCM, apply Hann window
    for (int i = 0; i < FFT_SIZE; i++) {
        double w = 0.5 * (1.0 - cos(2.0 * M_PI * i / (FFT_SIZE - 1)));
        vReal[i]  = (double)audioProcessBuffer[i] * w;
        vImag[i]  = 0.0;
    }
    audioBufferReady = false;
 
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();
 
    for (int b = 0; b < NUM_BANDS; b++) {
        double sum = 0.0;
        int    cnt = 0;
        for (int k = bandBinStart[b]; k < bandBinEnd[b]; k++) {
            sum += vReal[k];
            cnt++;
        }
        outRaw[b] = (cnt > 0) ? (float)(sum / cnt) : 0.0f;
    }
}
 
// ── Boot-time calibration ──────────────────────────────────────────────────────
// Called once from setup() (via fftCalibrate() public wrapper below).
// Collects FFT_CALIBRATE_MS worth of frames, tracks the per-band peak,
// then seeds adaptedFloor[] = peak × FFT_CALIBRATE_MARGIN.
//
// Visual cue: the matrix is NOT yet initialised when this runs (visualizerInit()
// is called after fftCalibrate() in setup()), so there is nothing to drive here.
// The ~1 second delay is the implicit cue. If you want an LED indicator, move
// visualizerInit() before fftCalibrate() and add your own sweep call here.
static void _runCalibration() {
    float bootPeak[NUM_BANDS] = {0};
    uint32_t start = millis();
 
    while (millis() - start < FFT_CALIBRATE_MS) {
        audioUpdate();
 
        if (audioBufferReady) {
            float raw[NUM_BANDS];
            _fftRawBands(raw);   // consumes audioBufferReady internally
 
            for (int b = 0; b < NUM_BANDS; b++) {
                if (raw[b] > bootPeak[b]) bootPeak[b] = raw[b];
            }
        }
    }
 
    // Seed adaptive floor from the measured boot peak.
    // Guard: if the room was noisy during calibration, bootPeak can be orders
    // of magnitude above the static (lab-measured) floor and will blind the
    // floor for minutes (FALL=0.9997 decays slowly).  Cap the seed at
    // 4× NOISE_FLOOR_STATIC — enough headroom for a genuinely loud environment
    // but prevents a single transient during boot from corrupting the floor.
    for (int b = 0; b < NUM_BANDS; b++) {
        float measured = bootPeak[b] * FFT_CALIBRATE_MARGIN;
        float cap      = NOISE_FLOOR_STATIC[b] * 4.0f;
        measured       = fminf(measured, cap);
        adaptedFloor[b] = (measured > NOISE_FLOOR_STATIC[b] * 0.5f)
                            ? measured
                            : NOISE_FLOOR_STATIC[b];
    }
}
 
#endif // FFT_AUTO_CALIBRATE
 
// ─── fftInit ──────────────────────────────────────────────────────────────────
void fftInit() {
    fftResetFloors();      // seed with factory constants
    fftResetSens();
    fftLoadFromNVS();      // override with user-saved values if present
    #if FFT_AUTO_CALIBRATE
    // Seed adaptive floor with static values; fftCalibrate() will refine them.
    for (int b = 0; b < NUM_BANDS; b++) {
        adaptedFloor[b] = NOISE_FLOOR_STATIC[b];
    }
#endif
}
 
// ─── fftCalibrate — public API ────────────────────────────────────────────────
// Call from setup() after audioInit() + fftInit(), before visualizerInit().
// When FFT_AUTO_CALIBRATE 0 this compiles to an empty inline stub (see fft.h
// declaration) so the call site in setup() never needs an #if guard.
void fftCalibrate() {
#if FFT_AUTO_CALIBRATE
    _runCalibration();
#endif
}
 
// ─── fftProcess — call every loop() when audioBufferReady ─────────────────────
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
 
    for (int b = 0; b < NUM_BANDS; b++) {
        double sum = 0.0;
        int count = 0;
 
#if FFT_AUTO_CALIBRATE
        // ── Adaptive floor path ───────────────────────────────────────────────
        // Accumulate raw magnitudes (no pre-filter against the floor — the
        // floor is subtracted after averaging, not bin-by-bin).
        for (int k = bandBinStart[b]; k < bandBinEnd[b]; k++) {
            sum += vReal[k];
            count++;
        }
        float raw = (count > 0) ? (float)(sum / count) : 0.0f;
        bandRaw[b] = raw;

        // Update adaptive floor with asymmetric IIR.
        // Rise: per-band coefficient (faster for low-freq bands where EMI pools).
        // Fall: slow global decay so musical silences don't collapse the floor.
        if (raw > adaptedFloor[b]) {
            adaptedFloor[b] = adaptedFloor[b] * FLOOR_RISE_COEFF[b]
                              + raw * (1.0f - FLOOR_RISE_COEFF[b]);
        } else {
            adaptedFloor[b] = adaptedFloor[b] * FFT_FLOOR_FALL_COEFF
                              + raw * (1.0f - FFT_FLOOR_FALL_COEFF);
        }
        // Clamp floor to its measured ambient minimum so it can't collapse to zero.
        adaptedFloor[b] = fmaxf(adaptedFloor[b], FLOOR_MIN[b]);
 
        // Subtract floor — any signal below the floor is clamped to zero
        float avg = fmaxf(0.0f, raw - adaptedFloor[b]);
 
#else
        // ── Static floor path (original behaviour) ────────────────────────────
        // Only bins above the static floor contribute to the average.
        for (int k = bandBinStart[b]; k < bandBinEnd[b]; k++) {
            double mag = vReal[k];
            if (mag > runtimeFloor[b]) {
                sum += mag;
                count++;
            }
        }
        float raw_s = (count > 0) ? (float)(sum / count) : 0.0f;
        bandRaw[b] = raw_s;
        float avg = fmaxf(0.0f, raw_s - runtimeFloor[b]);
#endif
 
        // ── 5. Normalise against fixed per-band scale, apply sensitivity ──────
        float normalised = fminf(1.0f, (avg / BAND_SCALE[b]) * runtimeSensitivity[b]);

        // ── 8. Smooth output (per-band IIR low-pass) ─────────────────────────
        bandMagnitude[b] = bandMagnitude[b] * BAND_SMOOTH[b]
                         + normalised * (1.0f - BAND_SMOOTH[b]);
    }
bleCalOnFftFrame(); 
}

// ─────────────────────────────────────────────────────────────────────────────

