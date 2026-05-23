#pragma once
#include <Arduino.h>
#include "audio.h"
 
// 8 perceptually-spaced frequency bands (logarithmic, ~1/3 octave groupings)
// At 16 kHz sample rate, FFT bin resolution = 16000 / 512 = 31.25 Hz/bin
//
//  Band  Freq Range      Description
//  0     20  – 150 Hz    Sub-bass
//  1     150 – 400 Hz    Bass
//  2     400 – 800 Hz    Low-mid
//  3     800 – 2000 Hz   Mid
//  4     2k  – 4k  Hz    Upper-mid
//  5     4k  – 6k  Hz    Presence
//  6     6k  – 10k Hz    Brilliance
//  7     10k – 8k  Hz    Air (Nyquist limited to 8 kHz)
 
#define NUM_BANDS 8
 
// ══════════════════════════════════════════════════════════════════════════════
// AUTO-CALIBRATION — compile-time opt-in
// ══════════════════════════════════════════════════════════════════════════════
//
//   FFT_AUTO_CALIBRATE  1   Boot-time + continuous adaptive noise floor.
//                           fftCalibrate() must be called from setup() after
//                           audioInit() and fftInit() but before
//                           visualizerInit(). The matrix shows a dim left-to-
//                           right wipe during the measurement window so the
//                           user knows to stay quiet for ~1 second.
//
//   FFT_AUTO_CALIBRATE  0   Static NOISE_FLOOR[] constants in fft.cpp are
//                           used as-is. Tune them by hand for your hardware.
//                           fftCalibrate() compiles to a no-op stub.
//
// If the adaptive floor causes the spectrum to look dead (floor rises too
// fast during music) set FFT_AUTO_CALIBRATE 0 and hand-tune NOISE_FLOOR[].
//
#define FFT_AUTO_CALIBRATE  0
 
// ── Calibration window ────────────────────────────────────────────────────────
// Duration of the boot-time silent measurement in milliseconds.
// Keep the room quiet during this window. A dim matrix sweep plays as a cue.
// Shorter = faster boot but noisier floor estimate. 1200 ms is a safe default.
#define FFT_CALIBRATE_MS    1200
 
// ── Adaptive floor time constants (only used when FFT_AUTO_CALIBRATE 1) ───────
// The floor tracks the per-band minimum raw magnitude using an asymmetric IIR:
//   • Rises quickly when raw > floor  → suppresses sudden new noise sources
//   • Falls slowly  when raw < floor  → doesn't collapse on musical silences
//
// Coefficients are applied every FFT frame (~31 fps at 16 kHz / 512 samples).
// RISE: floor tracks slow ambient drift only — matches FALL rate so the floor
//       does not chase transients or music (~3000 frames / ~97 s to converge).
// FALL: floor decays to a lower level in ~3000 frames (~97 s)
// FFT_FLOOR_RISE_COEFF removed — replaced by per-band FLOOR_RISE_COEFF[] array in fft.cpp
#define FFT_FLOOR_FALL_COEFF  0.9997f // weight on old value when falling (slow)
 
// ── Safety margin applied to the boot-measured peak ──────────────────────────
// adaptedFloor[b] = bootPeak[b] * FFT_CALIBRATE_MARGIN after calibration.
// 2.0 = 100 % headroom above the measured noise peak. Raise if phantom bars
// appear in silence; lower if quiet music is being eaten.
#define FFT_CALIBRATE_MARGIN  1.5f
 
// Per-band magnitude (0.0 – 1.0 after normalisation)
extern float bandMagnitude[NUM_BANDS];
// Per-band raw average magnitude before floor subtraction — use for floor tuning
extern float bandRaw[NUM_BANDS];
// Per-band trust flag — 0 if band is too noisy for reliable use, 1 if trusted
extern const uint8_t BAND_TRUSTED[NUM_BANDS];
 
void fftInit();
// Run FFT on audioProcessBuffer, populate bandMagnitude[], clear audioBufferReady
void fftProcess();
 
// Boot-time silent calibration — call once from setup() when FFT_AUTO_CALIBRATE 1.
// Measures ambient noise for FFT_CALIBRATE_MS ms and seeds the adaptive floor.
// Compiles to a no-op when FFT_AUTO_CALIBRATE 0.
void fftCalibrate();