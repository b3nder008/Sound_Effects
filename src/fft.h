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

// Per-band magnitude (0.0 – 1.0 after normalisation)
extern float bandMagnitude[NUM_BANDS];

void fftInit();
// Run FFT on audioProcessBuffer, populate bandMagnitude[], clear audioBufferReady
void fftProcess();
