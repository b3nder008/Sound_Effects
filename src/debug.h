#pragma once
// ─── Debug serial configuration ───────────────────────────────────────────────
//
// DEBUG_SERIAL     Master switch. Set 0 to eliminate ALL serial output and
//                  free the UART entirely. When 0, every sub-selector below
//                  is also silenced regardless of its value.
//
// Per-mode selectors — only active when DEBUG_SERIAL 1:
//
//   DEBUG_FFT_BANDS  Print FFT band magnitude + raw value every audio frame
//                    (from main.cpp). Produces one line per loop() call —
//                    fast and noisy. Useful for floor calibration; disable
//                    when working on visual modes.
//
//   DEBUG_LIFE       Print Conway's Game of Life state after every simulation
//                    step (from visualizer.cpp). Outputs: generation count,
//                    active seed category, palette, live cell count, stasis
//                    counter, and an 8×8 ASCII art grid of the live cells.
//                    Also prints on seed change, reset, and beat-flip events.
// ─────────────────────────────────────────────────────────────────────────────
#define DEBUG_SERIAL    1

#define DEBUG_FFT_BANDS 1   // 0 = silent  1 = band magnitudes every frame
#define DEBUG_LIFE      0   // 0 = silent  1 = GoL per-step state + grid
