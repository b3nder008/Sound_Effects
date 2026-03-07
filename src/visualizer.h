#pragma once
/*
 * visualizer.h
 * ─────────────────────────────────────────────────────────────────────────────
 * Hardware constants and the single mode-select switch.
 *
 * ══════════════════════════════════════════════════════════════════════════════
 * HOW TO SWITCH MODES
 * ══════════════════════════════════════════════════════════════════════════════
 * Change ACTIVE_MODE to any one value from the list below. Recompile.
 * Only the selected mode's code is compiled — everything else costs zero flash.
 *
 *   MODE_SPECTRUM          Audio spectrum analyzer bars (the original display)
 *   MODE_FIRE              Fire2012 simulation, HeatColors palette
 *   MODE_TORCH             Lukas Zeller torch (orange flame, green_energy = 20)
 *   MODE_TORCH2            Torch variant with more yellow (green_energy = 80)
 *   MODE_PULSE             Expanding concentric rings, palette colours
 *   MODE_WAVE              Sine wave sweeping with dimAll trail
 *   MODE_RAINBOW_NOISE     Perlin noise field, RainbowColors palette
 *   MODE_RAINBOW_STRIPE_NOISE  Perlin noise, RainbowStripe palette
 *   MODE_PARTY_NOISE       Perlin noise, PartyColors palette
 *   MODE_FOREST_NOISE      Perlin noise, ForestColors palette
 *   MODE_CLOUD_NOISE       Perlin noise, CloudColors palette
 *   MODE_FIRE_NOISE        Perlin noise, HeatColors palette (hueReduce=60)
 *   MODE_LAVA_NOISE        Perlin noise, LavaColors palette
 *   MODE_OCEAN_NOISE       Perlin noise, OceanColors palette
 *   MODE_CONFETTI          Random palette speckles fading to black
 *   MODE_JUGGLE            3 hue-dots weaving in/out with beatsin16 positions
 *   MODE_SINELON           Dot sweeping back and forth with fading trail
 *   MODE_PRIDE             Pride2015 shifting rainbow (sin16 / beatsin88)
 *   MODE_COLOR_WAVES       colorwaves() with cycling gradient palettes
 *   MODE_RAINBOW           fill_rainbow(gHue, 1)
 *   MODE_RAINBOW_GLITTER   fill_rainbow + random white sparkle
 *   MODE_HUE_CYCLE         fill_solid with slowly cycling hue
 *   MODE_CLOUD_TWINKLES    colortwinkles — CloudColors palette
 *   MODE_RAINBOW_TWINKLES  colortwinkles — RainbowColors palette
 *   MODE_RAIN              Falling rain streaks, grey-blue, ghost trails, lightning
 *   MODE_STARFIELD         Classic warp-speed starfield, accelerating outward from center
 *   MODE_DUNE               Desert dune ridges, sand palette, slow S-curve morphing, dust twinkles
 *
 * ══════════════════════════════════════════════════════════════════════════════
 * BEAT DETECTION
 * ══════════════════════════════════════════════════════════════════════════════
 * Beat detection runs for every mode. Each mode's render function receives
 * a beatEnergy float (1.0 on a kick, decaying to 0.0 over ~7 frames) and a
 * beatFired bool (true for exactly one frame per detected beat).
 * Use them — or ignore them — inside each mode's render function.
 *
 * For MODE_SPECTRUM the beat produces a global brightness flash, exactly
 * as the original code did.
 */

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_NeoMatrix.h>

// ── Active mode — change this one line ───────────────────────────────────────
#define ACTIVE_MODE  MODE_DUNE

// ── Mode identifiers ─────────────────────────────────────────────────────────
#define MODE_SPECTRUM              0
#define MODE_FIRE                  1
#define MODE_TORCH                 2
#define MODE_TORCH2                3
#define MODE_PULSE                 4
#define MODE_WAVE                  5
#define MODE_RAINBOW_NOISE         6
#define MODE_RAINBOW_STRIPE_NOISE  7
#define MODE_PARTY_NOISE           8
#define MODE_FOREST_NOISE          9
#define MODE_CLOUD_NOISE          10
#define MODE_FIRE_NOISE           11
#define MODE_LAVA_NOISE           12
#define MODE_OCEAN_NOISE          13
#define MODE_CONFETTI             14
#define MODE_JUGGLE               15
#define MODE_SINELON              16
#define MODE_PRIDE                17
#define MODE_COLOR_WAVES          18
#define MODE_RAINBOW              19
#define MODE_RAINBOW_GLITTER      20
#define MODE_HUE_CYCLE            21
#define MODE_CLOUD_TWINKLES       22
#define MODE_RAINBOW_TWINKLES     23

// ── Hardware ──────────────────────────────────────────────────────────────────
#define MATRIX_PIN   D10
#define MATRIX_COLS  8
#define MATRIX_ROWS  8
#define NUM_LEDS     (MATRIX_COLS * MATRIX_ROWS)

// NeoMatrix wiring flags — adjust if your pixel 0 is somewhere else
#define MATRIX_TYPE  (NEO_MATRIX_BOTTOM + NEO_MATRIX_LEFT + \
                      NEO_MATRIX_ROWS   + NEO_MATRIX_ZIGZAG)

// NeoMatrix Y=0 is the physical top; our row 0 is the bottom
#define DRAW_Y(row)  ((MATRIX_ROWS - 1) - (row))

// ── Public API ────────────────────────────────────────────────────────────────
void visualizerInit();
void visualizerUpdate();
#define MODE_RAIN                 24
#define MODE_STARFIELD            25
#define MODE_DUNE                 26