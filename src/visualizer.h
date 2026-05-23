#pragma once
/*
 * visualizer.h
 * ─────────────────────────────────────────────────────────────────────────────
 * Hardware constants, the mode registry, and the visualizer public API.
 *
 * ══════════════════════════════════════════════════════════════════════════════
 * ADDING OR RENAMING A MODE — edit exactly ONE place
 * ══════════════════════════════════════════════════════════════════════════════
 *
 * Everything — numeric IDs, button cycling order, debug names — is derived
 * from the single X-macro table VISUALIZER_MODE_TABLE below.
 *
 * To add a new mode:
 *   1. Append a new  X(MODE_MYMODE, "My mode description")  line to the table.
 *   2. Implement  renderMymode()  and  _mymodeInit()  in visualizer.cpp.
 *   3. Add  if (mode == MODE_MYMODE) _mymodeInit();  in visualizerSetMode().
 *   4. Add  case MODE_MYMODE: renderMymode(); break;  in visualizerUpdate().
 *
 * That's it.  buttons.cpp, the numeric IDs, and the button cycling order all
 * update automatically.  No other file needs to change.
 *
 * To change button cycling order, reorder rows in VISUALIZER_MODE_TABLE.
 * To exclude a mode from button cycling, comment out its row.
 *
 * ══════════════════════════════════════════════════════════════════════════════
 * HOW IT WORKS — X-macro pattern
 * ══════════════════════════════════════════════════════════════════════════════
 *
 * VISUALIZER_MODE_TABLE(X) expands X(id, description) for every mode.
 * Different consumers define X differently:
 *
 *   Numeric IDs (enum):
 *     #define X(id, desc) id,
 *     enum { VISUALIZER_MODE_TABLE(X) };        → MODE_SPECTRUM=0, etc.
 *     #undef X
 *
 *   Button cycling array (buttons.cpp):
 *     #define X(id, desc) id,
 *     static const uint8_t MODE_TABLE[] = { VISUALIZER_MODE_TABLE(X) };
 *     #undef X
 *
 *   Debug name lookup:
 *     #define X(id, desc) desc,
 *     static const char* MODE_NAMES[] = { VISUALIZER_MODE_TABLE(X) };
 *     #undef X
 *
 * ══════════════════════════════════════════════════════════════════════════════
 * BEAT DETECTION
 * ══════════════════════════════════════════════════════════════════════════════
 * Beat detection runs for every mode. Each mode's render function receives
 * a beatEnergy float (1.0 on a kick, decaying to 0.0 over ~7 frames) and a
 * beatFired bool (true for exactly one frame per detected beat).
 */

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_NeoMatrix.h>

// ══════════════════════════════════════════════════════════════════════════════
// MODE REGISTRY — edit here only
// X(identifier, "description")
// Order = button cycling order. Comment out a row to exclude from cycling.
// ══════════════════════════════════════════════════════════════════════════════
#define VISUALIZER_MODE_TABLE(X) \
    X(MODE_SPECTRUM,             "Spectrum analyser bars")            \
    X(MODE_FIRE,                 "Fire2012 simulation")               \
    X(MODE_TORCH,                "Torch (cool)")                      \
    X(MODE_TORCH2,               "Torch (warm)")                      \
    X(MODE_RAIN,                 "Falling rain + lightning")          \
    X(MODE_STARFIELD,            "Warp-speed starfield")              \
    X(MODE_WISP,                 "Will-o'-the-wisp")                  \
    X(MODE_PCBA,                 "PCB traces")                        \
    X(MODE_ATOM,                 "Bohr atom model")                   \
    X(MODE_GEOMETRIC,            "Geometric rectangles")              \
    X(MODE_DUNE,                 "Desert dunes")                      \
    X(MODE_PULSE,                "Expanding rings")                   \
    X(MODE_WAVE,                 "Sine wave sweep")                   \
    X(MODE_RAINBOW_NOISE,        "Rainbow Perlin noise")              \
    X(MODE_RAINBOW_STRIPE_NOISE, "Rainbow stripe noise")              \
    X(MODE_PARTY_NOISE,          "Party noise")                       \
    X(MODE_FOREST_NOISE,         "Forest noise")                      \
    X(MODE_CLOUD_NOISE,          "Cloud noise")                       \
    X(MODE_FIRE_NOISE,           "Fire noise")                        \
    X(MODE_LAVA_NOISE,           "Lava noise")                        \
    X(MODE_OCEAN_NOISE,          "Ocean noise")                       \
    X(MODE_CONFETTI,             "Confetti speckles")                 \
    X(MODE_PRIDE,                "Pride shifting rainbow")            \
    X(MODE_COLOR_WAVES,          "Color waves")                       \
    X(MODE_CLOUD_TWINKLES,       "Cloud twinkles")                    \
    X(MODE_RAINBOW_TWINKLES,     "Rainbow twinkles")                  \
    X(MODE_SINELON,              "Sinelon dot")                       \
    X(MODE_JUGGLE,               "Juggle dots")                       \
    X(MODE_LIFE,                 "Conway's Game of Life")             \
// ── Auto-generate numeric IDs from the table ──────────────────────────────────
// Each MODE_xxx constant equals its zero-based position in the table.
// Never assign these manually — let the enum do it.
#define _VMODE_X_ENUM(id, desc) id,
enum { VISUALIZER_MODE_TABLE(_VMODE_X_ENUM) _VMODE_COUNT };
#undef _VMODE_X_ENUM

// ── Compile-time default mode ─────────────────────────────────────────────────
// Used when BUTTONS_ENABLED 0 — device boots straight into this effect.
// When BUTTONS_ENABLED 1 the MODE button overrides this at runtime via buttons.
#define ACTIVE_MODE  MODE_ATOM

// ── Hardware ──────────────────────────────────────────────────────────────────
#define MATRIX_PIN   D10
#define MATRIX_COLS  8
#define MATRIX_ROWS  8
#define NUM_LEDS     (MATRIX_COLS * MATRIX_ROWS)

#define MATRIX_TYPE  (NEO_MATRIX_BOTTOM + NEO_MATRIX_LEFT + \
                      NEO_MATRIX_ROWS   + NEO_MATRIX_ZIGZAG)

// NeoMatrix Y=0 is the physical top; our row 0 is the bottom
#define DRAW_Y(row)  ((MATRIX_ROWS - 1) - (row))

// ── Public API ────────────────────────────────────────────────────────────────
void visualizerInit();
void visualizerUpdate();
void visualizerSetMode(uint8_t mode);  // runtime mode switch
uint8_t visualizerModeCount();         // returns _VMODE_COUNT
const char* visualizerModeName(uint8_t mode); // returns description string
void visualizerBlank();   // blank the matrix immediately, call on power-off