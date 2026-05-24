/*
 * visualizer.cpp
 * ─────────────────────────────────────────────────────────────────────────────
 * Architecture
 * ─────────────────────────────────────────────────────────────────────────────
 *
 *   ┌─────────────────────────────────────────┐
 *   │              visualizerUpdate()          │
 *   │                                          │
 *   │  1. beatDetect()  ← runs every frame     │
 *   │       │                                  │
 *   │       └─► beatFired / beatEnergy         │
 *   │                    │                     │
 *   │  2. renderXxx() ◄──┘  (one per mode)     │
 *   │       │                                  │
 *   │  3. matrix.show()                        │
 *   └─────────────────────────────────────────┘
 *
 * Beat detection is a shared service. Every render function receives
 * beatFired and beatEnergy and uses them however is appropriate for
 * that mode. MODE_SPECTRUM uses them for a brightness flash.
 * Ambient modes use them to modulate sparking rate, speed, colour shift, etc.
 *
 * Only the render function selected by ACTIVE_MODE is compiled.
 * All others are compiled out entirely by #if / #elif chains.
 */

#include "visualizer.h"
#include "fft.h"
#include "fastled_compat.h"   // CRGB, inoise8, beatsin88, sin16, palettes…
#include "debug.h"            // DEBUG_SERIAL, DEBUG_LIFE

// ─── NeoMatrix ───────────────────────────────────────────────────────────────
static Adafruit_NeoMatrix matrix(
    MATRIX_COLS, MATRIX_ROWS,
    MATRIX_PIN,
    MATRIX_TYPE,
    NEO_GRB + NEO_KHZ800
);

// ─── Global brightness ───────────────────────────────────────────────────────
// BRIGHTNESS resolves to the button-controlled level when buttons are
// enabled, or a fixed default when BUTTONS_ENABLED 0. Every render
// function calls matrix.setBrightness(BRIGHTNESS) — no other changes needed.
#include "buttons.h"
#if BUTTONS_ENABLED
  #define BRIGHTNESS  buttonsBrightness()
#else
  #define BRIGHTNESS  150   // fixed default when buttons disabled
#endif

// ─── Beat detector ───────────────────────────────────────────────────────────
// Shared by all modes. Outputs:
//   beatFired  — true for exactly one frame when a kick is detected
//   beatEnergy — 1.0 on that frame, decays by BEAT_DECAY each subsequent frame

#define BEAT_BAND           2     // which FFT band to watch (mid-bass works well)
#define BEAT_RISE_THRESH  0.20f   // minimum rise in smoothed magnitude to count
#define BEAT_MIN_INTERVAL  12     // minimum frames between beats (debounce)
#define BEAT_SMOOTH       0.40f   // IIR coefficient for magnitude smoother
#define BEAT_DECAY        0.18f   // how fast beatEnergy falls per frame (1/~6 frames)

// For MODE_SPECTRUM only: brightness flash on beat
#define BEAT_PULSE_FRAMES       6
#define BEAT_PULSE_BRIGHTNESS 255

static float   _beatSmoothed  = 0.0f;
static float   _beatPrev      = 0.0f;
static uint8_t _beatCooldown  = 0;
static uint8_t _beatPulseFrames = 0;  // counts down for spectrum flash
static float   beatEnergy     = 0.0f; // global — used by all render functions
static bool    beatFired      = false; // global — true for one frame

// ─── Runtime mode ────────────────────────────────────────────────────────────
// Declared here so all render functions can reference it.
// Defined and managed by visualizerSetMode() near the bottom of this file.
static uint8_t _runtimeMode = ACTIVE_MODE;

static void beatDetect() {
    float raw = bandMagnitude[BEAT_BAND];
    _beatSmoothed += (raw - _beatSmoothed) * BEAT_SMOOTH;
    float rise = _beatSmoothed - _beatPrev;
    _beatPrev = _beatSmoothed;

    if (_beatCooldown > 0)      _beatCooldown--;
    if (_beatPulseFrames > 0)   _beatPulseFrames--;

    beatFired = false;
    if (rise > BEAT_RISE_THRESH && _beatCooldown == 0) {
        _beatCooldown    = BEAT_MIN_INTERVAL;
        _beatPulseFrames = BEAT_PULSE_FRAMES;
        beatFired        = true;
        beatEnergy       = 1.0f;
    }

    // Decay energy every frame
    if (!beatFired && beatEnergy > 0.0f) {
        beatEnergy -= BEAT_DECAY;
        if (beatEnergy < 0.0f) beatEnergy = 0.0f;
    }
}

// ─── Shared: XY mapping ───────────────────────────────────────────────────────
// Matches the original torchv2.ino XY() exactly.
// Used only by the ambient-effect render functions that maintain a leds[] buffer.
static inline uint16_t XY(int16_t x, int16_t y) {
    if (x < 0) x = 0; if (x >= MATRIX_COLS) x = MATRIX_COLS - 1;
    if (y < 0) y = 0; if (y >= MATRIX_ROWS) y = MATRIX_ROWS - 1;
    uint16_t i = (uint16_t)(y * MATRIX_COLS) + (MATRIX_COLS - x);
    i = (NUM_LEDS - 1) - i;
    return (i < NUM_LEDS) ? i : NUM_LEDS - 1;
}

// Flush a CRGB leds[] buffer to NeoMatrix (used by ambient modes)
static void flushLeds(CRGB* leds) {
    for (int x = 0; x < MATRIX_COLS; x++)
        for (int y = 0; y < MATRIX_ROWS; y++)
            matrix.drawPixel(x, DRAW_Y(y),
                matrix.Color(leds[XY(x,y)].r,
                             leds[XY(x,y)].g,
                             leds[XY(x,y)].b));
}

static void dimAll(CRGB* leds, uint8_t value) {
    for (int i = 0; i < NUM_LEDS; i++) leds[i].nscale8(value);
}

void visualizerBlank() {
    matrix.fillScreen(0);
    matrix.show();
    // Reset FFT bands so first frame after power-on is clean
    memset(bandMagnitude, 0, sizeof(bandMagnitude));
}

// ═════════════════════════════════════════════════════════════════════════════
// MODE_SPECTRUM — audio spectrum analyser bars
// Unchanged from the original working implementation.
// Beat modulation: global brightness flash on kick.
// ═════════════════════════════════════════════════════════════════════════════

// Tuning
#define PEAK_HOLD_FRAMES   12
#define PEAK_GRAVITY     0.06f
#define PEAK_GLOW_ALPHA   120
#define BAR_SMOOTH       0.55f
#define GHOST_DECAY      0.75f

static float   _barHeight[MATRIX_COLS]                = {0};
static float   _peakPos  [MATRIX_COLS]                = {0};
static float   _peakVel  [MATRIX_COLS]                = {0};
static uint8_t _peakHold [MATRIX_COLS]                = {0};
static float   _ghost    [MATRIX_COLS][MATRIX_ROWS]   = {{0}};

// Standard HSV → NeoMatrix 16-bit colour (no FastLED dependency needed here)
static uint16_t _hsv565(uint8_t h, uint8_t s, uint8_t v) {
    uint8_t r, g, b;
    if (s == 0) { r = g = b = v; }
    else {
        uint8_t region = h / 43;
        uint8_t rem    = (h - region * 43) * 6;
        uint8_t p = (uint16_t)v * (255 - s) >> 8;
        uint8_t q = (uint16_t)v * (255 - ((uint16_t)s * rem >> 8)) >> 8;
        uint8_t t = (uint16_t)v * (255 - ((uint16_t)s * (255 - rem) >> 8)) >> 8;
        switch (region) {
            case 0: r=v; g=t; b=p; break;
            case 1: r=q; g=v; b=p; break;
            case 2: r=p; g=v; b=t; break;
            case 3: r=p; g=q; b=v; break;
            case 4: r=t; g=p; b=v; break;
            default: r=v; g=p; b=q; break;
        }
    }
    return matrix.Color(r, g, b);
}
static uint16_t _barColor(uint8_t row, uint8_t brightness) {
    uint8_t hue = map(row, 0, MATRIX_ROWS - 1, 160, 0);
    return _hsv565(hue, 255, brightness);
}

static void renderSpectrum() {
    // Beat → global brightness flash (exact original behaviour)
    uint8_t bri = BRIGHTNESS;
    if (_beatPulseFrames > 0) {
        float t     = (float)_beatPulseFrames / BEAT_PULSE_FRAMES;
        float eased = t * t;
        bri = (uint8_t)min(255, (int)BRIGHTNESS +
              (int)(eased * (BEAT_PULSE_BRIGHTNESS - BRIGHTNESS)));
    }
    matrix.setBrightness(bri);
    matrix.fillScreen(0);

    for (uint8_t col = 0; col < MATRIX_COLS; col++) {
        // 1. IIR-smooth bar height — reverse band index so bass (B0) is left, treble right
        float target = bandMagnitude[(NUM_BANDS - 1) - col] * MATRIX_ROWS;
        if (target < 0.05f * MATRIX_ROWS) target = 0.0f;
        _barHeight[col] += (target - _barHeight[col]) * BAR_SMOOTH;
        float bh = min((float)MATRIX_ROWS, _barHeight[col]);

        uint8_t fullRows = (uint8_t)bh;
        float   frac     = bh - fullRows;

        // 2. Decay ghost trail
        for (uint8_t row = 0; row < MATRIX_ROWS; row++)
            _ghost[col][row] *= GHOST_DECAY;

        // 3. Full bar rows
        for (uint8_t row = 0; row < fullRows; row++) {
            _ghost[col][row] = fmaxf(_ghost[col][row], 255.0f);
            matrix.drawPixel(col, row, _barColor(row, 255));
        }

        // 4. Fractional tip pixel
        if (fullRows < MATRIX_ROWS && frac > 0.01f) {
            uint8_t tipBright = (uint8_t)(frac * 255.0f);
            _ghost[col][fullRows] = fmaxf(_ghost[col][fullRows], (float)tipBright);
            matrix.drawPixel(col, fullRows, _barColor(fullRows, tipBright));
        }

        // 5. Ghost trail below bar tip
        for (uint8_t row = fullRows + 1; row < MATRIX_ROWS; row++) {
            uint8_t gb = (uint8_t)_ghost[col][row];
            if (gb > 4)
                matrix.drawPixel(col, row, _barColor(row, gb));
        }

        // 6. Peak dot: latch to rising bar
        if (bh >= _peakPos[col]) {
            _peakPos[col]  = bh;
            _peakVel[col]  = 0.0f;
            _peakHold[col] = PEAK_HOLD_FRAMES;
        }
        // 7. Peak dot: hold, then gravity fall
        if (_peakHold[col] > 0) {
            _peakHold[col]--;
        } else {
            _peakVel[col] += PEAK_GRAVITY;
            _peakPos[col] -= _peakVel[col];
            if (_peakPos[col] < 0.0f) { _peakPos[col] = 0.0f; _peakVel[col] = 0.0f; }
        }

        // 8. Peak dot + warm halo
        uint8_t pRow = (uint8_t)_peakPos[col];
        if (pRow < MATRIX_ROWS && _peakPos[col] > 0.5f) {
            matrix.drawPixel(col, pRow, matrix.Color(255, 255, 255));
            if (pRow > 0)
                matrix.drawPixel(col, pRow - 1,
                    matrix.Color(PEAK_GLOW_ALPHA, PEAK_GLOW_ALPHA, PEAK_GLOW_ALPHA >> 1));
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// MODE_FIRE — Fire2012WithPalette.h (exact port)
// Beat modulation: increases sparking probability.
// ═════════════════════════════════════════════════════════════════════════════

#define _FIRE_COOLING   55
#define _FIRE_SPARKING  50

static CRGB _fireLeds[NUM_LEDS];
static uint8_t _fireHeat[MATRIX_COLS][MATRIX_ROWS];

static void renderFire() {
    uint8_t sparking = _FIRE_SPARKING +
                       (uint8_t)(beatEnergy * 80.0f); // beat boosts sparks

    for (uint8_t x = 0; x < MATRIX_COLS; x++) {
        // Step 1: cool
        for (int i = 0; i < MATRIX_ROWS; i++)
            _fireHeat[x][i] = qsub8(_fireHeat[x][i],
                random8(0, ((_FIRE_COOLING * 10) / MATRIX_ROWS) + 2));
        // Step 2: drift up
        for (int k = MATRIX_ROWS - 1; k >= 2; k--)
            _fireHeat[x][k] = (_fireHeat[x][k-1] +
                                _fireHeat[x][k-2] +
                                _fireHeat[x][k-2]) / 3;
        // Step 3: spark
        if (random8() < sparking) {
            int y = random8(2);
            _fireHeat[x][y] = qadd8(_fireHeat[x][y], random8(160, 255));
        }
        // Step 4: colour
        for (int j = 0; j < MATRIX_ROWS; j++) {
            uint8_t ci = scale8(_fireHeat[x][j], 240);
            _fireLeds[XY(x, (MATRIX_ROWS-1)-j)] =
                ColorFromPalette(HeatColors_p, ci);
        }
    }
    matrix.setBrightness(BRIGHTNESS);
    flushLeds(_fireLeds);
}

// ═════════════════════════════════════════════════════════════════════════════
// MODE_TORCH / MODE_TORCH2 — Torch.h / Torch2.h (exact port)
// Beat modulation: increases spark probability.
// ═════════════════════════════════════════════════════════════════════════════

static const uint8_t _energymap[32] = {
    0,64,96,112,128,144,152,160,168,176,184,184,192,200,200,208,
    208,216,216,224,224,224,232,232,232,240,240,240,240,248,248,248
};
enum { _TP=0, _TN=1, _TS=2, _TST=3 }; // passive, nop, spark, spark_temp

static CRGB    _torchLeds[NUM_LEDS];
static uint8_t _tCur[NUM_LEDS], _tNxt[NUM_LEDS], _tMode[NUM_LEDS];

static inline void _tReduce(uint8_t& b, uint8_t amt) {
    int r = (int)b - amt; b = r < 0 ? 0 : (uint8_t)r;
}
static inline void _tIncrease(uint8_t& b, uint8_t amt) {
    int r = (int)b + amt; b = r > 255 ? 255 : (uint8_t)r;
}
static uint16_t _tRand(uint16_t lo, uint16_t hi) {
    return lo + (uint16_t)(rand() % (hi - lo + 1));
}

static void renderTorch() {
    const uint8_t  FLAME_MIN  = 100, FLAME_MAX  = 220;
    const uint8_t  SPARK_MIN  = 200, SPARK_MAX  = 255;
    const uint8_t  SPARK_TFR  = 40;
    const uint16_t SPARK_CAP  = 200;
    const uint16_t UP_RAD     = 40,  SIDE_RAD   = 35, HEAT_CAP = 0;
    const uint8_t  RED_BIAS   = 10,  GREEN_BIAS = 0,  BLUE_BIAS = 0;
    const int      RED_EN     = 180, BLUE_EN    = 0;
    // Torch2 has green_energy=80; Torch has green_energy=20
    const int GREEN_EN = (_runtimeMode == MODE_TORCH2) ? 80 : 20;
    const uint8_t RED_BG = 0, GREEN_BG = 0, BLUE_BG = 0;

    // Beat boosts spark probability
    uint8_t sparkProb = 2 + (uint8_t)(beatEnergy * 12.0f);

    // Inject random energy at bottom rows (injectRandom)
    for (int i = 0; i < MATRIX_COLS; i++) {
        _tCur[i]  = _tRand(FLAME_MIN, FLAME_MAX);
        _tMode[i] = _TN;
    }
    for (int i = MATRIX_COLS; i < 2 * MATRIX_COLS; i++) {
        if (_tMode[i] != _TS && _tRand(0, 99) < sparkProb) {
            _tCur[i]  = _tRand(SPARK_MIN, SPARK_MAX);
            _tMode[i] = _TS;
        }
    }

    // calcNextEnergy
    int idx = 0;
    for (int y = 0; y < MATRIX_ROWS; y++) {
        for (int x = 0; x < MATRIX_COLS; x++) {
            uint8_t e = _tCur[idx];
            switch (_tMode[idx]) {
                case _TS: {
                    _tReduce(e, SPARK_TFR);
                    if (y < MATRIX_ROWS - 1) _tMode[idx + MATRIX_COLS] = _TST;
                    break;
                }
                case _TST: {
                    uint8_t e2 = _tCur[idx - MATRIX_COLS];
                    if (e2 < SPARK_TFR) {
                        _tMode[idx - MATRIX_COLS] = _TP;
                        _tIncrease(e, e2);
                        e = ((int)e * SPARK_CAP) >> 8;
                        _tMode[idx] = _TS;
                    } else {
                        _tIncrease(e, SPARK_TFR);
                    }
                    break;
                }
                case _TP: {
                    e = ((int)e * HEAT_CAP) >> 8;
                    int left  = (x > 0)             ? _tCur[idx-1]           : 0;
                    int right = (x < MATRIX_COLS-1) ? _tCur[idx+1]           : 0;
                    int above = (y < MATRIX_ROWS-1) ? _tCur[idx+MATRIX_COLS] : 0;
                    _tIncrease(e, (uint8_t)(((left + right) * SIDE_RAD) >> 9)
                                + (uint8_t)((above * UP_RAD) >> 8));
                    break;
                }
                default: break;
            }
            _tNxt[idx++] = e;
        }
    }

    // calcNextColors
    for (int i = 0; i < NUM_LEDS; i++) {
        uint16_t e = _tNxt[i];
        _tCur[i] = (uint8_t)e;
        if (e > 250) {
            _torchLeds[i] = CRGB(170, 170, (uint8_t)e);
        } else if (e > 0) {
            uint8_t eb = _energymap[e >> 3];
            uint8_t r = RED_BIAS, g = GREEN_BIAS, b = BLUE_BIAS;
            _tIncrease(r, (eb * RED_EN)   >> 8);
            _tIncrease(g, (eb * GREEN_EN) >> 8);
            _tIncrease(b, (eb * BLUE_EN)  >> 8);
            _torchLeds[i] = CRGB(r, g, b);
        } else {
            _torchLeds[i] = CRGB(RED_BG, GREEN_BG, BLUE_BG);
        }
    }

    matrix.setBrightness(BRIGHTNESS);
    flushLeds(_torchLeds);
}

// ═════════════════════════════════════════════════════════════════════════════
// MODE_PULSE — Pulse.h (exact port)
// Beat modulation: new pulse triggered immediately on beat.
// ═════════════════════════════════════════════════════════════════════════════

static CRGB _pulseLeds[NUM_LEDS];
static CRGBPalette16 _pulsePal;

static void _drawCircle(int16_t x0, int16_t y0, uint16_t radius, const CRGB& color) {
    int a = radius, b = 0, err = 1 - a;
    if (radius == 0) { _pulseLeds[XY(x0,y0)] = color; return; }
    while (a >= b) {
        _pulseLeds[XY( a+x0, b+y0)] = color; _pulseLeds[XY( b+x0, a+y0)] = color;
        _pulseLeds[XY(-a+x0, b+y0)] = color; _pulseLeds[XY(-b+x0, a+y0)] = color;
        _pulseLeds[XY(-a+x0,-b+y0)] = color; _pulseLeds[XY(-b+x0,-a+y0)] = color;
        _pulseLeds[XY( a+x0,-b+y0)] = color; _pulseLeds[XY( b+x0,-a+y0)] = color;
        b++;
        if (err < 0) err += 2*b+1;
        else { a--; err += 2*(b-a+1); }
    }
}

static void renderPulse() {
    static uint8_t hue    = 0;
    static uint8_t cx     = 0, cy = 0;
    static uint8_t step   = 0;
    static const uint8_t maxSteps = 16;
    static const float   fadeRate = 0.8f;

    dimAll(_pulseLeds, 235);

    // Beat fires a new pulse immediately
    if (beatFired) step = 0;

    if (step == 0) {
        cx  = random(32);
        cy  = random(32);
        hue = random8();
        _drawCircle(cx, cy, 0, ColorFromPalette(_pulsePal, hue));
        step++;
    } else if (step < maxSteps) {
        uint8_t bri = (uint8_t)(powf(fadeRate, (float)(step - 2)) * 255.0f);
        _drawCircle(cx, cy, step, ColorFromPalette(_pulsePal, hue, bri));
        if (step > 3)
            _drawCircle(cx, cy, step-3, ColorFromPalette(_pulsePal, hue, bri));
        step++;
    } else {
        step = 0;
    }

    matrix.setBrightness(BRIGHTNESS);
    flushLeds(_pulseLeds);
}

// ═════════════════════════════════════════════════════════════════════════════
// MODE_WAVE — Wave.h (exact port)
// Beat modulation: hue jump and theta kick on beat.
// ═════════════════════════════════════════════════════════════════════════════

static CRGB _waveLeds[NUM_LEDS];
static CRGBPalette16 _wavePal;

static void renderWave() {
    static uint8_t rotation  = 3;
    static uint8_t waveCount = 1;
    static uint8_t hue       = 1;
    static uint8_t theta     = 0;

    const uint8_t scaleW = 256 / MATRIX_COLS;
    const uint8_t scaleH = 256 / MATRIX_ROWS;
    const uint8_t maxX   = MATRIX_COLS - 1;
    const uint8_t maxY   = MATRIX_ROWS - 1;

    int n = 0;
    switch (rotation) {
        case 0:
            for (int x = 0; x < MATRIX_COLS; x++) {
                n = quadwave8(x*2 + theta) / scaleH;
                _waveLeds[XY(x,n)] = ColorFromPalette(_wavePal, x+hue);
                if (waveCount==2) _waveLeds[XY(x,maxY-n)] = ColorFromPalette(_wavePal, x+hue);
            } break;
        case 1:
            for (int y = 0; y < MATRIX_ROWS; y++) {
                n = quadwave8(y*2 + theta) / scaleW;
                _waveLeds[XY(n,y)] = ColorFromPalette(_wavePal, y+hue);
                if (waveCount==2) _waveLeds[XY(maxX-n,y)] = ColorFromPalette(_wavePal, y+hue);
            } break;
        case 2:
            for (int x = 0; x < MATRIX_COLS; x++) {
                n = quadwave8(x*2 - theta) / scaleH;
                _waveLeds[XY(x,n)] = ColorFromPalette(_wavePal, x+hue);
                if (waveCount==2) _waveLeds[XY(x,maxY-n)] = ColorFromPalette(_wavePal, x+hue);
            } break;
        case 3:
            for (int y = 0; y < MATRIX_ROWS; y++) {
                n = quadwave8(y*2 - theta) / scaleW;
                _waveLeds[XY(n,y)] = ColorFromPalette(_wavePal, y+hue);
                if (waveCount==2) _waveLeds[XY(maxX-n,y)] = ColorFromPalette(_wavePal, y+hue);
            } break;
    }

    dimAll(_waveLeds, 254);

    EVERY_N_MILLISECONDS(10) {
        theta++;
        hue++;
        if (beatFired) { hue += 16; theta += 4; } // beat: snap hue + speed kick
    }

    matrix.setBrightness(BRIGHTNESS);
    flushLeds(_waveLeds);
}

// ═════════════════════════════════════════════════════════════════════════════
// NOISE MODES — Noise.h (exact port, all 8 variants)
// Beat modulation: noise speed boost proportional to beatEnergy.
// ═════════════════════════════════════════════════════════════════════════════

static CRGB _noiseLeds[NUM_LEDS];

#define _MAX_DIM ((MATRIX_COLS > MATRIX_ROWS) ? MATRIX_COLS : MATRIX_ROWS)
static uint8_t  _noiseArr[_MAX_DIM][_MAX_DIM];
static uint16_t _nx = 0, _ny = 0, _nz = 0;
static bool     _noiseInit = false;

static uint32_t _nSpeedX, _nSpeedY, _nSpeedZ;
static uint16_t _nScale;
static uint8_t  _nColorLoop;

static void _fillNoise8() {
    if (!_noiseInit) {
        _noiseInit = true;
        _nx = random16(); _ny = random16(); _nz = random16();
    }
    uint8_t dataSmoothing = 0;
    uint16_t lo = (uint16_t)min(_nSpeedX, min(_nSpeedY, _nSpeedZ));
    if (lo < 8) dataSmoothing = 200 - (lo * 4);

    for (int i = 0; i < _MAX_DIM; i++) {
        int ioff = _nScale * i;
        for (int j = 0; j < _MAX_DIM; j++) {
            uint8_t data = inoise8(_nx + ioff, _ny + _nScale*j, _nz);
            data = qsub8(data, 16);
            data = qadd8(data, scale8(data, 39));
            if (dataSmoothing) {
                data = scale8(_noiseArr[i][j], dataSmoothing)
                     + scale8(data, 255 - dataSmoothing);
            }
            _noiseArr[i][j] = data;
        }
    }
    _nx += _nSpeedX; _ny += _nSpeedY; _nz += _nSpeedZ;
}

static void _mapNoiseToLeds(const CRGBPalette16& pal, uint8_t hueReduce = 0) {
    static uint8_t ihue = 0;
    for (int i = 0; i < MATRIX_COLS; i++) {
        for (int j = 0; j < MATRIX_ROWS; j++) {
            uint8_t index = _noiseArr[j][i];
            uint8_t bri   = _noiseArr[i][j];
            if (_nColorLoop) index += ihue;
            bri = (bri > 127) ? 255 : dim8_raw(bri * 2);
            if (hueReduce > 0) index = (index < hueReduce) ? 0 : index - hueReduce;
            _noiseLeds[XY(i,j)] = ColorFromPalette(pal, index, bri);
        }
    }
    ihue++;
}

static void renderNoise() {
    // Per-mode parameters — exact values from Noise.h
    const CRGBPalette16* palPtr = &RainbowColors_p;
    uint8_t hr = 0;
    switch (_runtimeMode) {
        case MODE_RAINBOW_NOISE:
            _nSpeedX=9;  _nSpeedY=0; _nSpeedZ=0;  _nScale=30;  _nColorLoop=0;
            palPtr=&RainbowColors_p;       hr=0;  break;
        case MODE_RAINBOW_STRIPE_NOISE:
            _nSpeedX=9;  _nSpeedY=0; _nSpeedZ=0;  _nScale=20;  _nColorLoop=0;
            palPtr=&RainbowStripeColors_p; hr=0;  break;
        case MODE_PARTY_NOISE:
            _nSpeedX=9;  _nSpeedY=0; _nSpeedZ=0;  _nScale=30;  _nColorLoop=0;
            palPtr=&PartyColors_p;         hr=0;  break;
        case MODE_FOREST_NOISE:
            _nSpeedX=9;  _nSpeedY=0; _nSpeedZ=0;  _nScale=120; _nColorLoop=0;
            palPtr=&ForestColors_p;        hr=0;  break;
        case MODE_CLOUD_NOISE:
            _nSpeedX=9;  _nSpeedY=0; _nSpeedZ=0;  _nScale=30;  _nColorLoop=0;
            palPtr=&CloudColors_p;         hr=0;  break;
        case MODE_FIRE_NOISE:
            _nSpeedX=8;  _nSpeedY=0; _nSpeedZ=8;  _nScale=50;  _nColorLoop=0;
            palPtr=&HeatColors_p;          hr=60; break;
        case MODE_LAVA_NOISE:
            _nSpeedX=32; _nSpeedY=0; _nSpeedZ=16; _nScale=50;  _nColorLoop=0;
            palPtr=&LavaColors_p;          hr=0;  break;
        case MODE_OCEAN_NOISE:
        default:
            _nSpeedX=9;  _nSpeedY=0; _nSpeedZ=0;  _nScale=90;  _nColorLoop=0;
            palPtr=&OceanColors_p;         hr=0;  break;
    }

    // Beat: temporarily speed up noise drift
    if (beatEnergy > 0.0f) {
        _nSpeedX += (uint32_t)(beatEnergy * 12.0f);
        _nSpeedZ += (uint32_t)(beatEnergy * 8.0f);
    }

    _fillNoise8();
    _mapNoiseToLeds(*palPtr, hr);

    matrix.setBrightness(BRIGHTNESS);
    flushLeds(_noiseLeds);
}

// ═════════════════════════════════════════════════════════════════════════════
// MODE_CONFETTI — torchv2.ino confetti() (exact port)
// Beat modulation: extra splats on beat.
// ═════════════════════════════════════════════════════════════════════════════

static CRGB          _confLeds[NUM_LEDS];
static CRGBPalette16 _confPal;
static uint8_t       _confGHue = 0;

static void renderConfetti() {
    EVERY_N_MILLISECONDS(20) { _confGHue++; }

    fadeToBlackBy(_confLeds, NUM_LEDS, 10);
    int pos = random16(NUM_LEDS);
    _confLeds[pos] += ColorFromPalette(_confPal, _confGHue + random8(64), 255);

    if (beatFired) {
        for (int i = 0; i < 4; i++) {
            pos = random16(NUM_LEDS);
            _confLeds[pos] += ColorFromPalette(_confPal, _confGHue + random8(64), 255);
        }
    }

    matrix.setBrightness(BRIGHTNESS);
    flushLeds(_confLeds);
}

// ═════════════════════════════════════════════════════════════════════════════
// MODE_JUGGLE — torchv2.ino juggle() (exact port)
// Beat modulation: dot count or speed can increase on beat if desired.
// ═════════════════════════════════════════════════════════════════════════════

static CRGB _juggleLeds[NUM_LEDS];

static void renderJuggle() {
    fadeToBlackBy(_juggleLeds, NUM_LEDS, 20);
    uint8_t dothue   = 0;
    uint8_t dotCount = 3;
    for (int i = 0; i < dotCount; i++) {
        _juggleLeds[beatsin16(i + dotCount - 1, 0, NUM_LEDS)] |=
            hsv2rgb_rainbow(CHSV(dothue, 200, 255));
        dothue += 256 / dotCount;
    }
    matrix.setBrightness(BRIGHTNESS);
    flushLeds(_juggleLeds);
}

// ═════════════════════════════════════════════════════════════════════════════
// MODE_SINELON — torchv2.ino sinelon() (exact port)
// Beat modulation: hue jump on beat.
// ═════════════════════════════════════════════════════════════════════════════

static CRGB    _sinLeds[NUM_LEDS];
static uint8_t _sinGHue = 0;

static void renderSinelon() {
    EVERY_N_MILLISECONDS(20) { _sinGHue++; }
    if (beatFired) _sinGHue += 32;

    fadeToBlackBy(_sinLeds, NUM_LEDS, 20);
    uint16_t pos = beatsin16(13, 0, NUM_LEDS);
    static uint16_t prev = 0;
    if (pos < prev)
        fill_solid(_sinLeds + pos,  prev - pos + 1, hsv2rgb_rainbow(CHSV(_sinGHue, 220, 255)));
    else
        fill_solid(_sinLeds + prev, pos - prev + 1, hsv2rgb_rainbow(CHSV(_sinGHue, 220, 255)));
    prev = pos;

    matrix.setBrightness(BRIGHTNESS);
    flushLeds(_sinLeds);
}

// ═════════════════════════════════════════════════════════════════════════════
// MODE_PRIDE — torchv2.ino pride() / Pride2015 by Mark Kriegsman (exact port)
// Beat modulation: none needed — beatsin88 already drives animation.
// ═════════════════════════════════════════════════════════════════════════════

static CRGB _prideLeds[NUM_LEDS];

static void renderPride() {
    static uint16_t sPseudotime = 0;
    static uint16_t sLastMillis = 0;
    static uint16_t sHue16      = 0;

    uint8_t  sat8     = (uint8_t)beatsin88(87,  220, 250);
    uint8_t  bdepth   = (uint8_t)beatsin88(341, 96,  224);
    uint16_t btinc    = beatsin88(203, 25*256, 40*256);
    uint8_t  msmult   = (uint8_t)beatsin88(147, 23,  60);
    uint16_t hue16    = sHue16;
    uint16_t hueinc16 = beatsin88(113, 1, 3000);

    uint16_t ms      = (uint16_t)millis();
    uint16_t deltams = ms - sLastMillis;
    sLastMillis      = ms;
    sPseudotime     += deltams * msmult;
    sHue16          += deltams * (uint16_t)beatsin88(400, 5, 9);
    uint16_t btheta  = sPseudotime;

    for (int i = 0; i < NUM_LEDS; i++) {
        hue16  += hueinc16;
        uint8_t hue8 = (uint8_t)(hue16 / 256);
        btheta += btinc;
        uint16_t b16  = (uint16_t)(sin16(btheta) + 32768);
        uint16_t bri16 = (uint32_t)b16 * b16 / 65536;
        uint8_t  bri8  = (uint32_t)bri16 * bdepth / 65536;
        bri8 += (255 - bdepth);
        CRGB nc = hsv2rgb_rainbow(CHSV(hue8, sat8, bri8));
        nblend(_prideLeds[(NUM_LEDS-1)-i], nc, 64);
    }

    matrix.setBrightness(BRIGHTNESS);
    flushLeds(_prideLeds);
}

// ═════════════════════════════════════════════════════════════════════════════
// MODE_COLOR_WAVES — torchv2.ino colorwaves() (exact port)
// Beat modulation: none needed — beatsin88 already drives animation.
// ═════════════════════════════════════════════════════════════════════════════

static CRGB _cwLeds[NUM_LEDS];

static void renderColorWaves() {
    static CRGBPalette16 gCur, gTgt;
    static bool cwInit = false;
    if (!cwInit) { cwInit=true; gCur=RainbowColors_p; gTgt=OceanColors_p; }

    EVERY_N_SECONDS(10) {
        static const CRGBPalette16* pals[] = {
            &RainbowColors_p, &OceanColors_p, &CloudColors_p,
            &LavaColors_p,    &ForestColors_p, &PartyColors_p
        };
        static uint8_t pi = 0; pi = (pi+1) % 6;
        gTgt = *pals[pi];
    }
    EVERY_N_MILLISECONDS(40) { nblendPaletteTowardPalette(gCur, gTgt, 16); }

    static uint16_t sPseudotime = 0, sLastMillis = 0, sHue16 = 0;
    uint8_t  bdepth   = (uint8_t)beatsin88(341, 96,  224);
    uint16_t btinc    = beatsin88(203, 25*256, 40*256);
    uint8_t  msmult   = (uint8_t)beatsin88(147, 23,  60);
    uint16_t hue16    = sHue16;
    uint16_t hueinc16 = beatsin88(113, 300, 1500);

    uint16_t ms      = (uint16_t)millis();
    uint16_t deltams = ms - sLastMillis;
    sLastMillis      = ms;
    sPseudotime     += deltams * msmult;
    sHue16          += deltams * (uint16_t)beatsin88(400, 5, 9);
    uint16_t btheta  = sPseudotime;

    for (uint16_t i = 0; i < NUM_LEDS; i++) {
        hue16 += hueinc16;
        uint8_t hue8;
        uint16_t h128 = hue16 >> 7;
        hue8 = (h128 & 0x100) ? 255-(uint8_t)(h128>>1) : (uint8_t)(h128>>1);
        btheta += btinc;
        uint16_t b16   = (uint16_t)(sin16(btheta) + 32768);
        uint16_t bri16 = (uint32_t)b16 * b16 / 65536;
        uint8_t  bri8  = (uint32_t)bri16 * bdepth / 65536;
        bri8 += (255 - bdepth);
        CRGB nc = ColorFromPalette(gCur, scale8(hue8, 240), bri8);
        nblend(_cwLeds[(NUM_LEDS-1)-i], nc, 128);
    }

    matrix.setBrightness(BRIGHTNESS);
    flushLeds(_cwLeds);
}

// ═════════════════════════════════════════════════════════════════════════════
// MODE_CLOUD_TWINKLES / MODE_RAINBOW_TWINKLES
// torchv2.ino colortwinkles() (exact port)
// Beat modulation: density spike on beat.
// ═════════════════════════════════════════════════════════════════════════════

static CRGB    _twLeds[NUM_LEDS];
static uint8_t _twDirFlags[(NUM_LEDS + 7) / 8];

#define _TW_START_BRI  64
#define _TW_FADE_IN    32
#define _TW_FADE_OUT   20

enum { _TW_DARKER=0, _TW_BRIGHTER=1 };
static bool _twGetDir(uint16_t i) { return (_twDirFlags[i/8] & (1<<(i&7))) != 0; }
static void _twSetDir(uint16_t i, bool d) {
    uint8_t m = 1<<(i&7);
    if (d) _twDirFlags[i/8] |= m; else _twDirFlags[i/8] &= ~m;
}
static CRGB _twBrighter(const CRGB& c, fract8 h) {
    CRGB inc = c; inc.nscale8(h);
    return CRGB(qadd8(c.r,inc.r), qadd8(c.g,inc.g), qadd8(c.b,inc.b));
}
static CRGB _twDarker(const CRGB& c, fract8 h) { CRGB n=c; n.nscale8(255-h); return n; }

static void renderTwinkles() {
    // brightenOrDarkenEachPixel
    for (uint16_t i = 0; i < NUM_LEDS; i++) {
        if (_twGetDir(i) == _TW_DARKER) {
            _twLeds[i] = _twDarker(_twLeds[i], _TW_FADE_OUT);
        } else {
            _twLeds[i] = _twBrighter(_twLeds[i], _TW_FADE_IN);
            if (_twLeds[i].r==255 || _twLeds[i].g==255 || _twLeds[i].b==255)
                _twSetDir(i, _TW_DARKER);
        }
    }

    uint8_t density = beatFired ? 255 : 200; // beat → sudden burst
    const CRGBPalette16& pal =
        (_runtimeMode == MODE_CLOUD_TWINKLES) ? CloudColors_p : RainbowColors_p;
    if (random8() < density) {
        int pos = random16(NUM_LEDS);
        if (!_twLeds[pos]) {
            _twLeds[pos] = ColorFromPalette(pal, random8(), _TW_START_BRI, NOBLEND);
            _twSetDir(pos, _TW_BRIGHTER);
        }
    }

    matrix.setBrightness(BRIGHTNESS);
    flushLeds(_twLeds);
}


// =============================================================================
// MODE_RAIN -- Falling rain streaks, grey-blue tones, ghost trails, lightning
//
// Architecture
// ------------
//   - Up to RAIN_MAX_STREAKS simultaneous streaks, one pool shared across all
//     columns (multiple streaks can share a column, creating density variation).
//   - Each streak has a head position (float 0=top .. MATRIX_ROWS-1=bottom),
//     a speed, a lead brightness, and a per-streak blue tint amount.
//   - A ghost buffer [col][row] (float 0-255) decays each frame independently.
//     The streak head writes into it at full brightness; the buffer fades the
//     trail behind it naturally.
//   - Rainfall intensity is a slow sinusoidal oscillator cycling over
//     RAIN_CYCLE_MS milliseconds.  Intensity drives spawn rate and speed range.
//   - Lightning fires only when intensity > RAIN_LIGHTNING_THRESH.  A strike
//     lasts RAIN_FLASH_FRAMES frames: the first frame is a full white bloom,
//     subsequent frames invert the brightness of any ghost pixel that was lit
//     at the moment of the strike (captured into a snapshot buffer).
//   - Beat modulation: a detected beat can trigger an immediate lightning
//     strike regardless of intensity, giving an audio-reactive flash.
// =============================================================================

// ── Tuning ────────────────────────────────────────────────────────────────────
#define RAIN_MAX_STREAKS       12    // simultaneous falling streaks
#define RAIN_TRAIL_DECAY     0.78f   // ghost brightness multiplier per frame
                                     // lower = shorter trails, higher = longer
#define RAIN_SPEED_LIGHT     0.35f   // head pixels/frame at zero intensity
#define RAIN_SPEED_HEAVY     1.10f   // head pixels/frame at full intensity
#define RAIN_SPAWN_LIGHT     0.04f   // probability per frame of new streak (light)
#define RAIN_SPAWN_HEAVY     0.55f   // probability per frame of new streak (heavy)
#define RAIN_CYCLE_MS       75000UL  // full light→heavy→light cycle length (ms)
#define RAIN_LIGHTNING_THRESH 0.70f  // intensity level above which lightning fires
#define RAIN_LIGHTNING_PROB  0.004f  // probability per frame of a lightning strike
#define RAIN_FLASH_FRAMES       4    // frames a lightning event lasts
#define RAIN_BLOOM_FRAMES       1    // first N frames = full white bloom

// ── Streak ────────────────────────────────────────────────────────────────────
struct RainStreak {
    bool    active;
    uint8_t col;
    float   pos;       // 0.0 = top row, MATRIX_ROWS-1 = bottom row
    float   speed;     // rows per frame
    uint8_t headBri;   // lead pixel brightness 180-255
    uint8_t blueTint;  // 0 = pure grey, 255 = full blue (kept subtle, 10-55)
};

// ── State ─────────────────────────────────────────────────────────────────────
static RainStreak _rainStreaks[RAIN_MAX_STREAKS];
static float      _rainGhost[MATRIX_COLS][MATRIX_ROWS];   // decay buffer
static float      _rainSnap[MATRIX_COLS][MATRIX_ROWS];    // lightning snapshot
static uint8_t    _rainFlashFrames = 0;   // countdown during lightning event
static bool       _rainBeatFlash   = false;

static float _rainIntensity() {
    // Returns 0.0 (light drizzle) .. 1.0 (heavy downpour)
    // Slow sine wave with a mild random wander component
    uint32_t t = millis();
    float phase = (float)(t % RAIN_CYCLE_MS) / (float)RAIN_CYCLE_MS;
    // Primary: sin cycle. Secondary: offset sin at different phase/period
    float primary   = 0.5f + 0.5f * sinf(phase * 2.0f * (float)M_PI);
    float secondary = 0.5f + 0.5f * sinf(phase * 2.0f * (float)M_PI * 1.618f + 1.2f);
    return 0.6f * primary + 0.4f * secondary;  // weighted blend, always 0..1
}

static void _rainSpawnStreak(float intensity) {
    // Find a free slot
    int slot = -1;
    for (int i = 0; i < RAIN_MAX_STREAKS; i++) {
        if (!_rainStreaks[i].active) { slot = i; break; }
    }
    if (slot < 0) return;

    RainStreak& s = _rainStreaks[slot];
    s.active   = true;
    s.col      = random8(MATRIX_COLS);
    s.pos      = 0.0f;  // starts at top
    // Speed: linear interpolation across intensity, plus small random variance
    float baseSpeed = RAIN_SPEED_LIGHT + intensity * (RAIN_SPEED_HEAVY - RAIN_SPEED_LIGHT);
    s.speed    = baseSpeed * (0.75f + 0.50f * ((float)random8() / 255.0f));
    // Head brightness: heavier rain = brighter leads
    s.headBri  = (uint8_t)(160.0f + intensity * 90.0f + random8(0, 20));
    // Blue tint: very subtle, randomised per streak (10..55 range)
    s.blueTint = 10 + random8(0, 45);
}

static void _rainTriggerLightning() {
    // Snapshot current ghost buffer — these pixels will be inverted
    memcpy(_rainSnap, _rainGhost, sizeof(_rainGhost));
    _rainFlashFrames = RAIN_FLASH_FRAMES;
}

static void renderRain() {
    float intensity = _rainIntensity();

    // ── 1. Lightning: check for new strike ───────────────────────────────────
    bool lightningNow = false;
    if (_rainFlashFrames == 0) {
        // Beat always triggers a strike (dramatic audio reaction)
        if (beatFired) {
            lightningNow = true;
        }
        // Spontaneous strike only during heavy rain
        else if (intensity > RAIN_LIGHTNING_THRESH
                 && (float)random8() / 255.0f < RAIN_LIGHTNING_PROB) {
            lightningNow = true;
        }
        if (lightningNow) _rainTriggerLightning();
    }

    // ── 2. Spawn new streaks ──────────────────────────────────────────────────
    float spawnProb = RAIN_SPAWN_LIGHT
                    + intensity * (RAIN_SPAWN_HEAVY - RAIN_SPAWN_LIGHT);
    if ((float)random8() / 255.0f < spawnProb) {
        _rainSpawnStreak(intensity);
    }

    // ── 3. Decay all ghost trails ─────────────────────────────────────────────
    for (int c = 0; c < MATRIX_COLS; c++)
        for (int r = 0; r < MATRIX_ROWS; r++)
            _rainGhost[c][r] *= RAIN_TRAIL_DECAY;

    // ── 4. Advance streaks and stamp head into ghost buffer ───────────────────
    for (int i = 0; i < RAIN_MAX_STREAKS; i++) {
        RainStreak& s = _rainStreaks[i];
        if (!s.active) continue;

        s.pos += s.speed;

        if (s.pos >= (float)MATRIX_ROWS) {
            // Streak has fallen off the bottom — retire it
            s.active = false;
            continue;
        }

        // Stamp head at current position (integer row)
        int row = (int)s.pos;
        if (row >= 0 && row < MATRIX_ROWS) {
            // Head pixel always at full lead brightness
            _rainGhost[s.col][row] = fmaxf(_rainGhost[s.col][row],
                                           (float)s.headBri);
        }
        // Fractional sub-pixel: slightly light the next row for smooth motion
        float frac = s.pos - (float)row;
        if (frac > 0.1f && row + 1 < MATRIX_ROWS) {
            float tipBri = (float)s.headBri * frac * 0.6f;
            _rainGhost[s.col][row + 1] = fmaxf(_rainGhost[s.col][row + 1], tipBri);
        }
    }

    // ── 5. Render ─────────────────────────────────────────────────────────────
    matrix.setBrightness(BRIGHTNESS);
    matrix.fillScreen(0);

    if (_rainFlashFrames > 0) {
        // ── Lightning frame ───────────────────────────────────────────────────
        if (_rainFlashFrames > RAIN_FLASH_FRAMES - RAIN_BLOOM_FRAMES) {
            // Bloom frame: entire matrix floods white
            matrix.fillScreen(matrix.Color(255, 255, 255));
            // Active raindrop positions glow electric blue during bloom
            for (int c = 0; c < MATRIX_COLS; c++) {
                for (int r = 0; r < MATRIX_ROWS; r++) {
                    if (_rainSnap[c][r] > 30.0f) {
                        matrix.drawPixel(c, DRAW_Y(r), matrix.Color(120, 180, 255));
                    }
                }
            }
        } else {
            // Post-bloom frames: dark background, inverted raindrop brightness
            // gives a negative/afterimage effect
            for (int c = 0; c < MATRIX_COLS; c++) {
                for (int r = 0; r < MATRIX_ROWS; r++) {
                    float snap = _rainSnap[c][r];
                    if (snap > 10.0f) {
                        // Invert: bright pixels become dark, dark pixels glow
                        uint8_t inv = 255 - (uint8_t)fminf(255.0f, snap);
                        // Keep the blue tint in the inverted afterimage
                        uint8_t g = (uint8_t)(inv * 0.88f);
                        matrix.drawPixel(c, DRAW_Y(r), matrix.Color(g, g, inv));
                    }
                    // Also draw current live ghost underneath
                    float gb = _rainGhost[c][r];
                    if (gb > 6.0f && snap <= 10.0f) {
                        uint8_t bri = (uint8_t)fminf(255.0f, gb);
                        uint8_t g2  = (uint8_t)(bri * 0.88f);
                        matrix.drawPixel(c, DRAW_Y(r), matrix.Color(g2, g2, bri));
                    }
                }
            }
        }
        _rainFlashFrames--;

    } else {
        // ── Normal rain frame ─────────────────────────────────────────────────
        for (int c = 0; c < MATRIX_COLS; c++) {
            for (int r = 0; r < MATRIX_ROWS; r++) {
                float gb = _rainGhost[c][r];
                if (gb < 4.0f) continue;
                uint8_t bri = (uint8_t)fminf(255.0f, gb);
                // Grey-blue: reduce red and green channels slightly relative to blue
                // Amount of blue shift varies with rain intensity: heavier = bluer
                float blueBoost = 1.0f + intensity * 0.22f;
                uint8_t r_ch = (uint8_t)(bri * 0.82f);
                uint8_t g_ch = (uint8_t)(bri * 0.88f);
                uint8_t b_ch = (uint8_t)fminf(255.0f, (float)bri * blueBoost);
                matrix.drawPixel(c, DRAW_Y(r), matrix.Color(r_ch, g_ch, b_ch));
            }
        }
    }
}


// =============================================================================
// MODE_STARFIELD -- Classic warp-speed starfield
//
// Architecture
// ------------
//   Each star lives in a float coordinate space centered on the matrix center
//   (origin = 0,0). Every frame its offset vector (dx, dy) is scaled outward
//   by STAR_ACCEL raised to a power proportional to its current distance from
//   center -- stars start slow near the origin and accelerate exponentially as
//   they approach the edges, exactly like the classic warp effect.
//
//   Trail: each star remembers up to STAR_TRAIL_LEN previous integer screen
//   positions. Each trail step is rendered at a fraction of the head brightness,
//   giving a directional streak that always points back toward center.
//
//   Color: STAR_COLOR_CHANCE percent of stars are assigned a faint hue tint
//   (red, amber, cyan, or blue) at birth. All others are pure greyscale.
//
//   Beat modulation: a beat briefly boosts STAR_ACCEL, causing a momentary
//   warp-jump surge that snaps back to normal over ~8 frames.
//
// Tuning constants (all at the top of this block):
//   STAR_COUNT         -- number of simultaneous stars (density)
//   STAR_ACCEL         -- base outward acceleration factor per frame (speed)
//   STAR_TRAIL_DECAY   -- brightness fraction each trail step (trail length feel)
//   STAR_TRAIL_LEN     -- how many past positions to keep (trail pixel count)
//   STAR_COLOR_CHANCE  -- % of stars that get a color tint (0-100)
//   STAR_SPAWN_RADIUS  -- how close to center new stars spawn (smaller = tighter)
//   STAR_BEAT_SURGE    -- extra accel multiplier on a beat hit
// =============================================================================

// ── Tuning ────────────────────────────────────────────────────────────────────
#define STAR_COUNT          18    // number of simultaneous stars
                                  // 8-12 = sparse, 18-24 = busy, 30+ = dense
#define STAR_ACCEL        1.18f   // outward scale factor per frame
                                  // 1.10 = gentle drift, 1.18 = warp, 1.28 = fast
#define STAR_TRAIL_DECAY  0.45f   // brightness of each successive trail step
                                  // 0.25 = short dim trail, 0.55 = long bright trail
#define STAR_TRAIL_LEN       3    // number of trail positions stored per star
                                  // 1 = single ghost pixel, 4 = visible streak
#define STAR_COLOR_CHANCE    5    // % of stars that get a color tint (0-100)
#define STAR_SPAWN_RADIUS  0.8f   // spawn within this radius of center (matrix half-width units)
                                  // 0.3 = tight cluster at center, 1.2 = wider spawn zone
#define STAR_BEAT_SURGE   1.45f   // extra accel multiplier on a beat (layered on top of STAR_ACCEL)

// ── Matrix geometry ───────────────────────────────────────────────────────────
// Half-extents in float space. Stars are culled when abs(dx) or abs(dy) exceeds these.
#define _SF_HW  ((float)(MATRIX_COLS) * 0.5f)   // 4.0 for 8-wide
#define _SF_HH  ((float)(MATRIX_ROWS) * 0.5f)   // 4.0 for 8-tall
#define _SF_CX  (_SF_HW - 0.5f)                  // center X pixel (3.5)
#define _SF_CY  (_SF_HH - 0.5f)                  // center Y pixel (3.5)

// ── Star color tints (faint — blended with grey at low weight) ─────────────
// Each tint is (r_bias, g_bias, b_bias) added to the greyscale base.
// Kept subtle so the field reads as greyscale with occasional colour stars.
static const int8_t _sfTints[][3] = {
    { 60, -10, -10 },   // warm red-orange
    { 40,  20, -20 },   // amber
    {-20,  10,  60 },   // cold blue
    {-10,  50,  50 },   // cyan
    { 50, -20,  50 },   // violet
};
#define _SF_TINT_COUNT  5

// ── Star ──────────────────────────────────────────────────────────────────────
struct Star {
    float   dx, dy;         // position offset from center (float space)
    float   vx, vy;         // velocity (derived from position, kept for continuity)
    uint8_t bri;            // head brightness
    int8_t  tintIdx;        // -1 = greyscale, 0..4 = color tint index
    // Trail: ring buffer of recent integer screen positions
    int8_t  trailX[STAR_TRAIL_LEN];
    int8_t  trailY[STAR_TRAIL_LEN];
    uint8_t trailHead;      // ring buffer write index
    bool    active;
};

// ── State ─────────────────────────────────────────────────────────────────────
static Star    _sfStars[STAR_COUNT];
static float   _sfAccelBoost = 0.0f;   // extra accel from beat, decays each frame

// ── Helpers ───────────────────────────────────────────────────────────────────
static void _sfSpawnStar(Star& s) {
    // Random position within spawn radius, avoiding the exact center pixel
    float angle = (float)random16() * (2.0f * (float)M_PI / 65536.0f);
    float r     = STAR_SPAWN_RADIUS * ((float)random8(8, 255) / 255.0f);
    s.dx = r * cosf(angle);
    s.dy = r * sinf(angle);
    // Initial velocity tiny — let acceleration build it up naturally
    s.vx = s.dx * 0.01f;
    s.vy = s.dy * 0.01f;
    s.bri = random8(140, 255);
    // Color tint assignment
    if ((int)random8(0, 100) < STAR_COLOR_CHANCE) {
        s.tintIdx = (int8_t)random8(0, _SF_TINT_COUNT);
    } else {
        s.tintIdx = -1;
    }
    // Clear trail
    for (int i = 0; i < STAR_TRAIL_LEN; i++) {
        s.trailX[i] = (int8_t)_SF_CX;
        s.trailY[i] = (int8_t)_SF_CY;
    }
    s.trailHead = 0;
    s.active    = true;
}

// Convert float offset from center to integer screen pixel col/row
static inline int _sfToCol(float dx) { return (int)(_SF_CX + dx); }
static inline int _sfToRow(float dy) { return (int)(_SF_CY + dy); }
static inline bool _sfOnScreen(int col, int row) {
    return col >= 0 && col < MATRIX_COLS && row >= 0 && row < MATRIX_ROWS;
}

static void _sfDrawPixel(int col, int row, uint8_t bri, int8_t tintIdx) {
    if (!_sfOnScreen(col, row)) return;
    uint8_t r = bri, g = bri, b = bri;
    if (tintIdx >= 0) {
        // Apply tint: clamp each channel to 0-255
        r = (uint8_t)constrain((int)bri + _sfTints[tintIdx][0], 0, 255);
        g = (uint8_t)constrain((int)bri + _sfTints[tintIdx][1], 0, 255);
        b = (uint8_t)constrain((int)bri + _sfTints[tintIdx][2], 0, 255);
    }
    matrix.drawPixel(col, DRAW_Y(row), matrix.Color(r, g, b));
}

// ── Render ────────────────────────────────────────────────────────────────────
static void renderStarfield() {

    // Beat: inject a temporary accel boost that decays over ~8 frames
    if (beatFired) {
        _sfAccelBoost = STAR_BEAT_SURGE - 1.0f;  // surplus above base accel
    }
    float accel = STAR_ACCEL + _sfAccelBoost;
    if (_sfAccelBoost > 0.0f) {
        _sfAccelBoost *= 0.75f;   // decay boost
        if (_sfAccelBoost < 0.01f) _sfAccelBoost = 0.0f;
    }

    // Clear matrix
    matrix.setBrightness(BRIGHTNESS);
    matrix.fillScreen(0);

    for (int i = 0; i < STAR_COUNT; i++) {
        Star& s = _sfStars[i];
        if (!s.active) { _sfSpawnStar(s); continue; }

        // ── Draw trail (oldest first, dimmest first) ──────────────────────────
        // Walk backwards through ring buffer: trailHead-1 is most recent,
        // trailHead-(STAR_TRAIL_LEN) is oldest.
        float trailBri = (float)s.bri * STAR_TRAIL_DECAY;
        for (int t = 1; t <= STAR_TRAIL_LEN; t++) {
            int idx = (s.trailHead - t + STAR_TRAIL_LEN) % STAR_TRAIL_LEN;
            uint8_t tb = (uint8_t)(trailBri * powf(STAR_TRAIL_DECAY, (float)(t - 1)));
            if (tb > 6) {
                _sfDrawPixel(s.trailX[idx], s.trailY[idx], tb, s.tintIdx);
            }
        }

        // ── Draw head ─────────────────────────────────────────────────────────
        int col = _sfToCol(s.dx);
        int row = _sfToRow(s.dy);
        _sfDrawPixel(col, row, s.bri, s.tintIdx);

        // ── Store current position in trail ring buffer ───────────────────────
        s.trailX[s.trailHead] = (int8_t)col;
        s.trailY[s.trailHead] = (int8_t)row;
        s.trailHead = (s.trailHead + 1) % STAR_TRAIL_LEN;

        // ── Accelerate outward ────────────────────────────────────────────────
        // Scale the position vector by accel factor. Distance from center grows
        // exponentially, which is what produces the warp acceleration feel.
        // Stars close to center move slowly; stars near the edge move fast.
        s.dx *= accel;
        s.dy *= accel;

        // ── Cull star if it leaves the screen ─────────────────────────────────
        // Use a slight overshoot margin so stars don't pop out before their
        // trail has also cleared the visible area.
        if (fabsf(s.dx) > _SF_HW + 1.5f || fabsf(s.dy) > _SF_HH + 1.5f) {
            _sfSpawnStar(s);
        }
    }
}

// =============================================================================
// MODE_DUNE -- Desert dune ridges, sand palette, slow organic morphing
//
// Architecture
// ------------
//   The scene is a smooth mathematical field evaluated per-pixel each frame.
//   Two S-ridge curves define the dune shapes. Each ridge is a sine-based
//   ridge function: bright at the curve spine, falling off smoothly to the
//   sides. The two ridges sum to produce the final field value (0.0-1.0)
//   which maps to a sand colour palette (deep brown -> golden -> pale yellow).
//
//   A very slow inoise8 micro-texture term prevents the static periods from
//   looking digitally frozen -- it drifts imperceptibly, like heat shimmer.
//
//   S-curve shape: each curve is defined by (cx, cy, amplitude, frequency,
//   angle, phase). An S is produced by a sine whose argument is the projected
//   distance along the curve axis, creating the characteristic double-bend.
//   Ridge width (falloff) is a separate parameter.
//
//   State machine:
//     STATIC   45-60s  parameters completely frozen (micro-texture still drifts)
//     MORPHING 10-20s  all curve parameters lerp smoothly toward new targets
//     SETTLING  2- 3s  brief deceleration pause before returning to STATIC
//
//   Dust twinkles: independent 45-60s timer, spawns 3-6 bright highlight pixels
//   that fade over ~40 frames. Completely decoupled from morphing state.
//
//   Beat modulation: a beat briefly brightens the micro-texture contrast,
//   producing a subtle shimmer across the whole scene.
// =============================================================================

// ── Tuning ────────────────────────────────────────────────────────────────────
#define DUNE_NUM_RIDGES      2      // number of S-curve ridges
#define DUNE_RIDGE_WIDTH   1.8f     // falloff half-width in pixel units
                                    // smaller = sharper ridges, larger = softer
#define DUNE_TEXTURE_SPEED   3      // inoise z-drift per frame (micro-texture)
                                    // 0 = frozen, 8 = visible shimmer
#define DUNE_TEXTURE_DEPTH 0.12f    // how much micro-texture modulates brightness
                                    // 0.0 = none, 0.25 = noticeable grain
#define DUNE_STATIC_MIN   45000UL   // minimum static hold time (ms)
#define DUNE_STATIC_MAX   60000UL   // maximum static hold time (ms)
#define DUNE_MORPH_MIN    10000UL   // minimum morph duration (ms)
#define DUNE_MORPH_MAX    20000UL   // maximum morph duration (ms)
#define DUNE_SETTLE_MS     2500UL   // settling pause after morph (ms)
#define DUNE_DUST_MIN     45000UL   // minimum time between dust twinkles (ms)
#define DUNE_DUST_MAX     60000UL   // maximum time between dust twinkles (ms)
#define DUNE_DUST_COUNT_MIN  3      // minimum dust pixels per event
#define DUNE_DUST_COUNT_MAX  6      // maximum dust pixels per event
#define DUNE_DUST_FADE      40      // frames a dust pixel takes to fade out

// ── Sand colour palette ───────────────────────────────────────────────────────
// field value 0.0 = deep shadowed brown
// field value 0.5 = rich golden sand
// field value 1.0 = bright pale sun-bleached crest
// Each entry is {r, g, b} for field value i/7.
static const uint8_t _dunePal[][3] = {
    {  42,  22,   5 },   // 0.00 -- deep shadow brown
    {  72,  38,  10 },   // 0.14 -- dark amber brown
    { 120,  68,  18 },   // 0.29 -- warm brown
    { 175, 110,  30 },   // 0.43 -- golden brown
    { 210, 150,  45 },   // 0.57 -- rich gold
    { 235, 185,  75 },   // 0.71 -- bright sandy gold
    { 248, 218, 130 },   // 0.86 -- pale warm yellow
    { 255, 240, 185 },   // 1.00 -- bleached crest
};
#define _DUNE_PAL_LEN  8

// Interpolate sand palette for field value 0.0-1.0
static uint16_t _duneColor(float v) {
    v = fmaxf(0.0f, fminf(1.0f, v));
    float fi  = v * (_DUNE_PAL_LEN - 1);
    int   lo  = (int)fi;
    int   hi  = lo + 1;
    if (hi >= _DUNE_PAL_LEN) hi = _DUNE_PAL_LEN - 1;
    float t   = fi - (float)lo;
    uint8_t r = (uint8_t)(_dunePal[lo][0] + t * ((float)_dunePal[hi][0] - _dunePal[lo][0]));
    uint8_t g = (uint8_t)(_dunePal[lo][1] + t * ((float)_dunePal[hi][1] - _dunePal[lo][1]));
    uint8_t b = (uint8_t)(_dunePal[lo][2] + t * ((float)_dunePal[hi][2] - _dunePal[lo][2]));
    return matrix.Color(r, g, b);
}

// ── Ridge parameters ──────────────────────────────────────────────────────────
// Each ridge described by: center offset (cx,cy), S-amplitude, S-frequency,
// rotation angle (radians), and a ridge brightness peak value.
struct DuneRidge {
    float cx;      // horizontal center offset (-1.0 to 1.0, in pixel units from matrix center)
    float cy;      // vertical center offset
    float amp;     // S-curve amplitude (half-swing in pixels)
    float freq;    // S-curve spatial frequency
    float angle;   // ridge axis rotation (radians)
    float peak;    // brightness contribution at ridge spine (0.0-1.0)
};

static DuneRidge _duneCur[DUNE_NUM_RIDGES];   // current (displayed) params
static DuneRidge _duneFrom[DUNE_NUM_RIDGES];  // morph start snapshot
static DuneRidge _duneTo[DUNE_NUM_RIDGES];    // morph target

// ── State machine ─────────────────────────────────────────────────────────────
enum DuneState { DS_STATIC, DS_MORPHING, DS_SETTLING };
static DuneState _duneState     = DS_STATIC;
static uint32_t  _duneStateEnd  = 0;   // millis() when current state ends
static float     _duneMorphT    = 0.0f; // 0.0->1.0 progress through morph
static uint32_t  _duneMorphStart = 0;   // millis() when current morph started
static uint32_t  _duneMorphDur   = 0;   // total morph duration in ms

// ── Dust twinkle state ────────────────────────────────────────────────────────
#define _DUNE_MAX_DUST  6
struct DuneDust {
    uint8_t col, row;
    int     life;    // frames remaining (counts down)
    bool    active;
};
static DuneDust  _duneDust[_DUNE_MAX_DUST];
static uint32_t  _duneDustNext = 0;   // millis() when next dust event fires

// ── Micro-texture ─────────────────────────────────────────────────────────────
static uint32_t _duneNoiseZ   = 0;
static float    _duneBeatGlow = 0.0f;   // brief brightness boost on beat

// ── Helpers ───────────────────────────────────────────────────────────────────

// Evaluate the ridge field contribution at pixel (px, py) for one ridge.
// Returns 0.0 (far from ridge) to ridge.peak (at spine).
static float _duneRidgeField(const DuneRidge& r, float px, float py) {
    // Translate to ridge-local coordinates
    float lx = px - r.cx;
    float ly = py - r.cy;
    // Rotate into ridge axis frame
    float cosA = cosf(r.angle), sinA = sinf(r.angle);
    float ax =  lx * cosA + ly * sinA;   // along ridge axis
    float ay = -lx * sinA + ly * cosA;   // across ridge axis

    // S-curve: spine position across axis as function of along-axis position
    // sin(ax * freq) * amp gives the S shape
    float spine = sinf(ax * r.freq) * r.amp;

    // Distance from this pixel to the spine
    float dist = fabsf(ay - spine);

    // Smooth falloff: Gaussian-like using exp
    float falloff = expf(-(dist * dist) / (DUNE_RIDGE_WIDTH * DUNE_RIDGE_WIDTH));

    return r.peak * falloff;
}

// Randomise a ridge's target parameters — keep values plausible for dune shapes
static void _duneRandomRidge(DuneRidge& r) {
    // Center: anywhere in the matrix, biased toward the middle band
    r.cx  = ((float)(int8_t)random8() / 128.0f) * 3.0f;
    r.cy  = ((float)(int8_t)random8() / 128.0f) * 3.0f;
    // Amplitude: 0.8 to 2.5 pixels — controls how pronounced the S bend is
    r.amp  = 0.8f + ((float)random8() / 255.0f) * 1.7f;
    // Frequency: how tightly the S winds — lower = lazy S, higher = tight zigzag
    r.freq = 0.35f + ((float)random8() / 255.0f) * 0.55f;
    // Angle: biased near horizontal/diagonal for dune-like orientation
    r.angle = ((float)random8() / 255.0f) * (float)M_PI;
    // Peak brightness: 0.55 to 1.0
    r.peak = 0.55f + ((float)random8() / 255.0f) * 0.45f;
}

// Lerp a single ridge between from and to by t (0.0-1.0)
// Uses smoothstep so motion eases in and out naturally
static void _duneLerpRidge(DuneRidge& out,
                            const DuneRidge& a, const DuneRidge& b, float t) {
    // Smoothstep: 3t^2 - 2t^3
    float s = t * t * (3.0f - 2.0f * t);
    out.cx    = a.cx    + s * (b.cx    - a.cx);
    out.cy    = a.cy    + s * (b.cy    - a.cy);
    out.amp   = a.amp   + s * (b.amp   - a.amp);
    out.freq  = a.freq  + s * (b.freq  - a.freq);
    out.angle = a.angle + s * (b.angle - a.angle);
    out.peak  = a.peak  + s * (b.peak  - a.peak);
}

static void _duneStartMorph() {
    // Snapshot current as morph start
    memcpy(_duneFrom, _duneCur, sizeof(_duneCur));
    // Randomise targets
    for (int i = 0; i < DUNE_NUM_RIDGES; i++) _duneRandomRidge(_duneTo[i]);
    _duneMorphT     = 0.0f;
    _duneMorphDur   = DUNE_MORPH_MIN +
                      (uint32_t)(((float)random16() / 65535.0f) *
                                  (float)(DUNE_MORPH_MAX - DUNE_MORPH_MIN));
    _duneMorphStart = millis();
    _duneStateEnd   = _duneMorphStart + _duneMorphDur;
    _duneState      = DS_MORPHING;
}

static void _duneStartStatic() {
    uint32_t dur = DUNE_STATIC_MIN +
                   (uint32_t)(((float)random16() / 65535.0f) *
                               (float)(DUNE_STATIC_MAX - DUNE_STATIC_MIN));
    _duneStateEnd = millis() + dur;
    _duneState    = DS_STATIC;
}

static void _duneSpawnDust() {
    int count = DUNE_DUST_COUNT_MIN +
                random8(0, DUNE_DUST_COUNT_MAX - DUNE_DUST_COUNT_MIN + 1);
    int spawned = 0;
    for (int i = 0; i < _DUNE_MAX_DUST && spawned < count; i++) {
        if (!_duneDust[i].active) {
            _duneDust[i].col    = random8(MATRIX_COLS);
            _duneDust[i].row    = random8(MATRIX_ROWS);
            _duneDust[i].life   = DUNE_DUST_FADE;
            _duneDust[i].active = true;
            spawned++;
        }
    }
    // Schedule next dust event
    _duneDustNext = millis() + DUNE_DUST_MIN +
                    (uint32_t)(((float)random16() / 65535.0f) *
                                (float)(DUNE_DUST_MAX - DUNE_DUST_MIN));
}

// ── Init (called from visualizerInit) ─────────────────────────────────────────
static void _duneInit() {
    // Seed ridges with reasonable initial values
    for (int i = 0; i < DUNE_NUM_RIDGES; i++) {
        _duneRandomRidge(_duneCur[i]);
        _duneRandomRidge(_duneTo[i]);
        memcpy(&_duneFrom[i], &_duneCur[i], sizeof(DuneRidge));
    }
    memset(_duneDust, 0, sizeof(_duneDust));
    _duneBeatGlow  = 0.0f;
    _duneNoiseZ    = random16();
    _duneStartStatic();
    _duneDustNext  = millis() + DUNE_DUST_MIN +
                     (uint32_t)(((float)random16() / 65535.0f) *
                                 (float)(DUNE_DUST_MAX - DUNE_DUST_MIN));
}

// ── Render ────────────────────────────────────────────────────────────────────
static void renderDune() {
    uint32_t now = millis();

    // ── Beat: shimmer burst ───────────────────────────────────────────────────
    if (beatFired) _duneBeatGlow = 0.18f;
    if (_duneBeatGlow > 0.0f) {
        _duneBeatGlow -= 0.025f;
        if (_duneBeatGlow < 0.0f) _duneBeatGlow = 0.0f;
    }

    // ── State machine ─────────────────────────────────────────────────────────
    switch (_duneState) {
        case DS_STATIC:
            if (now >= _duneStateEnd) _duneStartMorph();
            break;

        case DS_MORPHING: {
            if (now >= _duneStateEnd) {
                // Morph complete: snap to targets, enter settling
                memcpy(_duneCur, _duneTo, sizeof(_duneCur));
                _duneMorphT   = 1.0f;
                _duneStateEnd = now + DUNE_SETTLE_MS;
                _duneState    = DS_SETTLING;
            } else {
                // t derived from wall time: (elapsed / total duration)
                // _duneMorphStart stored when morph began, _duneStateEnd = start+dur
                uint32_t elapsed = now - _duneMorphStart;
                uint32_t total   = _duneMorphDur;
                _duneMorphT = (total > 0)
                    ? fminf(1.0f, (float)elapsed / (float)total)
                    : 1.0f;
                for (int i = 0; i < DUNE_NUM_RIDGES; i++)
                    _duneLerpRidge(_duneCur[i], _duneFrom[i], _duneTo[i], _duneMorphT);
            }
            break;
        }

        case DS_SETTLING:
            if (now >= _duneStateEnd) _duneStartStatic();
            break;
    }

    // ── Dust twinkle scheduler ────────────────────────────────────────────────
    if (now >= _duneDustNext) _duneSpawnDust();

    // ── Advance micro-texture ─────────────────────────────────────────────────
    _duneNoiseZ += DUNE_TEXTURE_SPEED;

    // ── Render field ──────────────────────────────────────────────────────────
    matrix.setBrightness(BRIGHTNESS);

    // Matrix center in pixel coords
    float cx = (float)(MATRIX_COLS) * 0.5f - 0.5f;   // 3.5 for 8-wide matrix
    float cy = (float)(MATRIX_ROWS) * 0.5f - 0.5f;   // 3.5 for 8-tall matrix

    for (int col = 0; col < MATRIX_COLS; col++) {
        for (int row = 0; row < MATRIX_ROWS; row++) {
            // Pixel position relative to matrix center
            float px = (float)col - cx;
            float py = (float)row - cy;

            // Sum ridge field contributions
            float field = 0.0f;
            for (int i = 0; i < DUNE_NUM_RIDGES; i++) {
                field += _duneRidgeField(_duneCur[i], px, py);
            }
            // Clamp and normalise: each ridge contributes 0-peak,
            // two ridges could sum to 2.0; normalise to 0-1.
            field = fminf(1.0f, field / (float)DUNE_NUM_RIDGES);

            // Micro-texture: very slow inoise8 drift
            uint8_t nx = (uint8_t)(inoise8(
                (uint16_t)(col * 28),
                (uint16_t)(row * 28),
                (uint16_t)(_duneNoiseZ >> 2)
            ));
            float texture = ((float)nx / 255.0f - 0.5f) * DUNE_TEXTURE_DEPTH;
            field = fmaxf(0.0f, fminf(1.0f, field + texture + _duneBeatGlow));

            matrix.drawPixel(col, DRAW_Y(row), _duneColor(field));
        }
    }

    // ── Render dust twinkles on top of field ─────────────────────────────────
    for (int i = 0; i < _DUNE_MAX_DUST; i++) {
        DuneDust& d = _duneDust[i];
        if (!d.active) continue;
        // Brightness: full at life=DUNE_DUST_FADE, zero at life=0
        float t   = (float)d.life / (float)DUNE_DUST_FADE;
        // Ease out: t^2 for a quick bright flash then gentle fade
        uint8_t bri = (uint8_t)(t * t * 255.0f);
        // Dust colour: pale warm white with a hint of gold
        uint8_t r = bri;
        uint8_t g = (uint8_t)(bri * 0.92f);
        uint8_t b = (uint8_t)(bri * 0.60f);
        matrix.drawPixel(d.col, DRAW_Y(d.row), matrix.Color(r, g, b));
        d.life--;
        if (d.life <= 0) d.active = false;
    }
}

// =============================================================================
// MODE_GEOMETRIC -- Axis-aligned hollow rectangles, palette cycling
//
// Architecture
// ------------
//   GEO_NUM_SHAPES axis-aligned hollow rectangles (no rotation).
//   Each is defined by its centre (cx,cy) and half-extents (hw,hh).
//   Dimensions are always integers when rendered -- float values drive
//   smooth sub-pixel interpolated motion that snaps to integer pixels
//   each frame, giving crisp edges with smooth movement.
//
//   Wall thickness: exactly 1 pixel. Interior is always transparent (black).
//   Shape boundary always large enough to show all 4 walls: minimum
//   rendered dimension 2x2 pixels (hw>=1, hh>=1), maximum 8x8 (hw<=4,hh<=4).
//
//   Per-shape behaviour
//     Movement   : each shape has a constant velocity vector (vx, vy).
//                  Direction is one of: pure X, pure Y, or one of 4
//                  diagonals (45deg). Speed varies per shape. Direction
//                  and speed only change at a scheduled reversal interval,
//                  keeping motion smooth and predictable.
//     Morphing   : hw and hh are driven by independent sine oscillators
//                  at different rates. No sudden jumps -- purely smooth.
//     Boundary   : shapes always partially extend beyond the matrix edge.
//                  When the centre drifts too far out, it wraps to the
//                  opposite side so shapes re-enter continuously.
//
//   Rendering -- painter's algorithm
//     Shapes drawn back-to-front. At any pixel the frontmost shape wall
//     that covers it wins. Interiors are transparent -- you see through
//     to shapes beneath, making nested/overlapping rects clearly visible.
//     No dimming, no blending, no border effects.
//
//   Colour palettes
//     5 palettes of GEO_NUM_SHAPES colours. Every GEO_PAL_CYCLE_MS ms
//     the active palette snaps to the next -- no cross-fade, clean cut.
//
//   Beat modulation
//     A beat triggers a temporary speed boost on all shapes for ~8 frames.
// =============================================================================

// ── Tuning ────────────────────────────────────────────────────────────────────
#define GEO_NUM_SHAPES     5       // number of simultaneous rectangles

// Speed table: 8 possible speed magnitudes (pixels/frame).
// Each shape picks one independently. Higher = faster.
#define GEO_SPEED_MIN      0.08f   // slowest shape speed
#define GEO_SPEED_MAX      0.28f   // fastest shape speed

// Size oscillation: half-extents vary between these limits via sine wave
#define GEO_HW_MIN         1.0f    // minimum half-width  (renders as 2px wide)
#define GEO_HW_MAX         4.0f    // maximum half-width  (renders as 8px wide)
#define GEO_HH_MIN         1.0f    // minimum half-height (renders as 2px tall)
#define GEO_HH_MAX         4.0f    // maximum half-height (renders as 8px tall)

// Morph: independent sine rates per dimension, per shape
#define GEO_OSC_FREQ_MIN   0.008f  // slowest size oscillation (rad/frame)
#define GEO_OSC_FREQ_MAX   0.035f  // fastest size oscillation (rad/frame)

// Direction change: each shape reverses or picks new direction every N frames
#define GEO_DIR_CHANGE_MIN  180    // minimum frames before direction change
#define GEO_DIR_CHANGE_MAX  420    // maximum frames before direction change

// Wrap margin: how far past the edge the centre can go before wrapping
#define GEO_WRAP_MARGIN    5.0f

// Palette
#define GEO_NUM_PALETTES   5
#define GEO_PAL_CYCLE_MS   60000UL  // ms between palette snaps

// Beat
#define GEO_BEAT_BOOST     2.2f     // speed multiplier on beat
#define GEO_BEAT_DECAY     0.88f    // boost decay per frame

// ── Movement directions ───────────────────────────────────────────────────────
// 8 axis-aligned or diagonal unit vectors. No rotation, just these 8.
static const float _geoDirX[8] = { 1, 0,-1, 0, 1,-1,-1, 1 };
static const float _geoDirY[8] = { 0, 1, 0,-1, 1, 1,-1,-1 };
// Diagonal unit vectors are ~0.707 per component; normalise so speed is consistent
static const float _geoDirScale[8] = {
    1.0f, 1.0f, 1.0f, 1.0f,
    0.7071f, 0.7071f, 0.7071f, 0.7071f
};

// ── Colour palettes ───────────────────────────────────────────────────────────
static const uint8_t _geoPalettes[GEO_NUM_PALETTES][GEO_NUM_SHAPES][3] = {
    // 0: Ember
    { {200, 20,  5}, {230, 90, 10}, {255,150, 20}, {210, 55,  0}, {245,120, 35} },
    // 1: Arctic
    { { 15, 85,190}, { 10,145,225}, { 55,205,245}, {  5, 55,165}, {145,215,255} },
    // 2: Verdant
    { { 10,105, 30}, { 25,165, 50}, { 75,225, 80}, {  5, 75, 20}, {135,205, 60} },
    // 3: Dusk
    { {105, 10,165}, {175, 20,205}, {225, 60,185}, { 60,  5,125}, {205,100,235} },
    // 4: Mineral
    { { 20,145,135}, {185,155, 20}, { 55,185,165}, {155,115, 10}, {100,205,195} },
};

// ── Shape ─────────────────────────────────────────────────────────────────────
struct GeoShape {
    float   cx, cy;        // centre (float, may be outside matrix)
    float   hw, hh;        // current half-extents (float, clamped on render)
    float   hwBase, hhBase;// midpoint of oscillation range
    float   hwAmp,  hhAmp; // amplitude of sine oscillation
    float   oscPhW, oscPhH;// oscillator phase (radians)
    float   oscFrW, oscFrH;// oscillator frequency (rad/frame)
    float   speed;         // magnitude (pixels/frame)
    uint8_t dirIdx;        // index into _geoDirX/Y (0-7)
    int     dirTimer;      // frames until next direction change
    float   beatBoost;     // current speed multiplier from beat (decays to 1.0)
    uint8_t palIdx;        // palette colour slot
};

// ── State ─────────────────────────────────────────────────────────────────────
static GeoShape _geoShapes[GEO_NUM_SHAPES];
static uint8_t  _geoCurPal = 0;
static uint32_t _geoPalEnd = 0;

// ── Helpers ───────────────────────────────────────────────────────────────────
static inline float _geoRandRange(float lo, float hi) {
    return lo + ((float)random8() / 255.0f) * (hi - lo);
}

static void _geoSpawnShape(int i) {
    GeoShape& s = _geoShapes[i];

    // Start on-screen, possibly near an edge
    s.cx = _geoRandRange(0.5f, (float)(MATRIX_COLS - 1) + 0.5f);
    s.cy = _geoRandRange(0.5f, (float)(MATRIX_ROWS - 1) + 0.5f);

    // Size: midpoint and amplitude for each oscillator dimension
    // Ensure midpoint +/- amplitude stays within [MIN..MAX]
    float hwMid = _geoRandRange(GEO_HW_MIN + 0.8f, GEO_HW_MAX - 0.8f);
    float hhMid = _geoRandRange(GEO_HH_MIN + 0.8f, GEO_HH_MAX - 0.8f);
    float hwA   = _geoRandRange(0.5f, fminf(hwMid - GEO_HW_MIN, GEO_HW_MAX - hwMid));
    float hhA   = _geoRandRange(0.5f, fminf(hhMid - GEO_HH_MIN, GEO_HH_MAX - hhMid));
    s.hwBase = hwMid; s.hwAmp = hwA;
    s.hhBase = hhMid; s.hhAmp = hhA;
    s.oscPhW = _geoRandRange(0, 2.0f * (float)M_PI);
    s.oscPhH = _geoRandRange(0, 2.0f * (float)M_PI);
    s.oscFrW = _geoRandRange(GEO_OSC_FREQ_MIN, GEO_OSC_FREQ_MAX);
    s.oscFrH = _geoRandRange(GEO_OSC_FREQ_MIN, GEO_OSC_FREQ_MAX);

    // Movement
    s.speed    = _geoRandRange(GEO_SPEED_MIN, GEO_SPEED_MAX);
    s.dirIdx   = random8(8);
    s.dirTimer = GEO_DIR_CHANGE_MIN +
                 (int)(((float)random8() / 255.0f) *
                       (float)(GEO_DIR_CHANGE_MAX - GEO_DIR_CHANGE_MIN));
    s.beatBoost = 1.0f;
    s.palIdx    = (uint8_t)i;

    // Compute initial hw/hh
    s.hw = s.hwBase + s.hwAmp * sinf(s.oscPhW);
    s.hh = s.hhBase + s.hhAmp * sinf(s.oscPhH);
}

// ── Init ──────────────────────────────────────────────────────────────────────
static void _geoInit() {
    for (int i = 0; i < GEO_NUM_SHAPES; i++) _geoSpawnShape(i);
    _geoCurPal = 0;
    _geoPalEnd = millis() + GEO_PAL_CYCLE_MS;
}

// ── Render ────────────────────────────────────────────────────────────────────
static void renderGeometric() {
    uint32_t now = millis();

    // ── Palette snap ─────────────────────────────────────────────────────────
    if (now >= _geoPalEnd) {
        _geoCurPal = (_geoCurPal + 1) % GEO_NUM_PALETTES;
        _geoPalEnd = now + GEO_PAL_CYCLE_MS;
    }

    // ── Beat: boost all shapes briefly ───────────────────────────────────────
    if (beatFired) {
        for (int i = 0; i < GEO_NUM_SHAPES; i++)
            _geoShapes[i].beatBoost = GEO_BEAT_BOOST;
    }

    // ── Update all shapes ─────────────────────────────────────────────────────
    for (int i = 0; i < GEO_NUM_SHAPES; i++) {
        GeoShape& s = _geoShapes[i];

        // Decay beat boost
        if (s.beatBoost > 1.01f) s.beatBoost *= GEO_BEAT_DECAY;
        else                      s.beatBoost  = 1.0f;

        // Direction change timer
        s.dirTimer--;
        if (s.dirTimer <= 0) {
            // Pick a new direction that differs from current
            uint8_t newDir;
            do { newDir = random8(8); } while (newDir == s.dirIdx);
            s.dirIdx   = newDir;
            s.speed    = _geoRandRange(GEO_SPEED_MIN, GEO_SPEED_MAX);
            s.dirTimer = GEO_DIR_CHANGE_MIN +
                         (int)(((float)random8() / 255.0f) *
                               (float)(GEO_DIR_CHANGE_MAX - GEO_DIR_CHANGE_MIN));
        }

        // Move
        float spd = s.speed * s.beatBoost;
        s.cx += _geoDirX[s.dirIdx] * _geoDirScale[s.dirIdx] * spd;
        s.cy += _geoDirY[s.dirIdx] * _geoDirScale[s.dirIdx] * spd;

        // Wrap: when centre moves too far past any edge, re-enter from opposite side
        float wm = GEO_WRAP_MARGIN;
        float mw = (float)MATRIX_COLS, mh = (float)MATRIX_ROWS;
        if (s.cx < -wm)      s.cx += mw + wm * 2.0f;
        if (s.cx > mw + wm)  s.cx -= mw + wm * 2.0f;
        if (s.cy < -wm)      s.cy += mh + wm * 2.0f;
        if (s.cy > mh + wm)  s.cy -= mh + wm * 2.0f;

        // Morph size via sine oscillators
        s.oscPhW += s.oscFrW;
        s.oscPhH += s.oscFrH;
        s.hw = s.hwBase + s.hwAmp * sinf(s.oscPhW);
        s.hh = s.hhBase + s.hhAmp * sinf(s.oscPhH);
        // Clamp to valid range
        s.hw = fmaxf(GEO_HW_MIN, fminf(GEO_HW_MAX, s.hw));
        s.hh = fmaxf(GEO_HH_MIN, fminf(GEO_HH_MAX, s.hh));
    }

    // ── Render: painter's algorithm back-to-front ─────────────────────────────
    // Pixel ownership: -1 = background (black), 0..N-1 = shape index
    static int8_t _geoOwner[MATRIX_COLS][MATRIX_ROWS];
    for (int c = 0; c < MATRIX_COLS; c++)
        for (int r = 0; r < MATRIX_ROWS; r++)
            _geoOwner[c][r] = -1;

    for (int i = 0; i < GEO_NUM_SHAPES; i++) {
        GeoShape& s = _geoShapes[i];

        // Convert float centre + half-extents to integer pixel bounds
        // Round centre to nearest half-pixel for smooth but crisp motion
        int x0 = (int)roundf(s.cx - s.hw);  // left wall col
        int x1 = (int)roundf(s.cx + s.hw);  // right wall col
        int y0 = (int)roundf(s.cy - s.hh);  // top wall row
        int y1 = (int)roundf(s.cy + s.hh);  // bottom wall row

        // Enforce minimum size: at least 2 pixels per dimension
        if (x1 - x0 < 1) x1 = x0 + 1;
        if (y1 - y0 < 1) y1 = y0 + 1;

        // Draw exactly the 4 walls (1 pixel thick). Interior is NOT filled.
        // Only draw pixels that fall on-screen.
        // Top wall (y0): x0..x1
        // Bottom wall (y1): x0..x1
        // Left wall (x0): y0..y1
        // Right wall (x1): y0..y1

        for (int col = x0; col <= x1; col++) {
            if (col < 0 || col >= MATRIX_COLS) continue;
            if (y0 >= 0 && y0 < MATRIX_ROWS) _geoOwner[col][y0] = (int8_t)i;
            if (y1 >= 0 && y1 < MATRIX_ROWS) _geoOwner[col][y1] = (int8_t)i;
        }
        for (int row = y0 + 1; row < y1; row++) {
            if (row < 0 || row >= MATRIX_ROWS) continue;
            if (x0 >= 0 && x0 < MATRIX_COLS) _geoOwner[x0][row] = (int8_t)i;
            if (x1 >= 0 && x1 < MATRIX_COLS) _geoOwner[x1][row] = (int8_t)i;
        }
    }

    // Write final pixel colours
    matrix.setBrightness(BRIGHTNESS);
    matrix.fillScreen(0);

    for (int col = 0; col < MATRIX_COLS; col++) {
        for (int row = 0; row < MATRIX_ROWS; row++) {
            int8_t si = _geoOwner[col][row];
            if (si < 0) continue;
            const uint8_t* c = _geoPalettes[_geoCurPal][(int)si];
            matrix.drawPixel(col, DRAW_Y(row), matrix.Color(c[0], c[1], c[2]));
        }
    }
}





// =============================================================================
// MODE_PCBA — PCB Artwork
//
// Architecture
// ------------
//   Six hand-crafted 8×8 board outlines cycle every PCBA_BOARD_CYCLE_MS ms.
//   A single Manhattan route is generated per round (PCBA_ROUND_MS = 300 s).
//   The route is drawn on the board in lighter green for the full round,
//   visible as a "copper track" even between spark bursts.
//
//   Board colours — exactly two
//   ---------------------------
//   Board interior (non-trace) : flat dark green  (0, 45, 4)
//   Route trace                : light green      (0, 105, 32)
//
//   Route
//   -----
//   One route per round. Starts from a random top-edge pixel, walks downward
//   with occasional side-steps (Manhattan, constrained inside board mask).
//   Length PCBA_ROUTE_LEN_MIN–MAX. Re-generated at round start and board change.
//
//   Sparks
//   ------
//   Pool of PCBA_MAX_SPARKS. Bursts of 1–6 sparks are fired per round with a
//   3–15 second pause (millisecond timer) between bursts. Burst members are
//   each separated by PCBA_SPARK_GAP_MS (2 s) after the prior spark clears.
//
//   Head  : white with aggressive per-frame flicker — randomly drops to near-off
//           (~20% chance each frame) for a sharp electrical-arc stutter.
//
//   Tail  : PCBA_TAIL_LEN pixels long, piecewise colour gradient:
//             t=0        → white (head; handled separately)
//             t=1..40%   → white → bright gold
//             t=40%..80% → bright gold → amber
//             t=80%..end → amber → trace green  (0,105,32)
//           Brightness uses a linear-then-slow-decay profile so the gold zone
//           stays visibly bright rather than collapsing immediately.
//
//   Beat  : brief speed surge on all active sparks.
// =============================================================================

// ── Tuning ────────────────────────────────────────────────────────────────────
#define PCBA_MAX_SPARKS         18
#define PCBA_TAIL_LEN           12   // longer tail — more gold on screen
#define PCBA_SPARK_SPEED       0.576f // base pixels/frame (+20%)
#define PCBA_BURST_SIZE_MIN      1   // min sparks per burst
#define PCBA_BURST_SIZE_MAX      6   // max sparks per burst
#define PCBA_SPARK_GAP_MS     2000UL // ms gap after spark clears before next spark in burst
#define PCBA_BURST_PAUSE_MIN  8000UL // ms minimum between bursts
#define PCBA_BURST_PAUSE_MAX 25000UL // ms maximum between bursts
#define PCBA_BOARD_CYCLE_MS 300000UL // ms per board (= round — one board per round)
#define PCBA_ROUND_MS       300000UL // ms per route round
#define PCBA_ROUTE_LEN_MIN       6   // minimum route length (steps)
#define PCBA_ROUTE_LEN_MAX      14   // maximum route length
#define PCBA_BEAT_SURGE         1.8f // speed multiplier on beat
#define PCBA_BEAT_DECAY         0.88f
// Head flicker — single pixel, oscillates 10%–100% brightness each frame
#define PCBA_HEAD_MIN           26   // 10% of 255 — dimmest flicker value
#define PCBA_HEAD_MAX          255   // 100% — brightest flicker value

// ── Colours ───────────────────────────────────────────────────────────────────
// Board interior — single flat dark green
#define PCBA_COL_BOARD_R  0
#define PCBA_COL_BOARD_G 45
#define PCBA_COL_BOARD_B  4
// Route trace — light green
#define PCBA_COL_TRACE_R  0
#define PCBA_COL_TRACE_G 105
#define PCBA_COL_TRACE_B  32
// Tail colour stops
//   near-head zone : bright gold
#define PCBA_GOLD_R  255
#define PCBA_GOLD_G  200
#define PCBA_GOLD_B   10
//   mid zone       : amber
#define PCBA_AMBER_R 210
#define PCBA_AMBER_G  80
#define PCBA_AMBER_B   0
//   tip zone       : trace green (aliases so it auto-tracks trace colour)
#define PCBA_TIP_R   PCBA_COL_TRACE_R
#define PCBA_TIP_G   PCBA_COL_TRACE_G
#define PCBA_TIP_B   PCBA_COL_TRACE_B
// ── Board outline bitmasks ────────────────────────────────────────────────────
// 6 shapes. Each row byte: bit7 = col 0 (left), bit0 = col 7 (right).
// Row order: row 0 = bottom, row 7 = top.
static const uint8_t _pcbaBoards[6][8] = {
    // 0: Full rectangle with top notches (component keepout)
    { 0b11111111,
      0b11111111,
      0b11111111,
      0b11111111,
      0b11111111,
      0b11111111,
      0b11111111,
      0b11011011 },

    // 1: Large rectangle, bottom-left corner cut, right-edge notch
    { 0b00111111,
      0b01111111,
      0b11111111,
      0b11111111,
      0b11111111,
      0b11111111,
      0b11111111,
      0b11111110 },

    // 2: D-shape — right side has a bite taken out of the centre
    { 0b11111110,
      0b11111111,
      0b11111111,
      0b11110111,
      0b11111111,
      0b11111111,
      0b11111110,
      0b11111100 },

    // 3: L-shape — upper-right quadrant absent
    { 0b11111111,
      0b11111111,
      0b11111111,
      0b11111111,
      0b11110000,
      0b11110000,
      0b11110000,
      0b11110000 },

    // 4: Castellated edges top + bottom (stamp-out module)
    { 0b10101010,
      0b11111111,
      0b11111111,
      0b11111111,
      0b11111111,
      0b11111111,
      0b11111111,
      0b10101010 },

    // 5: Plus/cross shape — corners removed
    { 0b00111100,
      0b01111110,
      0b11111111,
      0b11111111,
      0b11111111,
      0b11111111,
      0b01111110,
      0b00111100 },
};

static inline bool _pcbaInBoard(int x, int y, uint8_t shape) {
    if (x < 0 || x > 7 || y < 0 || y > 7) return false;
    return (_pcbaBoards[shape][y] >> (7 - x)) & 1;
}

// ── Single route ──────────────────────────────────────────────────────────────
#define PCBA_ROUTE_CAP  PCBA_ROUTE_LEN_MAX

static int8_t  _pcbaRtX[PCBA_ROUTE_CAP]; // x coords of each step
static int8_t  _pcbaRtY[PCBA_ROUTE_CAP]; // y coords of each step
static uint8_t _pcbaRtLen = 0;           // actual number of steps

// ── Spark pool ────────────────────────────────────────────────────────────────
struct PcbaSpark {
    float   pos;         // float position along route (0 .. rtLen-1)
    float   speed;
    float   speedBoost;
    bool    active;
};
static PcbaSpark _pcbaSparks[PCBA_MAX_SPARKS];

// ── Spark sequencer ────────────────────────────────────────────────────────────
// One spark travels at a time. After it fully clears the route (head + tail
// both past the end), a PCBA_SPARK_GAP_MS pause fires before the next spark
// in the burst. After a burst drains, PCBA_BURST_PAUSE_MIN/MAX applies.
static uint32_t _pcbaNextSparkAt;    // millis() when next spark may launch
static uint8_t  _pcbaBurstRemain;    // sparks still to launch in current burst

// ── Timing / shape ────────────────────────────────────────────────────────────
static uint32_t _pcbaBoardAt = 0;  // millis of last board change
static uint32_t _pcbaRoundAt = 0;  // millis of last route regen
static uint8_t  _pcbaShape   = 0;

// ── Helpers ───────────────────────────────────────────────────────────────────
static inline uint8_t _pcbaRandRange(uint8_t lo, uint8_t hi) {
    if (hi <= lo) return lo;
    return lo + random8(hi - lo + 1);
}

// ── Route generator ───────────────────────────────────────────────────────────
// One route: starts at a random top-edge pixel, walks downward with
// occasional side-steps. Strictly inside board mask.
static void _pcbaGenRoute(uint8_t shape) {
    _pcbaRtLen = 0;

    // Collect top-edge (row 7) entry points
    int8_t topPx[8]; uint8_t nTop = 0;
    for (int x = 0; x < 8; x++)
        if (_pcbaInBoard(x, 7, shape)) topPx[nTop++] = x;
    if (nTop == 0) return;

    int8_t cx = topPx[random8(nTop)];
    int8_t cy = 7;
    _pcbaRtX[_pcbaRtLen] = cx;
    _pcbaRtY[_pcbaRtLen] = cy;
    _pcbaRtLen = 1;

    uint8_t stuck = 0;
    while (_pcbaRtLen < PCBA_ROUTE_LEN_MAX && stuck < 5) {
        // Bias: 60% down, 20% left, 20% right; never up
        uint8_t roll = random8(5);
        int8_t ndx, ndy;
        if      (roll <= 2) { ndx =  0; ndy = -1; } // down
        else if (roll == 3) { ndx = -1; ndy =  0; } // left
        else                { ndx =  1; ndy =  0; } // right

        int8_t nx = cx + ndx, ny = cy + ndy;
        if (!_pcbaInBoard(nx, ny, shape)) { stuck++; continue; }

        // Avoid revisiting last 3 steps
        bool revisit = false;
        int look = (int)_pcbaRtLen - 1;
        for (int k = look; k >= look - 2 && k >= 0; k--)
            if (_pcbaRtX[k] == nx && _pcbaRtY[k] == ny)
                { revisit = true; break; }
        if (revisit) { stuck++; continue; }

        stuck = 0;
        cx = nx; cy = ny;
        _pcbaRtX[_pcbaRtLen] = cx;
        _pcbaRtY[_pcbaRtLen] = cy;
        _pcbaRtLen++;
    }
    // Pad to minimum length by accepting whatever we got
    // (even a short route is valid)
}

// ── Spark launcher ────────────────────────────────────────────────────────────
static void _pcbaLaunchSpark() {
    for (int i = 0; i < PCBA_MAX_SPARKS; i++) {
        if (_pcbaSparks[i].active) continue;
        _pcbaSparks[i].active     = true;
        _pcbaSparks[i].pos        = 0.0f;
        _pcbaSparks[i].speed      = PCBA_SPARK_SPEED
                                    * (0.82f + (float)random8(36) / 100.0f);
        _pcbaSparks[i].speedBoost = 1.0f;
        return;
    }
}

// ── Init ──────────────────────────────────────────────────────────────────────
static void _pcbaInit() {
    _pcbaShape   = 0;
    _pcbaBoardAt = millis();
    _pcbaRoundAt = millis();
    memset(_pcbaSparks, 0, sizeof(_pcbaSparks));
    _pcbaGenRoute(_pcbaShape);
    {
        _pcbaNextSparkAt = millis() + 300UL;  // first spark within 300 ms
    }
    _pcbaBurstRemain = 0;
}

// ── Render ────────────────────────────────────────────────────────────────────
static void renderPcba() {
    uint32_t now = millis();

    // ── Board shape cycle ─────────────────────────────────────────────────────
    if (now - _pcbaBoardAt >= PCBA_BOARD_CYCLE_MS) {
        _pcbaBoardAt = now;
        _pcbaShape   = (_pcbaShape + 1) % 6;
        memset(_pcbaSparks, 0, sizeof(_pcbaSparks));
        _pcbaGenRoute(_pcbaShape);
        _pcbaRoundAt = now;
        _pcbaNextSparkAt = now + 300UL;   // first spark within 300 ms
        _pcbaBurstRemain = 0;
    }

    // ── Route round cycle (300 s) ─────────────────────────────────────────────
    if (now - _pcbaRoundAt >= PCBA_ROUND_MS) {
        _pcbaRoundAt = now;
        memset(_pcbaSparks, 0, sizeof(_pcbaSparks));
        _pcbaGenRoute(_pcbaShape);
        _pcbaNextSparkAt = now + 300UL;   // first spark within 300 ms
        _pcbaBurstRemain = 0;
    }

    // ── Beat ──────────────────────────────────────────────────────────────────
    if (beatFired)
        for (int i = 0; i < PCBA_MAX_SPARKS; i++)
            if (_pcbaSparks[i].active)
                _pcbaSparks[i].speedBoost = PCBA_BEAT_SURGE;

    // ── Spark sequencer ───────────────────────────────────────────────────────
    // Only one spark travels at a time. A spark is "clear" when its tail tip
    // (pos - PCBA_TAIL_LEN) has passed the end of the route. After clearance
    // the sequencer waits PCBA_SPARK_GAP_MS before the next spark in the burst,
    // then PCBA_BURST_PAUSE_MIN/MAX between bursts.
    {
        // Check whether the active spark (if any) has fully cleared the route
        bool anyActive = false;
        for (int i = 0; i < PCBA_MAX_SPARKS; i++)
            if (_pcbaSparks[i].active) { anyActive = true; break; }

        if (!anyActive && now >= _pcbaNextSparkAt) {
            if (_pcbaBurstRemain > 0) {
                // Still sparks left in this burst — launch the next one
                _pcbaLaunchSpark();
                _pcbaBurstRemain--;
                // Next spark allowed after current clears + gap
                // (will be re-evaluated each frame via anyActive check above)
            } else {
                // Burst drained — start a new burst after a long pause
                _pcbaBurstRemain = PCBA_BURST_SIZE_MIN
                                   + random8(PCBA_BURST_SIZE_MAX
                                             - PCBA_BURST_SIZE_MIN + 1);
                _pcbaLaunchSpark();
                _pcbaBurstRemain--;  // consumed the first immediately
                // Next burst pause will be set when this burst finishes
            }
        }
    }

    // ── Update sparks ─────────────────────────────────────────────────────────
    // A spark retires when its tail tip clears the route end, i.e. when
    // pos >= (rtLen - 1) + PCBA_TAIL_LEN.  On retirement set the gap timer.
    float clearThresh = (float)(_pcbaRtLen - 1) + (float)PCBA_TAIL_LEN;
    for (int i = 0; i < PCBA_MAX_SPARKS; i++) {
        PcbaSpark& s = _pcbaSparks[i];
        if (!s.active) continue;
        if (s.speedBoost > 1.01f) s.speedBoost *= PCBA_BEAT_DECAY;
        else s.speedBoost = 1.0f;
        s.pos += s.speed * s.speedBoost;
        if (s.pos >= clearThresh) {
            s.active = false;
            // Set gap: PCBA_SPARK_GAP_MS within a burst, long pause between bursts
            if (_pcbaBurstRemain > 0) {
                _pcbaNextSparkAt = now + PCBA_SPARK_GAP_MS;
            } else {
                uint32_t pause = PCBA_BURST_PAUSE_MIN
                                 + (uint32_t)random16()
                                   % (PCBA_BURST_PAUSE_MAX
                                      - PCBA_BURST_PAUSE_MIN + 1);
                _pcbaNextSparkAt = now + pause;
            }
        }
    }

    // ── Draw board — two colours only ─────────────────────────────────────────
    matrix.setBrightness(BRIGHTNESS);
    matrix.fillScreen(0);

    for (int y = 0; y < MATRIX_ROWS; y++)
        for (int x = 0; x < MATRIX_COLS; x++)
            if (_pcbaInBoard(x, y, _pcbaShape))
                matrix.drawPixel(x, DRAW_Y(y),
                    matrix.Color(PCBA_COL_BOARD_R,
                                 PCBA_COL_BOARD_G,
                                 PCBA_COL_BOARD_B));

    // ── Draw permanent route trace ────────────────────────────────────────────
    for (int k = 0; k < _pcbaRtLen; k++) {
        int x = _pcbaRtX[k], y = _pcbaRtY[k];
        if (x >= 0 && x < MATRIX_COLS && y >= 0 && y < MATRIX_ROWS)
            matrix.drawPixel(x, DRAW_Y(y),
                matrix.Color(PCBA_COL_TRACE_R, PCBA_COL_TRACE_G, PCBA_COL_TRACE_B));
    }

    // ── Build spark frame buffer ──────────────────────────────────────────────
    static uint8_t fbR[8][8], fbG[8][8], fbB[8][8];
    memset(fbR, 0, sizeof(fbR));
    memset(fbG, 0, sizeof(fbG));
    memset(fbB, 0, sizeof(fbB));

    for (int i = 0; i < PCBA_MAX_SPARKS; i++) {
        PcbaSpark& s = _pcbaSparks[i];
        if (!s.active) continue;

        for (int t = 0; t <= PCBA_TAIL_LEN; t++) {
            float tp = s.pos - (float)t;
            if (tp < 0.0f) break;

            // Interpolate pixel position along route
            int   wi   = (int)tp;
            float frac = tp - (float)wi;
            if (wi >= _pcbaRtLen - 1) { wi = _pcbaRtLen - 2; frac = 1.0f; }
            if (wi < 0) wi = 0;

            float fx = (float)_pcbaRtX[wi]
                       + frac * (float)(_pcbaRtX[wi+1] - _pcbaRtX[wi]);
            float fy = (float)_pcbaRtY[wi]
                       + frac * (float)(_pcbaRtY[wi+1] - _pcbaRtY[wi]);
            int px = (int)roundf(fx);
            int py = (int)roundf(fy);
            if (px < 0 || px >= MATRIX_COLS || py < 0 || py >= MATRIX_ROWS)
                continue;

            uint8_t pr, pg, pb;

            if (t == 0) {
                // ── Head: single white pixel, flickers 10%–100% per frame ──────
                uint8_t bri = PCBA_HEAD_MIN
                              + random8(PCBA_HEAD_MAX - PCBA_HEAD_MIN + 1);
                pr = bri; pg = bri; pb = bri;
            } else {
                // ── Tail: piecewise gradient, broad gold zone ─────────────────
                // tn: 0.0 = just behind head, 1.0 = tip
                float tn = (float)t / (float)PCBA_TAIL_LEN;

                // Brightness: near-full for first 60%, then linear fade to 0
                float bri;
                if (tn < 0.60f) {
                    bri = 1.0f - tn * 0.40f;   // 100% → 76% over first 60%
                } else {
                    bri = 0.76f * (1.0f - (tn - 0.60f) / 0.40f); // 76% → 0
                }

                // Colour zones (amber zone compressed to 70–85%):
                //  0%–70%  : white (255,255,255) → gold  (PCBA_GOLD)
                //  70%–85% : gold  → amber (PCBA_AMBER)  ← narrow
                //  85%–100%: amber → trace green (PCBA_TIP)
                float cr, cg, cb;
                if (tn < 0.70f) {
                    float f = tn / 0.70f;
                    cr = 255.0f + f * (PCBA_GOLD_R - 255.0f);
                    cg = 255.0f + f * (PCBA_GOLD_G - 255.0f);
                    cb = 255.0f + f * (PCBA_GOLD_B - 255.0f);
                } else if (tn < 0.85f) {
                    float f = (tn - 0.70f) / 0.15f;
                    cr = PCBA_GOLD_R  + f * (PCBA_AMBER_R - PCBA_GOLD_R);
                    cg = PCBA_GOLD_G  + f * (PCBA_AMBER_G - PCBA_GOLD_G);
                    cb = PCBA_GOLD_B  + f * (PCBA_AMBER_B - PCBA_GOLD_B);
                } else {
                    float f = (tn - 0.85f) / 0.15f;
                    cr = PCBA_AMBER_R + f * (PCBA_TIP_R - PCBA_AMBER_R);
                    cg = PCBA_AMBER_G + f * (PCBA_TIP_G - PCBA_AMBER_G);
                    cb = PCBA_AMBER_B + f * (PCBA_TIP_B - PCBA_AMBER_B);
                }

                pr = (uint8_t)(cr * bri);
                pg = (uint8_t)(cg * bri);
                pb = (uint8_t)(cb * bri);
            }

            // Per-channel max — sparks bloom, don't cancel
            if (pr > fbR[px][py]) fbR[px][py] = pr;
            if (pg > fbG[px][py]) fbG[px][py] = pg;
            if (pb > fbB[px][py]) fbB[px][py] = pb;
        }
    }

    // Composite spark buffer over board
    for (int y = 0; y < MATRIX_ROWS; y++)
        for (int x = 0; x < MATRIX_COLS; x++)
            if (fbR[x][y] || fbG[x][y] || fbB[x][y])
                matrix.drawPixel(x, DRAW_Y(y),
                    matrix.Color(fbR[x][y], fbG[x][y], fbB[x][y]));
}


// =============================================================================
// MODE_WISP — Will-o'-the-wisp
//
// A pale blue-green blob drifts slowly around the matrix on a Lissajous path,
// breathing in and out with a slow radius pulse.  Its edge is ragged and
// organic — per-pixel Perlin noise perturbs the falloff radius at each angle,
// giving a soft, irregular silhouette that shifts every frame.  A pool of
// sparkles orbit just outside the bright core at varying radii and angular
// speeds; each sparkle has its own brightness envelope so they twinkle and
// fade independently.
//
// Colour model
//   Core centre : R=160  G=255  B=255  (cold blue-white)
//   Core fringe : R=0    G=180  B=200  (deep teal)
//   Sparkles    : white-hot → pale blue-green; rare yellow-green flash
//
// Beat response
//   The blob flares (radius spike) and all sparkles surge outward.
// =============================================================================

// ── Tuning ────────────────────────────────────────────────────────────────────
#define WISP_CORE_RADIUS    2.4f   // base blob radius (pixels)
#define WISP_BREATH_DEPTH   0.8f   // ± how much radius breathes
#define WISP_BREATH_PERIOD  280    // frames per full breath cycle
#define WISP_EDGE_DEPTH     1.4f   // ± noise perturbation on edge (pixels)
#define WISP_EDGE_ZSPEED    3      // z-axis Perlin drift speed (0-255/frame)
#define WISP_DRIFT_RADIUS   2.2f   // max drift from matrix centre (pixels)
#define WISP_DRIFT_PER_A    0.0071f// drift oscillator A frequency (rad/frame)
#define WISP_DRIFT_PER_B    0.0113f// drift oscillator B frequency (irrational ratio)

#define WISP_NUM_SPARKS     10     // sparkle pool size
#define WISP_SPARK_ORBT_MIN 2.6f   // min orbit radius
#define WISP_SPARK_ORBT_MAX 4.2f   // max orbit radius
#define WISP_SPARK_LIFE_MIN 40     // min sparkle lifetime (frames)
#define WISP_SPARK_LIFE_MAX 110    // max sparkle lifetime (frames)
#define WISP_SPARK_SPD_MIN  0.022f // min angular speed (rad/frame)
#define WISP_SPARK_SPD_MAX  0.085f // max angular speed (rad/frame)

#define WISP_BEAT_FLARE     1.2f   // blob radius boost on beat
#define WISP_BEAT_DECAY     0.88f  // flare decay per frame
#define WISP_BEAT_SURGE     1.5f   // sparkle orbit surge multiplier on beat
#define WISP_BEAT_SDECAY    0.91f  // surge decay per frame

// ── Sparkle struct ────────────────────────────────────────────────────────────
struct WispSpark {
    float   angle;        // current orbital angle (radians)
    float   orbitR;       // orbit radius (pixels from blob centre)
    float   angSpd;       // angular speed (rad/frame); sign = CW or CCW
    float   phase;        // brightness phase (0–2π)
    float   phaseSpd;     // how fast phase advances (rad/frame)
    uint8_t life;         // frames remaining
    uint8_t maxLife;      // total lifetime (for fade envelope)
    bool    yellowGreen;  // rare warm-tint variant
    float   surgeBoost;   // extra outward push from beat (decays to 0)
};

// ── State ─────────────────────────────────────────────────────────────────────
static WispSpark _wispSparks[WISP_NUM_SPARKS];
static float     _wispDriftT  = 0.0f;  // drift oscillator time
static uint16_t  _wispBreathT = 0;     // breath counter
static uint16_t  _wispNoiseZ  = 0;     // Perlin z-slice for edge noise
static float     _wispFlare   = 0.0f;  // beat flare (decays to 0)

// Blob centre in pixel coordinates (derived each frame from drift)
static float     _wispCX = 0.0f, _wispCY = 0.0f;

// ── Helpers ───────────────────────────────────────────────────────────────────
static inline float _wispRandF(float lo, float hi) {
    return lo + ((float)random8() / 255.0f) * (hi - lo);
}

static void _wispSpawnSpark(WispSpark& s, bool randomPhase) {
    s.angle     = _wispRandF(0.0f, 6.2832f);
    s.orbitR    = _wispRandF(WISP_SPARK_ORBT_MIN, WISP_SPARK_ORBT_MAX);
    // 50% clockwise, 50% counter
    float spd   = _wispRandF(WISP_SPARK_SPD_MIN, WISP_SPARK_SPD_MAX);
    s.angSpd    = (random8(2) == 0) ? spd : -spd;
    s.phase     = randomPhase ? _wispRandF(0.0f, 6.2832f) : 0.0f;
    s.phaseSpd  = _wispRandF(0.04f, 0.13f);
    s.maxLife   = WISP_SPARK_LIFE_MIN
                  + random8(WISP_SPARK_LIFE_MAX - WISP_SPARK_LIFE_MIN);
    s.life      = s.maxLife;
    s.yellowGreen = (random8(100) < 12); // 12% chance of warm variant
    s.surgeBoost  = 0.0f;
}

// ── Init ──────────────────────────────────────────────────────────────────────
static void _wispInit() {
    _wispDriftT  = 0.0f;
    _wispBreathT = 0;
    _wispNoiseZ  = random16();
    _wispFlare   = 0.0f;
    float cx = (float)MATRIX_COLS * 0.5f - 0.5f;
    float cy = (float)MATRIX_ROWS * 0.5f - 0.5f;
    _wispCX = cx;  _wispCY = cy;
    for (int i = 0; i < WISP_NUM_SPARKS; i++) {
        _wispSpawnSpark(_wispSparks[i], true);
        // Stagger lifetimes so they don't all die at once at startup
        _wispSparks[i].life = 1 + random8(_wispSparks[i].maxLife);
    }
}

// ── Render ────────────────────────────────────────────────────────────────────
static void renderWisp() {

    // ── Beat ──────────────────────────────────────────────────────────────────
    if (beatFired) {
        _wispFlare = WISP_BEAT_FLARE;
        for (int i = 0; i < WISP_NUM_SPARKS; i++)
            _wispSparks[i].surgeBoost = WISP_BEAT_SURGE;
    }
    if (_wispFlare > 0.01f) _wispFlare *= WISP_BEAT_DECAY; else _wispFlare = 0.0f;

    // ── Drift — Lissajous: two sine oscillators at irrational ratio ───────────
    float matCX = (float)MATRIX_COLS * 0.5f - 0.5f;
    float matCY = (float)MATRIX_ROWS * 0.5f - 0.5f;
    _wispCX = matCX + sinf(_wispDriftT * WISP_DRIFT_PER_A) * WISP_DRIFT_RADIUS;
    _wispCY = matCY + sinf(_wispDriftT * WISP_DRIFT_PER_B) * WISP_DRIFT_RADIUS;
    _wispDriftT += 1.0f;

    // ── Breathe ───────────────────────────────────────────────────────────────
    _wispBreathT = (_wispBreathT + 1) % WISP_BREATH_PERIOD;
    float breathPhase = (float)_wispBreathT / (float)WISP_BREATH_PERIOD;
    float breathScale = 1.0f + WISP_BREATH_DEPTH
                        * sinf(breathPhase * 6.2832f); // 0..2π over one period
    float baseR = WISP_CORE_RADIUS * breathScale + _wispFlare;

    // Advance Perlin z for edge wobble
    _wispNoiseZ += WISP_EDGE_ZSPEED;

    // ── Draw background + blob ────────────────────────────────────────────────
    matrix.setBrightness(BRIGHTNESS);
    matrix.fillScreen(0);

    for (int py = 0; py < MATRIX_ROWS; py++) {
        for (int px = 0; px < MATRIX_COLS; px++) {
            float dx = (float)px - _wispCX;
            float dy = (float)py - _wispCY;
            float dist = sqrtf(dx*dx + dy*dy);

            // Sample Perlin noise along the angle to perturb the edge radius.
            // Use angle quantised to 0-255 as x, z-slice for time.
            // atan2f → -π..π, remap to 0..255 for inoise8.
            uint8_t angIdx = (uint8_t)(((atan2f(dy, dx) + 3.14159f)
                                        / 6.28318f) * 255.0f);
            uint8_t edgeNoise = inoise8(angIdx, (uint8_t)(_wispNoiseZ >> 2));
            // Map 0-255 noise to ± WISP_EDGE_DEPTH
            float edgePerturb = ((float)(int8_t)(edgeNoise - 128))
                                / 128.0f * WISP_EDGE_DEPTH;
            float effectiveR  = baseR + edgePerturb;
            effectiveR = fmaxf(effectiveR, 0.4f); // never collapse to nothing

            if (dist > effectiveR + 1.5f) continue; // fully outside — skip

            // Brightness: 1.0 at centre, falls as gaussian-ish curve
            float t;
            if (dist <= effectiveR) {
                // Inside edge: full→dim with smooth falloff
                t = 1.0f - (dist / effectiveR) * 0.55f;
            } else {
                // Outside edge: fringe fade-out over 1.5px
                t = 0.45f * (1.0f - (dist - effectiveR) / 1.5f);
                t = fmaxf(t, 0.0f);
            }

            // Colour: lerp from cold-white core (t≈1) to deep teal fringe (t≈0)
            // Core: R=160 G=255 B=255   Fringe: R=0 G=140 B=180
            uint8_t rv = (uint8_t)(t * t * 160.0f);          // core only
            uint8_t gv = (uint8_t)((t * 115.0f + 140.0f) * t);
            uint8_t bv = (uint8_t)((t *  75.0f + 180.0f) * t);

            // Clamp
            if (rv > 255) rv = 255;
            if (gv > 255) gv = 255;
            if (bv > 255) bv = 255;

            matrix.drawPixel(px, DRAW_Y(py), matrix.Color(rv, gv, bv));
        }
    }

    // ── Update and draw sparkles ───────────────────────────────────────────────
    for (int i = 0; i < WISP_NUM_SPARKS; i++) {
        WispSpark& s = _wispSparks[i];

        // Decay beat surge
        if (s.surgeBoost > 0.01f) s.surgeBoost *= WISP_BEAT_SDECAY;
        else s.surgeBoost = 0.0f;

        // Age — respawn when expired
        if (s.life == 0) { _wispSpawnSpark(s, false); }
        s.life--;

        // Orbit
        s.angle += s.angSpd;
        s.phase += s.phaseSpd;

        float r = s.orbitR + s.surgeBoost;
        float sx = _wispCX + cosf(s.angle) * r;
        float sy = _wispCY + sinf(s.angle) * r;
        int   px = (int)roundf(sx);
        int   py = (int)roundf(sy);
        if (px < 0 || px >= MATRIX_COLS || py < 0 || py >= MATRIX_ROWS) continue;

        // Brightness envelope: ramp up, sustain, ramp down + twinkle modulation
        float lifeT = (float)s.life / (float)s.maxLife; // 1→0 as spark ages
        float rampUp   = fminf(1.0f, (float)(s.maxLife - s.life) / 12.0f);
        float rampDown  = fminf(1.0f, lifeT * (float)s.maxLife / 12.0f);
        float envelope  = fminf(rampUp, rampDown);
        float twinkle   = 0.55f + 0.45f * sinf(s.phase); // 0.1 .. 1.0
        float bri       = envelope * twinkle;

        uint8_t sb;
        uint8_t sr, sg;
        if (s.yellowGreen) {
            // Rare warm flash: yellow-green "foolish fire"
            sb = (uint8_t)(bri * 80.0f);
            sg = (uint8_t)(bri * 255.0f);
            sr = (uint8_t)(bri * 200.0f);
        } else {
            // Normal sparkle: white-hot → pale blue-green
            sr = (uint8_t)(bri * 180.0f);
            sg = (uint8_t)(bri * 255.0f);
            sb = (uint8_t)(bri * 255.0f);
        }
        matrix.drawPixel(px, DRAW_Y(py), matrix.Color(sr, sg, sb));
    }
}


// =============================================================================
// MODE_ATOM — Bohr Atom Model
//
// Architecture
// ------------
//   Up to ATOM_MAX_ATOMS atoms, each with an element (H or He), a nucleus
//   rendered as hard pixel stamps, and electrons on precessing elliptical orbits.
//
//   Ghost tails
//   -----------
//   Each electron carries a tail of ATOM_TAIL_STEPS ghost positions sampled
//   backwards along the orbit at ATOM_TAIL_DTHETA radians per step.  The head
//   is drawn at full brightness; each tail step scales linearly to 0 at the tip.
//   Head brightness is 10× the first tail step, i.e. the tail runs at ≤10% of
//   head brightness (head is 90% brighter than the ghost trail).
//
//   Depth cue: both head and each tail pixel are dimmed when "behind" the nucleus
//   (sin(θ)·sin(planeAngle) < 0), giving a 3-D passing-behind-the-nucleus look.
//
//   All electrons are the same blue.  Speed is ATOM_ORBIT_SPEED rad/frame.
// =============================================================================

// ── Master parameters ─────────────────────────────────────────────────────────
#define ATOM_NUM_ATOMS          1    // 1, 2, or 3 atoms
#define ATOM_ELEMENT            2    // 1=H  2=He

// ── Electron orbit tuning ─────────────────────────────────────────────────────
#define ATOM_ORBIT_RADIUS     2.8f   // orbital radius (pixels)
#define ATOM_ORBIT_SPEED      0.38f  // angular speed (rad/frame) — fast
#define ATOM_ORBIT_SPEED_VAR  0.04f  // ± per-electron variation
#define ATOM_PRECESS_SPEED    0.012f // plane-precession rate (rad/frame)
#define ATOM_PRECESS_VAR      0.005f // ± variation

// ── Ghost tail ────────────────────────────────────────────────────────────────
#define ATOM_TAIL_STEPS       18     // number of tail ghost pixels
#define ATOM_TAIL_DTHETA      0.18f  // angle step between tail samples (radians)
// Head is drawn at ATOM_HEAD_BRI; tail step k=1 is at 45% of head brightness,
// fading to 0 at the tip.  Tail pixels near the head are tinted toward white.
#define ATOM_HEAD_BRI        255     // head brightness (0–255)

// ── Beat response ─────────────────────────────────────────────────────────────
#define ATOM_BEAT_SURGE       1.6f
#define ATOM_BEAT_DECAY       0.90f
#define ATOM_NUCLEUS_FLASH    180
#define ATOM_NUCLEUS_FDECAY   0.82f

// ── Drift (stubbed) ───────────────────────────────────────────────────────────
#define ATOM_DRIFT_ENABLE     0
#define ATOM_DRIFT_RADIUS     0.0f
#define ATOM_DRIFT_FREQ_A     0.0071f
#define ATOM_DRIFT_FREQ_B     0.0113f

// ── Colours ───────────────────────────────────────────────────────────────────
#define ATOM_PROTON_R   220
#define ATOM_PROTON_G    30
#define ATOM_PROTON_B    20
#define ATOM_NEUTRON_R  200
#define ATOM_NEUTRON_G  200
#define ATOM_NEUTRON_B  200
// All electrons: blue
#define ATOM_ELEC_R      30
#define ATOM_ELEC_G      80
#define ATOM_ELEC_B     255

// ── Element definitions ───────────────────────────────────────────────────────
struct AtomNucleonPx { int8_t dx, dy; uint8_t type; };

static const AtomNucleonPx _atomNucH[] = {
    {0, 0, 0},
};
static const AtomNucleonPx _atomNucHe[] = {
    {0, 0, 0},  {1, 0, 1},
    {0, 1, 1},  {1, 1, 0},
};

struct AtomElement {
    uint8_t protons, electrons;
    const AtomNucleonPx* nucPx;
    uint8_t nucLen;
    float   nucOffX, nucOffY;
};

static const AtomElement _atomElements[2] = {
    { 1, 1, _atomNucH,  1, 0.0f, 0.0f },
    { 2, 2, _atomNucHe, 4, 0.5f, 0.5f },
};

// ── Per-electron state ────────────────────────────────────────────────────────
#define ATOM_MAX_ELECTRONS  2

struct AtomElectron {
    float theta;
    float angSpd;
    float planeAngle;
    float precessSpd;
    float speedBoost;
};

// ── Per-atom state ────────────────────────────────────────────────────────────
#define ATOM_MAX_ATOMS_CAP  3

struct AtomState {
    float        cx, cy;
    float        driftT;
    float        nucleusFlash;
    AtomElectron elec[ATOM_MAX_ELECTRONS];
    uint8_t      numElec;
};

static AtomState _atoms[ATOM_MAX_ATOMS_CAP];

// ── Helper ────────────────────────────────────────────────────────────────────
static inline float _atomRandF(float lo, float hi) {
    return lo + ((float)random8() / 255.0f) * (hi - lo);
}

// ── Depth cue helper ──────────────────────────────────────────────────────────
// Returns brightness multiplier: full (1.0) when in front, dimmer when behind.
static inline float _atomDepth(float theta, float planeAngle) {
    float d = sinf(theta) * sinf(planeAngle);
    return (d < 0.0f) ? (0.55f - d * 0.45f) : 1.0f;
    // behind: 0.55 + 0.45*|d|, never below 0.55; front: 1.0
}

// ── Init one atom ─────────────────────────────────────────────────────────────
static void _atomInitOne(AtomState& a, float cx, float cy) {
    a.cx           = cx;
    a.cy           = cy;
    a.driftT       = _atomRandF(0.0f, 62.83f);
    a.nucleusFlash = 0.0f;

    const AtomElement& el = _atomElements[ATOM_ELEMENT - 1];
    a.numElec = el.electrons;

    for (int e = 0; e < a.numElec; e++) {
        a.elec[e].theta      = (float)e * (6.28318f / (float)a.numElec)
                               + _atomRandF(0.0f, 6.28318f);
        float spd            = ATOM_ORBIT_SPEED
                               + _atomRandF(-ATOM_ORBIT_SPEED_VAR,
                                             ATOM_ORBIT_SPEED_VAR);
        // Alternate CW/CCW so electrons chase each other on opposite arcs
        a.elec[e].angSpd     = (e % 2 == 0) ? spd : -spd;
        a.elec[e].planeAngle = _atomRandF(0.0f, 6.28318f);
        float pspd           = ATOM_PRECESS_SPEED
                               + _atomRandF(-ATOM_PRECESS_VAR, ATOM_PRECESS_VAR);
        a.elec[e].precessSpd = (e % 2 == 0) ? pspd : -pspd;
        a.elec[e].speedBoost = 1.0f;
    }
}

// ── Init all atoms ────────────────────────────────────────────────────────────
static void _atomInit() {
    float cx = (float)MATRIX_COLS * 0.5f - 0.5f;
    float cy = (float)MATRIX_ROWS * 0.5f - 0.5f;

    int n = ATOM_NUM_ATOMS;
    if (n < 1) n = 1;
    if (n > ATOM_MAX_ATOMS_CAP) n = ATOM_MAX_ATOMS_CAP;

    static const float _offX[3][3] = {
        { 0.0f,  0.0f,  0.0f },
        {-1.8f,  1.8f,  0.0f },
        { 0.0f, -2.0f,  2.0f },
    };
    static const float _offY[3][3] = {
        { 0.0f,  0.0f,  0.0f },
        { 0.0f,  0.0f,  0.0f },
        {-2.0f,  1.4f,  1.4f },
    };

    for (int i = 0; i < n; i++)
        _atomInitOne(_atoms[i],
                     cx + _offX[n-1][i],
                     cy + _offY[n-1][i]);
}

// ── Render ────────────────────────────────────────────────────────────────────
static void renderAtom() {
    int n = ATOM_NUM_ATOMS;
    if (n < 1) n = 1;
    if (n > ATOM_MAX_ATOMS_CAP) n = ATOM_MAX_ATOMS_CAP;

    const AtomElement& el = _atomElements[ATOM_ELEMENT - 1];

    // ── Beat ──────────────────────────────────────────────────────────────────
    if (beatFired) {
        for (int i = 0; i < n; i++) {
            _atoms[i].nucleusFlash = (float)ATOM_NUCLEUS_FLASH;
            for (int e = 0; e < _atoms[i].numElec; e++)
                _atoms[i].elec[e].speedBoost = ATOM_BEAT_SURGE;
        }
    }

    // ── Update ────────────────────────────────────────────────────────────────
    for (int i = 0; i < n; i++) {
        AtomState& a = _atoms[i];
#if ATOM_DRIFT_ENABLE
        float matCX = (float)MATRIX_COLS * 0.5f - 0.5f;
        float matCY = (float)MATRIX_ROWS * 0.5f - 0.5f;
        a.cx = matCX + sinf(a.driftT * ATOM_DRIFT_FREQ_A) * ATOM_DRIFT_RADIUS;
        a.cy = matCY + sinf(a.driftT * ATOM_DRIFT_FREQ_B) * ATOM_DRIFT_RADIUS;
#endif
        a.driftT += 1.0f;
        if (a.nucleusFlash > 0.5f) a.nucleusFlash *= ATOM_NUCLEUS_FDECAY;
        else a.nucleusFlash = 0.0f;

        for (int e = 0; e < a.numElec; e++) {
            AtomElectron& el2 = a.elec[e];
            if (el2.speedBoost > 1.01f) el2.speedBoost *= ATOM_BEAT_DECAY;
            else el2.speedBoost = 1.0f;
            el2.theta      += el2.angSpd * el2.speedBoost;
            el2.planeAngle += el2.precessSpd;
        }
    }

    // ── Draw ──────────────────────────────────────────────────────────────────
    matrix.setBrightness(BRIGHTNESS);
    matrix.fillScreen(0);

    // Use a per-pixel accumulation buffer so overlapping tails add correctly
    // and the nucleus (drawn last) always wins.
    static uint8_t fbR[8][8], fbG[8][8], fbB[8][8];
    memset(fbR, 0, sizeof(fbR));
    memset(fbG, 0, sizeof(fbG));
    memset(fbB, 0, sizeof(fbB));

    for (int i = 0; i < n; i++) {
        AtomState& a = _atoms[i];

        for (int e = 0; e < a.numElec; e++) {
            const AtomElectron& el2 = a.elec[e];

            // Draw tail first (oldest → newest), then head on top
            for (int k = ATOM_TAIL_STEPS; k >= 0; k--) {
                // k=0 is the head; k=ATOM_TAIL_STEPS is the tip
                float sampleTheta = el2.theta - (float)k * ATOM_TAIL_DTHETA
                                    * (el2.angSpd >= 0.0f ? 1.0f : -1.0f);

                float ex = a.cx + cosf(sampleTheta) * ATOM_ORBIT_RADIUS;
                float ey = a.cy + sinf(sampleTheta) * ATOM_ORBIT_RADIUS
                                * cosf(el2.planeAngle);

                int px = (int)roundf(ex);
                int py = (int)roundf(ey);
                if (px < 0 || px >= MATRIX_COLS || py < 0 || py >= MATRIX_ROWS)
                    continue;

                // Brightness: head = ATOM_HEAD_BRI; tail fades from 45% at k=1
                // down to 0 at k=ATOM_TAIL_STEPS (much brighter ghost trail).
                // Colour: tail pixels near the head are tinted toward white,
                // fading back to pure blue toward the tip.
                //   whiteness = (1 - k/TAIL_STEPS)^1.5  → strong near head
                float bri;
                float whiteness = 0.0f;
                if (k == 0) {
                    bri      = (float)ATOM_HEAD_BRI;
                    whiteness = 0.0f; // head stays pure colour
                } else {
                    float tailFrac = 1.0f - (float)k / (float)ATOM_TAIL_STEPS;
                    bri      = (float)ATOM_HEAD_BRI * 0.45f * tailFrac;
                    // whiteness strongest just behind head, zero at tip
                    whiteness = tailFrac * tailFrac;  // quadratic: 1→0
                }

                // Depth cue
                float depth = _atomDepth(sampleTheta, el2.planeAngle);
                bri *= depth;

                // Base colour scaled by brightness, then blend toward white
                float br_f = ATOM_ELEC_R * bri / 255.0f;
                float bg_f = ATOM_ELEC_G * bri / 255.0f;
                float bb_f = ATOM_ELEC_B * bri / 255.0f;
                // White contribution: add (bri * whiteness) to each channel, cap 255
                float wc   = bri * whiteness * 0.72f; // 0.72 keeps it tinted, not stark
                uint8_t br = (uint8_t)fminf(255.0f, br_f + wc);
                uint8_t bg = (uint8_t)fminf(255.0f, bg_f + wc);
                uint8_t bb = (uint8_t)fminf(255.0f, bb_f + wc);

                // Accumulate: take max per channel so head wins over tail
                if (br > fbR[px][py]) fbR[px][py] = br;
                if (bg > fbG[px][py]) fbG[px][py] = bg;
                if (bb > fbB[px][py]) fbB[px][py] = bb;
            }
        }

        // ── Nucleus — drawn over electron buffer ──────────────────────────────
        float flash = a.nucleusFlash / (float)ATOM_NUCLEUS_FLASH;

        for (int k = 0; k < el.nucLen; k++) {
            int px = (int)roundf(a.cx - el.nucOffX + el.nucPx[k].dx);
            int py = (int)roundf(a.cy - el.nucOffY + el.nucPx[k].dy);
            if (px < 0 || px >= MATRIX_COLS || py < 0 || py >= MATRIX_ROWS)
                continue;

            uint8_t r, g, b;
            if (el.nucPx[k].type == 0) {
                r = (uint8_t)(ATOM_PROTON_R  + flash * (255 - ATOM_PROTON_R));
                g = (uint8_t)(ATOM_PROTON_G  + flash * (255 - ATOM_PROTON_G));
                b = (uint8_t)(ATOM_PROTON_B  + flash * (255 - ATOM_PROTON_B));
            } else {
                r = (uint8_t)fminf(255.0f, ATOM_NEUTRON_R + flash * 55.0f);
                g = (uint8_t)fminf(255.0f, ATOM_NEUTRON_G + flash * 55.0f);
                b = (uint8_t)fminf(255.0f, ATOM_NEUTRON_B + flash * 55.0f);
            }
            // Nucleus always overrides electron buffer
            fbR[px][py] = r;
            fbG[px][py] = g;
            fbB[px][py] = b;
        }
    }

    // Flush frame buffer to matrix
    for (int y = 0; y < MATRIX_ROWS; y++)
        for (int x = 0; x < MATRIX_COLS; x++)
            matrix.drawPixel(x, DRAW_Y(y),
                matrix.Color(fbR[x][y], fbG[x][y], fbB[x][y]));
}
// ═════════════════════════════════════════════════════════════════════════════
// MODE_LIFE — Conway's Game of Life demo
// ═════════════════════════════════════════════════════════════════════════════
//
// ── What this does ───────────────────────────────────────────────────────────
//
//   Runs a full Conway's Game of Life simulation on an 8×8 toroidal grid
//   (edges wrap — left/right and top/bottom connect). Wrapping is essential:
//   on a finite non-wrapping grid, gliders die at the border in ~8 steps.
//
//   The display cycles through five distinct seed categories, each
//   demonstrating a different class of GoL behavior:
//
//     SEED_RANDOM     — pure random fill at ~35 % density.  Chaotic start,
//                       usually settles into still-lifes and oscillators.
//     SEED_STILLLIFE  — a curated mix of stable patterns: Block, Beehive,
//                       Loaf, Boat, Tub — placed randomly on the grid so
//                       the display is never empty but never changes.
//     SEED_OSCILLATOR — period-2 and period-3 oscillators: Blinker, Toad,
//                       Beacon, Pulsar fragment, Pentadecathlon fragment.
//                       The grid breathes rhythmically.
//     SEED_SPACESHIP  — Gliders and lightweight spaceships (LWSS) placed at
//                       varied positions and orientations.  They drift across
//                       the toroidal grid indefinitely.
//     SEED_METHUSELAH — R-pentomino and Acorn: tiny seeds that evolve for
//                       hundreds of generations before stabilising.
//
//   After GOL_RESET_MIN..GOL_RESET_MAX seconds (random each cycle) the grid
//   fades out, advances to the next category, and fades back in. Holding the
//   mode for a long time therefore cycles through all five classes.
//
// ── Color theory ─────────────────────────────────────────────────────────────
//
//   Cell age (frames alive since last birth) drives the hue position inside
//   the active palette:
//
//     Age 1  (newborn)   →  palette index 0   — peak energy color
//     Age 2-4 (young)    →  palette index ~64  — settling
//     Age 5-12 (mature)  →  palette index ~160 — stable, deep color
//     Age 13+ (ancient)  →  palette index 220  — cool, faded
//
//   This maps directly onto GoL metaphysics:
//     • Birth  = hot energy = warm/bright hues (reds, yellows, cyans)
//     • Survival = cooling = the cell "settles into" its environment
//     • Ancient cells = cold, structural = blues, purples, deep greens
//
//   Eight hand-crafted palettes rotate between resets. Each has a thematic
//   motivation tied to the categories above:
//
//     LIFE_PAL_EMBER    — birth=white-hot, old=deep red. Cellular metabolism.
//     LIFE_PAL_AURORA   — birth=cyan-white, old=deep violet. Arctic lights.
//     LIFE_PAL_TOXIC    — birth=yellow, old=deep green. Radioactive decay.
//     LIFE_PAL_COSMOS   — birth=white, old=deep indigo. Stars forming.
//     LIFE_PAL_MAGMA    — birth=bright orange, old=near-black red. Lava crust.
//     LIFE_PAL_BIOLUM   — birth=bright teal, old=dark navy. Bioluminescence.
//     LIFE_PAL_VOID     — birth=magenta, old=black. Dark energy.
//     LIFE_PAL_SPECTRUM — birth=white, cycles through full hue as cell ages.
//
//   A slow hue drift (GOL_HUE_DRIFT_SPEED) rotates all palettes slightly
//   over time so long-running sessions never feel static.
//
// ── Stasis detection ─────────────────────────────────────────────────────────
//
//   If the grid reaches a fixed point (no cells change for GOL_STASIS_FRAMES
//   consecutive generations) it triggers an early reset so the display is
//   never stuck on a static image. A brief "shiver" flash signals the reset.
//
// ── Beat interaction ─────────────────────────────────────────────────────────
//
//   beatFired injects a small amount of random noise into the grid: a handful
//   of cells flip state, temporarily disturbing stable patterns and kicking
//   off new gliders. This keeps the simulation reactive to music.
//   beatEnergy modulates the overall brightness (+20 % on hard kicks).
//
// ── Timing ───────────────────────────────────────────────────────────────────
//
//   GOL_STEP_MS     — milliseconds between simulation steps (default 120 ms
//                     ≈ ~8 generations/sec — readable on 8×8).
//   GOL_FADE_FRAMES — length of the cross-fade at resets (in render frames,
//                     not simulation steps — render runs every loop()).
//
// ═════════════════════════════════════════════════════════════════════════════

// ── Timing & simulation constants ────────────────────────────────────────────
#define GOL_STEP_MS        240      // ms between generations (halved from original 120)
#define GOL_RESET_MIN      120      // minimum seconds before forced reset
#define GOL_RESET_MAX      300      // maximum seconds before forced reset
#define GOL_FADE_FRAMES    40       // render frames for fade-out/in transition
#define GOL_STASIS_FRAMES  10       // consecutive identical generations → reset
#define GOL_BEAT_FLIPS     4        // cells to randomly flip on each beat
#define GOL_HUE_DRIFT_SPEED 1       // palette hue offset increment per reset

// ── Grid ─────────────────────────────────────────────────────────────────────
#define GOL_W  MATRIX_COLS   // 8
#define GOL_H  MATRIX_ROWS   // 8

// Cell age: 0 = dead, 1..255 = alive (capped). Age drives color.
static uint8_t _golAge[GOL_H][GOL_W];      // current generation
static uint8_t _golNext[GOL_H][GOL_W];     // scratch buffer for next gen

// ── Seed categories ───────────────────────────────────────────────────────────
#define SEED_RANDOM      0
#define SEED_STILLLIFE   1
#define SEED_OSCILLATOR  2
#define SEED_SPACESHIP   3
#define SEED_METHUSELAH  4
#define SEED_LENIA       5   // multi-kernel continuous cellular automaton
#define SEED_COUNT       6

static uint8_t _golSeedType  = 0;
static uint8_t _golPalIdx    = 0;
static uint8_t _golHueDrift  = 0;   // global hue offset, incremented each reset

// ── Reset timer ───────────────────────────────────────────────────────────────
static uint32_t _golResetAt   = 0;   // millis() when next forced reset fires
static uint8_t  _golStasis    = 0;   // consecutive unchanged-generation counter

// ── Fade state ────────────────────────────────────────────────────────────────
// States: 0 = running, 1 = fading out, 2 = fading in
static uint8_t  _golFadeState  = 0;
static uint8_t  _golFadeFrame  = 0;   // 0..GOL_FADE_FRAMES
// Snapshot of the grid at fade-out start (drawn at diminishing brightness)
static uint8_t  _golFadeSnap[GOL_H][GOL_W];

// ── Step timer ────────────────────────────────────────────────────────────────
static uint32_t _golLastStep   = 0;
static uint32_t _golGeneration = 0;  // simulation steps since current seed was applied

// ── Period-2 oscillation detector ────────────────────────────────────────────
// Blinkers, toads, and beacons cycle continuously so changed=true every step
// and the stasis counter never fires. We detect this by comparing the current
// grid to the grid 2 steps ago: if they match for GOL_OSC_RESET_N consecutive
// steps the pattern is stuck and a forced fade-out reset fires (~30 s).
static uint8_t _golSnap1Ago[GOL_H][GOL_W];  // grid 1 step ago
static uint8_t _golSnap2Ago[GOL_H][GOL_W];  // grid 2 steps ago
static uint8_t _golOscCount = 0;
// 125 steps × 240 ms/step = 30 000 ms = 30 s
#define GOL_OSC_RESET_N  125

// ── Lenia continuous field ─────────────────────────────────────────────────────
// Active when _golSeedType == SEED_LENIA.
// _lenField[y][x] ∈ [0.0, 1.0] holds the continuous activation level.
// After each Lenia step, _golAge[][] is recomputed from _lenField so the
// existing colour mapper, fade, and draw code works without modification.
static float _lenField[GOL_H][GOL_W];   // current continuous field
static float _lenNext[GOL_H][GOL_W];    // Euler scratch buffer

// Precomputed kernel tables [ky+3][kx+3] covering offsets dx,dy ∈ [−3,+3].
// Built once in _lenBuildKernels() at mode init.
static float _lenK1[7][7];   // inner-ring kernel  (R≈2.0)
static float _lenK2[7][7];   // outer-ring kernel  (R≈3.5)
static float _lenK1sum;      // normalisation denominators
static float _lenK2sum;

// ms between Lenia steps — faster than GoL (8 gens/s vs ~12 steps/s @ 80ms)
#define LEN_STEP_MS   60
// Field threshold below which a cell renders as dark (age = 0)
#define LEN_ALIVE_THR 0.04f

// ═════════════════════════════════════════════════════════════════════════════
// PALETTES
// Each palette maps cell age (0=newborn, 255=ancient) to a color.
// Index 0 = youngest (most energetic), index 255 = oldest (most settled).
// Built as 16-stop CRGBPalette16; ColorFromPalette interpolates.
// ═════════════════════════════════════════════════════════════════════════════

#define GOL_PAL_COUNT  8

static CRGBPalette16 _golPalettes[GOL_PAL_COUNT];

static void _golBuildPalettes() {
    // EMBER: white-hot birth → orange → deep crimson (cellular heat death)
    _golPalettes[0] = CRGBPalette16(
        CRGB(255,255,220),  CRGB(255,240,80),   CRGB(255,160,20),   CRGB(255,80,0),
        CRGB(220,40,0),     CRGB(180,10,0),     CRGB(140,0,0),      CRGB(100,0,0),
        CRGB(80,0,0),       CRGB(60,0,0),       CRGB(40,0,0),       CRGB(28,0,0),
        CRGB(18,0,0),       CRGB(10,0,0),       CRGB(5,0,0),        CRGB(2,0,0)
    );
    // AURORA: cyan-white birth → violet → deep indigo (arctic electromagnetic)
    _golPalettes[1] = CRGBPalette16(
        CRGB(220,255,255),  CRGB(100,255,240),  CRGB(0,220,255),    CRGB(0,160,255),
        CRGB(20,80,255),    CRGB(60,20,220),    CRGB(100,0,200),    CRGB(120,0,180),
        CRGB(100,0,140),    CRGB(70,0,100),     CRGB(40,0,80),      CRGB(20,0,60),
        CRGB(10,0,40),      CRGB(5,0,25),       CRGB(2,0,15),       CRGB(0,0,8)
    );
    // TOXIC: bright yellow birth → acid green → dark green (radioactive decay)
    _golPalettes[2] = CRGBPalette16(
        CRGB(255,255,100),  CRGB(200,255,0),    CRGB(120,230,0),    CRGB(60,200,0),
        CRGB(20,180,0),     CRGB(0,160,10),     CRGB(0,130,20),     CRGB(0,100,15),
        CRGB(0,80,10),      CRGB(0,60,5),       CRGB(0,40,0),       CRGB(0,28,0),
        CRGB(0,18,0),       CRGB(0,10,0),       CRGB(0,5,0),        CRGB(0,2,0)
    );
    // COSMOS: white birth → electric blue → deep indigo (stellar formation)
    _golPalettes[3] = CRGBPalette16(
        CRGB(255,255,255),  CRGB(180,220,255),  CRGB(80,160,255),   CRGB(20,100,255),
        CRGB(0,60,220),     CRGB(0,30,180),     CRGB(0,10,140),     CRGB(0,0,120),
        CRGB(0,0,100),      CRGB(0,0,80),       CRGB(5,0,60),       CRGB(10,0,45),
        CRGB(8,0,30),       CRGB(5,0,18),       CRGB(2,0,10),       CRGB(1,0,4)
    );
    // MAGMA: bright orange birth → dark orange → near-black (cooling lava crust)
    _golPalettes[4] = CRGBPalette16(
        CRGB(255,200,50),   CRGB(255,130,0),    CRGB(230,70,0),     CRGB(200,30,0),
        CRGB(160,10,0),     CRGB(120,5,0),      CRGB(90,2,0),       CRGB(64,0,0),
        CRGB(48,0,0),       CRGB(34,0,0),       CRGB(22,0,0),       CRGB(14,0,0),
        CRGB(8,0,0),        CRGB(4,0,0),        CRGB(2,0,0),        CRGB(1,0,0)
    );
    // BIOLUM: bright teal birth → cyan → deep navy (deep-sea bioluminescence)
    _golPalettes[5] = CRGBPalette16(
        CRGB(200,255,240),  CRGB(0,255,200),    CRGB(0,220,160),    CRGB(0,180,120),
        CRGB(0,140,100),    CRGB(0,100,80),     CRGB(0,70,70),      CRGB(0,50,60),
        CRGB(0,30,50),      CRGB(0,20,40),      CRGB(0,12,30),      CRGB(0,6,22),
        CRGB(0,3,15),       CRGB(0,1,10),       CRGB(0,0,6),        CRGB(0,0,3)
    );
    // VOID: bright magenta birth → purple → near-black (dark energy / void)
    _golPalettes[6] = CRGBPalette16(
        CRGB(255,180,255),  CRGB(255,60,220),   CRGB(220,0,180),    CRGB(180,0,140),
        CRGB(140,0,110),    CRGB(110,0,90),     CRGB(80,0,70),      CRGB(60,0,55),
        CRGB(40,0,40),      CRGB(28,0,28),      CRGB(18,0,18),      CRGB(10,0,12),
        CRGB(6,0,8),        CRGB(3,0,5),        CRGB(1,0,3),        CRGB(0,0,1)
    );
    // SPECTRUM: white birth then full hue wheel as cell ages (purest GoL demo)
    _golPalettes[7] = CRGBPalette16(
        CRGB(255,255,255),  CRGB(255,80,0),     CRGB(255,200,0),    CRGB(100,255,0),
        CRGB(0,255,80),     CRGB(0,255,220),    CRGB(0,160,255),    CRGB(0,40,255),
        CRGB(80,0,255),     CRGB(180,0,255),    CRGB(255,0,160),    CRGB(255,0,60),
        CRGB(200,0,0),      CRGB(120,0,0),      CRGB(50,0,0),       CRGB(10,0,0)
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// LENIA KERNEL PRECOMPUTATION
//
// Two annular ("ring") kernels. Each kernel weight at offset (dx, dy) is:
//
//   K(dx,dy) = exp(–((r/R – mu_K)²) / (2·sigma_K²))   for r/R ≤ 1
//            = 0                                        otherwise
//
//   where r = sqrt(dx²+dy²) is Euclidean distance to the neighbour cell.
//
// K1  (inner ring, R=2.0):  peaks at r ≈ 1.0 — weights nearest neighbours.
// K2  (outer ring, R=3.5):  peaks at r ≈ 1.75 — reaches across the 8×8 grid.
//
// The centre cell (0,0) is always excluded (self does not contribute to u).
// Normalisation sums (_lenK1sum, _lenK2sum) are stored for use in _lenStep().
// ─────────────────────────────────────────────────────────────────────────────

static void _lenBuildKernels() {
    const float R1 = 2.0f,  muK1 = 0.50f, sgK1 = 0.15f;
    const float R2 = 3.5f,  muK2 = 0.50f, sgK2 = 0.15f;

    _lenK1sum = 0.0f;
    _lenK2sum = 0.0f;

    for (int dy = -3; dy <= 3; dy++) {
        for (int dx = -3; dx <= 3; dx++) {
            if (dx == 0 && dy == 0) {
                _lenK1[dy + 3][dx + 3] = 0.0f;
                _lenK2[dy + 3][dx + 3] = 0.0f;
                continue;
            }
            float r  = sqrtf((float)(dx * dx + dy * dy));

            float r1n = r / R1;
            float v1  = (r1n <= 1.0f)
                      ? expf(-((r1n - muK1) * (r1n - muK1)) / (2.0f * sgK1 * sgK1))
                      : 0.0f;
            _lenK1[dy + 3][dx + 3] = v1;
            _lenK1sum += v1;

            float r2n = r / R2;
            float v2  = (r2n <= 1.0f)
                      ? expf(-((r2n - muK2) * (r2n - muK2)) / (2.0f * sgK2 * sgK2))
                      : 0.0f;
            _lenK2[dy + 3][dx + 3] = v2;
            _lenK2sum += v2;
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// SEED PATTERNS
// All coordinates are (col, row) = (x, y), placed with wrapping so patterns
// near the edge appear on the other side.
// ═════════════════════════════════════════════════════════════════════════════

// Helper: set a cell alive (age=1), coordinates wrap toroidally
static inline void _golSet(uint8_t x, uint8_t y) {
    _golAge[y % GOL_H][x % GOL_W] = 1;
}

// Helper: place a pattern given as (dx,dy) offsets from origin (ox,oy)
static void _golPlace(uint8_t ox, uint8_t oy,
                      const int8_t* pts, uint8_t npts) {
    for (uint8_t i = 0; i < npts; i++) {
        int8_t dx = pts[i * 2];
        int8_t dy = pts[i * 2 + 1];
        _golSet((uint8_t)((ox + dx + GOL_W * 4) % GOL_W),
                (uint8_t)((oy + dy + GOL_H * 4) % GOL_H));
    }
}

// ── Still life patterns ───────────────────────────────────────────────────────
// Block (2×2): simplest stable object
static const int8_t _golBlock[]    = { 0,0, 1,0, 0,1, 1,1 };
// Beehive (6 cells): most common naturally-occurring still life
static const int8_t _golBeehive[]  = { 1,0, 2,0, 0,1, 3,1, 1,2, 2,2 };
// Loaf (7 cells)
static const int8_t _golLoaf[]     = { 1,0, 2,0, 0,1, 3,1, 1,2, 3,2, 2,3 };
// Boat (5 cells)
static const int8_t _golBoat[]     = { 0,0, 1,0, 0,1, 2,1, 1,2 };
// Tub (4 cells) — diamond
static const int8_t _golTub[]      = { 1,0, 0,1, 2,1, 1,2 };

// ── Oscillator patterns ───────────────────────────────────────────────────────
// Blinker (period 2): three cells in a row
static const int8_t _golBlinker[]  = { 0,0, 1,0, 2,0 };
// Toad (period 2): two offset rows of 3
static const int8_t _golToad[]     = { 1,0, 2,0, 3,0, 0,1, 1,1, 2,1 };
// Beacon (period 2): two touching blocks
static const int8_t _golBeacon[]   = { 0,0, 1,0, 0,1, 3,2, 2,3, 3,3 };
// Pulsar fragment (period 3): place the top arm; the rest builds from symmetry
// Full pulsar is 13×13 — too big. This is a compact 3-period trigger cluster.
static const int8_t _golClock[]    = { 1,0, 0,1, 1,1, 2,1, 1,2 }; // period 2 "clock" cross

// ── Spaceship patterns ────────────────────────────────────────────────────────
// Glider (period 4, moves diagonally)
static const int8_t _golGlider[]   = { 1,0, 2,1, 0,2, 1,2, 2,2 };
// Glider variant — mirrored (moves opposite diagonal)
static const int8_t _golGliderM[]  = { 1,0, 0,1, 2,1, 1,2, 2,2 };  // reflected
// Glider variant — flipped vertically
static const int8_t _golGliderV[]  = { 0,0, 1,0, 2,0, 0,1, 1,2 };
// Lightweight Spaceship / LWSS (period 4, moves horizontally)
static const int8_t _golLWSS[]     = { 1,0, 4,0, 0,1, 0,2, 4,2, 0,3, 1,3, 2,3, 3,3 };

// ── Methuselah patterns ───────────────────────────────────────────────────────
// R-pentomino: 5 cells, evolves for 1103 generations before stabilising
static const int8_t _golRPento[]   = { 1,0, 2,0, 0,1, 1,1, 1,2 };
// Acorn: 7 cells, evolves for 5206 generations
static const int8_t _golAcorn[]    = { 1,0, 3,1, 0,2, 1,2, 4,2, 5,2, 6,2 };
// Pi-heptomino: chaotic methuselah
static const int8_t _golPiHept[]   = { 0,0, 1,0, 2,0, 0,1, 2,1, 0,2, 1,2, 2,2 };

// ── Seed initialisation functions ─────────────────────────────────────────────

static void _golSeedRandom() {
    // ~35 % density — higher gives early death, lower gives sparse start
    for (uint8_t y = 0; y < GOL_H; y++)
        for (uint8_t x = 0; x < GOL_W; x++)
            _golAge[y][x] = (random8(100) < 35) ? 1 : 0;
}

static void _golSeedStillLife() {
    memset(_golAge, 0, sizeof(_golAge));
    // Place 2-3 non-overlapping still-life patterns at random positions
    uint8_t count = 2 + random8(2);  // 2 or 3
    for (uint8_t i = 0; i < count; i++) {
        uint8_t ox = random8(GOL_W);
        uint8_t oy = random8(GOL_H);
        uint8_t which = random8(5);
        switch (which) {
            case 0: _golPlace(ox, oy, _golBlock,   4); break;
            case 1: _golPlace(ox, oy, _golBeehive, 6); break;
            case 2: _golPlace(ox, oy, _golLoaf,    7); break;
            case 3: _golPlace(ox, oy, _golBoat,    5); break;
            case 4: _golPlace(ox, oy, _golTub,     4); break;
        }
    }
}

static void _golSeedOscillator() {
    memset(_golAge, 0, sizeof(_golAge));
    // Place 2-4 oscillators
    uint8_t count = 2 + random8(3);
    for (uint8_t i = 0; i < count; i++) {
        uint8_t ox = random8(GOL_W);
        uint8_t oy = random8(GOL_H);
        uint8_t which = random8(4);
        switch (which) {
            case 0: _golPlace(ox, oy, _golBlinker, 3); break;
            case 1: _golPlace(ox, oy, _golToad,    6); break;
            case 2: _golPlace(ox, oy, _golBeacon,  6); break;
            case 3: _golPlace(ox, oy, _golClock,   5); break;
        }
    }
}

static void _golSeedSpaceship() {
    memset(_golAge, 0, sizeof(_golAge));
    // 2-4 gliders/spaceships at varied positions and orientations
    uint8_t count = 2 + random8(3);
    for (uint8_t i = 0; i < count; i++) {
        uint8_t ox = random8(GOL_W);
        uint8_t oy = random8(GOL_H);
        uint8_t which = random8(4);
        switch (which) {
            case 0: _golPlace(ox, oy, _golGlider,  5); break;
            case 1: _golPlace(ox, oy, _golGliderM, 5); break;
            case 2: _golPlace(ox, oy, _golGliderV, 5); break;
            // LWSS fits on 8-wide toroidal grid — place near top half
            case 3: _golPlace(ox % 4, oy, _golLWSS, 9); break;
        }
    }
}

static void _golSeedMethuselah() {
    memset(_golAge, 0, sizeof(_golAge));
    // Single methuselah near center — let it fill the grid organically
    uint8_t ox = 2 + random8(4);
    uint8_t oy = 2 + random8(4);
    uint8_t which = random8(3);
    switch (which) {
        case 0: _golPlace(ox, oy, _golRPento,  5); break;
        case 1: _golPlace(ox, oy, _golAcorn,   7); break;
        case 2: _golPlace(ox, oy, _golPiHept,  8); break;
    }
}

// ── Lenia seed: smooth Gaussian blobs ─────────────────────────────────────────
// Places 2–4 overlapping blobs with random positions, amplitudes, and spreads.
// The continuous field is then quantised into _golAge using an inverted mapping
// so high activation (≈1.0) maps to a low age value (hot palette colour) and
// low activation maps to a high age value (cool palette colour).
static void _golSeedLenia() {
    memset(_lenField, 0, sizeof(_lenField));
    uint8_t nblobs = 2 + random8(3);          // 2..4 blobs per seed
    for (uint8_t b = 0; b < nblobs; b++) {
        float cx  = (float)random8(GOL_W);
        float cy  = (float)random8(GOL_H);
        float amp = 0.40f + random8(6) * 0.10f;  // 0.40..0.90
        float sp2 = 2.00f + random8(20) * 0.15f; // spread² → blob radius
        for (uint8_t y = 0; y < GOL_H; y++) {
            for (uint8_t x = 0; x < GOL_W; x++) {
                float ddx = fabsf((float)x - cx);
                if (ddx > GOL_W * 0.5f) ddx = GOL_W - ddx;  // toroidal wrap
                float ddy = fabsf((float)y - cy);
                if (ddy > GOL_H * 0.5f) ddy = GOL_H - ddy;
                float v = _lenField[y][x] + amp * expf(-(ddx * ddx + ddy * ddy) / sp2);
                _lenField[y][x] = fminf(1.0f, v);
            }
        }
    }
    // Quantise: high field (active) → low age (hot), low field → high age (cool)
    for (uint8_t y = 0; y < GOL_H; y++)
        for (uint8_t x = 0; x < GOL_W; x++) {
            float v = _lenField[y][x];
            _golAge[y][x] = (v > LEN_ALIVE_THR)
                          ? (uint8_t)fmaxf(1.0f, (1.0f - v) * 250.0f)
                          : 0;
        }
}

// Master seed dispatch
static void _golApplySeed(uint8_t seedType) {
    switch (seedType) {
        case SEED_RANDOM:      _golSeedRandom();      break;
        case SEED_STILLLIFE:   _golSeedStillLife();   break;
        case SEED_OSCILLATOR:  _golSeedOscillator();  break;
        case SEED_SPACESHIP:   _golSeedSpaceship();   break;
        case SEED_METHUSELAH:  _golSeedMethuselah();  break;
        case SEED_LENIA:       _golSeedLenia();       break;
        default:               _golSeedRandom();      break;
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// SIMULATION STEP
// Standard Moore-neighbourhood rules on a toroidal grid:
//   Birth:    dead cell with exactly 3 neighbours → alive
//   Survival: live cell with 2 or 3 neighbours   → survives (age++)
//   Death:    all other live cells                → die
// ═════════════════════════════════════════════════════════════════════════════

static uint8_t _golCountNeighbours(uint8_t x, uint8_t y) {
    uint8_t n = 0;
    for (int8_t dy = -1; dy <= 1; dy++) {
        for (int8_t dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            uint8_t nx = (uint8_t)((x + dx + GOL_W) % GOL_W);
            uint8_t ny = (uint8_t)((y + dy + GOL_H) % GOL_H);
            if (_golAge[ny][nx] > 0) n++;
        }
    }
    return n;
}

// ── Debug helpers (compiled out when DEBUG_SERIAL or DEBUG_LIFE is 0) ─────────
#if DEBUG_SERIAL && DEBUG_LIFE

static const char* const _golSeedNames[SEED_COUNT] = {
    "RANDOM", "STILLLIFE", "OSCILLATOR", "SPACESHIP", "METHUSELAH", "LENIA"
};
static const char* const _golPalNames[GOL_PAL_COUNT] = {
    "EMBER", "AURORA", "TOXIC", "COSMOS", "MAGMA", "BIOLUM", "VOID", "SPECTRUM"
};

// Count live cells on the current grid (max 64).
static uint8_t _golCountCells() {
    uint8_t n = 0;
    for (uint8_t y = 0; y < GOL_H; y++)
        for (uint8_t x = 0; x < GOL_W; x++)
            if (_golAge[y][x] > 0) n++;
    return n;
}

// Print the 8×8 grid as ASCII art.
//   '#' = live cell (any age)   '.' = dead cell
// Row 0 is the top of the simulation coordinate space, which corresponds to
// the BOTTOM of the physical matrix (DRAW_Y inverts Y before drawPixel).
static void _golPrintGrid() {
    for (uint8_t y = 0; y < GOL_H; y++) {
        if (Serial) {
            Serial.print(F("[GoL]   "));
            for (uint8_t x = 0; x < GOL_W; x++)
                Serial.print(_golAge[y][x] > 0 ? '#' : '.');
            Serial.println();
        }
    }
}

#endif // DEBUG_SERIAL && DEBUG_LIFE

// ═════════════════════════════════════════════════════════════════════════════
// LENIA SIMULATION STEP
//
// Multi-kernel continuous cellular automaton (Chan 2019 — "Lenia").
//
// Update rule — Euler integration:
//
//   u_i(x,y)  = Σ K_i(dx,dy) · A(x+dx, y+dy) / K_i_sum     (convolution)
//   g_i       = G_i(u_i)  ∈ [−1, +1]                        (growth)
//   A(t+dt)   = clip(A(t) + dt · Σᵢ wᵢ · gᵢ,  0, 1)        (Euler step)
//
// Growth function  G(u; μ, σ) = 2·exp(–(u–μ)²/(2σ²)) – 1:
//   • G = +1  when u = μ  (maximum growth at the "right" neighbourhood density)
//   • G → −1  when u ≫ μ or u ≪ μ  (decay at wrong density)
//
// Two kernels compete on different length scales:
//   K1 (short-range, R≈2): stabilises dense local clusters  μ₁=0.35, σ₁=0.12
//   K2 (long-range,  R≈3.5): provides global coupling       μ₂=0.15, σ₂=0.08
//
// Colour: _golAge[][] is written as (1−field)×250 so high activation (bright)
// maps to a low age (hot palette entry) — a natural "temperature" metaphor.
// ═════════════════════════════════════════════════════════════════════════════

// Lenia growth function G(u; μ, σ) → [−1, +1]
static inline float _lenGrowth(float u, float mu, float sigma) {
    float d = (u - mu) / sigma;
    return 2.0f * expf(-0.5f * d * d) - 1.0f;
}

// Helper: write _golAge from the current _lenField (used after every Lenia step)
static void _lenSyncAge() {
    for (uint8_t y = 0; y < GOL_H; y++)
        for (uint8_t x = 0; x < GOL_W; x++) {
            float v = _lenField[y][x];
            // Invert: high activation → low age → hot palette colour
            _golAge[y][x] = (v > LEN_ALIVE_THR)
                          ? (uint8_t)fmaxf(1.0f, (1.0f - v) * 250.0f)
                          : 0;
        }
}

static bool _lenStep() {
    // Growth function parameters per kernel
    const float mu1 = 0.35f, sg1 = 0.12f, w1 = 0.60f;   // K1 short-range
    const float mu2 = 0.15f, sg2 = 0.08f, w2 = 0.40f;   // K2 long-range
    const float dt  = 0.10f;   // Euler step size (stable for these σ values)

    bool changed = false;

    for (uint8_t y = 0; y < GOL_H; y++) {
        for (uint8_t x = 0; x < GOL_W; x++) {

            // ── Kernel convolution (toroidal wrap) ────────────────────────────
            // Both kernels share the same spatial loop; zero-weight entries
            // are skipped for efficiency.
            float u1 = 0.0f, u2 = 0.0f;
            for (int ky = -3; ky <= 3; ky++) {
                for (int kx = -3; kx <= 3; kx++) {
                    float k1 = _lenK1[ky + 3][kx + 3];
                    float k2 = _lenK2[ky + 3][kx + 3];
                    if (k1 < 1e-4f && k2 < 1e-4f) continue;
                    uint8_t nx = (uint8_t)((x + kx + GOL_W * 4) % GOL_W);
                    uint8_t ny = (uint8_t)((y + ky + GOL_H * 4) % GOL_H);
                    float   v  = _lenField[ny][nx];
                    u1 += k1 * v;
                    u2 += k2 * v;
                }
            }
            // Normalise so u ∈ [0,1] regardless of which kernel entries fired
            if (_lenK1sum > 1e-6f) u1 /= _lenK1sum;
            if (_lenK2sum > 1e-6f) u2 /= _lenK2sum;

            // ── Weighted growth update ────────────────────────────────────────
            float g  = w1 * _lenGrowth(u1, mu1, sg1)
                     + w2 * _lenGrowth(u2, mu2, sg2);
            float nv = _lenField[y][x] + dt * g;
            nv = fmaxf(0.0f, fminf(1.0f, nv));
            _lenNext[y][x] = nv;

            // Stasis: any alive↔dead threshold crossing counts as "changed"
            bool wasAlive = (_lenField[y][x] > LEN_ALIVE_THR);
            bool nowAlive = (nv             > LEN_ALIVE_THR);
            if (wasAlive != nowAlive) changed = true;
        }
    }

    memcpy(_lenField, _lenNext, sizeof(_lenField));
    _lenSyncAge();
    return changed;
}

// Returns true if anything changed (stasis detection)
static bool _golStep() {
    bool changed = false;
    for (uint8_t y = 0; y < GOL_H; y++) {
        for (uint8_t x = 0; x < GOL_W; x++) {
            uint8_t neighbours = _golCountNeighbours(x, y);
            bool    alive      = (_golAge[y][x] > 0);

            if (alive && (neighbours == 2 || neighbours == 3)) {
                // Survival: increment age, cap at 255
                _golNext[y][x] = (_golAge[y][x] < 255) ? _golAge[y][x] + 1 : 255;
                if (_golNext[y][x] != _golAge[y][x]) changed = true;
            } else if (!alive && neighbours == 3) {
                // Birth
                _golNext[y][x] = 1;
                changed = true;
            } else {
                // Death (or remains dead)
                if (_golAge[y][x] > 0) changed = true;
                _golNext[y][x] = 0;
            }
        }
    }
    memcpy(_golAge, _golNext, sizeof(_golAge));
    return changed;
}

// ═════════════════════════════════════════════════════════════════════════════
// COLOR MAPPING
// age=1 → palette index 0 (youngest/hottest)
// age→∞ → palette index 255 (oldest/coolest), asymptotically
// A logarithmic curve keeps young cells bright for longer — important on 8×8
// where cells rarely survive more than ~30 generations anyway.
// ═════════════════════════════════════════════════════════════════════════════

static CRGB _golAgeToColor(uint8_t age, uint8_t palIdx, uint8_t hueDrift) {
    if (age == 0) return CRGB(0, 0, 0);

    // Map age 1..255 → palette index 0..255 logarithmically.
    // age=1  → idx≈0    (newborn, hottest color)
    // age=8  → idx≈100
    // age=20 → idx≈180
    // age=60 → idx≈230
    // Use integer approximation: idx = 255 * log2(age) / log2(255) ≈ 255*log2(age)/8
    // Faster: use a lookup-free formula: idx = min(255, (age * 48) >> 3) for small ages
    // then clamp. Tuned so age 1 = 0, age 32 ≈ 192, age 60+ = 240+.
    uint16_t idx16;
    if (age < 4)        idx16 = (uint16_t)age * 20;        // 0..80
    else if (age < 16)  idx16 = 80 + (uint16_t)(age-4) * 8; // 80..176
    else if (age < 40)  idx16 = 176 + (uint16_t)(age-16) * 3; // 176..248
    else                idx16 = 248;
    uint8_t idx = (uint8_t)(idx16 > 255 ? 255 : idx16);

    // Apply hue drift as a rotation around the palette
    idx = (uint8_t)((idx + hueDrift) & 0xFF);

    return ColorFromPalette(_golPalettes[palIdx], idx, 255, LINEARBLEND);
}

// ═════════════════════════════════════════════════════════════════════════════
// INIT
// ═════════════════════════════════════════════════════════════════════════════

static void _golInit() {
    _golBuildPalettes();
    _lenBuildKernels();
    memset(_golAge,    0, sizeof(_golAge));
    memset(_golNext,   0, sizeof(_golNext));
    memset(_lenField,  0, sizeof(_lenField));
    memset(_lenNext,   0, sizeof(_lenNext));

    _golSeedType   = 0;
    _golPalIdx     = 0;
    _golHueDrift   = 0;
    _golStasis     = 0;
    _golFadeState  = 0;
    _golFadeFrame  = 0;
    _golLastStep   = millis();
    _golGeneration = 0;
    memset(_golSnap1Ago, 0, sizeof(_golSnap1Ago));
    memset(_golSnap2Ago, 0, sizeof(_golSnap2Ago));
    _golOscCount   = 0;

    // Randomise first reset time: GOL_RESET_MIN..GOL_RESET_MAX seconds
    _golResetAt = millis() + (uint32_t)(GOL_RESET_MIN + random8(GOL_RESET_MAX - GOL_RESET_MIN)) * 1000UL;

    _golApplySeed(_golSeedType);

#if DEBUG_SERIAL && DEBUG_LIFE
    if (Serial) {
        Serial.println(F("[GoL] ══ MODE_LIFE initialised ════════════════════════════"));
        Serial.print(F("[GoL]   seed  : ")); Serial.println(_golSeedNames[_golSeedType]);
        Serial.print(F("[GoL]   pal   : ")); Serial.println(_golPalNames[_golPalIdx]);
        Serial.print(F("[GoL]   step  : ")); Serial.print(GOL_STEP_MS); Serial.println(F(" ms / generation"));
        Serial.print(F("[GoL]   reset : ")); Serial.print(GOL_RESET_MIN); Serial.print('-');
                                             Serial.print(GOL_RESET_MAX); Serial.println(F(" s (random)"));
        Serial.print(F("[GoL]   stasis: ")); Serial.print(GOL_STASIS_FRAMES); Serial.println(F(" frozen gens → reset"));
        Serial.print(F("[GoL]   beat  : flips ")); Serial.print(GOL_BEAT_FLIPS); Serial.println(F(" random cells"));
        _golPrintGrid();
    }
#endif
}

// ═════════════════════════════════════════════════════════════════════════════
// RENDER
// ═════════════════════════════════════════════════════════════════════════════

static void renderLife() {
    matrix.setBrightness((uint8_t)fminf(255.0f, BRIGHTNESS * (1.0f + beatEnergy * 0.20f)));
    matrix.fillScreen(0);

    uint32_t now = millis();

    // ── Fade-out phase ────────────────────────────────────────────────────────
    if (_golFadeState == 1) {
        _golFadeFrame++;
        float t = (float)_golFadeFrame / GOL_FADE_FRAMES;  // 0→1
        // Ease-out: slower at start so the existing pattern is readable, fast at end
        float alpha = (1.0f - t) * (1.0f - t);
        uint8_t a8 = (uint8_t)(alpha * 255.0f);

        for (uint8_t y = 0; y < GOL_H; y++) {
            for (uint8_t x = 0; x < GOL_W; x++) {
                uint8_t age = _golFadeSnap[y][x];
                if (age == 0) continue;
                CRGB c = _golAgeToColor(age, _golPalIdx, _golHueDrift);
                c.r = scale8(c.r, a8);
                c.g = scale8(c.g, a8);
                c.b = scale8(c.b, a8);
                matrix.drawPixel(x, DRAW_Y(y), matrix.Color(c.r, c.g, c.b));
            }
        }

        if (_golFadeFrame >= GOL_FADE_FRAMES) {
            // Cross-fade done — apply the new seed and start fade-in
            _golSeedType = (_golSeedType + 1) % SEED_COUNT;
            _golPalIdx   = (_golPalIdx   + 1) % GOL_PAL_COUNT;
            _golHueDrift = (_golHueDrift + GOL_HUE_DRIFT_SPEED) & 0xFF;
            _golStasis   = 0;
            _golGeneration = 0;
            memset(_golSnap1Ago, 0, sizeof(_golSnap1Ago));
            memset(_golSnap2Ago, 0, sizeof(_golSnap2Ago));
            _golOscCount = 0;
            _golApplySeed(_golSeedType);
#if DEBUG_SERIAL && DEBUG_LIFE
            if (Serial) {
                // Printed once per reset cycle when the old grid has fully
                // faded out and the new seed pattern has just been written
                // into _golAge[]. The fade-in animation will now bring it in.
                Serial.println(F("[GoL] ── fade-out done: new seed written ─────────────────────"));
                Serial.print(F("[GoL]   seed  : ")); Serial.println(_golSeedNames[_golSeedType]);
                Serial.print(F("[GoL]   pal   : ")); Serial.println(_golPalNames[_golPalIdx]);
                Serial.print(F("[GoL]   cells : ")); Serial.print(_golCountCells()); Serial.println(F(" live at gen 0"));
                _golPrintGrid();
            }
#endif
            _golResetAt = now + (uint32_t)(GOL_RESET_MIN + random8(GOL_RESET_MAX - GOL_RESET_MIN)) * 1000UL;
            _golFadeState = 2;
            _golFadeFrame = 0;
        }
        return;
    }

    // ── Fade-in phase ─────────────────────────────────────────────────────────
    if (_golFadeState == 2) {
        _golFadeFrame++;
        float t     = (float)_golFadeFrame / GOL_FADE_FRAMES;
        // Ease-in: slow start so the new pattern "materialises" gently
        float alpha = t * t;
        uint8_t a8  = (uint8_t)(alpha * 255.0f);

        for (uint8_t y = 0; y < GOL_H; y++) {
            for (uint8_t x = 0; x < GOL_W; x++) {
                uint8_t age = _golAge[y][x];
                if (age == 0) continue;
                CRGB c = _golAgeToColor(age, _golPalIdx, _golHueDrift);
                c.r = scale8(c.r, a8);
                c.g = scale8(c.g, a8);
                c.b = scale8(c.b, a8);
                matrix.drawPixel(x, DRAW_Y(y), matrix.Color(c.r, c.g, c.b));
            }
        }

        if (_golFadeFrame >= GOL_FADE_FRAMES) {
            _golFadeState = 0;  // running
            _golFadeFrame = 0;
            _golLastStep  = now;
#if DEBUG_SERIAL && DEBUG_LIFE
            if (Serial) {
                // Fade-in animation is complete. The simulation is now fully
                // visible and stepping normally. Generation counter is 0 here
                // because _golGeneration was reset when the seed was applied.
                Serial.println(F("[GoL] ── fade-in done: simulation running ────────────────────"));
                Serial.print(F("[GoL]   seed  : ")); Serial.println(_golSeedNames[_golSeedType]);
                Serial.print(F("[GoL]   pal   : ")); Serial.println(_golPalNames[_golPalIdx]);
                Serial.print(F("[GoL]   cells : ")); Serial.print(_golCountCells()); Serial.println(F(" live"));
            }
#endif
        }
        return;
    }

    // ── Normal running phase ──────────────────────────────────────────────────

    // Beat: inject energy into the simulation.
    //   GoL  — randomly flip GOL_BEAT_FLIPS cells.
    //   Lenia — add or remove a smooth Gaussian blob at a random location.
    if (beatFired) {
        if (_golSeedType == SEED_LENIA) {
#if DEBUG_SERIAL && DEBUG_LIFE
            if (Serial) {
                Serial.print(F("[GoL] beat @ gen=")); Serial.print(_golGeneration);
                Serial.println(F("  Lenia: injecting Gaussian blob"));
            }
#endif
            // Sign alternates: beat sometimes adds energy, sometimes removes it.
            // Spread factor 3.0 gives a blob ~1 cell in radius.
            float cx  = (float)random8(GOL_W);
            float cy  = (float)random8(GOL_H);
            float amp = (random8(2) == 0) ? 0.35f : -0.35f;
            for (uint8_t by = 0; by < GOL_H; by++) {
                for (uint8_t bx = 0; bx < GOL_W; bx++) {
                    float ddx = fabsf((float)bx - cx);
                    if (ddx > GOL_W * 0.5f) ddx = GOL_W - ddx;
                    float ddy = fabsf((float)by - cy);
                    if (ddy > GOL_H * 0.5f) ddy = GOL_H - ddy;
                    float nv = _lenField[by][bx] + amp * expf(-(ddx * ddx + ddy * ddy) * 0.5f);
                    _lenField[by][bx] = fmaxf(0.0f, fminf(1.0f, nv));
                }
            }
            _lenSyncAge();
        } else {
#if DEBUG_SERIAL && DEBUG_LIFE
            // Printed once per detected beat. Lists the (col, row) of every cell
            // that was toggled. "alive→dead" means the cell was live and is now
            // killed; "dead→alive" means it was empty and is now spawned at age 1.
            if (Serial) {
                Serial.print(F("[GoL] beat @ gen=")); Serial.print(_golGeneration);
                Serial.print(F("  flipping ")); Serial.print(GOL_BEAT_FLIPS); Serial.println(F(" cells:"));
            }
#endif
            for (uint8_t i = 0; i < GOL_BEAT_FLIPS; i++) {
                uint8_t bx = random8(GOL_W);
                uint8_t by = random8(GOL_H);
                bool wasAlive = (_golAge[by][bx] > 0);
                _golAge[by][bx] = wasAlive ? 0 : 1;
#if DEBUG_SERIAL && DEBUG_LIFE
                if (Serial) {
                    Serial.print(F("[GoL]   col=")); Serial.print(bx);
                    Serial.print(F(" row=")); Serial.print(by);
                    Serial.println(wasAlive ? F("  alive→dead") : F("  dead→alive"));
                }
#endif
            }
        }
    }

    // Advance simulation: Lenia steps faster and uses continuous rules.
    uint32_t stepMs = (_golSeedType == SEED_LENIA) ? LEN_STEP_MS : GOL_STEP_MS;
    if ((uint32_t)(now - _golLastStep) >= stepMs) {
        bool changed = (_golSeedType == SEED_LENIA) ? _lenStep() : _golStep();
        _golLastStep = now;
        _golGeneration++;

        if (!changed) {
            _golStasis++;
            if (_golStasis >= GOL_STASIS_FRAMES) {
#if DEBUG_SERIAL && DEBUG_LIFE
                if (Serial) {
                    // The grid has produced the same pattern for GOL_STASIS_FRAMES
                    // consecutive generations. This means it reached a fixed point
                    // (still life) or a cycle that _golStep() can't detect as changed.
                    // A fade-out/seed-change reset will now begin.
                    Serial.print(F("[GoL] STASIS at gen=")); Serial.print(_golGeneration);
                    Serial.print(F("  (")); Serial.print(GOL_STASIS_FRAMES);
                    Serial.println(F(" frozen gens) → triggering fade-out"));
                }
#endif
                // Grid is frozen — trigger reset
                memcpy(_golFadeSnap, _golAge, sizeof(_golAge));
                _golFadeState = 1;
                _golFadeFrame = 0;
                _golStasis    = 0;
            }
        } else {
            _golStasis = 0;
        }

        // Period-2 oscillation detection (GoL only — Lenia has its own dynamics)
        if (_golSeedType != SEED_LENIA && _golFadeState == 0 && _golGeneration > 2) {
            if (memcmp(_golAge, _golSnap2Ago, sizeof(_golAge)) == 0) {
                if (++_golOscCount >= GOL_OSC_RESET_N) {
#if DEBUG_SERIAL && DEBUG_LIFE
                    if (Serial) {
                        Serial.print(F("[GoL] OSC-STUCK at gen=")); Serial.print(_golGeneration);
                        Serial.print(F("  osc-count=")); Serial.print(_golOscCount);
                        Serial.println(F("  → fade-out"));
                    }
#endif
                    memcpy(_golFadeSnap, _golAge, sizeof(_golAge));
                    _golFadeState = 1;
                    _golFadeFrame = 0;
                    _golStasis    = 0;
                    _golOscCount  = 0;
                }
            } else {
                _golOscCount = 0;
            }
            memcpy(_golSnap2Ago, _golSnap1Ago, sizeof(_golAge));
            memcpy(_golSnap1Ago, _golAge,      sizeof(_golAge));
        }

#if DEBUG_SERIAL && DEBUG_LIFE
        if (Serial) {
            // Printed every simulation step (every GOL_STEP_MS milliseconds).
            // gen    = Conway generations since the current seed was applied.
            // seed   = active seed category (what initial pattern was loaded).
            // pal    = active colour palette name.
            // cells  = number of live cells on the 8×8 grid (0–64).
            // stasis = consecutive unchanged generations / threshold.
            //          When stasis reaches the threshold a reset is triggered.
            // osc    = consecutive period-2 matches / threshold.
            // The ASCII grid follows: '#' = live cell, '.' = dead cell.
            // Row 0 of the grid = bottom row of the physical matrix (DRAW_Y).
            Serial.print(F("[GoL] gen=")); Serial.print(_golGeneration);
            Serial.print(F("  seed=")); Serial.print(_golSeedNames[_golSeedType]);
            Serial.print(F("  pal=")); Serial.print(_golPalNames[_golPalIdx]);
            Serial.print(F("  cells=")); Serial.print(_golCountCells());
            Serial.print(F("  stasis=")); Serial.print(_golStasis);
            Serial.print(F("/")); Serial.print(GOL_STASIS_FRAMES);
            Serial.print(F("  osc=")); Serial.print(_golOscCount);
            Serial.print(F("/")); Serial.println(GOL_OSC_RESET_N);
            _golPrintGrid();
        }
#endif
    }

    // Check forced reset timer
    if (_golFadeState == 0 && (int32_t)(now - _golResetAt) >= 0) {
#if DEBUG_SERIAL && DEBUG_LIFE
        if (Serial) {
            // The scheduled wall-clock reset timer fired. This happens every
            // GOL_RESET_MIN..GOL_RESET_MAX seconds regardless of stasis, so
            // even a healthy, changing grid will eventually cycle to a new seed.
            // gen= shows how many steps ran during this seed's lifetime.
            Serial.print(F("[GoL] TIMEOUT at gen=")); Serial.print(_golGeneration);
            Serial.print(F("  cells=")); Serial.print(_golCountCells());
            Serial.println(F("  → scheduled fade-out"));
        }
#endif
        memcpy(_golFadeSnap, _golAge, sizeof(_golAge));
        _golFadeState = 1;
        _golFadeFrame = 0;
        _golStasis    = 0;
    }

    // Draw current grid
    for (uint8_t y = 0; y < GOL_H; y++) {
        for (uint8_t x = 0; x < GOL_W; x++) {
            uint8_t age = _golAge[y][x];
            if (age == 0) continue;
            CRGB c = _golAgeToColor(age, _golPalIdx, _golHueDrift);
            matrix.drawPixel(x, DRAW_Y(y), matrix.Color(c.r, c.g, c.b));
        }
    }
}
// ═════════════════════════════════════════════════════════════════════════════
// MODE_MICROBE — paramecium-inspired single-organism simulation
// ═════════════════════════════════════════════════════════════════════════════
//
// ── Behaviour overview ────────────────────────────────────────────────────────
//
//   One organism lives on the 8×8 toroidal grid (edges wrap).
//   It is 3–6 pixels in size, moves with gentle curves, and interacts
//   with food pixels scattered across the grid.
//
//   BODY SHAPE
//     Stored as up to MCB_MAX_BODY relative (dx, dy) offsets from a head
//     pixel plus a separate nucleus offset.  Five hand-crafted seed shapes
//     are chosen at spawn.  They are rotated to match the direction of travel
//     using 8-direction (45°) quantised rotation so pixels stay on the integer
//     LED grid.  The nucleus is always the most interior pixel — never on the
//     leading or trailing edge.
//
//   MOVEMENT
//     Sub-pixel position (float x, y) drifts at MCB_SPEED pixels/ms.
//     Direction evolves slowly: each step a small random angular nudge
//     ±MCB_CURVE_MAX is added (gentle curve), and every MCB_VEER_INTERVAL ms
//     a larger spontaneous veer fires (abrupt direction change).
//     Position wraps toroidally.
//
//   FOOD
//     MCB_FOOD_COUNT single green pixels are scattered at init, never on the
//     organism.  When the head pixel comes within 1 pixel of food:
//       50% EATING:   organism halts MCB_EAT_PAUSE_MS, food flashes white,
//                     organism grows one pixel (up to MCB_MAX_BODY), food
//                     reappears elsewhere after MCB_FOOD_RESPAWN_MS.
//       50% AVOIDANCE: angle swings 72°–108° away, food flashes red briefly.
//                      No growth; food stays.
//     Only the nearest food triggers per frame.
//
//   AGING + COLOR
//     Lifespan randomised MCB_LIFE_MIN_MS – MCB_LIFE_MAX_MS at spawn.
//     Age fraction t drives colour:
//       0.0–0.5  healthy  : saturated cyan-green (H=140)
//       0.5–0.8  maturing : hue slides toward orange (H=30)
//       0.8–1.0  dying    : hue → red, desaturation, dimming
//     At t=1.0: organism disappears for MCB_DEATH_FADE_MS then respawns.
//
//   FLAGELLA
//     The 1–2 rearmost body pixels flicker on/off every 80–140 ms.
//     Flagella are drawn MCB_FLAG_DIM brightness units darker than the body.
//     Flicker is suppressed during eat-pause (organism is stationary).
//
//   BEAT RESPONSE
//     beatFired triggers a brief nucleus brightness burst (+60, 3 frames).
//
// ── Tuning constants ──────────────────────────────────────────────────────────

#define MCB_MAX_BODY         4
#define MCB_MIN_BODY         1
#define MCB_FOOD_COUNT       1
#define MCB_SPEED            0.004f    // pixels / ms
#define MCB_CURVE_MAX        0.08f     // max angular nudge / step (radians)
#define MCB_VEER_INTERVAL   1400UL    // ms between spontaneous veers
#define MCB_VEER_AMOUNT      1.3f     // magnitude of spontaneous veer (radians)
#define MCB_EAT_PAUSE_MS     420UL
#define MCB_FOOD_RESPAWN_MS 2800UL
#define MCB_LIFE_MIN_MS   300000UL    // 5 minutes
#define MCB_LIFE_MAX_MS   600000UL    // 10 minutes
#define MCB_DEATH_FADE_MS   1200UL
#define MCB_FLAG_DIM         60       // brightness reduction on flagella pixels
#define MCB_NUC_FLASH_FRAMES  3

// ── Body shape definitions ─────────────────────────────────────────────────────
// Offsets relative to head pixel (0,0).  Defined for east travel (dx>0=forward).
// Rotation is applied at draw time.  Index 0 is always the head.

struct McbPixel { int8_t dx; int8_t dy; };

// Shape 0: straight 3-px slug  ●──●──●
static const McbPixel _mcbS0[] = {{0,0},{-1,0},{-2,0}};
// Shape 1: diagonal 4-px      ●
//                               ●
//                                ●
//                                 ●
static const McbPixel _mcbS1[] = {{0,0},{-1,0},{-1,-1},{-2,-1}};
// Shape 2: 2×2 compact         ●●
//                               ●●
static const McbPixel _mcbS2[] = {{0,0},{-1,0},{0,-1},{-1,-1}};
// Shape 3: elongated 5-px with bump  ●──●──●──●──●
//                                              ●
static const McbPixel _mcbS3[] = {{0,0},{-1,0},{-2,0},{-3,0},{-2,-1}};
// Shape 4: chunky 6-px 3×2 blob  ●●●
//                                  ●●●
static const McbPixel _mcbS4[] = {{0,0},{-1,0},{-2,0},{0,-1},{-1,-1},{-2,-1}};

static const McbPixel* const _mcbShapes[]    = {_mcbS0,_mcbS1,_mcbS2,_mcbS3,_mcbS4};
static const uint8_t         _mcbShapeLen[]  = {3,4,4,5,6};
static const uint8_t         _mcbNucIdx[]    = {1,2,3,2,4};
// Nucleus indices chosen as the most interior pixel of each shape.
#define MCB_SHAPE_COUNT 5

// ── Food structure ─────────────────────────────────────────────────────────────
struct McbFood {
    int8_t   x, y;
    bool     alive;
    uint32_t respawnAt;
    uint8_t  flashR, flashG, flashB;
    uint8_t  flashFrames;
};

// ── Organism state ─────────────────────────────────────────────────────────────
static struct {
    float    x, y;
    float    angle;
    uint8_t  bodyLen;
    uint8_t  shapeIdx;
    uint8_t  nucIdx;
    uint32_t spawnMs;
    uint32_t lifespanMs;
    bool     eating;
    uint32_t eatEndMs;
    uint32_t lastMoveMs;
    uint32_t lastVeerMs;
    uint8_t  flagCount;
    uint8_t  flagBodyIdx[2];
    uint32_t flagNextMs[2];
    bool     flagOn[2];
    bool     dying;
    uint32_t deathStartMs;
    uint8_t  nucFlashFrames;
} _mcb;

static McbFood _mcbFood[MCB_FOOD_COUNT];

// ── Helpers ────────────────────────────────────────────────────────────────────

static inline float _mcbWrapF(float v, float lim) {
    while (v < 0.0f)   v += lim;
    while (v >= lim)   v -= lim;
    return v;
}

static inline int8_t _mcbWrapI(int v, int lim) {
    v %= lim;
    if (v < 0) v += lim;
    return (int8_t)v;
}

static void _mcbHsv(uint8_t h, uint8_t s, uint8_t v,
                    uint8_t &r, uint8_t &g, uint8_t &b) {
    if (s == 0) { r = g = b = v; return; }
    uint8_t region = h / 43;
    uint8_t rem    = (h - region * 43) * 6;
    uint8_t p = (uint16_t)v * (255 - s) >> 8;
    uint8_t q = (uint16_t)v * (255 - ((uint16_t)s * rem >> 8)) >> 8;
    uint8_t t = (uint16_t)v * (255 - ((uint16_t)s * (255 - rem) >> 8)) >> 8;
    switch (region) {
        case 0: r=v;g=t;b=p; break; case 1: r=q;g=v;b=p; break;
        case 2: r=p;g=v;b=t; break; case 3: r=p;g=q;b=v; break;
        case 4: r=t;g=p;b=v; break; default:r=v;g=p;b=q; break;
    }
}

// Rotate shape pixel by angle, quantised to 8 directions (45° steps)
static void _mcbRotate(int8_t dx, int8_t dy, float angle, int &rx, int &ry) {
    int sector = (int)roundf(angle / (M_PI * 0.25f)) & 7;
    static const int8_t cT[8] = { 1, 1, 0,-1,-1,-1, 0, 1};
    static const int8_t sT[8] = { 0,-1,-1,-1, 0, 1, 1, 1};
    rx = (int)dx * cT[sector] - (int)dy * sT[sector];
    ry = (int)dx * sT[sector] + (int)dy * cT[sector];
}

static bool _mcbOrgHas(int8_t x, int8_t y) {
    const McbPixel* sh = _mcbShapes[_mcb.shapeIdx];
    int hx = (int)roundf(_mcb.x);
    int hy = (int)roundf(_mcb.y);
    if (hx < 0) hx = 0; if (hx >= MATRIX_COLS) hx = MATRIX_COLS-1;
    if (hy < 0) hy = 0; if (hy >= MATRIX_ROWS) hy = MATRIX_ROWS-1;
    for (uint8_t i = 0; i < _mcb.bodyLen; i++) {
        int rx, ry;
        _mcbRotate(sh[i].dx, sh[i].dy, _mcb.angle, rx, ry);
        int px = hx + rx;
        int py = hy + ry;
        if (px == (int)x && py == (int)y) return true;
    }
    return false;
}

static void _mcbSpawnFood(uint8_t idx) {
    for (uint8_t att = 0; att < 40; att++) {
        int8_t fx = (int8_t)(random8() % MATRIX_COLS);
        int8_t fy = (int8_t)(random8() % MATRIX_ROWS);
        if (_mcbOrgHas(fx, fy)) continue;
        bool clash = false;
        for (uint8_t j = 0; j < MCB_FOOD_COUNT; j++) {
            if (j != idx && _mcbFood[j].alive &&
                _mcbFood[j].x==fx && _mcbFood[j].y==fy) { clash=true; break; }
        }
        if (!clash) {
            _mcbFood[idx]={fx,fy,true,0,0,0,0,0};
            return;
        }
    }
    _mcbFood[idx]={(int8_t)(random8()%MATRIX_COLS),(int8_t)(random8()%MATRIX_ROWS),true,0,0,0,0,0};
}

static inline float _mcbAgeFrac() {
    uint32_t e = millis() - _mcb.spawnMs;
    float f = (float)e / (float)_mcb.lifespanMs;
    return f > 1.0f ? 1.0f : f;
}

static void _mcbBodyColor(float t, uint8_t &r, uint8_t &g, uint8_t &b) {
    uint8_t h, s, v;
    if (t < 0.5f)      { h=140; s=240; v=200; }
    else if (t < 0.8f) { float u=(t-0.5f)/0.3f; h=(uint8_t)(140-u*110); s=230; v=190; }
    else               { float u=(t-0.8f)/0.2f; h=(uint8_t)(30-u*25); s=(uint8_t)(230-u*130); v=(uint8_t)(180*(1.0f-u*0.5f)); }
    _mcbHsv(h,s,v,r,g,b);
}

static void _mcbNucColor(float t, uint8_t &r, uint8_t &g, uint8_t &b) {
    uint8_t h = t<0.5f ? 180 : t<0.8f ? 60 : 0;
    uint8_t s = t<0.8f ? 255 : (uint8_t)(255*(1.0f-(t-0.8f)/0.2f*0.4f));
    _mcbHsv(h, s, 255, r, g, b);
}

// ── Init ───────────────────────────────────────────────────────────────────────
static void _mcbInit() {
    uint32_t now = millis();
    _mcb.shapeIdx     = random8() % MCB_SHAPE_COUNT;
    // Always start at 1 pixel; grow by eating food (up to MCB_MAX_BODY,
    // but only during the first half of life).
    _mcb.bodyLen      = 1;
    // Nucleus must be within the active body; clamp to last pixel if needed.
    _mcb.nucIdx       = _mcbNucIdx[_mcb.shapeIdx];
    if (_mcb.nucIdx >= _mcb.bodyLen) _mcb.nucIdx = _mcb.bodyLen - 1;
    _mcb.x            = (float)(random8() % MATRIX_COLS);
    _mcb.y            = (float)(random8() % MATRIX_ROWS);
    _mcb.angle        = (float)(random8()) / 255.0f * 2.0f * M_PI;
    _mcb.spawnMs      = now;
    _mcb.lifespanMs   = MCB_LIFE_MIN_MS +
                        (uint32_t)(random16() % (MCB_LIFE_MAX_MS - MCB_LIFE_MIN_MS));
    _mcb.lastMoveMs   = now;
    _mcb.lastVeerMs   = now;
    _mcb.eating       = false;
    _mcb.dying        = false;
    _mcb.nucFlashFrames = 0;
    // No flagella at size 1; they are added when the organism grows to size 2.
    _mcb.flagCount    = 0;
    for (uint8_t i = 0; i < MCB_FOOD_COUNT; i++) {
        _mcbFood[i] = {0,0,false,0,0,0,0,0};
        _mcbSpawnFood(i);
    }
}

// ── Render ─────────────────────────────────────────────────────────────────────
static void renderMicrobe() {
    matrix.setBrightness(BRIGHTNESS);
    matrix.fillScreen(0);
    uint32_t now = millis();

    // Death: blank display, wait, respawn
    if (_mcb.dying) {
        if ((now - _mcb.deathStartMs) >= MCB_DEATH_FADE_MS) _mcbInit();
        return;
    }
    // Lifespan expired
    if (_mcbAgeFrac() >= 1.0f) { _mcb.dying=true; _mcb.deathStartMs=now; return; }

    // ── Movement ──────────────────────────────────────────────────────────────
    if (!_mcb.eating) {
        uint32_t dt = now - _mcb.lastMoveMs;
        _mcb.lastMoveMs = now;

        // Spontaneous large veer
        if ((now - _mcb.lastVeerMs) >= MCB_VEER_INTERVAL) {
            _mcb.lastVeerMs = now;
            float v = MCB_VEER_AMOUNT * (((float)(random8())/255.0f) - 0.5f) * 2.0f;
            _mcb.angle += v;
        }
        // Gentle curve nudge
        _mcb.angle += MCB_CURVE_MAX * (((float)(random8())/255.0f) - 0.5f) * 2.0f;
        while (_mcb.angle < 0.0f)          _mcb.angle += 2.0f*M_PI;
        while (_mcb.angle >= 2.0f*M_PI)    _mcb.angle -= 2.0f*M_PI;

        float dist = MCB_SPEED * (float)dt;
        float nx = _mcb.x + cosf(_mcb.angle) * dist;
        float ny = _mcb.y - sinf(_mcb.angle) * dist;
        // Bounce off edges — reflect the velocity component that crossed a wall
        if (nx < 0.0f)                     { nx = -nx;                              _mcb.angle = M_PI - _mcb.angle; }
        if (nx >= (float)MATRIX_COLS)      { nx = 2.0f*(float)MATRIX_COLS - nx - 1.0f; _mcb.angle = M_PI - _mcb.angle; }
        if (ny < 0.0f)                     { ny = -ny;                              _mcb.angle = -_mcb.angle; }
        if (ny >= (float)MATRIX_ROWS)      { ny = 2.0f*(float)MATRIX_ROWS - ny - 1.0f; _mcb.angle = -_mcb.angle; }
        while (_mcb.angle < 0.0f)          _mcb.angle += 2.0f * M_PI;
        while (_mcb.angle >= 2.0f * M_PI)  _mcb.angle -= 2.0f * M_PI;
        _mcb.x = nx;
        _mcb.y = ny;

        // Food interaction (no toroidal distance — direct euclidean on bounded grid)
        int hx = (int)roundf(_mcb.x);
        int hy = (int)roundf(_mcb.y);
        if (hx < 0) hx = 0; if (hx >= MATRIX_COLS) hx = MATRIX_COLS-1;
        if (hy < 0) hy = 0; if (hy >= MATRIX_ROWS) hy = MATRIX_ROWS-1;
        for (uint8_t fi = 0; fi < MCB_FOOD_COUNT; fi++) {
            if (!_mcbFood[fi].alive) continue;
            int fdx = abs((int)_mcbFood[fi].x - hx);
            int fdy = abs((int)_mcbFood[fi].y - hy);
            if (fdx<=1 && fdy<=1) {
                if (random8() & 1) {
                    // EAT
                    _mcb.eating = true;
                    _mcb.eatEndMs = now + MCB_EAT_PAUSE_MS;
                    _mcbFood[fi].flashR=255; _mcbFood[fi].flashG=255;
                    _mcbFood[fi].flashB=255; _mcbFood[fi].flashFrames=6;
                    _mcbFood[fi].alive=false;
                    _mcbFood[fi].respawnAt = now + MCB_FOOD_RESPAWN_MS;
                    // Only grow in the first half of life, up to MCB_MAX_BODY
                    if (_mcb.bodyLen < MCB_MAX_BODY && _mcbAgeFrac() < 0.5f) {
                        _mcb.bodyLen++;
                        if (_mcb.bodyLen == 2) {
                            // First growth: add one flagellum
                            _mcb.flagBodyIdx[0] = 1;
                            _mcb.flagNextMs[0]  = now + 200 + (uint32_t)(random8() % 150);
                            _mcb.flagOn[0]      = true;
                            _mcb.flagCount      = 1;
                        } else if (_mcb.flagCount < 2) {
                            _mcb.flagBodyIdx[_mcb.flagCount] = _mcb.bodyLen-1;
                            _mcb.flagNextMs[_mcb.flagCount]  = now + 200 + (uint32_t)(random8() % 150);
                            _mcb.flagOn[_mcb.flagCount]      = true;
                            _mcb.flagCount++;
                        } else {
                            _mcb.flagBodyIdx[0] = _mcb.bodyLen-1;
                            _mcb.flagBodyIdx[1] = _mcb.bodyLen-2;
                        }
                    }
                } else {
                    // AVOID
                    float fa = atan2f((float)(_mcbFood[fi].y-hy),
                                      (float)(_mcbFood[fi].x-hx));
                    float veer = (M_PI*0.4f)*(random8()&1 ? 1.0f:-1.0f);
                    _mcb.angle = fa + M_PI + veer;
                    while (_mcb.angle<0.0f)       _mcb.angle += 2.0f*M_PI;
                    while (_mcb.angle>=2.0f*M_PI) _mcb.angle -= 2.0f*M_PI;
                    _mcbFood[fi].flashR=200; _mcbFood[fi].flashG=20;
                    _mcbFood[fi].flashB=20;  _mcbFood[fi].flashFrames=4;
                }
                break;
            }
        }
    } else {
        if (now >= _mcb.eatEndMs) _mcb.eating=false;
    }

    // Flagella flicker — slower and more deliberate than before
    if (!_mcb.eating) {
        for (uint8_t i=0;i<_mcb.flagCount;i++) {
            if (now >= _mcb.flagNextMs[i]) {
                _mcb.flagOn[i]     = !_mcb.flagOn[i];
                _mcb.flagNextMs[i] = now + 200 + (uint32_t)(random8() % 150);
            }
        }
    }

    // Food respawn
    for (uint8_t i=0;i<MCB_FOOD_COUNT;i++)
        if (!_mcbFood[i].alive && now>=_mcbFood[i].respawnAt)
            _mcbSpawnFood(i);

    // Beat nucleus flash
    if (beatFired) _mcb.nucFlashFrames = MCB_NUC_FLASH_FRAMES;
    if (_mcb.nucFlashFrames > 0) _mcb.nucFlashFrames--;

    // Colors
    float ageFrac = _mcbAgeFrac();
    uint8_t br,bg,bb,nr,ng,nb;
    _mcbBodyColor(ageFrac, br,bg,bb);
    _mcbNucColor (ageFrac, nr,ng,nb);
    if (_mcb.nucFlashFrames > 0) {
        float f=(float)_mcb.nucFlashFrames/MCB_NUC_FLASH_FRAMES;
        nr=(uint8_t)min(255,(int)nr+(int)(f*60));
        ng=(uint8_t)min(255,(int)ng+(int)(f*60));
        nb=(uint8_t)min(255,(int)nb+(int)(f*60));
    }

    // Draw food
    for (uint8_t fi=0;fi<MCB_FOOD_COUNT;fi++) {
        if (_mcbFood[fi].flashFrames > 0) {
            float f=(float)_mcbFood[fi].flashFrames/6.0f;
            matrix.drawPixel(_mcbFood[fi].x, DRAW_Y(_mcbFood[fi].y),
                matrix.Color((uint8_t)(_mcbFood[fi].flashR*f),
                             (uint8_t)(_mcbFood[fi].flashG*f),
                             (uint8_t)(_mcbFood[fi].flashB*f)));
            _mcbFood[fi].flashFrames--;
        } else if (_mcbFood[fi].alive) {
            matrix.drawPixel(_mcbFood[fi].x, DRAW_Y(_mcbFood[fi].y),
                matrix.Color(40,180,40));
        }
    }

    // Draw organism
    const McbPixel* sh = _mcbShapes[_mcb.shapeIdx];
    int hx = (int)roundf(_mcb.x);
    int hy = (int)roundf(_mcb.y);
    if (hx < 0) hx = 0; if (hx >= MATRIX_COLS) hx = MATRIX_COLS-1;
    if (hy < 0) hy = 0; if (hy >= MATRIX_ROWS) hy = MATRIX_ROWS-1;

    for (uint8_t i=0;i<_mcb.bodyLen;i++) {
        int rx,ry;
        _mcbRotate(sh[i].dx, sh[i].dy, _mcb.angle, rx,ry);
        int px = hx + rx;
        int py = hy + ry;
        if (px < 0 || px >= MATRIX_COLS || py < 0 || py >= MATRIX_ROWS) continue;

        bool isFlagellum=false, flagOn=true;
        for (uint8_t fi=0;fi<_mcb.flagCount;fi++) {
            if (_mcb.flagBodyIdx[fi]==i) { isFlagellum=true; flagOn=_mcb.flagOn[fi]; break; }
        }
        if (isFlagellum && !_mcb.eating && !flagOn) continue;

        if (i == _mcb.nucIdx) {
            matrix.drawPixel(px, DRAW_Y(py), matrix.Color(nr,ng,nb));
        } else if (isFlagellum) {
            matrix.drawPixel(px, DRAW_Y(py),
                matrix.Color((uint8_t)max(0,(int)br-MCB_FLAG_DIM),
                             (uint8_t)max(0,(int)bg-MCB_FLAG_DIM),
                             (uint8_t)max(0,(int)bb-MCB_FLAG_DIM)));
        } else if (i==0) {
            matrix.drawPixel(px, DRAW_Y(py),
                matrix.Color((uint8_t)min(255,(int)br+40),
                             (uint8_t)min(255,(int)bg+40),
                             (uint8_t)min(255,(int)bb+40)));
        } else {
            matrix.drawPixel(px, DRAW_Y(py), matrix.Color(br,bg,bb));
        }
    }
}
// ═════════════════════════════════════════════════════════════════════════════
// Init + Update — always compiled
// ═════════════════════════════════════════════════════════════════════════════

// ── Mode name table (auto-generated from VISUALIZER_MODE_TABLE) ──────────────
#define _VMODE_X_NAME(id, desc) desc,
static const char* const _vModeNames[] = { VISUALIZER_MODE_TABLE(_VMODE_X_NAME) };
#undef _VMODE_X_NAME

uint8_t visualizerModeCount() { return (uint8_t)_VMODE_COUNT; }

const char* visualizerModeName(uint8_t mode) {
    if (mode < _VMODE_COUNT) return _vModeNames[mode];
    return "Unknown";
}

void visualizerInit() {
    matrix.begin();
    matrix.setBrightness(BRIGHTNESS);
    matrix.fillScreen(0);
    matrix.show();
    initPalettes(); // populate HeatColors_p, RainbowColors_p etc.

    // Initialise all mode state — all modes are now always compiled
    _pulsePal = RainbowColors_p; memset(_pulseLeds,    0, sizeof(_pulseLeds));
    _wavePal  = RainbowColors_p; memset(_waveLeds,     0, sizeof(_waveLeds));
    _confPal  = RainbowColors_p; memset(_confLeds,     0, sizeof(_confLeds));
    memset(_tCur,      0, sizeof(_tCur));
    memset(_tNxt,      0, sizeof(_tNxt));
    memset(_tMode,     0, sizeof(_tMode));
    memset(_twLeds,    0, sizeof(_twLeds));
    memset(_twDirFlags,0, sizeof(_twDirFlags));
    memset(_rainStreaks,0, sizeof(_rainStreaks));
    memset(_rainGhost,  0, sizeof(_rainGhost));
    memset(_rainSnap,   0, sizeof(_rainSnap));
    _rainFlashFrames = 0;
    for (int i = 0; i < STAR_COUNT; i++) _sfSpawnStar(_sfStars[i]);
    _sfAccelBoost = 0.0f;
    _duneInit();
    _geoInit();
    _wispInit();
    _pcbaInit();
    _atomInit();
    _golInit();
    _mcbInit();
}

// ── Runtime mode state (variable declared near top of file) ─────────────────

void visualizerSetMode(uint8_t mode) {
    if (mode == _runtimeMode) return;
    _runtimeMode = mode;
    if (mode == MODE_PULSE)   { _pulsePal = RainbowColors_p; memset(_pulseLeds,   0, sizeof(_pulseLeds)); }
    if (mode == MODE_WAVE)    { _wavePal  = RainbowColors_p; memset(_waveLeds,    0, sizeof(_waveLeds)); }
    if (mode == MODE_CONFETTI){ _confPal  = RainbowColors_p; memset(_confLeds,    0, sizeof(_confLeds)); }
    if (mode == MODE_TORCH || mode == MODE_TORCH2) {
        memset(_tCur, 0, sizeof(_tCur));
        memset(_tNxt, 0, sizeof(_tNxt));
        memset(_tMode,0, sizeof(_tMode));
    }
    if (mode == MODE_CLOUD_TWINKLES || mode == MODE_RAINBOW_TWINKLES) {
        memset(_twLeds,    0, sizeof(_twLeds));
        memset(_twDirFlags,0, sizeof(_twDirFlags));
    }
    if (mode == MODE_RAIN) {
        memset(_rainStreaks, 0, sizeof(_rainStreaks));
        memset(_rainGhost,  0, sizeof(_rainGhost));
        memset(_rainSnap,   0, sizeof(_rainSnap));
        _rainFlashFrames = 0;
    }
    if (mode == MODE_STARFIELD) {
        for (int i = 0; i < STAR_COUNT; i++) _sfSpawnStar(_sfStars[i]);
        _sfAccelBoost = 0.0f;
    }
    if (mode == MODE_DUNE)      _duneInit();
    if (mode == MODE_GEOMETRIC) _geoInit();
    if (mode == MODE_WISP)      _wispInit();
    if (mode == MODE_PCBA)      _pcbaInit();
    if (mode == MODE_ATOM)      _atomInit();
    if (mode == MODE_LIFE)      _golInit();
    if (mode == MODE_MICROBE) _mcbInit();
    matrix.fillScreen(0);
    matrix.show();
}

void visualizerUpdate() {
    // Beat detection runs for every mode
    beatDetect();

    // Runtime dispatch — all render functions always compiled
    switch (_runtimeMode) {
        case MODE_SPECTRUM:             renderSpectrum();   break;
        case MODE_FIRE:                 renderFire();       break;
        case MODE_TORCH:
        case MODE_TORCH2:               renderTorch();      break;
        case MODE_PULSE:                renderPulse();      break;
        case MODE_WAVE:                 renderWave();       break;
        case MODE_RAINBOW_NOISE:
        case MODE_RAINBOW_STRIPE_NOISE:
        case MODE_PARTY_NOISE:
        case MODE_FOREST_NOISE:
        case MODE_CLOUD_NOISE:
        case MODE_FIRE_NOISE:
        case MODE_LAVA_NOISE:
        case MODE_OCEAN_NOISE:          renderNoise();      break;
        case MODE_CONFETTI:             renderConfetti();   break;
        case MODE_JUGGLE:               renderJuggle();     break;
        case MODE_SINELON:              renderSinelon();    break;
        case MODE_PRIDE:                renderPride();      break;
        case MODE_COLOR_WAVES:          renderColorWaves(); break;
        case MODE_CLOUD_TWINKLES:
        case MODE_RAINBOW_TWINKLES:     renderTwinkles();   break;
        case MODE_RAIN:                 renderRain();       break;
        case MODE_STARFIELD:            renderStarfield();  break;
        case MODE_DUNE:                 renderDune();       break;
        case MODE_GEOMETRIC:            renderGeometric();  break;
        case MODE_WISP:                 renderWisp();       break;
        case MODE_PCBA:                 renderPcba();       break;
        case MODE_ATOM:                 renderAtom();       break;
        case MODE_LIFE:                 renderLife();       break;
        case MODE_MICROBE:              renderMicrobe();    break;
        default:                        renderSpectrum();   break;
    }

    matrix.show();
}