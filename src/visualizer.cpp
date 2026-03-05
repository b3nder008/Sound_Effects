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

// ─── NeoMatrix ───────────────────────────────────────────────────────────────
static Adafruit_NeoMatrix matrix(
    MATRIX_COLS, MATRIX_ROWS,
    MATRIX_PIN,
    MATRIX_TYPE,
    NEO_GRB + NEO_KHZ800
);

// ─── Global brightness ───────────────────────────────────────────────────────
#define BRIGHTNESS  100   // base brightness 0–255

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

// ═════════════════════════════════════════════════════════════════════════════
// MODE_SPECTRUM — audio spectrum analyser bars
// Unchanged from the original working implementation.
// Beat modulation: global brightness flash on kick.
// ═════════════════════════════════════════════════════════════════════════════
#if ACTIVE_MODE == MODE_SPECTRUM

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
        // 1. IIR-smooth bar height
        float target = bandMagnitude[col] * MATRIX_ROWS;
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
            matrix.drawPixel(col, DRAW_Y(row), _barColor(row, 255));
        }

        // 4. Fractional tip pixel
        if (fullRows < MATRIX_ROWS && frac > 0.01f) {
            uint8_t tipBright = (uint8_t)(frac * 255.0f);
            _ghost[col][fullRows] = fmaxf(_ghost[col][fullRows], (float)tipBright);
            matrix.drawPixel(col, DRAW_Y(fullRows), _barColor(fullRows, tipBright));
        }

        // 5. Ghost trail above bar
        for (uint8_t row = fullRows + 1; row < MATRIX_ROWS; row++) {
            uint8_t gb = (uint8_t)_ghost[col][row];
            if (gb > 4)
                matrix.drawPixel(col, DRAW_Y(row), _barColor(row, gb));
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
            matrix.drawPixel(col, DRAW_Y(pRow), matrix.Color(255, 255, 255));
            if (pRow > 0)
                matrix.drawPixel(col, DRAW_Y(pRow - 1),
                    matrix.Color(PEAK_GLOW_ALPHA, PEAK_GLOW_ALPHA, PEAK_GLOW_ALPHA >> 1));
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// MODE_FIRE — Fire2012WithPalette.h (exact port)
// Beat modulation: increases sparking probability.
// ═════════════════════════════════════════════════════════════════════════════
#elif ACTIVE_MODE == MODE_FIRE

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
#elif ACTIVE_MODE == MODE_TORCH || ACTIVE_MODE == MODE_TORCH2

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
#if ACTIVE_MODE == MODE_TORCH2
    const int GREEN_EN = 80;
#else
    const int GREEN_EN = 20;
#endif
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
#elif ACTIVE_MODE == MODE_PULSE

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
#elif ACTIVE_MODE == MODE_WAVE

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
#elif ACTIVE_MODE == MODE_RAINBOW_NOISE      || \
      ACTIVE_MODE == MODE_RAINBOW_STRIPE_NOISE || \
      ACTIVE_MODE == MODE_PARTY_NOISE        || \
      ACTIVE_MODE == MODE_FOREST_NOISE       || \
      ACTIVE_MODE == MODE_CLOUD_NOISE        || \
      ACTIVE_MODE == MODE_FIRE_NOISE         || \
      ACTIVE_MODE == MODE_LAVA_NOISE         || \
      ACTIVE_MODE == MODE_OCEAN_NOISE

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
#if   ACTIVE_MODE == MODE_RAINBOW_NOISE
    _nSpeedX=9; _nSpeedY=0; _nSpeedZ=0; _nScale=30; _nColorLoop=0;
    const CRGBPalette16& pal = RainbowColors_p; uint8_t hr = 0;
#elif ACTIVE_MODE == MODE_RAINBOW_STRIPE_NOISE
    _nSpeedX=9; _nSpeedY=0; _nSpeedZ=0; _nScale=20; _nColorLoop=0;
    const CRGBPalette16& pal = RainbowStripeColors_p; uint8_t hr = 0;
#elif ACTIVE_MODE == MODE_PARTY_NOISE
    _nSpeedX=9; _nSpeedY=0; _nSpeedZ=0; _nScale=30; _nColorLoop=0;
    const CRGBPalette16& pal = PartyColors_p; uint8_t hr = 0;
#elif ACTIVE_MODE == MODE_FOREST_NOISE
    _nSpeedX=9; _nSpeedY=0; _nSpeedZ=0; _nScale=120; _nColorLoop=0;
    const CRGBPalette16& pal = ForestColors_p; uint8_t hr = 0;
#elif ACTIVE_MODE == MODE_CLOUD_NOISE
    _nSpeedX=9; _nSpeedY=0; _nSpeedZ=0; _nScale=30; _nColorLoop=0;
    const CRGBPalette16& pal = CloudColors_p; uint8_t hr = 0;
#elif ACTIVE_MODE == MODE_FIRE_NOISE
    _nSpeedX=8; _nSpeedY=0; _nSpeedZ=8; _nScale=50; _nColorLoop=0;
    const CRGBPalette16& pal = HeatColors_p; uint8_t hr = 60;
#elif ACTIVE_MODE == MODE_LAVA_NOISE
    _nSpeedX=32; _nSpeedY=0; _nSpeedZ=16; _nScale=50; _nColorLoop=0;
    const CRGBPalette16& pal = LavaColors_p; uint8_t hr = 0;
#elif ACTIVE_MODE == MODE_OCEAN_NOISE
    _nSpeedX=9; _nSpeedY=0; _nSpeedZ=0; _nScale=90; _nColorLoop=0;
    const CRGBPalette16& pal = OceanColors_p; uint8_t hr = 0;
#endif

    // Beat: temporarily speed up noise drift
    if (beatEnergy > 0.0f) {
        _nSpeedX += (uint32_t)(beatEnergy * 12.0f);
        _nSpeedZ += (uint32_t)(beatEnergy * 8.0f);
    }

    _fillNoise8();
    _mapNoiseToLeds(pal, hr);

    matrix.setBrightness(BRIGHTNESS);
    flushLeds(_noiseLeds);
}

// ═════════════════════════════════════════════════════════════════════════════
// MODE_CONFETTI — torchv2.ino confetti() (exact port)
// Beat modulation: extra splats on beat.
// ═════════════════════════════════════════════════════════════════════════════
#elif ACTIVE_MODE == MODE_CONFETTI

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
#elif ACTIVE_MODE == MODE_JUGGLE

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
#elif ACTIVE_MODE == MODE_SINELON

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
#elif ACTIVE_MODE == MODE_PRIDE

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
#elif ACTIVE_MODE == MODE_COLOR_WAVES

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
// MODE_RAINBOW / MODE_RAINBOW_GLITTER / MODE_HUE_CYCLE
// torchv2.ino rainbow(), rainbowWithGlitter(), hueCycle() (exact ports)
// Beat modulation: extra glitter burst on beat; hue jump for hue cycle.
// ═════════════════════════════════════════════════════════════════════════════
#elif ACTIVE_MODE == MODE_RAINBOW      || \
      ACTIVE_MODE == MODE_RAINBOW_GLITTER || \
      ACTIVE_MODE == MODE_HUE_CYCLE

static CRGB    _simLeds[NUM_LEDS];
static uint8_t _simGHue = 0;

static void renderSimple() {
    EVERY_N_MILLISECONDS(20) { _simGHue++; }
    if (beatFired) _simGHue += 16;

#if ACTIVE_MODE == MODE_RAINBOW
    fill_rainbow(_simLeds, NUM_LEDS, _simGHue, 1);

#elif ACTIVE_MODE == MODE_RAINBOW_GLITTER
    fill_rainbow(_simLeds, NUM_LEDS, _simGHue, 1);
    if (random8() < 80 || beatFired)
        _simLeds[random16(NUM_LEDS)] += CRGB(CRGB::White);
    if (beatFired) // extra glitter burst on beat
        for (int i = 0; i < 5; i++)
            _simLeds[random16(NUM_LEDS)] += CRGB(CRGB::White);

#elif ACTIVE_MODE == MODE_HUE_CYCLE
    uint8_t bri = beatFired ? 255 : 200; // brief brightness pop on beat
    fill_solid(_simLeds, NUM_LEDS, hsv2rgb_rainbow(CHSV(_simGHue, 255, bri)));
#endif

    matrix.setBrightness(BRIGHTNESS);
    flushLeds(_simLeds);
}

// ═════════════════════════════════════════════════════════════════════════════
// MODE_CLOUD_TWINKLES / MODE_RAINBOW_TWINKLES
// torchv2.ino colortwinkles() (exact port)
// Beat modulation: density spike on beat.
// ═════════════════════════════════════════════════════════════════════════════
#elif ACTIVE_MODE == MODE_CLOUD_TWINKLES || \
      ACTIVE_MODE == MODE_RAINBOW_TWINKLES

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
#if ACTIVE_MODE == MODE_CLOUD_TWINKLES
    const CRGBPalette16& pal = CloudColors_p;
#else
    const CRGBPalette16& pal = RainbowColors_p;
#endif
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

#endif // end of mode implementations

// ═════════════════════════════════════════════════════════════════════════════
// Init + Update — always compiled
// ═════════════════════════════════════════════════════════════════════════════

void visualizerInit() {
    matrix.begin();
    matrix.setBrightness(BRIGHTNESS);
    matrix.fillScreen(0);
    matrix.show();
    initPalettes(); // populate HeatColors_p, RainbowColors_p etc.

    // Mode-specific one-time setup
#if ACTIVE_MODE == MODE_PULSE
    _pulsePal = RainbowColors_p;
    memset(_pulseLeds, 0, sizeof(_pulseLeds));
#elif ACTIVE_MODE == MODE_WAVE
    _wavePal = RainbowColors_p;
    memset(_waveLeds, 0, sizeof(_waveLeds));
#elif ACTIVE_MODE == MODE_CONFETTI
    _confPal = RainbowColors_p;
    memset(_confLeds, 0, sizeof(_confLeds));
#elif ACTIVE_MODE == MODE_TORCH || ACTIVE_MODE == MODE_TORCH2
    memset(_tCur,  0, sizeof(_tCur));
    memset(_tNxt,  0, sizeof(_tNxt));
    memset(_tMode, 0, sizeof(_tMode));
#elif ACTIVE_MODE == MODE_CLOUD_TWINKLES || ACTIVE_MODE == MODE_RAINBOW_TWINKLES
    memset(_twLeds,    0, sizeof(_twLeds));
    memset(_twDirFlags, 0, sizeof(_twDirFlags));
#endif
}

void visualizerUpdate() {
    // Beat detection runs for every mode
    beatDetect();

    // Dispatch to exactly one render function
#if   ACTIVE_MODE == MODE_SPECTRUM
    renderSpectrum();
#elif ACTIVE_MODE == MODE_FIRE
    renderFire();
#elif ACTIVE_MODE == MODE_TORCH || ACTIVE_MODE == MODE_TORCH2
    renderTorch();
#elif ACTIVE_MODE == MODE_PULSE
    renderPulse();
#elif ACTIVE_MODE == MODE_WAVE
    renderWave();
#elif ACTIVE_MODE == MODE_RAINBOW_NOISE      || \
      ACTIVE_MODE == MODE_RAINBOW_STRIPE_NOISE || \
      ACTIVE_MODE == MODE_PARTY_NOISE        || \
      ACTIVE_MODE == MODE_FOREST_NOISE       || \
      ACTIVE_MODE == MODE_CLOUD_NOISE        || \
      ACTIVE_MODE == MODE_FIRE_NOISE         || \
      ACTIVE_MODE == MODE_LAVA_NOISE         || \
      ACTIVE_MODE == MODE_OCEAN_NOISE
    renderNoise();
#elif ACTIVE_MODE == MODE_CONFETTI
    renderConfetti();
#elif ACTIVE_MODE == MODE_JUGGLE
    renderJuggle();
#elif ACTIVE_MODE == MODE_SINELON
    renderSinelon();
#elif ACTIVE_MODE == MODE_PRIDE
    renderPride();
#elif ACTIVE_MODE == MODE_COLOR_WAVES
    renderColorWaves();
#elif ACTIVE_MODE == MODE_RAINBOW      || \
      ACTIVE_MODE == MODE_RAINBOW_GLITTER || \
      ACTIVE_MODE == MODE_HUE_CYCLE
    renderSimple();
#elif ACTIVE_MODE == MODE_CLOUD_TWINKLES || \
      ACTIVE_MODE == MODE_RAINBOW_TWINKLES
    renderTwinkles();
#endif

    matrix.show();
}
