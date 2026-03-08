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
#elif ACTIVE_MODE == MODE_RAIN

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
#elif ACTIVE_MODE == MODE_STARFIELD

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
#elif ACTIVE_MODE == MODE_DUNE

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
#elif ACTIVE_MODE == MODE_GEOMETRIC

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
// MODE_ASTEROIDS -- Asteroids-style drifting rocks with an evasive 3-pixel ship
//
// Architecture
// ------------
//   AST_NUM_ROCKS asteroids of varying radii (0-4) drift across the matrix.
//   Each has a float centre, constant velocity vector (one of 8 directions),
//   and a greyscale brightness. They wrap at the edges identical to MODE_GEOMETRIC.
//
//   Asteroid shapes are stored as compile-time pixel stamp tables: arrays of
//   (dx, dy) integer offsets from the centre that approximate circles without
//   anti-aliasing. Radius 0 = 1 pixel. Radius 4 = ~29 pixels (chunky circle).
//
//   The ship is exactly 3 pixels: a bright nose pixel and two dimmer rear
//   pixels. Their positions are derived from the ship's integer centre (sx, sy)
//   and heading direction (0-7, same direction table as MODE_GEOMETRIC).
//   The ship moves at a fixed speed. Every AST_SHIP_THINK_FRAMES frames the
//   ship AI evaluates whether its current heading will collide with any asteroid
//   within AST_SHIP_LOOKAHEAD steps and steers toward the clearest heading.
//
//   Rendering: black background, asteroids drawn back-to-front (overlapping
//   freely), ship drawn last so it always appears on top. No anti-aliasing.
//
//   Colour: asteroids are grey-scale with slight brightness variation matching
//   the starfield palette. Ship nose = pure white, rear pixels = dim white.
//
//   Beat modulation: beat brightens all asteroids briefly and injects a small
//   speed surge (same decay pattern as MODE_STARFIELD).
// =============================================================================
#elif ACTIVE_MODE == MODE_ASTEROIDS

// ── Tuning ────────────────────────────────────────────────────────────────────
#define AST_NUM_ROCKS          8    // number of asteroids
#define AST_SPEED_MIN       0.06f   // slowest asteroid (pixels/frame)
#define AST_SPEED_MAX       0.22f   // fastest asteroid (pixels/frame)
#define AST_WRAP_MARGIN      6.0f   // pixels past edge before wrap
#define AST_SHIP_SPEED      0.18f   // ship movement speed (pixels/frame)
#define AST_SHIP_THINK_FRAMES  4    // frames between ship AI decisions
#define AST_SHIP_LOOKAHEAD    14    // steps ahead the AI scans for collisions
#define AST_BEAT_BRIGHT_BOOST 60    // extra brightness added on beat
#define AST_BEAT_BRIGHT_DECAY 0.85f // brightness boost decay per frame
#define AST_BEAT_SPEED_SURGE  1.6f  // speed multiplier on beat
#define AST_BEAT_SPEED_DECAY  0.90f // speed boost decay per frame

// ── Direction table (shared with geometric, redefined here) ───────────────────
// 8 directions: E, SE, S, SW, W, NW, N, NE
static const float _astDX[8] = { 1, 1, 0,-1,-1,-1, 0, 1 };
static const float _astDY[8] = { 0, 1, 1, 1, 0,-1,-1,-1 };
// Pre-normalised diagonal scale
static const float _astDS[8] = {
    1.0f, 0.7071f, 1.0f, 0.7071f,
    1.0f, 0.7071f, 1.0f, 0.7071f
};

// ── Asteroid pixel stamps ─────────────────────────────────────────────────────
// Each stamp is a list of (dx, dy) pixel offsets from the centre.
// Hand-crafted to approximate circles of radius 0-4 with no anti-aliasing.
// Intentionally rough — real Asteroids looked chunky.

// Radius 0: single pixel
static const int8_t _astR0[][2] = { {0,0} };

// Radius 1: single pixel (halved from 3x3 cross)
static const int8_t _astR1[][2] = {
    {0,0}
};

// Radius 2: 5-pixel cross (halved from 5x5 blob)
static const int8_t _astR2[][2] = {
    {0,0},
    {1,0},{-1,0},{0,1},{0,-1}
};

// Radius 3: 9-pixel filled 3x3 (halved from 7x7 circle)
static const int8_t _astR3[][2] = {
    {0,0},
    {1,0},{-1,0},{0,1},{0,-1},
    {1,1},{-1,1},{1,-1},{-1,-1}
};

// Radius 4: 13-pixel circle radius~2 (halved from 9x9 circle)
static const int8_t _astR4[][2] = {
    {0,0},
    {1,0},{-1,0},{0,1},{0,-1},
    {2,0},{-2,0},{0,2},{0,-2},
    {1,1},{-1,1},{1,-1},{-1,-1}
};

#define _AST_R0_LEN  (sizeof(_astR0)/sizeof(_astR0[0]))
#define _AST_R1_LEN  (sizeof(_astR1)/sizeof(_astR1[0]))
#define _AST_R2_LEN  (sizeof(_astR2)/sizeof(_astR2[0]))
#define _AST_R3_LEN  (sizeof(_astR3)/sizeof(_astR3[0]))
#define _AST_R4_LEN  (sizeof(_astR4)/sizeof(_astR4[0]))

// Pointer + length pair for runtime dispatch
struct AstStamp { const int8_t (*pts)[2]; uint8_t len; };
static const AstStamp _astStamps[5] = {
    { _astR0, _AST_R0_LEN },
    { _astR1, _AST_R1_LEN },
    { _astR2, _AST_R2_LEN },
    { _astR3, _AST_R3_LEN },
    { _astR4, _AST_R4_LEN },
};

// ── Ship pixel layout per heading ─────────────────────────────────────────────
// For each of 8 headings, define 3 pixel offsets from ship centre:
//   [0] = nose (bright white)
//   [1] = rear-left  (dim)
//   [2] = rear-right (dim)
// Heading indices match _astDX/DY: 0=E, 1=SE, 2=S, 3=SW, 4=W, 5=NW, 6=N, 7=NE
static const int8_t _astShipPx[8][3][2] = {
    // 0 E:  nose right, rears upper-left / lower-left
    { { 1, 0}, {-1,-1}, {-1, 1} },
    // 1 SE: nose lower-right, rears upper-left / upper-right  (roughly)
    { { 1, 1}, {-1, 0}, { 0,-1} },
    // 2 S:  nose down, rears upper-left / upper-right
    { { 0, 1}, {-1,-1}, { 1,-1} },
    // 3 SW: nose lower-left, rears upper-right / upper-left
    { {-1, 1}, { 1, 0}, { 0,-1} },
    // 4 W:  nose left, rears upper-right / lower-right
    { {-1, 0}, { 1,-1}, { 1, 1} },
    // 5 NW: nose upper-left, rears lower-right / lower-left
    { {-1,-1}, { 1, 0}, { 0, 1} },
    // 6 N:  nose up, rears lower-left / lower-right
    { { 0,-1}, {-1, 1}, { 1, 1} },
    // 7 NE: nose upper-right, rears lower-left / lower-right
    { { 1,-1}, {-1, 0}, { 0, 1} },
};

// ── Rock ──────────────────────────────────────────────────────────────────────
struct AstRock {
    float   cx, cy;      // centre (float)
    uint8_t radius;      // 0-4
    uint8_t dirIdx;      // movement direction (0-7)
    float   speed;       // pixels/frame base
    uint8_t brightness;  // base greyscale brightness 130-240
    float   brightBoost; // extra from beat (decays to 0)
    float   speedBoost;  // extra speed multiplier from beat (decays to 1)
};

// ── Ship ──────────────────────────────────────────────────────────────────────
struct AstShip {
    float   cx, cy;      // centre (float)
    uint8_t heading;     // 0-7
    int     thinkTimer;  // counts down to next AI decision
};

// ── State ─────────────────────────────────────────────────────────────────────
static AstRock  _astRocks[AST_NUM_ROCKS];
static AstShip  _astShip;

// ── Helpers ───────────────────────────────────────────────────────────────────
static inline float _astRandRange(float lo, float hi) {
    return lo + ((float)random8() / 255.0f) * (hi - lo);
}

static inline void _astWrap(float& v, float lo, float hi) {
    float range = hi - lo;
    if (v < lo) v += range;
    if (v >= hi) v -= range;
}

static void _astSpawnRock(int i, bool offscreen) {
    AstRock& r = _astRocks[i];
    if (offscreen) {
        // Spawn at a random edge so it drifts into view
        uint8_t edge = random8(4);
        float m = AST_WRAP_MARGIN * 0.5f;
        switch (edge) {
            case 0: r.cx = _astRandRange(0, MATRIX_COLS); r.cy = -m; break;
            case 1: r.cx = _astRandRange(0, MATRIX_COLS); r.cy = MATRIX_ROWS + m; break;
            case 2: r.cx = -m; r.cy = _astRandRange(0, MATRIX_ROWS); break;
            default: r.cx = MATRIX_COLS + m; r.cy = _astRandRange(0, MATRIX_ROWS); break;
        }
    } else {
        r.cx = _astRandRange(1, MATRIX_COLS - 1);
        r.cy = _astRandRange(1, MATRIX_ROWS - 1);
    }
    r.radius     = random8(5);           // 0-4
    r.dirIdx     = random8(8);
    r.speed      = _astRandRange(AST_SPEED_MIN, AST_SPEED_MAX);
    r.brightness = 130 + random8(110);   // 130-240
    r.brightBoost = 0.0f;
    r.speedBoost  = 1.0f;
}

// Pixel occupancy check: is pixel (px, py) covered by rock i given its stamp?
static bool _astRockCoversPixel(int i, int px, int py) {
    AstRock& r = _astRocks[i];
    int cx = (int)roundf(r.cx);
    int cy = (int)roundf(r.cy);
    const AstStamp& st = _astStamps[r.radius];
    for (int k = 0; k < st.len; k++) {
        if (cx + st.pts[k][0] == px && cy + st.pts[k][1] == py) return true;
    }
    return false;
}

// Check if any asteroid occupies pixel (px, py) — used by ship AI
static bool _astAnyRockAt(int px, int py) {
    for (int i = 0; i < AST_NUM_ROCKS; i++)
        if (_astRockCoversPixel(i, px, py)) return true;
    return false;
}

// Ship AI: scan heading for asteroid pixels, return safest heading
static uint8_t _astPickHeading(float sx, float sy, uint8_t curHeading) {
    // Try directions in order of preference: current, ±1, ±2, ±3, opposite
    // Return first heading that is clear for AST_SHIP_LOOKAHEAD steps
    const uint8_t tryOrder[8] = {0, 1, 7, 2, 6, 3, 5, 4}; // relative offsets

    for (int t = 0; t < 8; t++) {
        uint8_t h = (curHeading + tryOrder[t]) & 7;
        bool clear = true;
        float tx = sx, ty = sy;
        for (int step = 1; step <= AST_SHIP_LOOKAHEAD; step++) {
            tx += _astDX[h] * _astDS[h] * AST_SHIP_SPEED;
            ty += _astDY[h] * _astDS[h] * AST_SHIP_SPEED;
            // Wrap test position
            float wtx = tx, wty = ty;
            _astWrap(wtx, -AST_WRAP_MARGIN, MATRIX_COLS + AST_WRAP_MARGIN);
            _astWrap(wty, -AST_WRAP_MARGIN, MATRIX_ROWS + AST_WRAP_MARGIN);
            int ipx = (int)roundf(wtx);
            int ipy = (int)roundf(wty);
            // Check the 3 ship pixels that would be at this position
            for (int p = 0; p < 3; p++) {
                int spx = ipx + _astShipPx[h][p][0];
                int spy = ipy + _astShipPx[h][p][1];
                if (_astAnyRockAt(spx, spy)) { clear = false; break; }
            }
            if (!clear) break;
        }
        if (clear) return h;
    }
    // All directions blocked — maintain current (rare, just hold course)
    return curHeading;
}

// ── Init ──────────────────────────────────────────────────────────────────────
static void _astInit() {
    for (int i = 0; i < AST_NUM_ROCKS; i++) _astSpawnRock(i, false);
    // Ship starts near centre
    _astShip.cx         = (float)(MATRIX_COLS / 2);
    _astShip.cy         = (float)(MATRIX_ROWS / 2);
    _astShip.heading    = 0;
    _astShip.thinkTimer = AST_SHIP_THINK_FRAMES;
}

// ── Render ────────────────────────────────────────────────────────────────────
static void renderAsteroids() {

    // ── Beat ──────────────────────────────────────────────────────────────────
    if (beatFired) {
        for (int i = 0; i < AST_NUM_ROCKS; i++) {
            _astRocks[i].brightBoost = (float)AST_BEAT_BRIGHT_BOOST;
            _astRocks[i].speedBoost  = AST_BEAT_SPEED_SURGE;
        }
    }

    // ── Update rocks ──────────────────────────────────────────────────────────
    for (int i = 0; i < AST_NUM_ROCKS; i++) {
        AstRock& r = _astRocks[i];

        // Decay beat effects
        if (r.brightBoost > 0.5f) r.brightBoost *= AST_BEAT_BRIGHT_DECAY;
        else r.brightBoost = 0.0f;
        if (r.speedBoost > 1.01f) r.speedBoost *= AST_BEAT_SPEED_DECAY;
        else r.speedBoost = 1.0f;

        // Move
        float spd = r.speed * r.speedBoost;
        r.cx += _astDX[r.dirIdx] * _astDS[r.dirIdx] * spd;
        r.cy += _astDY[r.dirIdx] * _astDS[r.dirIdx] * spd;

        // Wrap
        float lo = -AST_WRAP_MARGIN, hiX = MATRIX_COLS + AST_WRAP_MARGIN;
        float hiY = MATRIX_ROWS + AST_WRAP_MARGIN;
        _astWrap(r.cx, lo, hiX);
        _astWrap(r.cy, lo, hiY);
    }

    // ── Ship AI ───────────────────────────────────────────────────────────────
    _astShip.thinkTimer--;
    if (_astShip.thinkTimer <= 0) {
        _astShip.heading    = _astPickHeading(_astShip.cx, _astShip.cy,
                                               _astShip.heading);
        _astShip.thinkTimer = AST_SHIP_THINK_FRAMES;
    }

    // Move ship
    _astShip.cx += _astDX[_astShip.heading] * _astDS[_astShip.heading] * AST_SHIP_SPEED;
    _astShip.cy += _astDY[_astShip.heading] * _astDS[_astShip.heading] * AST_SHIP_SPEED;
    _astWrap(_astShip.cx, -AST_WRAP_MARGIN, MATRIX_COLS + AST_WRAP_MARGIN);
    _astWrap(_astShip.cy, -AST_WRAP_MARGIN, MATRIX_ROWS + AST_WRAP_MARGIN);

    // ── Draw ──────────────────────────────────────────────────────────────────
    matrix.setBrightness(BRIGHTNESS);
    matrix.fillScreen(0);

    // Draw asteroids (back-to-front, they overlap freely)
    for (int i = 0; i < AST_NUM_ROCKS; i++) {
        AstRock& r   = _astRocks[i];
        int cx       = (int)roundf(r.cx);
        int cy       = (int)roundf(r.cy);
        uint8_t bri  = (uint8_t)fminf(255.0f, (float)r.brightness + r.brightBoost);
        // Slight blue tint for larger rocks (matching starfield colour feel)
        // Small rocks are pure grey; large rocks have a faint cold cast
        uint8_t rv = (uint8_t)(bri * (1.0f - r.radius * 0.025f));
        uint8_t gv = (uint8_t)(bri * (1.0f - r.radius * 0.015f));
        uint8_t bv = bri;
        uint16_t col16 = matrix.Color(rv, gv, bv);

        const AstStamp& st = _astStamps[r.radius];
        for (int k = 0; k < st.len; k++) {
            int px = cx + st.pts[k][0];
            int py = cy + st.pts[k][1];
            if (px >= 0 && px < MATRIX_COLS && py >= 0 && py < MATRIX_ROWS)
                matrix.drawPixel(px, DRAW_Y(py), col16);
        }
    }

    // Draw ship on top (always visible)
    int sx = (int)roundf(_astShip.cx);
    int sy = (int)roundf(_astShip.cy);
    uint8_t h = _astShip.heading;
    for (int p = 0; p < 3; p++) {
        int px = sx + _astShipPx[h][p][0];
        int py = sy + _astShipPx[h][p][1];
        if (px < 0 || px >= MATRIX_COLS || py < 0 || py >= MATRIX_ROWS) continue;
        // Nose = pure white, rear pixels = dim white
        uint8_t bri = (p == 0) ? 255 : 160;
        matrix.drawPixel(px, DRAW_Y(py), matrix.Color(bri, bri, bri));
    }
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
#elif ACTIVE_MODE == MODE_RAIN
    memset(_rainStreaks, 0, sizeof(_rainStreaks));
    memset(_rainGhost,  0, sizeof(_rainGhost));
    memset(_rainSnap,   0, sizeof(_rainSnap));
    _rainFlashFrames = 0;
#elif ACTIVE_MODE == MODE_STARFIELD
    for (int i = 0; i < STAR_COUNT; i++) _sfSpawnStar(_sfStars[i]);
    _sfAccelBoost = 0.0f;
#elif ACTIVE_MODE == MODE_DUNE
    _duneInit();
#elif ACTIVE_MODE == MODE_GEOMETRIC
    _geoInit();
#elif ACTIVE_MODE == MODE_ASTEROIDS
    _astInit();
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
#elif ACTIVE_MODE == MODE_RAIN
    renderRain();
#elif ACTIVE_MODE == MODE_STARFIELD
    renderStarfield();
#elif ACTIVE_MODE == MODE_DUNE
    renderDune();
#elif ACTIVE_MODE == MODE_GEOMETRIC
    renderGeometric();
#elif ACTIVE_MODE == MODE_ASTEROIDS
    renderAsteroids();
#endif

    matrix.show();
}