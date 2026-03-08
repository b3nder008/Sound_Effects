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
// MODE_RAINBOW / MODE_RAINBOW_GLITTER / MODE_HUE_CYCLE
// torchv2.ino rainbow(), rainbowWithGlitter(), hueCycle() (exact ports)
// Beat modulation: extra glitter burst on beat; hue jump for hue cycle.
// ═════════════════════════════════════════════════════════════════════════════

static CRGB    _simLeds[NUM_LEDS];
static uint8_t _simGHue = 0;

static void renderSimple() {
    EVERY_N_MILLISECONDS(20) { _simGHue++; }
    if (beatFired) _simGHue += 16;

    if (_runtimeMode == MODE_RAINBOW) {
        fill_rainbow(_simLeds, NUM_LEDS, _simGHue, 1);
    } else if (_runtimeMode == MODE_RAINBOW_GLITTER) {
        fill_rainbow(_simLeds, NUM_LEDS, _simGHue, 1);
        if (random8() < 80 || beatFired)
            _simLeds[random16(NUM_LEDS)] += CRGB(CRGB::White);
        if (beatFired)
            for (int i = 0; i < 5; i++)
                _simLeds[random16(NUM_LEDS)] += CRGB(CRGB::White);
    } else { // MODE_HUE_CYCLE
        uint8_t bri = beatFired ? 255 : 200;
        fill_solid(_simLeds, NUM_LEDS, hsv2rgb_rainbow(CHSV(_simGHue, 255, bri)));
    }

    matrix.setBrightness(BRIGHTNESS);
    flushLeds(_simLeds);
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
//   Each outline is a bitmask (one uint8_t per row, bit7=col0).
//   The board cycles with a hard cut; inside pixels are rendered as dark/mid
//   green PCB substrate with a faint grid of brighter green traces.
//
//   Routes are Manhattan paths traced inside the board mask, always starting
//   from a top-edge pixel and walking downward (occasionally side-stepping).
//   Between 6 and 10 routes are generated per board. After PCBA_ROUTE_CYCLE_MS
//   all routes are re-randomised. Routes regenerate automatically on board
//   change too.
//
//   Sparks — pool of PCBA_MAX_SPARKS structs. Each spark rides one route,
//   advancing at PCBA_SPARK_SPEED pixels/frame. A burst scheduler fires
//   bursts of 1–3 sparks per route every PCBA_BURST_GAP frames. The head
//   pixel is bright gold; a PCBA_TAIL_LEN-pixel ghost tail decays
//   exponentially from gold → amber → dark brown.
//
//   Beat: brief speed surge on all active sparks.
// =============================================================================

// ── Tuning ────────────────────────────────────────────────────────────────────
#define PCBA_MAX_SPARKS        18
#define PCBA_TAIL_LEN           6    // pixels of ghost tail behind head
#define PCBA_SPARK_SPEED      0.45f  // base pixels/frame (fast)
#define PCBA_BURST_GAP_MIN     18    // min frames between bursts on one route
#define PCBA_BURST_GAP_MAX     45    // max frames between bursts on one route
#define PCBA_BOARD_CYCLE_MS  14000UL // ms between board shape changes
#define PCBA_ROUTE_CYCLE_MS  60000UL // ms between route re-randomisation
#define PCBA_MAX_ROUTES        10    // max routes per board
#define PCBA_ROUTE_LEN_MIN      5    // minimum route length (pixels)
#define PCBA_ROUTE_LEN_MAX     14    // maximum route length
#define PCBA_BEAT_SURGE        1.8f  // speed multiplier on beat
#define PCBA_BEAT_DECAY        0.88f // surge decay per frame

// ── Colour palette ────────────────────────────────────────────────────────────
// PCB substrate: dark green
#define PCBA_COL_DARK_R   0
#define PCBA_COL_DARK_G  55
#define PCBA_COL_DARK_B  18
// PCB trace grid: mid green
#define PCBA_COL_MID_R    0
#define PCBA_COL_MID_G   90
#define PCBA_COL_MID_B   28
// Board edge highlight: slightly brighter
#define PCBA_COL_EDGE_R   8
#define PCBA_COL_EDGE_G  110
#define PCBA_COL_EDGE_B   35
// Spark head: bright gold
#define PCBA_HEAD_R      255
#define PCBA_HEAD_G      195
#define PCBA_HEAD_B       10
// Tail tip: dark amber
#define PCBA_TAIL_R       60
#define PCBA_TAIL_G       28
#define PCBA_TAIL_B        0

// ── Board outline bitmasks ────────────────────────────────────────────────────
// 6 shapes. Each row byte: bit7 = col 0 (left), bit0 = col 7 (right).
// Shapes fill 6-7 columns/rows with PCB-plausible outlines.
// Row order: row 0 = bottom, row 7 = top (matching DRAW_Y convention).

static const uint8_t _pcbaBoards[6][8] = {
    // Shape 0: full rectangle with top notch (component keepout)
    { 0b11111111,   // row 0  bottom
      0b11111111,
      0b11111111,
      0b11111111,
      0b11111111,
      0b11111111,
      0b11111111,
      0b11011011 }, // row 7  top: notches at col1 and col5

    // Shape 1: large rectangle, bottom-left corner cut
    { 0b00111111,   // row 0  bottom-left 2px cut
      0b01111111,
      0b11111111,
      0b11111111,
      0b11111111,
      0b11111111,
      0b11111111,
      0b11111110 }, // row 7  top: right edge notch

    // Shape 2: D-shape — left side flat, right side has bite taken out centre
    { 0b11111110,
      0b11111111,
      0b11111111,
      0b11110111,   // centre-right pixel missing
      0b11111111,
      0b11111111,
      0b11111110,
      0b11111100 },

    // Shape 3: L-shape — upper-right quadrant absent
    { 0b11111111,
      0b11111111,
      0b11111111,
      0b11111111,
      0b11110000,   // top half: right half missing
      0b11110000,
      0b11110000,
      0b11110000 },

    // Shape 4: castellated edges top + bottom (mounting pads)
    { 0b10101010,   // bottom castellations
      0b11111111,
      0b11111111,
      0b11111111,
      0b11111111,
      0b11111111,
      0b11111111,
      0b10101010 }, // top castellations

    // Shape 5: plus/cross shape — corners removed
    { 0b00111100,
      0b01111110,
      0b11111111,
      0b11111111,
      0b11111111,
      0b11111111,
      0b01111110,
      0b00111100 },
};

static inline bool _pcbaInBoard(int x, int y, uint8_t shapeIdx) {
    if (x < 0 || x > 7 || y < 0 || y > 7) return false;
    return (_pcbaBoards[shapeIdx][y] >> (7 - x)) & 1;
}

// ── Route storage ─────────────────────────────────────────────────────────────
struct PcbaRoute {
    int8_t  px[PCBA_ROUTE_LEN_MAX]; // x coords
    int8_t  py[PCBA_ROUTE_LEN_MAX]; // y coords
    uint8_t len;                     // actual length
    uint8_t burstTimer;              // frames until next burst launch
    uint8_t burstRemain;             // sparks left to launch in current burst
    uint8_t burstSpacing;            // frames between burst members
    uint8_t burstSpacingTimer;
};

static PcbaRoute _pcbaRoutes[PCBA_MAX_ROUTES];
static uint8_t   _pcbaNumRoutes = 0;
static uint8_t   _pcbaShape     = 0;

// ── Spark storage ─────────────────────────────────────────────────────────────
struct PcbaSpark {
    float   pos;         // position along route (0..route.len-1)
    uint8_t routeIdx;
    float   speed;
    float   speedBoost;
    bool    active;
};

static PcbaSpark _pcbaSparks[PCBA_MAX_SPARKS];

// ── Timing ────────────────────────────────────────────────────────────────────
static uint32_t _pcbaBoardAt   = 0;  // millis of last board change
static uint32_t _pcbaRouteAt   = 0;  // millis of last route regen
static uint8_t  _pcbaFrameNo   = 0;  // wrapping frame counter

// ── Helpers ───────────────────────────────────────────────────────────────────
static inline uint8_t _pcbaRand8(uint8_t lo, uint8_t hi) {
    if (hi <= lo) return lo;
    return lo + random8(hi - lo + 1);
}

// ── Route generator ───────────────────────────────────────────────────────────
// Walks Manhattan-style inside the board mask.
// Always starts at a top-edge pixel (row 7) and tries to move downward.
// Direction bias: 60% down, 20% left, 20% right. Never moves up.
// Aborts (keeps partial path) if stuck for 4 consecutive steps.

static void _pcbaGenRoutes(uint8_t shape) {
    _pcbaNumRoutes = 0;
    memset(_pcbaRoutes, 0, sizeof(_pcbaRoutes));

    // Collect all top-edge pixels as potential starts
    int8_t topPx[8]; uint8_t nTop = 0;
    for (int x = 0; x < 8; x++)
        if (_pcbaInBoard(x, 7, shape)) topPx[nTop++] = x;
    if (nTop == 0) return;

    uint8_t target = _pcbaRand8(6, 10); // how many routes to generate
    uint8_t attempts = 0;

    while (_pcbaNumRoutes < target && _pcbaNumRoutes < PCBA_MAX_ROUTES
           && attempts < 40) {
        attempts++;

        // Pick a random top-edge start
        int8_t sx = topPx[random8(nTop)];
        int8_t sy = 7; // top row

        PcbaRoute rt;
        rt.len = 0;
        rt.px[rt.len] = sx;
        rt.py[rt.len] = sy;
        rt.len = 1;

        int8_t cx = sx, cy = sy;
        uint8_t stuck = 0;

        while (rt.len < PCBA_ROUTE_LEN_MAX && stuck < 4) {
            // Direction weights: down(2) left(1) right(1) — never up
            uint8_t roll = random8(4);
            int8_t ndx, ndy;
            if      (roll <= 1) { ndx =  0; ndy = -1; } // down (in board coords)
            else if (roll == 2) { ndx = -1; ndy =  0; } // left
            else                { ndx =  1; ndy =  0; } // right

            int8_t nx = cx + ndx, ny = cy + ndy;
            if (!_pcbaInBoard(nx, ny, shape)) { stuck++; continue; }

            // Avoid revisiting very recent pixels (last 3)
            bool revisit = false;
            for (int k = (int)rt.len - 1; k >= (int)rt.len - 3 && k >= 0; k--)
                if (rt.px[k] == nx && rt.py[k] == ny) { revisit = true; break; }
            if (revisit) { stuck++; continue; }

            stuck = 0;
            cx = nx; cy = ny;
            rt.px[rt.len] = cx;
            rt.py[rt.len] = cy;
            rt.len++;
        }

        if (rt.len < PCBA_ROUTE_LEN_MIN) continue;

        rt.burstTimer        = _pcbaRand8(PCBA_BURST_GAP_MIN, PCBA_BURST_GAP_MAX);
        rt.burstRemain       = 0;
        rt.burstSpacing      = 5;
        rt.burstSpacingTimer = 0;
        _pcbaRoutes[_pcbaNumRoutes++] = rt;
    }
}

// ── Spark launcher ────────────────────────────────────────────────────────────
static void _pcbaLaunchSpark(uint8_t routeIdx) {
    for (int i = 0; i < PCBA_MAX_SPARKS; i++) {
        if (_pcbaSparks[i].active) continue;
        _pcbaSparks[i].active     = true;
        _pcbaSparks[i].routeIdx   = routeIdx;
        _pcbaSparks[i].pos        = 0.0f;
        _pcbaSparks[i].speed      = PCBA_SPARK_SPEED
                                    * (0.85f + (float)random8(30) / 100.0f);
        _pcbaSparks[i].speedBoost = 1.0f;
        return;
    }
}

// ── Init ──────────────────────────────────────────────────────────────────────
static void _pcbaInit() {
    _pcbaShape   = 0;
    _pcbaBoardAt = millis();
    _pcbaRouteAt = millis();
    _pcbaFrameNo = 0;
    memset(_pcbaSparks, 0, sizeof(_pcbaSparks));
    _pcbaGenRoutes(_pcbaShape);
}

// ── Render ────────────────────────────────────────────────────────────────────
static void renderPcba() {
    uint32_t now = millis();
    _pcbaFrameNo++;

    // ── Board shape cycle ─────────────────────────────────────────────────────
    if (now - _pcbaBoardAt >= PCBA_BOARD_CYCLE_MS) {
        _pcbaBoardAt = now;
        _pcbaShape   = (_pcbaShape + 1) % 6;
        memset(_pcbaSparks, 0, sizeof(_pcbaSparks));
        _pcbaGenRoutes(_pcbaShape);
        _pcbaRouteAt = now;
    }

    // ── Route re-randomisation ────────────────────────────────────────────────
    if (now - _pcbaRouteAt >= PCBA_ROUTE_CYCLE_MS) {
        _pcbaRouteAt = now;
        memset(_pcbaSparks, 0, sizeof(_pcbaSparks));
        _pcbaGenRoutes(_pcbaShape);
    }

    // ── Beat ──────────────────────────────────────────────────────────────────
    if (beatFired) {
        for (int i = 0; i < PCBA_MAX_SPARKS; i++)
            if (_pcbaSparks[i].active)
                _pcbaSparks[i].speedBoost = PCBA_BEAT_SURGE;
    }

    // ── Burst scheduler — per route ───────────────────────────────────────────
    for (int r = 0; r < _pcbaNumRoutes; r++) {
        PcbaRoute& rt = _pcbaRoutes[r];

        if (rt.burstRemain > 0) {
            // Mid-burst: spacing timer
            if (rt.burstSpacingTimer == 0) {
                _pcbaLaunchSpark(r);
                rt.burstRemain--;
                rt.burstSpacingTimer = rt.burstSpacing;
            } else {
                rt.burstSpacingTimer--;
            }
        } else {
            // Between bursts
            if (rt.burstTimer == 0) {
                rt.burstRemain       = _pcbaRand8(1, 3);
                rt.burstSpacing      = _pcbaRand8(4, 8);
                rt.burstSpacingTimer = 0;
                rt.burstTimer        = _pcbaRand8(PCBA_BURST_GAP_MIN,
                                                   PCBA_BURST_GAP_MAX);
            } else {
                rt.burstTimer--;
            }
        }
    }

    // ── Update sparks ─────────────────────────────────────────────────────────
    for (int i = 0; i < PCBA_MAX_SPARKS; i++) {
        PcbaSpark& s = _pcbaSparks[i];
        if (!s.active) continue;

        // Decay speed boost
        if (s.speedBoost > 1.01f) s.speedBoost *= PCBA_BEAT_DECAY;
        else s.speedBoost = 1.0f;

        s.pos += s.speed * s.speedBoost;

        // Retire when past end of route
        PcbaRoute& rt = _pcbaRoutes[s.routeIdx];
        if (s.pos >= (float)(rt.len - 1)) s.active = false;
    }

    // ── Draw board background ─────────────────────────────────────────────────
    matrix.setBrightness(BRIGHTNESS);
    matrix.fillScreen(0);

    for (int y = 0; y < MATRIX_ROWS; y++) {
        for (int x = 0; x < MATRIX_COLS; x++) {
            if (!_pcbaInBoard(x, y, _pcbaShape)) continue;

            // Detect edge pixel: any 4-neighbour outside the board
            bool edge = !_pcbaInBoard(x+1,y,_pcbaShape) ||
                        !_pcbaInBoard(x-1,y,_pcbaShape) ||
                        !_pcbaInBoard(x,y+1,_pcbaShape) ||
                        !_pcbaInBoard(x,y-1,_pcbaShape);

            uint8_t r, g, b;
            if (edge) {
                r = PCBA_COL_EDGE_R; g = PCBA_COL_EDGE_G; b = PCBA_COL_EDGE_B;
            } else {
                // Subtle grid: brighten pixels where x or y is even
                bool grid = ((x & 1) == 0) || ((y & 1) == 0);
                r = grid ? PCBA_COL_MID_R  : PCBA_COL_DARK_R;
                g = grid ? PCBA_COL_MID_G  : PCBA_COL_DARK_G;
                b = grid ? PCBA_COL_MID_B  : PCBA_COL_DARK_B;
            }
            matrix.drawPixel(x, DRAW_Y(y), matrix.Color(r, g, b));
        }
    }

    // ── Draw route highlights (dim trace lines) ───────────────────────────────
    // Render a very subtle brightening along each route so the paths are
    // barely visible as PCB traces even when no spark is on them.
    for (int r = 0; r < _pcbaNumRoutes; r++) {
        PcbaRoute& rt = _pcbaRoutes[r];
        for (int k = 0; k < rt.len; k++) {
            int x = rt.px[k], y = rt.py[k];
            if (x < 0 || x >= MATRIX_COLS || y < 0 || y >= MATRIX_ROWS) continue;
            // Brighten trace pixel slightly above substrate
            matrix.drawPixel(x, DRAW_Y(y), matrix.Color(0, 105, 32));
        }
    }

    // ── Draw sparks (tail first, head last) ───────────────────────────────────
    // Pre-build a frame buffer so multiple sparks on the same pixel
    // take the brightest value (additive-ish).
    static uint8_t _pcbaFbR[8][8], _pcbaFbG[8][8], _pcbaFbB[8][8];
    memset(_pcbaFbR, 0, sizeof(_pcbaFbR));
    memset(_pcbaFbG, 0, sizeof(_pcbaFbG));
    memset(_pcbaFbB, 0, sizeof(_pcbaFbB));

    for (int i = 0; i < PCBA_MAX_SPARKS; i++) {
        PcbaSpark& s = _pcbaSparks[i];
        if (!s.active) continue;
        PcbaRoute& rt = _pcbaRoutes[s.routeIdx];

        // Draw head + tail
        for (int t = 0; t <= PCBA_TAIL_LEN; t++) {
            float tp = s.pos - (float)t;
            if (tp < 0.0f) break;

            // Interpolate between route waypoints
            int   wi  = (int)tp;
            float frac = tp - (float)wi;
            if (wi >= rt.len - 1) { wi = rt.len - 2; frac = 1.0f; }
            wi = (wi < 0) ? 0 : wi;

            float fx = (float)rt.px[wi] + frac * (float)(rt.px[wi+1] - rt.px[wi]);
            float fy = (float)rt.py[wi] + frac * (float)(rt.py[wi+1] - rt.py[wi]);
            int   px = (int)roundf(fx);
            int   py = (int)roundf(fy);
            if (px < 0 || px >= MATRIX_COLS || py < 0 || py >= MATRIX_ROWS) continue;

            // Brightness: head=255, tail decays exponentially
            float decay = (t == 0) ? 1.0f
                        : 1.0f / (1.0f + (float)t * (float)t * 0.38f);

            // Colour: head=bright gold, tail fades to amber→dark brown
            float headBlend = decay; // 1.0 at head, 0 at tip
            uint8_t pr = (uint8_t)(headBlend * PCBA_HEAD_R + (1.0f-headBlend) * PCBA_TAIL_R);
            uint8_t pg = (uint8_t)(headBlend * PCBA_HEAD_G + (1.0f-headBlend) * PCBA_TAIL_G);
            uint8_t pb = (uint8_t)(headBlend * PCBA_HEAD_B + (1.0f-headBlend) * PCBA_TAIL_B);
            pr = (uint8_t)((float)pr * decay);
            pg = (uint8_t)((float)pg * decay);
            pb = (uint8_t)((float)pb * decay);

            // Accumulate — take max per channel so sparks don't cancel
            if (pr > _pcbaFbR[px][py]) _pcbaFbR[px][py] = pr;
            if (pg > _pcbaFbG[px][py]) _pcbaFbG[px][py] = pg;
            if (pb > _pcbaFbB[px][py]) _pcbaFbB[px][py] = pb;
        }
    }

    // Composite spark buffer onto matrix (skip zero pixels — board shows through)
    for (int y = 0; y < MATRIX_ROWS; y++)
        for (int x = 0; x < MATRIX_COLS; x++)
            if (_pcbaFbR[x][y] || _pcbaFbG[x][y] || _pcbaFbB[x][y])
                matrix.drawPixel(x, DRAW_Y(y),
                    matrix.Color(_pcbaFbR[x][y], _pcbaFbG[x][y], _pcbaFbB[x][y]));
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

// ═════════════════════════════════════════════════════════════════════════════
// Init + Update — always compiled
// ═════════════════════════════════════════════════════════════════════════════

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
        case MODE_RAINBOW:
        case MODE_RAINBOW_GLITTER:
        case MODE_HUE_CYCLE:            renderSimple();     break;
        case MODE_CLOUD_TWINKLES:
        case MODE_RAINBOW_TWINKLES:     renderTwinkles();   break;
        case MODE_RAIN:                 renderRain();       break;
        case MODE_STARFIELD:            renderStarfield();  break;
        case MODE_DUNE:                 renderDune();       break;
        case MODE_GEOMETRIC:            renderGeometric();  break;
        case MODE_WISP:                 renderWisp();       break;
        case MODE_PCBA:                 renderPcba();       break;
        default:                        renderSpectrum();   break;
    }

    matrix.show();
}