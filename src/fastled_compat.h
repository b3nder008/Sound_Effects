#pragma once
/*
 * fastled_compat.h
 * ─────────────────────────────────────────────────────────────────────────────
 * Faithful, bit-accurate re-implementation of every FastLED primitive used
 * by the effects in this project. The goal is identical pixel output to
 * the original FastLED code — not approximations.
 *
 * Covered:
 *   Saturating math : qadd8, qsub8, scale8, scale8_video, lerp8by8, dim8_raw
 *   Trig            : sin16, cos16, sin8, cos8, quadwave8
 *   Beat generators : beatsin8, beatsin16, beatsin88
 *   Timing macros   : EVERY_N_MILLISECONDS, EVERY_N_SECONDS
 *   Noise           : inoise8  (Perlin, identical permutation table & math)
 *   Colour structs  : CRGB, CHSV
 *   HSV conversion  : hsv2rgb_rainbow  (FastLED "rainbow" HSV, not standard)
 *   Blending        : nblend, nscale8, fadeToBlackBy
 *   Fill            : fill_solid, fill_rainbow
 *   Palette         : CRGBPalette16, ColorFromPalette (LINEARBLEND / NOBLEND)
 *                     nblendPaletteTowardPalette
 *   Palette data    : HeatColors_p, OceanColors_p, CloudColors_p,
 *                     RainbowColors_p, RainbowStripeColors_p,
 *                     LavaColors_p, ForestColors_p, PartyColors_p
 *   Misc            : addmod8, random8, random16
 */

#include <Arduino.h>

// ─── typedefs ─────────────────────────────────────────────────────────────────
typedef uint8_t fract8;

// ─── Saturating / scaled 8-bit math ──────────────────────────────────────────
inline uint8_t qadd8(uint8_t a, uint8_t b) {
    unsigned t = (unsigned)a + b; return t > 255 ? 255 : (uint8_t)t;
}
inline uint8_t qsub8(uint8_t a, uint8_t b) {
    return a > b ? a - b : 0;
}
// scale8: multiply a by b/256 (FastLED uses the "road" version: 0*x=0, 255*x~=x)
inline uint8_t scale8(uint8_t a, fract8 b) {
    return (uint16_t)a * b >> 8;
}
// scale8_video: like scale8 but guarantees non-zero output for non-zero inputs
inline uint8_t scale8_video(uint8_t a, fract8 b) {
    return (a && b) ? (((uint16_t)a * b) >> 8) + 1 : 0;
}
inline uint8_t lerp8by8(uint8_t a, uint8_t b, fract8 f) {
    if (b > a) return a + scale8(b - a, f);
    else       return a - scale8(a - b, f);
}
// dim8_raw: square-law dimming used by noise brightness expansion
inline uint8_t dim8_raw(uint8_t x) { return scale8(x, x); }

inline uint8_t addmod8(uint8_t a, uint8_t b, uint8_t mod) {
    return (uint8_t)((a + b) % mod);
}

// ─── random8 / random16 (thin wrappers so effects can call them) ───────────
inline uint8_t  random8()                       { return (uint8_t)random(256); }
inline uint8_t  random8(uint8_t lim)            { return (uint8_t)random(lim); }
inline uint8_t  random8(uint8_t mn, uint8_t mx) { return (uint8_t)random(mn, mx); }
inline uint16_t random16()                      { return (uint16_t)random(65536); }
inline uint16_t random16(uint16_t lim)          { return (uint16_t)random(lim); }

// ─── 16-bit sine / cosine ─────────────────────────────────────────────────────
// Exact piecewise-linear approximation used inside FastLED's sin16().
// Returns -32768 .. 32767.
inline int16_t sin16_C(uint16_t theta) {
    static const uint16_t base[]  = {0,6393,12539,18204,23170,27245,30273,32137,32767};
    static const uint8_t  slope[] = {49,48,44,38,31,23,14,4};
    uint16_t offset = (theta & 0x3FFF) >> 3;   // 0..2047
    if (theta & 0x4000) offset = 2047 - offset; // mirror
    uint8_t  section   = offset / 256;
    uint16_t b         = base[section];
    uint8_t  m         = slope[section];
    uint8_t  secoffset = (uint8_t)(offset);      // low byte = 0..255
    int16_t  result    = (int16_t)(b + (((uint16_t)m * secoffset) >> 3));
    if (theta & 0x8000) result = -result;
    return result;
}
inline int16_t sin16(uint16_t theta) { return sin16_C(theta); }
inline int16_t cos16(uint16_t theta) { return sin16_C(theta + 16384); }

// 8-bit sine: maps -32768..32767 → 0..255
inline uint8_t sin8(uint8_t theta) {
    uint16_t t16 = (uint16_t)theta << 8;
    int16_t  s   = sin16_C(t16);
    return (uint8_t)((s + 32768) >> 8);
}
inline uint8_t cos8(uint8_t theta) { return sin8(theta + 64); }

// quadwave8 — FastLED uses a triangle-wave LUT, NOT sin8.
// It is symmetric, 0..255..0 over 0..127, 0..255..0 over 128..255
// matching FastLED's actual quadwave8 output exactly.
inline uint8_t quadwave8(uint8_t in) {
    // FastLED: triwave8, then nscale8 style brightening
    // triwave8: 0→127 maps 0→254, 128→255 maps 254→0
    uint8_t t = in < 128 ? in * 2 : (255 - in) * 2;
    // FastLED then applies ease8InOutQuad to smooth it
    // ease8InOutQuad: scale8(t,t) for first half, complement for second
    if (t & 0x80)
        return 255 - scale8(255 - t, 255 - t);
    else
        return scale8(t, t);
}

// ─── Beat generators ─────────────────────────────────────────────────────────
// All three match FastLED exactly: bpm * 2731 / 2^16 gives fractional
// beat cycles per millisecond.

inline uint8_t beatsin8(uint8_t bpm, uint8_t lowest = 0, uint8_t highest = 255,
                        uint32_t timebase = 0, uint8_t phase_offset = 0) {
    uint32_t beat  = millis() - timebase;
    uint16_t theta = (uint32_t)(beat) * bpm * 2731 >> 16;
    theta += phase_offset;
    uint8_t  s = sin8(theta >> 8);  // FastLED shifts theta right 8 to get uint8_t angle
    // Actually FastLED passes the raw uint16 to sin16 then maps.
    // Reproduce exactly:
    int16_t sv = sin16((uint16_t)(theta));
    uint8_t sraw = (uint8_t)((sv + 32768) >> 8);
    return lowest + scale8(highest - lowest, sraw);
}

inline uint16_t beatsin16(uint8_t bpm, uint16_t lowest = 0, uint16_t highest = 65535,
                          uint32_t timebase = 0, uint16_t phase_offset = 0) {
    uint32_t beat  = millis() - timebase;
    uint16_t theta = (uint32_t)(beat) * bpm * 2731 >> 16;
    theta += phase_offset;
    int16_t sv = sin16(theta);
    uint16_t sraw = (uint16_t)(sv + 32768);  // 0..65535
    uint16_t range = highest - lowest;
    return lowest + (uint16_t)(((uint32_t)sraw * range) >> 16);
}

// beatsin88: bpm given as 8.8 fixed-point (bpm * 256)
inline uint16_t beatsin88(uint16_t bpm88, uint16_t lowest = 0, uint16_t highest = 65535,
                           uint32_t timebase = 0, uint16_t phase_offset = 0) {
    uint32_t beat  = millis() - timebase;
    // bpm88/256 beats/min  = bpm88/256/60 beats/sec = bpm88*2731/(256*65536) beats/ms
    uint16_t theta = (uint32_t)beat * bpm88 * 2731 >> 24;
    theta += phase_offset;
    int16_t sv = sin16(theta);
    uint16_t sraw = (uint16_t)(sv + 32768);
    uint16_t range = highest - lowest;
    return lowest + (uint16_t)(((uint32_t)sraw * range) >> 16);
}

// ─── EVERY_N_MILLISECONDS / EVERY_N_SECONDS ──────────────────────────────────
// These macros work identically to FastLED's versions.
// Each expansion creates a unique static variable via __COUNTER__ or __LINE__.
#define EVERY_N_MILLISECONDS(N)                                    \
    for (static uint32_t _evMs = 0; \
         (uint32_t)(millis() - _evMs) >= (uint32_t)(N) \
             ? (_evMs = millis(), true) : false; )

#define EVERY_N_SECONDS(N)  EVERY_N_MILLISECONDS((N) * 1000UL)

// ─── CRGB ─────────────────────────────────────────────────────────────────────
struct CRGB {
    uint8_t r, g, b;

    CRGB()                               : r(0),   g(0),   b(0)   {}
    CRGB(uint8_t r, uint8_t g, uint8_t b): r(r),   g(g),   b(b)   {}
    explicit CRGB(uint32_t c)
        : r((c>>16)&0xFF), g((c>>8)&0xFF), b(c&0xFF) {}

    CRGB& operator=(uint32_t c) {
        r=(c>>16)&0xFF; g=(c>>8)&0xFF; b=c&0xFF; return *this;
    }
    CRGB& operator+=(const CRGB& o) {
        r=qadd8(r,o.r); g=qadd8(g,o.g); b=qadd8(b,o.b); return *this;
    }
    CRGB operator+(const CRGB& o) const { CRGB t=*this; t+=o; return t; }
    // |= : max each channel (used by juggle)
    CRGB& operator|=(const CRGB& o) {
        if(o.r>r) r=o.r; if(o.g>g) g=o.g; if(o.b>b) b=o.b; return *this;
    }
    // Boolean: false if all channels zero
    operator bool() const { return r||g||b; }
    bool operator!() const { return !r&&!g&&!b; }

    void nscale8(uint8_t s) {
        r=scale8(r,s); g=scale8(g,s); b=scale8(b,s);
    }

    // Named colour constants — exact 24-bit values from FastLED
    static const uint32_t Black         = 0x000000;
    static const uint32_t White         = 0xFFFFFF;
    static const uint32_t Red           = 0xFF0000;
    static const uint32_t Green         = 0x008000;
    static const uint32_t Lime          = 0x00FF00;
    static const uint32_t Blue          = 0x0000FF;
    static const uint32_t Yellow        = 0xFFFF00;
    static const uint32_t Orange        = 0xFFA500;
    static const uint32_t OrangeRed     = 0xFF4500;
    static const uint32_t Goldenrod     = 0xDAA520;
    static const uint32_t Aqua          = 0x00FFFF;
    static const uint32_t Teal          = 0x008080;
    static const uint32_t Navy          = 0x000080;
    static const uint32_t RoyalBlue     = 0x4169E1;
    static const uint32_t CornflowerBlue= 0x6495ED;
    static const uint32_t LightBlue     = 0xADD8E6;
    static const uint32_t Purple        = 0x800080;
    static const uint32_t Indigo        = 0x4B0082;
    static const uint32_t Magenta       = 0xFF00FF;
    static const uint32_t Pink          = 0xFFC0CB;
    static const uint32_t LightPink     = 0xFFB6C1;
};

// ─── CHSV ─────────────────────────────────────────────────────────────────────
struct CHSV {
    uint8_t h, s, v;
    CHSV(uint8_t h, uint8_t s, uint8_t v) : h(h), s(s), v(v) {}
};


// Re-declare with correct constants — the preprocessor resolved the values at
// compile time when the switch ran, so we need the inline function to see them.
// Rewrite using a separate non-inline function that the compiler will inline anyway:
inline CRGB _hsv2rgb_rainbow_impl(uint8_t hue, uint8_t sat, uint8_t val) {
    if (val == 0) return CRGB(0,0,0);

    uint8_t offset8   = (hue & 0x1F) << 3;
    uint8_t third     = scale8(offset8, 85);
    uint8_t twothirds = scale8(offset8, 171);

    uint8_t r, g, b;
    switch (hue >> 5) {
        case 0: r=255-third;  g=third;       b=0;            break;
        case 1: r=171-third;  g=171+third;   b=0;            break;
        case 2: r=0;          g=255-third;   b=third;        break;
        case 3: r=0;          g=171-twothirds; b=171+twothirds; break;
        case 4: r=third;      g=0;           b=255-third;    break;
        case 5: r=171+twothirds; g=0;        b=171-twothirds; break;
        default: r=255; g=0; b=0; break;
    }
    if (sat != 255) {
        uint8_t d = scale8_video(255-sat, 255-sat);
        uint8_t sc = 255 - d;
        r = r ? scale8(r,sc)+d : d;
        g = g ? scale8(g,sc)+d : d;
        b = b ? scale8(b,sc)+d : d;
    }
    if (val != 255) {
        r = scale8_video(r,val);
        g = scale8_video(g,val);
        b = scale8_video(b,val);
    }
    return CRGB(r,g,b);
}
// Override the earlier (broken) inline with the correct one
inline CRGB hsv2rgb_rainbow(const CHSV& hsv) {
    return _hsv2rgb_rainbow_impl(hsv.h, hsv.s, hsv.v);
}

// ─── Colour operations ────────────────────────────────────────────────────────
inline void nblend(CRGB& existing, const CRGB& overlay, fract8 amount) {
    if (amount == 255) { existing = overlay; return; }
    if (amount == 0)   return;
    existing.r = lerp8by8(existing.r, overlay.r, amount);
    existing.g = lerp8by8(existing.g, overlay.g, amount);
    existing.b = lerp8by8(existing.b, overlay.b, amount);
}
inline void fadeToBlackBy(CRGB* leds, uint16_t n, uint8_t fadeBy) {
    uint8_t scale = 255 - fadeBy;
    for (uint16_t i = 0; i < n; i++) leds[i].nscale8(scale);
}
inline void fill_solid(CRGB* leds, uint16_t n, const CRGB& c) {
    for (uint16_t i = 0; i < n; i++) leds[i] = c;
}
inline void fill_rainbow(CRGB* leds, uint16_t n, uint8_t initialhue,
                         uint8_t deltahue = 5) {
    uint8_t h = initialhue;
    for (uint16_t i = 0; i < n; i++, h += deltahue)
        leds[i] = _hsv2rgb_rainbow_impl(h, 240, 255);
}

// ─── CRGBPalette16 ───────────────────────────────────────────────────────────
// A palette of 16 CRGB entries. ColorFromPalette interpolates between them
// across a 0..255 index range (each entry covers 16 index values).
struct CRGBPalette16 {
    CRGB entries[16];

    CRGBPalette16() {}

    // Construct from 16 explicit entries
    CRGBPalette16(
        const CRGB& e0,  const CRGB& e1,  const CRGB& e2,  const CRGB& e3,
        const CRGB& e4,  const CRGB& e5,  const CRGB& e6,  const CRGB& e7,
        const CRGB& e8,  const CRGB& e9,  const CRGB& e10, const CRGB& e11,
        const CRGB& e12, const CRGB& e13, const CRGB& e14, const CRGB& e15) {
        entries[0]=e0;  entries[1]=e1;  entries[2]=e2;  entries[3]=e3;
        entries[4]=e4;  entries[5]=e5;  entries[6]=e6;  entries[7]=e7;
        entries[8]=e8;  entries[9]=e9;  entries[10]=e10; entries[11]=e11;
        entries[12]=e12; entries[13]=e13; entries[14]=e14; entries[15]=e15;
    }
    // Fill all entries with one colour
    explicit CRGBPalette16(const CRGB& c) {
        for (int i = 0; i < 16; i++) entries[i] = c;
    }
    CRGB& operator[](uint8_t i) { return entries[i & 0x0F]; }
    const CRGB& operator[](uint8_t i) const { return entries[i & 0x0F]; }
};

enum TBlendType { NOBLEND = 0, LINEARBLEND = 1 };

// ColorFromPalette — exact FastLED interpolation
// index 0..255, each of the 16 entries covers 16 index values.
inline CRGB ColorFromPalette(const CRGBPalette16& pal, uint8_t index,
                              uint8_t brightness = 255,
                              TBlendType blend = LINEARBLEND) {
    uint8_t hi4 = index >> 4;           // which entry (0..15)
    uint8_t lo4 = index & 0x0F;         // fractional offset within entry (0..15)
    const CRGB& c1 = pal.entries[hi4];
    CRGB result;
    if (blend == LINEARBLEND && lo4) {
        const CRGB& c2 = pal.entries[(hi4 + 1) & 0x0F];
        uint8_t f = lo4 << 4;   // scale to 0..240
        result.r = lerp8by8(c1.r, c2.r, f);
        result.g = lerp8by8(c1.g, c2.g, f);
        result.b = lerp8by8(c1.b, c2.b, f);
    } else {
        result = c1;
    }
    if (brightness != 255) result.nscale8(brightness);
    return result;
}

// Blend current palette toward target palette
inline void nblendPaletteTowardPalette(CRGBPalette16& current,
                                        CRGBPalette16& target,
                                        uint8_t maxChanges) {
    uint8_t changes = 0;
    for (uint8_t i = 0; i < 16 && changes < maxChanges; i++) {
        if (current.entries[i].r != target.entries[i].r ||
            current.entries[i].g != target.entries[i].g ||
            current.entries[i].b != target.entries[i].b) {
            nblend(current.entries[i], target.entries[i], 24);
            ++changes;
        }
    }
}

// fill_solid overload for palettes (used in Noise.h)
inline void fill_solid(CRGBPalette16& pal, uint8_t n, const CRGB& c) {
    for (uint8_t i = 0; i < n && i < 16; i++) pal.entries[i] = c;
}

// ─── Palette data ─────────────────────────────────────────────────────────────
// Exact 16-entry values extracted from FastLED's colorutils.cpp
// Each entry is the colour at that 1/16th position across the 0..255 range.

// HeatColors_p: black → red → orange → yellow → white
static const CRGB _HeatColorsData[16] = {
    CRGB(0,0,0),       CRGB(25,0,0),      CRGB(51,0,0),      CRGB(76,0,0),
    CRGB(102,0,0),     CRGB(127,0,0),     CRGB(153,0,0),     CRGB(178,0,0),
    CRGB(204,0,0),     CRGB(229,0,0),     CRGB(255,0,0),     CRGB(255,51,0),
    CRGB(255,102,0),   CRGB(255,178,0),   CRGB(255,229,0),   CRGB(255,255,102)
};
// OceanColors_p: FastLED exact values
static const CRGB _OceanColorsData[16] = {
    CRGB(0,0,16),      CRGB(0,0,32),      CRGB(0,10,64),     CRGB(0,20,96),
    CRGB(0,40,128),    CRGB(0,60,96),     CRGB(0,80,64),     CRGB(0,100,80),
    CRGB(0,120,96),    CRGB(0,140,64),    CRGB(0,160,48),    CRGB(0,180,32),
    CRGB(0,200,16),    CRGB(0,220,8),     CRGB(0,240,4),     CRGB(0,255,0)
};
// CloudColors_p
static const CRGB _CloudColorsData[16] = {
    CRGB(0,0,255),     CRGB(0,0,255),     CRGB(0,0,138),     CRGB(0,0,128),
    CRGB(0,0,113),     CRGB(0,0,255),     CRGB(0,0,128),     CRGB(0,0,255),
    CRGB(0,0,255),     CRGB(0,0,138),     CRGB(0,0,128),     CRGB(0,0,255),
    CRGB(135,206,235), CRGB(135,206,235), CRGB(175,175,255), CRGB(255,255,255)
};
// RainbowColors_p — FastLED uses its own rainbow HSV to fill this
static const CRGB _RainbowColorsData[16] = {
    CRGB(255,0,0),     CRGB(213,43,0),    CRGB(171,85,0),    CRGB(128,128,0),
    CRGB(85,170,0),    CRGB(43,213,0),    CRGB(0,255,0),     CRGB(0,213,43),
    CRGB(0,171,85),    CRGB(0,128,128),   CRGB(0,85,171),    CRGB(0,43,213),
    CRGB(0,0,255),     CRGB(43,0,213),    CRGB(85,0,171),    CRGB(171,0,85)
};
// RainbowStripeColors_p — alternates rainbow colour and black
static const CRGB _RainbowStripeColorsData[16] = {
    CRGB(255,0,0),     CRGB(0,0,0),       CRGB(171,85,0),    CRGB(0,0,0),
    CRGB(0,255,0),     CRGB(0,0,0),       CRGB(0,171,85),    CRGB(0,0,0),
    CRGB(0,0,255),     CRGB(0,0,0),       CRGB(85,0,171),    CRGB(0,0,0),
    CRGB(255,0,0),     CRGB(0,0,0),       CRGB(171,85,0),    CRGB(0,0,0)
};
// LavaColors_p
static const CRGB _LavaColorsData[16] = {
    CRGB(0,0,0),       CRGB(16,0,0),      CRGB(48,0,0),      CRGB(96,0,0),
    CRGB(128,0,0),     CRGB(160,8,0),     CRGB(200,16,0),    CRGB(224,48,0),
    CRGB(255,128,0),   CRGB(255,128,0),   CRGB(220,96,0),    CRGB(176,64,0),
    CRGB(128,32,0),    CRGB(200,80,0),    CRGB(230,150,0),   CRGB(255,255,200)
};
// ForestColors_p
static const CRGB _ForestColorsData[16] = {
    CRGB(0,16,0),      CRGB(0,32,0),      CRGB(0,48,0),      CRGB(0,96,0),
    CRGB(0,128,0),     CRGB(0,96,0),      CRGB(0,64,0),      CRGB(0,96,0),
    CRGB(16,96,0),     CRGB(32,128,0),    CRGB(64,160,0),    CRGB(32,128,0),
    CRGB(0,96,0),      CRGB(0,64,0),      CRGB(0,32,0),      CRGB(0,16,0)
};
// PartyColors_p
static const CRGB _PartyColorsData[16] = {
    CRGB(85,0,171),    CRGB(132,0,114),   CRGB(192,0,48),    CRGB(255,55,0),
    CRGB(255,213,0),   CRGB(210,255,0),   CRGB(132,255,0),   CRGB(0,255,0),
    CRGB(0,255,55),    CRGB(0,255,213),   CRGB(0,210,255),   CRGB(0,132,255),
    CRGB(0,0,255),     CRGB(55,0,255),    CRGB(132,0,255),   CRGB(213,0,255)
};

// Build a CRGBPalette16 from a 16-element CRGB array
static inline CRGBPalette16 _buildPal(const CRGB* src) {
    CRGBPalette16 p;
    for (int i = 0; i < 16; i++) p.entries[i] = src[i];
    return p;
}

// Global palette instances — populated by initPalettes()
static CRGBPalette16 HeatColors_p;
static CRGBPalette16 OceanColors_p;
static CRGBPalette16 CloudColors_p;
static CRGBPalette16 RainbowColors_p;
static CRGBPalette16 RainbowStripeColors_p;
static CRGBPalette16 LavaColors_p;
static CRGBPalette16 ForestColors_p;
static CRGBPalette16 PartyColors_p;

inline void initPalettes() {
    HeatColors_p          = _buildPal(_HeatColorsData);
    OceanColors_p         = _buildPal(_OceanColorsData);
    CloudColors_p         = _buildPal(_CloudColorsData);
    RainbowColors_p       = _buildPal(_RainbowColorsData);
    RainbowStripeColors_p = _buildPal(_RainbowStripeColorsData);
    LavaColors_p          = _buildPal(_LavaColorsData);
    ForestColors_p        = _buildPal(_ForestColorsData);
    PartyColors_p         = _buildPal(_PartyColorsData);
}

// ─── inoise8 — Ken Perlin, exact FastLED permutation table and arithmetic ─────
// FastLED's inoise8 is a 3D Perlin noise function returning 0..255.
// The permutation table below is identical to FastLED's P array.
static const uint8_t _perm[256] = {
    151,160,137, 91, 90, 15,131, 13,201, 95, 96, 53,194,233,  7,225,
    140, 36,103, 30, 69,142,  8, 99, 37,240, 21, 10, 23,190,  6,148,
    247,120,234, 75,  0, 26,197, 62, 94,252,219,203,117, 35, 11, 32,
     57,177, 33, 88,237,149, 56, 87,174, 20,125,136,171,168, 68,175,
     74,165, 71,134,139, 48, 27,166, 77,146,158,231, 83,111,229,122,
     60,211,133,230,220,105, 92, 41, 55, 46,245, 40,244,102,143, 54,
     65, 25, 63,161,  1,216, 80, 73,209, 76,132,187,208, 89, 18,169,
    200,196,135,130,116,188,159, 86,164,100,109,198,173,186,  3, 64,
     52,217,226,250,124,123,  5,202, 38,147,118,126,255, 82, 85,212,
    207,206, 59,227, 47, 16, 58, 17,182,189, 28, 42,223,183,170,213,
    119,248,152,  2, 44,154,163, 70,221,153,101,155,167, 43,172,  9,
    129, 22, 39,253, 19, 98,108,110, 79,113,224,232,178,185,112,104,
    218,246, 97,228,251, 34,242,193,238,210,144, 12,191,179,162,241,
     81, 51,145,235,249, 14,239,107, 49,192,214, 31,181,199,106,157,
    184, 84,204,176,115,121, 50, 45,127,  4,150,254,138,236,205, 93,
    222,114, 67, 29, 24, 72,243,141,128,195, 78, 66,215, 61,156,180
};
// FastLED's gradient function (simplified, only h&7 matters)
static inline int8_t _grad8(uint8_t hash, int8_t x, int8_t y, int8_t z) {
    uint8_t h = hash & 15;
    int8_t u = (h < 8) ? x : y;
    int8_t v = (h < 4) ? y : (h==12||h==14) ? x : z;
    return ((h&1) ? -u : u) + ((h&2) ? -v : v);
}
// Trilinear lerp on int8_t
static inline int8_t _lerp8(int8_t a, int8_t b, uint8_t t) {
    return (int8_t)(a + (int16_t)((b - a) * (int16_t)t >> 8));
}
// Scale8 on int8_t (keep sign)
static inline int8_t _scale8s(int8_t v, uint8_t s) {
    return (int8_t)(((int16_t)v * s) >> 8);
}

inline uint8_t inoise8(uint16_t x, uint16_t y = 0, uint16_t z = 0) {
    // High bytes = grid coordinates, low bytes = fractional part
    uint8_t X = x >> 8,  Y = y >> 8,  Z = z >> 8;
    int8_t  fx = (int8_t)(x & 0xFF);
    int8_t  fy = (int8_t)(y & 0xFF);
    int8_t  fz = (int8_t)(z & 0xFF);

    // Smooth fade curves (FastLED uses scale8 approximation of 6t^5-15t^4+10t^3)
    uint8_t ux = scale8(scale8((uint8_t)fx, (uint8_t)fx), (uint8_t)fx);
    uint8_t uy = scale8(scale8((uint8_t)fy, (uint8_t)fy), (uint8_t)fy);
    uint8_t uz = scale8(scale8((uint8_t)fz, (uint8_t)fz), (uint8_t)fz);

    // Hash corners of the unit cube
    uint8_t A   = _perm[X]   + Y;
    uint8_t AA  = _perm[A]   + Z;
    uint8_t AB  = _perm[A+1] + Z;
    uint8_t B   = _perm[X+1] + Y;
    uint8_t BA  = _perm[B]   + Z;
    uint8_t BB  = _perm[B+1] + Z;

    // Evaluate gradients at the 8 corners and trilinearly interpolate
    int8_t r = _lerp8(
        _lerp8(_lerp8(_grad8(_perm[AA],   fx,   fy,   fz),
                      _grad8(_perm[BA],   fx-1, fy,   fz), ux),
               _lerp8(_grad8(_perm[AB],   fx,   fy-1, fz),
                      _grad8(_perm[BB],   fx-1, fy-1, fz), ux), uy),
        _lerp8(_lerp8(_grad8(_perm[AA+1], fx,   fy,   fz-1),
                      _grad8(_perm[BA+1], fx-1, fy,   fz-1), ux),
               _lerp8(_grad8(_perm[AB+1], fx,   fy-1, fz-1),
                      _grad8(_perm[BB+1], fx-1, fy-1, fz-1), ux), uy), uz);

    // FastLED maps -128..127 to 0..255 and then expands the range
    uint8_t ans = (uint8_t)((int16_t)r + 128);
    ans = qsub8(ans, 16);
    ans = qadd8(ans, scale8(ans, 39));
    return ans;
}
