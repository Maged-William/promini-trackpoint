#include "PowerCurve.h"

#include <math.h>

PowerCurve::PowerCurve() {
    _sens  = 255;
    _rate  = 0;
    _exp   = 256;
    _start = 256;
    _rem_x = 0;
    _rem_y = 0;
    _dirty = 0;
    rebuildLut();
}

void PowerCurve::begin() {
    _sens  = 255;
    _rate  = 0;
    _exp   = 256;
    _start = 256; /* Q8.8 1.0 — safe identity default */
    _rem_x = 0;
    _rem_y = 0;
    rebuildLut();
}

void PowerCurve::setSens(uint8_t sens) {
    _sens = sens;
    _dirty = 1;
}

void PowerCurve::setParam(uint8_t reg, uint16_t value) {
    switch (reg) {
    case REG_RATE:
        _rate = value;
        break;
    case REG_EXP:
        _exp = value;
        break;
    case REG_START:
        _start = value;
        break;
    default:
        return; /* unknown register — ignore */
    }
    _dirty = 1;
}

void PowerCurve::update() {
    if (_dirty) {
        rebuildLut();
        _dirty = 0;
    }
}

void PowerCurve::apply(int8_t x, int8_t y, int8_t &out_x, int8_t &out_y) {
    if (x == 0 && y == 0) {
        out_x = 0;
        out_y = 0;
        _rem_x = 0;
        _rem_y = 0;
        return;
    }

    uint16_t v = isqrt16((uint16_t)(x * x) + (uint16_t)(y * y));
    if (v >= LUT_SIZE) {
        v = LUT_SIZE - 1;
    }
    uint16_t g = _lut[v];

    int32_t tx = (int32_t)x * g + _rem_x;
    int32_t ty = (int32_t)y * g + _rem_y;
    int16_t sx = tx / 256;
    int16_t sy = ty / 256;
    _rem_x = tx - (int32_t)sx * 256;
    _rem_y = ty - (int32_t)sy * 256;

    if (sx > OUT_MAX) sx = OUT_MAX;
    if (sx < -OUT_MAX) sx = -OUT_MAX;
    if (sy > OUT_MAX) sy = OUT_MAX;
    if (sy < -OUT_MAX) sy = -OUT_MAX;

    out_x = (int8_t)sx;
    out_y = (int8_t)sy;
}

void PowerCurve::rebuildLut() {
    double r   = _rate  / 256.0;
    double e   = _exp   / 256.0;
    double s   = _start / 256.0;
    double sn  = _sens  / 256.0;

    for (uint16_t v = 0; v < LUT_SIZE; v++) {
        double f = s + pow(r * (double)v, e);
        double val = sn * f;
        if (val > (double)CLAMP_MAX / 256.0) {
            val = (double)CLAMP_MAX / 256.0;
        }
        _lut[v] = (uint16_t)(val * 256.0 + 0.5);
    }
}

/* Digit-by-digit integer square root (floor), no libm dependency at runtime. */
uint16_t PowerCurve::isqrt16(uint16_t n) {
    uint16_t res = 0;
    uint16_t bit = (uint16_t)1 << 14;
    while (bit > n) {
        bit >>= 2;
    }
    while (bit != 0) {
        if (n >= res + bit) {
            n -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}