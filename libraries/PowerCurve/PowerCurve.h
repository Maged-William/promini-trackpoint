#ifndef POWERCURVE_H
#define POWERCURVE_H

#include <Arduino.h>

/*
 * PowerCurve — RawAccel-style "Power" velocity curve for the Pro Mini
 * trackpoint emulator (Exp60).
 *
 *   v     = |(x,y)| = sqrt(x*x + y*y)              (whole mode, L2 magnitude)
 *   f(v)  = start + (rate * v)^exponent            (Power sensitivity function)
 *   out   = (x * sens, y * sens) * f(v)            (applied to the whole vector)
 *
 * A Q8.8 gain LUT  lut[v] = sens * f(v) * 256  is built once at begin() and
 * rebuilt via update() when a param write marks it dirty (float pow() only
 * ever runs in the main loop — never in the I2C/ISR or sample path).
 * Per 50 Hz sample the class does a magnitude lookup, one fixed-point multiply
 * with fractional-remainder accumulation, and an int8 clamp.
 *
 * Register protocol shared with the ZMK driver (2-byte little-endian params):
 *   0x11 = sens (1 byte)
 *   0x13 = rate      Q8.8
 *   0x15 = exponent  Q8.8
 *   0x17 = start     Q8.8
 */
class PowerCurve {
public:
    PowerCurve();

    /* Registers wired to setParam(). */
    static const uint8_t REG_SENS  = 0x11;
    static const uint8_t REG_RATE  = 0x13;
    static const uint8_t REG_EXP   = 0x15;
    static const uint8_t REG_START = 0x17;

    /* Zero remainders and reset to safe defaults (sens=255, rate=0 -> identity
     * curve so an unconfigured build behaves like plain speed_scale). */
    void begin();

    /* Sens multiplier (Q8.8, 255 = 1.0). Stores + marks LUT dirty. */
    void setSens(uint8_t sens);

    /* Set one curve param by register address. 0x13/0x15/0x17 are Q8.8 (2-byte
     * little-endian); 0x11 is 1-byte sens. Only stores + marks the LUT dirty —
     * call update() from the main loop to rebuild (keeps float pow() out of
     * the I2C interrupt context). */
    void setParam(uint8_t reg, uint16_t value);

    /* Apply the curve to one PS/2 sample. Outputs are clamped to int8. */
    void apply(int8_t x, int8_t y, int8_t &out_x, int8_t &out_y);

    /* Rebuild the LUT if any param changed since the last call (cheap no-op
     * when clean). Call once per main loop iteration. */
    void update();

private:
    void rebuildLut();
    static uint16_t isqrt16(uint16_t n);

    static const uint8_t  LUT_SIZE   = 64;
    static const uint16_t CLAMP_MAX  = 2048; /* Q8.8: max out gain 8.0 */
    static const int      OUT_MAX    = 127;

    uint8_t  _sens;      /* Q8.8                                     */
    uint16_t _rate;      /* Q8.8                                     */
    uint16_t _exp;       /* Q8.8                                     */
    uint16_t _start;     /* Q8.8                                     */
    uint16_t _lut[LUT_SIZE]; /* Q8.8 gain for each speed bin         */
    int16_t  _rem_x;
    int16_t  _rem_y;
    uint8_t  _dirty;         /* LUT needs rebuild (set in ISR context) */
};

#endif