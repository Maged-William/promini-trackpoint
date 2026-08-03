#ifndef PS2TRACKPOINT_H
#define PS2TRACKPOINT_H

#include <Arduino.h>
#include <stdint.h>

class PS2Trackpoint {
public:
    PS2Trackpoint(uint8_t clkPin, uint8_t datPin);

    void begin();
    bool readPacket(int8_t &x, int8_t &y, uint8_t &buttons);

    /* Exp43: bound the PS/2 idle wait so long reads don't starve the I2C slave */
    void setReadTimeout(uint16_t timeout) { _readTimeout = timeout; }

    /* Exp43 debug: raw bytes of the last successfully-parsed packet */
    uint8_t last_status;
    uint8_t last_xraw;
    uint8_t last_yraw;
    uint16_t read_timeouts; /* Exp43: count of readByte idle-timeouts (0x00/0xFF/0xFE... returns) */

    /* Exp43 fix: discard packets whose |x| or |y| exceeds this (misaligned
     * PS/2 reads produce clamped extremes like 127/-128). Tune per sensor. */
    int8_t max_delta = 32;

private:
    uint8_t readByte(uint16_t timeout);

    uint16_t _readTimeout = 40000;

    /* Exp43 sync: readPacket first waits for a sustained CLK-high (packet
     * gap) so the status byte is read from a true packet boundary, not
     * mid-stream. Thresholds in busy-loop iterations (~0.75us each at 8MHz):
     * idle_count ~150us sits above intra-bit CLK highs (~50us) and below the
     * measured inter-packet gaps (>=0.7ms). */
    uint16_t _syncIdleCount = 200;
    uint16_t _syncTimeout = 5000;

    uint8_t _clkPin;
    uint8_t _datPin;
    uint8_t _clkMask;
    uint8_t _datMask;
    volatile uint8_t *_clkPinReg;
    volatile uint8_t *_clkDdrReg;
    volatile uint8_t *_clkPortReg;
    volatile uint8_t *_datPinReg;
    volatile uint8_t *_datDdrReg;
    volatile uint8_t *_datPortReg;
};

#endif
