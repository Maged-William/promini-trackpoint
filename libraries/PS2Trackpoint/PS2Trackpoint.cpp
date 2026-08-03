#include "PS2Trackpoint.h"

PS2Trackpoint::PS2Trackpoint(uint8_t clkPin, uint8_t datPin)
    : _clkPin(clkPin), _datPin(datPin)
{
}

void PS2Trackpoint::begin() {
    _clkMask = digitalPinToBitMask(_clkPin);
    _datMask = digitalPinToBitMask(_datPin);

    uint8_t clkPort = digitalPinToPort(_clkPin);
    uint8_t datPort = digitalPinToPort(_datPin);

    _clkPinReg  = portInputRegister(clkPort);
    _clkDdrReg  = portModeRegister(clkPort);
    _clkPortReg = portOutputRegister(clkPort);

    _datPinReg  = portInputRegister(datPort);
    _datDdrReg  = portModeRegister(datPort);
    _datPortReg = portOutputRegister(datPort);

    *_clkDdrReg  &= ~_clkMask;  * _clkPortReg |= _clkMask;
    *_datDdrReg  &= ~_datMask;  * _datPortReg |= _datMask;

    read_timeouts = 0;
}

uint8_t PS2Trackpoint::readByte(uint16_t timeout) {
    uint8_t out = 0;
    uint16_t t;

    t = timeout;
    while (*_clkPinReg & _clkMask) {
        if (--t == 0) { read_timeouts++; return 0; }
    }
    t = timeout;
    while (!(*_clkPinReg & _clkMask)) {
        if (--t == 0) { read_timeouts++; return 0xFF; }
    }
    for (int i = 0; i < 8; i++) {
        t = timeout;
        while (*_clkPinReg & _clkMask) {
            if (--t == 0) { read_timeouts++; return 0xFE; }
        }
        out |= ((*_datPinReg & _datMask) ? 1 : 0) << i;
        t = timeout;
        while (!(*_clkPinReg & _clkMask)) {
            if (--t == 0) { read_timeouts++; return 0xFD; }
        }
    }
    for (int i = 0; i < 2; i++) {
        t = timeout;
        while (*_clkPinReg & _clkMask) {
            if (--t == 0) { read_timeouts++; return 0xFC; }
        }
        t = timeout;
        while (!(*_clkPinReg & _clkMask)) {
            if (--t == 0) { read_timeouts++; return 0xFB; }
        }
    }

    return out;
}

bool PS2Trackpoint::readPacket(int8_t &x, int8_t &y, uint8_t &buttons) {
    /* Exp43 sync: wait for a sustained CLK-high (packet gap) so the status
       byte read starts at a true packet boundary instead of mid-stream. */
    uint16_t idle = 0;
    uint16_t t = _syncTimeout;
    while (idle < _syncIdleCount) {
        if (*_clkPinReg & _clkMask) {
            idle++;
        } else {
            idle = 0;
        }
        if (--t == 0) return false; /* trackpoint silent — no packet coming */
    }

    uint16_t tmo_before = read_timeouts;
    uint8_t s = readByte(_readTimeout);
    last_status = s;
    if (!(s & 0x08)) return false;

    uint8_t xraw = readByte(_readTimeout);
    uint8_t yraw = readByte(_readTimeout);

    /* Exp43 fix: any PS/2 timeout during the 3-byte read means the read
     * landed mid-stream (misaligned). readByte timeout markers (0xFE etc.)
     * have bit 3 set, so garbage can pass the status check above. Discard. */
    if (read_timeouts != tmo_before) return false;

    int ix = (int)xraw - ((s << 4) & 0x100);
    int iy = (int)yraw - ((s << 3) & 0x100);

    if (s & 0x40) ix = (ix < 0) ? -128 : 127;
    if (s & 0x80) iy = (iy < 0) ? -128 : 127;
    if (ix > 127) ix = 127;
    if (ix < -128) ix = -128;
    if (iy > 127) iy = 127;
    if (iy < -128) iy = -128;

    /* Exp43 fix: sanity gate — misaligned-but-timout-free reads yield
     * absurd deltas; real 100 Hz trackpoint deltas stay small. */
    if (ix > max_delta || ix < -max_delta || iy > max_delta || iy < -max_delta) return false;

    last_status = s;
    last_xraw = xraw;
    last_yraw = yraw;

    x = (int8_t)ix;
    y = (int8_t)iy;
    buttons = s & 0x07;

    return true;
}
