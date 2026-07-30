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
}

uint8_t PS2Trackpoint::readByte(uint16_t timeout) {
    uint8_t out = 0;
    uint16_t t;

    t = timeout;
    while (*_clkPinReg & _clkMask) {
        if (--t == 0) return 0;
    }
    t = timeout;
    while (!(*_clkPinReg & _clkMask)) {
        if (--t == 0) return 0xFF;
    }
    for (int i = 0; i < 8; i++) {
        t = timeout;
        while (*_clkPinReg & _clkMask) {
            if (--t == 0) return 0xFE;
        }
        out |= ((*_datPinReg & _datMask) ? 1 : 0) << i;
        t = timeout;
        while (!(*_clkPinReg & _clkMask)) {
            if (--t == 0) return 0xFD;
        }
    }
    for (int i = 0; i < 2; i++) {
        t = timeout;
        while (*_clkPinReg & _clkMask) {
            if (--t == 0) return 0xFC;
        }
        t = timeout;
        while (!(*_clkPinReg & _clkMask)) {
            if (--t == 0) return 0xFB;
        }
    }

    return out;
}

bool PS2Trackpoint::readPacket(int8_t &x, int8_t &y, uint8_t &buttons) {
    uint8_t s = readByte(40000);
    if (!(s & 0x08)) return false;

    uint8_t xraw = readByte(40000);
    uint8_t yraw = readByte(40000);

    int ix = (int)xraw - ((s << 4) & 0x100);
    int iy = (int)yraw - ((s << 3) & 0x100);

    if (s & 0x40) ix = (ix < 0) ? -128 : 127;
    if (s & 0x80) iy = (iy < 0) ? -128 : 127;
    if (ix > 127) ix = 127;
    if (ix < -128) ix = -128;
    if (iy > 127) iy = 127;
    if (iy < -128) iy = -128;

    x = (int8_t)ix;
    y = (int8_t)iy;
    buttons = s & 0x07;

    return true;
}
