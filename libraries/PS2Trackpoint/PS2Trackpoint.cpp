#include "PS2Trackpoint.h"

PS2Trackpoint::PS2Trackpoint(uint8_t clkPin, uint8_t datPin)
    : _clkPin(clkPin), _datPin(datPin),
      _lastError(ERR_OK)
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

    *_clkDdrReg  &= ~_clkMask;  * _clkPortReg |= _clkMask;   // input + pullup
    *_datDdrReg  &= ~_datMask;  * _datPortReg |= _datMask;   // input + pullup
}

uint8_t PS2Trackpoint::readByte(uint16_t timeout) {
    uint8_t out = 0;
    uint16_t t;

    if (timeout == 0) {
        while (*_clkPinReg & _clkMask);
        while (!(*_clkPinReg & _clkMask));
        for (int i = 0; i < 8; i++) {
            while (*_clkPinReg & _clkMask);
            out |= ((*_datPinReg & _datMask) ? 1 : 0) << i;
            while (!(*_clkPinReg & _clkMask));
        }
        for (int i = 0; i < 2; i++) {
            while (*_clkPinReg & _clkMask);
            while (!(*_clkPinReg & _clkMask));
        }
    } else {
        t = timeout;
        while (*_clkPinReg & _clkMask) {
            if (--t == 0) { _lastError = ERR_CLK_STUCK_HIGH; return 0; }
        }
        t = timeout;
        while (!(*_clkPinReg & _clkMask)) {
            if (--t == 0) { _lastError = ERR_CLK_STUCK_LOW; return 0xFF; }
        }
        for (int i = 0; i < 8; i++) {
            t = timeout;
            while (*_clkPinReg & _clkMask) {
                if (--t == 0) { _lastError = ERR_DATA_BIT_TIMEOUT; return 0xFE; }
            }
            out |= ((*_datPinReg & _datMask) ? 1 : 0) << i;
            t = timeout;
            while (!(*_clkPinReg & _clkMask)) {
                if (--t == 0) { _lastError = ERR_DATA_BIT_TIMEOUT; return 0xFD; }
            }
        }
        for (int i = 0; i < 2; i++) {
            t = timeout;
            while (*_clkPinReg & _clkMask) {
                if (--t == 0) { _lastError = ERR_PARITY_STOP_TIMEOUT; return 0xFC; }
            }
            t = timeout;
            while (!(*_clkPinReg & _clkMask)) {
                if (--t == 0) { _lastError = ERR_PARITY_STOP_TIMEOUT; return 0xFB; }
            }
        }
    }

    _lastError = ERR_OK;
    return out;
}

void PS2Trackpoint::sendByte(uint8_t data) {
    uint8_t parity = 1;

    *_clkDdrReg |= _clkMask;   *_clkPortReg &= ~_clkMask;    // pull CLK low
    delayMicroseconds(150);
    *_datDdrReg |= _datMask;   *_datPortReg &= ~_datMask;    // pull DAT low (start bit)
    *_clkDdrReg &= ~_clkMask;  *_clkPortReg |= _clkMask;    // release CLK

    for (int i = 0; i < 8; i++) {
        while (*_clkPinReg & _clkMask);
        uint8_t b = (data >> i) & 1;
        if (b) *_datPortReg |= _datMask; else *_datPortReg &= ~_datMask;
        parity ^= b;
        while (!(*_clkPinReg & _clkMask));
    }

    while (*_clkPinReg & _clkMask);
    if (parity) *_datPortReg |= _datMask; else *_datPortReg &= ~_datMask;
    while (!(*_clkPinReg & _clkMask));

    while (*_clkPinReg & _clkMask);
    *_datDdrReg &= ~_datMask;   *_datPortReg |= _datMask;     // release DAT (stop bit)
    while (!(*_clkPinReg & _clkMask));

    while (*_datPinReg & _datMask);     // wait for ACK (DAT low)
    while ((*_clkPinReg & _clkMask) && (*_datPinReg & _datMask));  // wait for idle
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

void PS2Trackpoint::reset() {
    sendByte(0xFF);
    readByte();
    readByte();
    readByte();
}

void PS2Trackpoint::enableStreaming() {
    sendByte(0xF4);
    readByte();
}
