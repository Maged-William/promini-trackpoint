#ifndef PS2TRACKPOINT_H
#define PS2TRACKPOINT_H

#include <Arduino.h>
#include <stdint.h>

class PS2Trackpoint {
public:
    PS2Trackpoint(uint8_t clkPin, uint8_t datPin);

    void begin();
    bool readPacket(int8_t &x, int8_t &y, uint8_t &buttons);

private:
    uint8_t readByte(uint16_t timeout = 40000);

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
