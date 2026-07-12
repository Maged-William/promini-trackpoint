#ifndef PS2TRACKPOINT_H
#define PS2TRACKPOINT_H

#include <Arduino.h>
#include <stdint.h>

class PS2Trackpoint {
public:
    PS2Trackpoint(uint8_t clkPin, uint8_t datPin);

    void begin();

    uint8_t readByte(uint16_t timeout = 0);
    void sendByte(uint8_t data);

    bool readPacket(int8_t &x, int8_t &y, uint8_t &buttons);
    void reset();
    void enableStreaming();

    enum Error : uint8_t {
        ERR_OK = 0,
        ERR_CLK_STUCK_HIGH,
        ERR_CLK_STUCK_LOW,
        ERR_DATA_BIT_TIMEOUT,
        ERR_PARITY_STOP_TIMEOUT,
    };

    Error lastError() const { return _lastError; }

private:
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

    Error _lastError;
};

#endif
