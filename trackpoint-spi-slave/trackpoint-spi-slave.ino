// PMW3610 SPI Slave Emulator — Exp02
// Pro Mini 3.3V 8MHz (ATmega328P)
//
// SPI wiring to NiceNano:
//   D10 (PB2/SS)   <- P0.20 (CS)
//   D11 (PB3/MOSI) <- P0.17 (MOSI)
//   D12 (PB4/MISO) -> P0.06 (MISO)
//   D13 (PB5/SCK)  <- P0.08 (SCK)
//   D2  (PD2)      -> P0.10 (MOT/IRQ, active LOW)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h>

#define MOT_PIN      2
#define MAX_RESPONSE 16
#define PULSE_MS     1
#define PERIOD_MS    50

static volatile uint8_t tx_buf[MAX_RESPONSE];
static volatile uint8_t tx_idx;
static volatile uint8_t byte_pos;
static volatile uint8_t cmd_addr;
static volatile bool   is_write;

static void prepare_read_response(uint8_t reg) {
    memset((void *)tx_buf, 0, MAX_RESPONSE);

    switch (reg) {
    case 0x2D:  // Observation — self-test pass mask
        tx_buf[0] = 0x0F;
        break;
    case 0x3F:  // Product ID (moved from 0x00)
        tx_buf[0] = 0x3E;
        break;
    case 0x02:  // Motion register
        tx_buf[0] = 0x80;  // motion detected
        break;
    case 0x03:  // Delta_X_L
        tx_buf[0] = 0x01;
        break;
    case 0x04:  // Delta_Y_L
        tx_buf[0] = 0x01;
        break;
    case 0x05:  // Delta_XY_H
        tx_buf[0] = 0x00;
        break;
    case 0x12:  // Motion burst (7 bytes)
        tx_buf[0] = 0x80;  // motion flag
        tx_buf[1] = 0x01;  // X delta low
        tx_buf[2] = 0x01;  // Y delta low
        tx_buf[3] = 0x00;  // XY high nibbles
        tx_buf[4] = 0x80;  // SQUAL
        tx_buf[5] = 0x30;  // Shutter high
        tx_buf[6] = 0x42;  // Shutter low
        break;
    default:
        break;  // unknown -> 0x00
    }
}

// PCINT on PB2 (SS/D10) — transaction framing
ISR(PCINT0_vect) {
    if (PINB & _BV(PB2)) {
        // CS HIGH -> transaction ended
        byte_pos  = 0;
        is_write  = false;
        SPDR      = 0x00;
    } else {
        // CS LOW -> transaction starting
        byte_pos  = 0;
        is_write  = false;
        SPDR      = 0x00;   // pre-load dummy for address phase
    }
}

// SPI transfer complete
ISR(SPI_STC_vect) {
    uint8_t rx = SPDR;

    if (byte_pos == 0) {
        // First byte: command address
        cmd_addr = rx;
        if (rx & 0x80) {
            is_write = true;
            memset((void *)tx_buf, 0, MAX_RESPONSE);
        } else {
            is_write = false;
            prepare_read_response(rx & 0x7F);
        }
        tx_idx   = 1;
        byte_pos = 1;
        SPDR     = tx_buf[0];
    } else if (is_write) {
        // Second byte: write data (absorb, ignore)
        is_write = false;
        byte_pos = 0;
        SPDR     = 0x00;
    } else {
        // Read continuation: send next byte
        if (tx_idx < MAX_RESPONSE) {
            SPDR = tx_buf[tx_idx++];
        } else {
            SPDR = 0x00;
        }
    }
}

void setup() {
    pinMode(MOT_PIN, OUTPUT);
    digitalWrite(MOT_PIN, HIGH);

    // SPI slave mode 3 (CPOL=1, CPHA=1)
    pinMode(MISO, OUTPUT);
    SPCR = _BV(SPE) | _BV(SPIE) | _BV(CPOL) | _BV(CPHA);
    SPDR = 0x00;

    // Pin Change Interrupt on PB2 (SS/D10)
    PCICR  |= _BV(PCIE0);
    PCMSK0 |= _BV(PCINT2);
}

void loop() {
    static unsigned long last_pulse = 0;
    static bool         pulsed      = false;

    if (!pulsed && (millis() - last_pulse >= PERIOD_MS)) {
        digitalWrite(MOT_PIN, LOW);   // trigger IRQ (active low)
        pulsed      = true;
        last_pulse  = millis();
    } else if (pulsed && (millis() - last_pulse >= PULSE_MS)) {
        digitalWrite(MOT_PIN, HIGH);  // release after short pulse
        pulsed = false;
    }
}
