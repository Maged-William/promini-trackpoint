// PMW3610 SPI Slave Emulator — Exp02 debug
// Pro Mini 3.3V 8MHz (ATmega328P)
//
// SPI wiring to NiceNano:
//   D10 (PB2/SS)   <- P0.20 (CS)
//   D11 (PB3/MOSI) <- P0.17 (MOSI)
//   D12 (PB4/MISO) -> P0.06 (MISO)
//   D13 (PB5/SCK)  <- P0.08 (SCK)
//   D2  (PD2)      -> P0.10 (MOT/IRQ, active LOW)
//
// Generates a tiny slow circle (radius 3, 8s/rotation) and
// prints every step over Serial for cross-checking with the
// NiceNano PMW3610 driver logs.

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>

#define MOT_PIN      2
#define PULSE_MS     2
#define PERIOD_MS    500

// 16-step circle, radius 3, {X_low, Y_low, XY_high}
static const uint8_t circle[16][3] PROGMEM = {
    {0x03, 0x00, 0x00},  //   0: X=3,  Y=0
    {0x03, 0x01, 0x00},  //   1: X=3,  Y=1
    {0x02, 0x02, 0x00},  //   2: X=2,  Y=2
    {0x01, 0x03, 0x00},  //   3: X=1,  Y=3
    {0x00, 0x03, 0x00},  //   4: X=0,  Y=3
    {0xFF, 0x03, 0xF0},  //   5: X=-1, Y=3
    {0xFE, 0x02, 0xF0},  //   6: X=-2, Y=2
    {0xFD, 0x01, 0xF0},  //   7: X=-3, Y=1
    {0xFD, 0x00, 0xF0},  //   8: X=-3, Y=0
    {0xFD, 0xFF, 0xFF},  //   9: X=-3, Y=-1
    {0xFE, 0xFE, 0xFF},  //  10: X=-2, Y=-2
    {0xFF, 0xFD, 0xFF},  //  11: X=-1, Y=-3
    {0x00, 0xFD, 0x0F},  //  12: X=0,  Y=-3
    {0x01, 0xFD, 0x0F},  //  13: X=1,  Y=-3
    {0x02, 0xFE, 0x0F},  //  14: X=2,  Y=-2
    {0x03, 0xFF, 0x0F},  //  15: X=3,  Y=-1
};

static volatile uint8_t byte_pos;
static volatile uint8_t burst_idx;
static volatile uint8_t is_write;

static uint8_t rsp0[128];
static volatile uint8_t burst[6];

ISR(PCINT0_vect) {
    byte_pos = 0;
    SPDR     = 0x00;
}

ISR(SPI_STC_vect) {
    uint8_t rx = SPDR;

    if (byte_pos == 0) {
        if (rx & 0x80) {
            is_write = 1;
            SPDR     = 0x00;
        } else {
            is_write  = 0;
            rx       &= 0x7F;
            SPDR      = rsp0[rx];
            if (rx == 0x12) {
                burst_idx = 0;
            } else {
                burst_idx = 0xFF;
            }
        }
        byte_pos = 1;
    } else if (is_write) {
        is_write  = 0;
        byte_pos  = 0;
        SPDR      = 0x00;
    } else if (burst_idx < 6) {
        SPDR = burst[burst_idx++];
    } else {
        SPDR = 0x00;
    }
}

static void update_circle_step(uint8_t step) {
    uint8_t i = step & 0x0F;
    uint8_t xl  = pgm_read_byte(&circle[i][0]);
    uint8_t yl  = pgm_read_byte(&circle[i][1]);
    uint8_t xyh = pgm_read_byte(&circle[i][2]);

    cli();
    burst[0] = xl;
    burst[1] = yl;
    burst[2] = xyh;
    sei();
}

void setup() {
    pinMode(MOT_PIN, OUTPUT);
    digitalWrite(MOT_PIN, HIGH);

    pinMode(MISO, OUTPUT);
    SPCR = _BV(SPE) | _BV(SPIE) | _BV(CPOL) | _BV(CPHA);
    SPDR = 0x00;

    PCICR  |= _BV(PCIE0);
    PCMSK0 |= _BV(PCINT2);

    for (uint8_t i = 0; i < 128; i++) rsp0[i] = 0x00;
    rsp0[0x02] = 0x80;
    rsp0[0x03] = 0x01;
    rsp0[0x04] = 0x01;
    rsp0[0x05] = 0x00;
    rsp0[0x2D] = 0x0F;
    rsp0[0x3F] = 0x3E;
    rsp0[0x12] = 0x80;

    burst[0] = 0x01;  // X low
    burst[1] = 0x01;  // Y low
    burst[2] = 0x00;  // XY high
    burst[3] = 0x80;  // SQUAL
    burst[4] = 0x30;  // Shutter high
    burst[5] = 0x42;  // Shutter low

    Serial.begin(9600);
    Serial.println("--- PMW3610 emulator debug ---");
    Serial.print("Burst bytes send order: [0x80, ");
    Serial.print("Xlo, Ylo, XYhi, SQUAL, ShutH, ShutL]");
    Serial.println();
}

void loop() {
    static unsigned long last_pulse = 0;
    static bool         pulsed      = false;
    static uint8_t      step        = 0;

    if (!pulsed && (millis() - last_pulse >= PERIOD_MS)) {
        uint8_t i = step & 0x0F;
        uint8_t xl  = pgm_read_byte(&circle[i][0]);
        uint8_t yl  = pgm_read_byte(&circle[i][1]);
        uint8_t xyh = pgm_read_byte(&circle[i][2]);

        Serial.print("step=");
        Serial.print(i);
        Serial.print(" xl=0x");
        Serial.print(xl, HEX);
        Serial.print(" yl=0x");
        Serial.print(yl, HEX);
        Serial.print(" xyh=0x");
        Serial.print(xyh, HEX);
        Serial.println();

        update_circle_step(step);
        step++;

        digitalWrite(MOT_PIN, LOW);
        pulsed      = true;
        last_pulse  = millis();
    } else if (pulsed && (millis() - last_pulse >= PULSE_MS)) {
        digitalWrite(MOT_PIN, HIGH);
        pulsed = false;
    }
}
