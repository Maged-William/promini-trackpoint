// PMW3610 SPI Slave Emulator — Exp03
// Pro Mini 3.3V 8MHz (ATmega328P)
//
// SPI wiring to NiceNano:
//   D10 (PB2/SS)   <- P0.20 (CS)
//   D11 (PB3/MOSI) <- P0.17 (MOSI)
//   D12 (PB4/MISO) -> P0.06 (MISO)
//   D13 (PB5/SCK)  <- P0.08 (SCK)
//   D2  (PD2)      -> P0.10 (MOT/IRQ, active LOW)
//
// Strategy: nRF52 SPIM has near-zero inter-byte gap, so the AVR SPI
// ISR is always ~4 cycles late.  This causes a deterministic 1-byte
// shift  —  master buf[N] = AVR burst[N-1] (for N>=1).
//
// Compensation: burst array layout is X_L at [0], Y_L at [1],
// XY_H at [2], which the driver reads at buf[1-3] perfectly.
// At 1 MHz SPI (8 us/byte, 64 AVR cycles) the ISR (~25 cycles)
// completes within the current byte, reliably pre‑loading the
// write buffer for byte N+2.
//
// Init checks (observation + product ID) are bypassed in the
// ZMK driver (Exp02).

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>

#define MOT_PIN      2
#define PULSE_MS     2
#define PERIOD_MS    500

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
static volatile uint8_t burst[6];
static volatile uint8_t spi_byte_count;

ISR(PCINT0_vect) {
    if (!(PINB & _BV(PB2))) {     // CS falling edge → new transaction
        byte_pos = 0;
        burst_idx = 0xFF;         // not in burst mode yet
        is_write  = 0;
        SPDR      = 0x00;
    }
}

ISR(SPI_STC_vect) {
    spi_byte_count++;
    uint8_t rx = SPDR;
    SPDR = 0x00;    // safe default; overwritten for burst data below

    if (byte_pos == 0) {
        if (rx & 0x80) {
            is_write = 1;
        } else {
            is_write = 0;
            rx &= 0x7F;
            if (rx == 0x12) {
                burst_idx = 0;
            }
        }
        byte_pos = 1;
    } else if (is_write) {
        is_write = 0;
        byte_pos = 0;
    } else if (burst_idx < 6) {
        SPDR = burst[burst_idx++];
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

    burst[0] = 0x01;
    burst[1] = 0x01;
    burst[2] = 0x00;
    burst[3] = 0x80;
    burst[4] = 0x30;
    burst[5] = 0x42;

    Serial.begin(9600);
    Serial.println("--- PMW3610 emulator Exp03 ---");
    Serial.println("1-byte shift compensation: burst[0-2]=X_L,Y_L,XY_H");
    Serial.println("Master sees buf[1-3]=burst[0-2] -> correct X/Y");
}

void loop() {
    static unsigned long last_pulse  = 0;
    static bool         pulsed       = false;
    static uint8_t      step         = 0;
    static uint8_t      last_spi_cnt = 0;
    static unsigned long last_spi_print = 0;

    if (millis() - last_spi_print >= 2000) {
        uint8_t cnt = spi_byte_count;
        if (cnt != last_spi_cnt) {
            Serial.print("SPI bytes: ");
            Serial.print(cnt);
            Serial.print(" (+");
            Serial.print(cnt - last_spi_cnt);
            Serial.println(")");
            last_spi_cnt = cnt;
        } else {
            Serial.println("SPI bytes: 0 (no activity)");
        }
        last_spi_print = millis();
    }

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
