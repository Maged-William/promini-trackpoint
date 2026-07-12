// PMW3610 SPI Slave Emulator — Exp04
// Per-byte SPI transactions with CS deassertion between each byte.
// The nRF52 sends each byte as a separate transaction (CS↓, 1 byte, CS↑).
// CS deassertion gives the AVR time to read SPDR and write the next response,
// eliminating the single-buffered SPDR overwrite race (Exp03 root cause).
//
// Writes remain as 2-byte continuous transactions — the 8µs window (at 1MHz)
// between bytes is enough for the SPIF polling loop to catch the address byte
// before the data byte overwrites SPDR.
//
// State machine:
//   S_IDLE       → address byte (first byte after CS↓)
//   S_ADDR_RCVD  → single register data (read or write)
//   S_BURST      → burst read data (sequential register values)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>

#define MOT_PIN      2
#define PULSE_US     100
#define PERIOD_MS    500

// Circle pattern: 16 steps of {X_L, Y_L, XY_H} forming a smooth circle
static const uint8_t circle[16][3] PROGMEM = {
    {0x03, 0x00, 0x00},  //  0: X=3,  Y=0
    {0x03, 0x01, 0x00},  //  1: X=3,  Y=1
    {0x02, 0x02, 0x00},  //  2: X=2,  Y=2
    {0x01, 0x03, 0x00},  //  3: X=1,  Y=3
    {0x00, 0x03, 0x00},  //  4: X=0,  Y=3
    {0xFF, 0x03, 0xF0},  //  5: X=-1, Y=3
    {0xFE, 0x02, 0xF0},  //  6: X=-2, Y=2
    {0xFD, 0x01, 0xF0},  //  7: X=-3, Y=1
    {0xFD, 0x00, 0xF0},  //  8: X=-3, Y=0
    {0xFD, 0xFF, 0xFF},  //  9: X=-3, Y=-1
    {0xFE, 0xFE, 0xFF},  // 10: X=-2, Y=-2
    {0xFF, 0xFD, 0xFF},  // 11: X=-1, Y=-3
    {0x00, 0xFD, 0x0F},  // 12: X=0,  Y=-3
    {0x01, 0xFD, 0x0F},  // 13: X=1,  Y=-3
    {0x02, 0xFE, 0x0F},  // 14: X=2,  Y=-2
    {0x03, 0xFF, 0x0F},  // 15: X=3,  Y=-1
};

#define BURST_SIZE     7
#define REG_BURST      0x12

enum { S_IDLE, S_ADDR_RCVD, S_BURST };

static volatile uint8_t burst[BURST_SIZE];
static uint8_t state;
static uint8_t last_addr;
static uint8_t write_pending;
static uint8_t burst_idx;
static volatile uint8_t spi_byte_count;
static uint8_t regs[128];

static void update_circle_step(uint8_t step) {
    uint8_t i = step & 0x0F;
    uint8_t xl  = pgm_read_byte(&circle[i][0]);
    uint8_t yl  = pgm_read_byte(&circle[i][1]);
    uint8_t xyh = pgm_read_byte(&circle[i][2]);

    cli();
    burst[1] = xl;
    burst[2] = yl;
    burst[3] = xyh;
    sei();
}

ISR(PCINT0_vect) {
    // CS changed state. For CS falling edge, the SPI hardware
    // automatically shifts out the SPDR value pre-loaded by the
    // previous SPIF handler — no action needed here.
}

void setup() {
    pinMode(MOT_PIN, OUTPUT);
    digitalWrite(MOT_PIN, HIGH);

    pinMode(MISO, OUTPUT);
    SPCR = _BV(SPE) | _BV(CPOL) | _BV(CPHA);  // SPI slave, mode 3
    SPDR = 0x00;

    PCICR  |= _BV(PCIE0);
    PCMSK0 |= _BV(PCINT2);  // PB2 = SS/D10 (CS)

    // Initialize burst with circle step 0 values
    burst[0] = 0x01;  // MOTION — motion detected
    burst[1] = 0x03;  // DELTA_X_L
    burst[2] = 0x00;  // DELTA_Y_L
    burst[3] = 0x00;  // DELTA_XY_H
    burst[4] = 0x00;  // SQUAL
    burst[5] = 0x00;  // SHUTTER_H
    burst[6] = 0x00;  // SHUTTER_L

    // Register file (reads use these for single-register access)
    // All zeros by default (global), which is fine.

    state = S_IDLE;

    Serial.begin(9600);
    Serial.println("--- PMW3610 emulator Exp04 ---");
    Serial.println("Per-byte transactions + CS deassertion");
}

void loop() {
    static unsigned long last_step     = 0;
    static bool          pulsed        = false;
    static unsigned long last_pulse_us = 0;
    static uint8_t       step          = 0;
    static uint8_t       last_spi_cnt  = 0;
    static unsigned long last_spi_print = 0;

    // ── SPI byte handling (poll SPIF) ──
    {
        uint8_t spsr_val = SPSR;
        if (spsr_val & _BV(SPIF)) {
            uint8_t received = SPDR;  // SPIF cleared (SPSR read above, now SPDR)
            spi_byte_count++;

            if (state == S_IDLE) {
                last_addr = received & 0x7F;
                write_pending = received & 0x80;

                if (write_pending) {
                    // Write: address received, expect data byte (same CS assertion)
                    state = S_ADDR_RCVD;
                    SPDR = 0x00;
                } else if (last_addr == REG_BURST) {
                    // Burst read start
                    state = S_BURST;
                    burst_idx = 0;
                    SPDR = burst[0];
                } else {
                    // Single register read
                    state = S_ADDR_RCVD;
                    SPDR = regs[last_addr];
                }
            } else if (state == S_ADDR_RCVD) {
                if (write_pending) {
                    regs[last_addr] = received;
                    write_pending = 0;
                }
                state = S_IDLE;
                SPDR = 0x00;
            } else if (state == S_BURST) {
                burst_idx++;
                if (burst_idx < BURST_SIZE) {
                    SPDR = burst[burst_idx];
                } else {
                    state = S_IDLE;
                    SPDR = 0x00;
                }
            }
        }
    }

    // ── MOT pin pulse (triggers ZMK burst read) ──
    if (!pulsed && (millis() - last_step >= PERIOD_MS)) {
        update_circle_step(step++);
        digitalWrite(MOT_PIN, LOW);
        pulsed = true;
        last_pulse_us = micros();
        last_step = millis();
    } else if (pulsed && (micros() - last_pulse_us >= PULSE_US)) {
        digitalWrite(MOT_PIN, HIGH);
        pulsed = false;
    }

    // ── Serial debug ──
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
}
