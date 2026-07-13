// PMW3610 SPI Slave Emulator — Exp05
// Rectangle speed test with 4 segments at different velocities.
// Per-byte SPI transactions with CS deassertion between each byte (Exp04 proven).
//
// MOT period reduced to 50ms (20 Hz) so the cursor updates smoothly enough
// to evaluate whether fast/slow movement is choppy or glitchy.
//
// Rectangle:
//   A→B: Fast right   (+10,  0) × 20 steps  ≈ 1s  @ 50ms
//   B→C: Slow down    (  0, +1) × 100 steps ≈ 5s
//   C→D: Normal left  ( -5,  0) × 40 steps  ≈ 2s
//   D→A: Extra fast up(  0,-10) × 10 steps  ≈ 0.5s

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>

#define MOT_PIN      2
#define PULSE_US     100
#define PERIOD_MS    50

// Rectangle segment data: {dx, dy, steps}
// dx/dy are signed 8-bit, converted to 12-bit two's complement at runtime
static const int8_t rect_data[4][3] PROGMEM = {
    { 10,  0, 20  },   // A→B: Fast right   (+10,  0) × 20
    {  0,  1, 100 },   // B→C: Slow down    (  0, +1) × 100
    { -5,  0, 40  },   // C→D: Normal left  ( -5,  0) × 40
    {  0, -10, 10  },   // D→A: Extra fast up(  0,-10) × 10
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

static uint8_t segment = 0;
static uint8_t step_in_seg = 0;

static void update_rect_step(void) {
    int8_t dx = pgm_read_byte(&rect_data[segment][0]);
    int8_t dy = pgm_read_byte(&rect_data[segment][1]);

    // Convert signed 8-bit to 12-bit two's complement
    uint16_t dx_12 = (dx >= 0) ? (uint16_t)dx : (uint16_t)(4096 + dx);
    uint16_t dy_12 = (dy >= 0) ? (uint16_t)dy : (uint16_t)(4096 + dy);

    cli();
    burst[1] = (uint8_t)(dx_12 & 0xFF);        // X_L
    burst[2] = (uint8_t)(dy_12 & 0xFF);        // Y_L
    burst[3] = ((uint8_t)(dx_12 >> 4) & 0xF0)
             | ((uint8_t)(dy_12 >> 8) & 0x0F); // XY_H
    sei();

    step_in_seg++;
    uint8_t steps = pgm_read_byte(&rect_data[segment][2]);
    if (step_in_seg >= steps) {
        segment = (segment + 1) & 3;
        step_in_seg = 0;
    }
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

    // Initialize burst with segment 0 values (A→B: +10, 0)
    burst[0] = 0x01;  // MOTION — motion detected
    burst[1] = 0x0A;  // DELTA_X_L = 10
    burst[2] = 0x00;  // DELTA_Y_L = 0
    burst[3] = 0x00;  // DELTA_XY_H
    burst[4] = 0x00;  // SQUAL
    burst[5] = 0x00;  // SHUTTER_H
    burst[6] = 0x00;  // SHUTTER_L

    // Register file (reads use these for single-register access)
    // All zeros by default (global), which is fine.

    state = S_IDLE;

    Serial.begin(9600);
    Serial.println("--- PMW3610 emulator Exp05 ---");
    Serial.println("Rectangle speed test (20 Hz MOT)");
}

void loop() {
    static unsigned long last_step     = 0;
    static bool          pulsed        = false;
    static unsigned long last_pulse_us = 0;

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
        update_rect_step();
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
