// PMW3610 SPI Slave Emulator — Exp05b
// Rectangle speed test — 100 Hz MOT for smooth movement.
// Per-byte SPI transactions with CS deassertion between each byte (Exp04 proven).
//
// MOT period 10ms (100 Hz) — each cursor step is smaller at the same screen
// speed, eliminating the "20 fps" choppiness of Exp05.
//
// Rectangle (200×200 px):
//   A→B: Fast right     (+2,  0) × 100 steps  ≈ 1.0s  @ 200 px/s
//   B→C: Slow down      ( 0, +1) × 200 steps  ≈ 2.0s  @ 100 px/s
//   C→D: Extrafast left (-4,  0) ×  50 steps  ≈ 0.5s  @ 400 px/s
//   D→A: Normal up      ( 0, -2) × 100 steps  ≈ 1.0s  @ 200 px/s

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>

#define MOT_PIN      2
#define PULSE_US     100
#define PERIOD_MS    10

// Rectangle segment data: {dx, dy, steps}
// dx/dy are signed 8-bit, converted to 12-bit two's complement at runtime
static const int8_t rect_data[4][3] PROGMEM = {
    {  2,  0, 100 },   // A→B: Fast right     (+2,  0) × 100
    {  0,  1, 200 },   // B→C: Slow down      ( 0, +1) × 200
    { -4,  0, 50  },   // C→D: Extrafast left (-4,  0) ×  50
    {  0, -2, 100 },   // D→A: Normal up      ( 0, -2) × 100
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

// ── SPI Interrupt ──
// Handles every byte immediately, even during Serial.print.
// Eliminates the polling gap that corrupts the state machine.
ISR(SPI_STC_vect) {
    uint8_t received = SPDR;  // clears SPIF
    spi_byte_count++;

    if (state == S_IDLE) {
        last_addr = received & 0x7F;
        write_pending = received & 0x80;

        if (write_pending) {
            state = S_ADDR_RCVD;
            SPDR = 0x00;
        } else if (last_addr == REG_BURST) {
            state = S_BURST;
            burst_idx = 0;
            SPDR = burst[0];
        } else {
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

ISR(PCINT0_vect) {
    // CS changed state — not needed with interrupt-driven SPI.
}

void setup() {
    pinMode(MOT_PIN, OUTPUT);
    digitalWrite(MOT_PIN, HIGH);

    pinMode(MISO, OUTPUT);
    SPCR = _BV(SPE) | _BV(SPIE) | _BV(CPOL) | _BV(CPHA);  // SPI slave, mode 3, interrupt
    SPDR = 0x00;

    PCICR  |= _BV(PCIE0);
    PCMSK0 |= _BV(PCINT2);  // PB2 = SS/D10 (CS)

    // Initialize burst with segment 0 values (A→B: +2, 0)
    burst[0] = 0x01;  // MOTION — motion detected
    burst[1] = 0x02;  // DELTA_X_L = 2
    burst[2] = 0x00;  // DELTA_Y_L = 0
    burst[3] = 0x00;  // DELTA_XY_H
    burst[4] = 0x00;  // SQUAL
    burst[5] = 0x00;  // SHUTTER_H
    burst[6] = 0x00;  // SHUTTER_L

    // Register file (reads use these for single-register access)
    // All zeros by default (global), which is fine.

    state = S_IDLE;

    Serial.begin(115200);
    Serial.println("--- PMW3610 emulator Exp05b ---");
    Serial.println("Rectangle speed test (100 Hz MOT)");
}

void loop() {
    static unsigned long last_step     = 0;
    static bool          pulsed        = false;
    static unsigned long last_pulse_us = 0;

    static uint8_t       last_spi_cnt  = 0;
    static unsigned long last_spi_print = 0;

    // ── SPI handled by ISR (SPI_STC_vect) — no polling needed

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
