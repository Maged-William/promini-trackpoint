// PMW3610 SPI Slave Emulator — Exp16
// Synthetic rectangle over BLE — clone of Exp05 with current pin mapping.
// Interrupt-driven SPI (SPI_STC_vect) — never misses a byte.
// 50 Hz MOT (20ms period) — match BLE throughput.
// Running-average smoothing (SMOOTH_FACTOR=4) — ramps each velocity change.
// Per-byte SPI transactions with CS deassertion (Exp04 proven).
// No PS/2, no sleep, no power switching — pure synthetic test.
//
// Rectangle (200×200 px, ~4.5s loop):
//   A→B: Fast right     (+4,  0) ×  50 steps  200 px/s
//   B→C: Slow down      ( 0, +2) × 100 steps  100 px/s
//   C→D: Extrafast left (-8,  0) ×  25 steps  400 px/s
//   D→A: Normal up      ( 0, -4) ×  50 steps  200 px/s

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>

#define MOT_PIN      14
#define PULSE_US     100
#define PERIOD_MS    20

static const int8_t rect_data[4][3] PROGMEM = {
    {  4,  0, 50  },   // A→B: Fast right     (+4,  0) ×  50  200 px/s
    {  0,  2, 100 },   // B→C: Slow down      ( 0, +2) × 100  100 px/s
    { -8,  0, 25  },   // C→D: Extrafast left (-8,  0) ×  25  400 px/s
    {  0, -4, 50  },   // D→A: Normal up      ( 0, -4) ×  50  200 px/s
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

#define SMOOTH_FACTOR 4

static int8_t dx_history[SMOOTH_FACTOR];
static int8_t dy_history[SMOOTH_FACTOR];
static uint8_t hist_idx = 0;
static uint8_t hist_filled = 0;

static void update_rect_step(void) {
    int8_t dx_target = pgm_read_byte(&rect_data[segment][0]);
    int8_t dy_target = pgm_read_byte(&rect_data[segment][1]);

    dx_history[hist_idx] = dx_target;
    dy_history[hist_idx] = dy_target;
    hist_idx = (hist_idx + 1) % SMOOTH_FACTOR;
    if (hist_filled < SMOOTH_FACTOR) hist_filled++;

    int16_t dx_sum = 0, dy_sum = 0;
    for (uint8_t i = 0; i < hist_filled; i++) {
        dx_sum += dx_history[i];
        dy_sum += dy_history[i];
    }

    int8_t dx = (dx_sum + SMOOTH_FACTOR / 2) / hist_filled;
    int8_t dy = (dy_sum + SMOOTH_FACTOR / 2) / hist_filled;

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

ISR(SPI_STC_vect) {
    uint8_t received = SPDR;
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
}

void setup() {
    pinMode(13, OUTPUT);
    for (uint8_t i = 0; i < 6; i++) {
        digitalWrite(13, i & 1);
        delay(100);
    }

    pinMode(MOT_PIN, OUTPUT);
    digitalWrite(MOT_PIN, HIGH);

    pinMode(MISO, OUTPUT);
    SPCR = _BV(SPE) | _BV(SPIE) | _BV(CPOL) | _BV(CPHA);
    SPDR = 0x00;

    PCICR  |= _BV(PCIE0);
    PCMSK0 |= _BV(PCINT2);

    burst[0] = 0x01;
    burst[1] = 0x02;
    burst[2] = 0x00;
    burst[3] = 0x00;
    burst[4] = 0x00;
    burst[5] = 0x00;
    burst[6] = 0x00;

    state = S_IDLE;

    Serial.begin(115200);
    Serial.println("--- PMW3610 emulator Exp16d ---");
    Serial.println("Synthetic rectangle over BLE (50 Hz)");
}

void loop() {
    static unsigned long last_step      = 0;
    static bool          pulsed         = false;
    static unsigned long last_pulse_us  = 0;

    static uint8_t       last_spi_cnt   = 0;
    static unsigned long last_spi_print = 0;

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
