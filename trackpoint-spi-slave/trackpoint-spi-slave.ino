// PMW3610 SPI Slave Emulator — Exp08
// Power-curve smoothing: activity ramp replaces flat /4 scaling.
// Real TrackPoint PS/2 data via SPI pipeline.
// Interrupt-driven SPI (SPI_STC_vect) — never misses a byte.
// 100 Hz MOT (10ms period) for smooth cursor movement.
// Per-byte SPI transactions with CS deassertion (Exp04 proven).

#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h>
#include <PS2Trackpoint.h>

#define MOT_PIN      2
#define PULSE_US     100
#define PERIOD_MS    10

#define PS2_CLK      7
#define PS2_DAT      3

PS2Trackpoint ps2(PS2_CLK, PS2_DAT);

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

static unsigned long  ps2_last_pkt_ms = 0;
static uint8_t        ramp             = 0;   // activity ramp 0..20 (Exp08)

static void update_from_ps2(int8_t x, int8_t y) {
    x = -x;  // reverse X axis — TrackPoint direction vs screen

    uint8_t mag = abs(x) + abs(y);
    if (mag > 0) {
        ramp += 2;
        if (ramp > 20) ramp = 20;
    }

    // Power curve: output = input * ramp / 20
    // At ramp=0: zero output, at ramp=20: full raw value
    int16_t out_x = ((int16_t)x * ramp) / 20;
    int16_t out_y = ((int16_t)y * ramp) / 20;

    if (out_x > 127) out_x = 127;
    if (out_x < -128) out_x = -128;
    if (out_y > 127) out_y = 127;
    if (out_y < -128) out_y = -128;

    uint16_t x_12 = (out_x >= 0) ? (uint16_t)out_x : (uint16_t)(4096 + out_x);
    uint16_t y_12 = (out_y >= 0) ? (uint16_t)out_y : (uint16_t)(4096 + out_y);

    cli();
    burst[0] = 0x01;
    burst[1] = (uint8_t)(x_12 & 0xFF);
    burst[2] = (uint8_t)(y_12 & 0xFF);
    burst[3] = ((uint8_t)(x_12 >> 4) & 0xF0)
             | ((uint8_t)(y_12 >> 8) & 0x0F);
    burst[4] = 0x00;
    burst[5] = 0x00;
    burst[6] = 0x00;
    sei();

    ps2_last_pkt_ms = millis();
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
    SPCR = _BV(SPE) | _BV(SPIE) | _BV(CPOL) | _BV(CPHA);
    SPDR = 0x00;

    PCICR  |= _BV(PCIE0);
    PCMSK0 |= _BV(PCINT2);

    burst[0] = 0x00;
    burst[1] = 0x00;
    burst[2] = 0x00;
    burst[3] = 0x00;
    burst[4] = 0x00;
    burst[5] = 0x00;
    burst[6] = 0x00;

    state = S_IDLE;

    ps2.begin();

    Serial.begin(115200);
    Serial.println("--- PMW3610 emulator Exp07 ---");
    Serial.println("TrackPoint PS/2 → SPI pipeline");
}

void loop() {
    static unsigned long last_mot      = 0;
    static bool          pulsed        = false;
    static unsigned long last_pulse_us = 0;

    static uint8_t       last_spi_cnt  = 0;
    static unsigned long last_spi_print = 0;

    // ── Read TrackPoint PS/2 packet ──
    int8_t x, y;
    uint8_t buttons;
    if (ps2.readPacket(x, y, buttons)) {
        update_from_ps2(x, y);
    } else {
        ramp = 0;  // instant decay — every touch starts from slow (Exp08)
    }

    // Clear stale motion if no PS/2 data for 500ms
    if (ps2_last_pkt_ms && (millis() - ps2_last_pkt_ms > 500)) {
        cli();
        burst[0] = 0x00;
        burst[1] = 0x00;
        burst[2] = 0x00;
        burst[3] = 0x00;
        sei();
        ps2_last_pkt_ms = 0;
    }

    // ── MOT pin pulse (triggers ZMK burst read) ──
    if (!pulsed && (millis() - last_mot >= PERIOD_MS)) {
        digitalWrite(MOT_PIN, LOW);
        pulsed = true;
        last_pulse_us = micros();
        last_mot = millis();
    } else if (pulsed && (micros() - last_pulse_us >= PULSE_US)) {
        digitalWrite(MOT_PIN, HIGH);
        cli();
        burst[0] = 0x00;
        burst[1] = 0x00;
        burst[2] = 0x00;
        burst[3] = 0x00;
        sei();
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
