// PMW3610 SPI Slave Emulator — Exp09
// Deep sleep with TTP223 touch wakeup on D19.
// AGENTS.md wiring: MOT=D14, PS2_CLK=D3, PS2_DAT=D2.
// Power-curve smoothing: activity ramp (Exp08).
// Interrupt-driven SPI (SPI_STC_vect).
// 100 Hz MOT. Per-byte SPI transactions.
// LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF) when idle.

#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h>
#include <PS2Trackpoint.h>
#include <LowPower.h>

#define MOT_PIN      14
#define TOUCH_PIN    19
#define PULSE_US     100
#define PERIOD_MS    10

#define PS2_CLK      3
#define PS2_DAT      2

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
static uint8_t        ramp             = 0;

static void update_from_ps2(int8_t x, int8_t y) {
    x = -x;

    uint8_t mag = abs(x) + abs(y);
    if (mag > 0) {
        ramp += 4;
        if (ramp > 20) ramp = 20;
    }

    int16_t out_x = ((int16_t)x * ramp * 3) / 40;
    int16_t out_y = ((int16_t)y * ramp * 3) / 40;

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

ISR(PCINT1_vect) {
}

static void enter_sleep() {
    Serial.println("Sleeping...");
    Serial.flush();
    delay(10);

    UCSR0B = 0;

    SPCR &= ~_BV(SPE);

    pinMode(MOT_PIN, INPUT_PULLUP);

    cli();
    PCMSK0 &= ~_BV(PCINT2);
    PCMSK1 |= _BV(PCINT5);
    PCICR  |= _BV(PCIE1);
    sei();

    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

    PCMSK1 &= ~_BV(PCINT5);

    PCMSK0 |= _BV(PCINT2);

    pinMode(MOT_PIN, OUTPUT);
    digitalWrite(MOT_PIN, HIGH);

    SPCR = _BV(SPE) | _BV(SPIE) | _BV(CPOL) | _BV(CPHA);
    SPDR = 0x00;

    state = S_IDLE;
    burst_idx = 0;

    cli();
    burst[0] = 0x00; burst[1] = 0x00; burst[2] = 0x00; burst[3] = 0x00;
    burst[4] = 0x00; burst[5] = 0x00; burst[6] = 0x00;
    sei();

    ramp = 0;
    ps2_last_pkt_ms = 0;

    Serial.begin(115200);
    Serial.println("Woke!");
}

void setup() {
    pinMode(MOT_PIN, OUTPUT);
    digitalWrite(MOT_PIN, HIGH);

    pinMode(TOUCH_PIN, INPUT);

    pinMode(MISO, OUTPUT);
    SPCR = _BV(SPE) | _BV(SPIE) | _BV(CPOL) | _BV(CPHA);
    SPDR = 0x00;

    PCICR  |= _BV(PCIE0);
    PCMSK0 |= _BV(PCINT2);

    burst[0] = 0x00; burst[1] = 0x00; burst[2] = 0x00; burst[3] = 0x00;
    burst[4] = 0x00; burst[5] = 0x00; burst[6] = 0x00;

    state = S_IDLE;

    ps2.begin();

    Serial.begin(115200);
    Serial.println("--- PMW3610 emulator Exp09 ---");
    Serial.println("Deep sleep + TTP223 touch wakeup");
}

void loop() {
    static unsigned long last_mot      = 0;
    static bool          pulsed        = false;
    static unsigned long last_pulse_us = 0;

    static uint8_t       last_spi_cnt   = 0;
    static unsigned long last_spi_print = 0;

    static unsigned long idle_start     = 0;
    static bool          boot_grace     = true;

    uint8_t touched = digitalRead(TOUCH_PIN);

    if (!touched) {
        if (idle_start == 0) {
            idle_start = millis();
        } else {
            unsigned long timeout = boot_grace ? 15000UL : 2000UL;
            if (millis() - idle_start >= timeout) {
                enter_sleep();
                last_mot   = millis();
                pulsed     = false;
                idle_start = 0;
                boot_grace = false;
            }
        }
    } else {
        idle_start = 0;
        boot_grace = false;
    }

    int8_t x, y;
    uint8_t buttons;
    if (ps2.readPacket(x, y, buttons)) {
        update_from_ps2(x, y);
    } else {
        ramp = 0;
    }

    if (ps2_last_pkt_ms && (millis() - ps2_last_pkt_ms > 500)) {
        cli();
        burst[0] = 0x00; burst[1] = 0x00; burst[2] = 0x00; burst[3] = 0x00;
        sei();
        ps2_last_pkt_ms = 0;
    }

    if (!pulsed && (millis() - last_mot >= PERIOD_MS)) {
        digitalWrite(MOT_PIN, LOW);
        pulsed = true;
        last_pulse_us = micros();
        last_mot = millis();
    } else if (pulsed && (micros() - last_pulse_us >= PULSE_US)) {
        digitalWrite(MOT_PIN, HIGH);
        cli();
        burst[0] = 0x00; burst[1] = 0x00; burst[2] = 0x00; burst[3] = 0x00;
        sei();
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
