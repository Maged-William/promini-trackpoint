// PMW3610 SPI Slave Emulator — Exp14
// Deep sleep with dual-rail power cut + PS/2 pin float.
// D4 = NPN (GND switch), D6 = AO3401 P-MOSFET (VCC switch).
// D3/D7 floated (INPUT, no pull-up) during sleep to kill parasitic power.
// TTP223 INT0 wakeup (D2). SPI + PS/2 re-initialized on wake.
// Power-curve smoothing: activity ramp (Exp08).
// Interrupt-driven SPI (SPI_STC_vect). 100 Hz MOT. Per-byte SPI transactions.

#include <avr/io.h>
#include <avr/interrupt.h>
#include <Arduino.h>
#include <PS2Trackpoint.h>
#include <LowPower.h>

#define MOT_PIN      14
#define TOUCH_PIN    2
#define PULSE_US     100
#define PERIOD_MS    10

#define PS2_CLK      3
#define PS2_DAT      7
#define NPN_PIN      4
#define VCC_PIN      6

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

    Serial.print("Burst: ");
    for (uint8_t i = 0; i < BURST_SIZE; i++) {
        if (burst[i] < 0x10) Serial.print("0");
        Serial.print(burst[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

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

static void wakeUp() {
}

static void enter_sleep() {
    Serial.println("Sleeping...");
    Serial.flush();
    delay(10);

    digitalWrite(MOT_PIN, HIGH);

    // Cut both power rails
    digitalWrite(NPN_PIN, LOW);   // NPN OFF — GND floating
    digitalWrite(VCC_PIN, HIGH);  // P-MOSFET OFF — VCC disconnected

    // Float PS/2 pins (no pull-ups) to kill parasitic power
    pinMode(PS2_CLK, INPUT);
    pinMode(PS2_DAT, INPUT);

    // Kill SPI and D13 LED
    SPCR &= ~_BV(SPE);
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);

    attachInterrupt(digitalPinToInterrupt(TOUCH_PIN), wakeUp, CHANGE);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    detachInterrupt(digitalPinToInterrupt(TOUCH_PIN));

    // --- WAKE ---

    // Restore power rails
    digitalWrite(VCC_PIN, LOW);   // P-MOSFET ON — VCC connected
    digitalWrite(NPN_PIN, HIGH);  // NPN ON — GND connected
    delay(100);  // let power stabilize

    // Re-init PS/2 pins (INPUT_PULLUP) and let TrackPoint power on
    ps2.begin();
    delay(100);

    // Restore SPI
    SPCR = _BV(SPE) | _BV(SPIE) | _BV(CPOL) | _BV(CPHA);
    SPDR = 0x00;

    state = S_IDLE;
    burst_idx = 0;

    cli();
    burst[0] = 0x00; burst[1] = 0x00; burst[2] = 0x00; burst[3] = 0x00;
    burst[4] = 0x00; burst[5] = 0x00; burst[6] = 0x00;
    sei();
    Serial.println("Burst cleared (idle)");

    ramp = 0;
    ps2_last_pkt_ms = 0;

    Serial.println("Woke!");
}

void setup() {
    // Blink D13 3x fast so we can confirm Exp14 is running
    pinMode(13, OUTPUT);
    for (uint8_t i = 0; i < 6; i++) {
        digitalWrite(13, i & 1);
        delay(100);
    }

    pinMode(MOT_PIN, OUTPUT);
    digitalWrite(MOT_PIN, HIGH);

    pinMode(NPN_PIN, OUTPUT);
    digitalWrite(NPN_PIN, HIGH);

    pinMode(VCC_PIN, OUTPUT);
    digitalWrite(VCC_PIN, LOW);  // LOW = P-MOSFET ON = VCC connected

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
    Serial.println("--- PMW3610 emulator Exp14 ---");
    Serial.println("MOT=D14  PS2 CLK=D3  DAT=D7  NPN=D4  VCC=D6  TTP223=D2");
}

void loop() {
    static unsigned long last_mot      = 0;
    static bool          pulsed        = false;
    static unsigned long last_pulse_us = 0;

    static uint8_t       last_spi_cnt   = 0;
    static unsigned long last_spi_print = 0;

    static unsigned long idle_start     = 0;
    static bool          boot_grace     = true;

    int8_t x, y;
    uint8_t buttons;
    if (ps2.readPacket(x, y, buttons)) {
        update_from_ps2(x, y);
        Serial.print("PS2: "); Serial.print(x); Serial.print(","); Serial.println(y);
        idle_start = 0;
        boot_grace = false;
    } else {
        ramp = 0;
        PS2Trackpoint::Error err = ps2.lastError();
        static PS2Trackpoint::Error last_err = PS2Trackpoint::ERR_OK;
        static unsigned long last_err_print = 0;
        if (err != last_err && millis() - last_err_print > 1000) {
            Serial.print("PS2 err: "); Serial.println(err);
            last_err = err;
            last_err_print = millis();
        }
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

    if (boot_grace && millis() > 15000) {
        boot_grace = false;
        idle_start = millis();
    }
    if (!boot_grace) {
        if (idle_start == 0) {
            idle_start = millis();
        } else if (millis() - idle_start >= 2000) {
            enter_sleep();
            last_mot   = millis();
            pulsed     = false;
            idle_start = 0;
        }
    }
}
