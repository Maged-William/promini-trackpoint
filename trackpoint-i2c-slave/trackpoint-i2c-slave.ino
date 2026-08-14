#define SLEEP_ENABLED 0

#include <Wire.h>
#include <PS2Trackpoint.h>
#include <PowerCurve.h>
#if SLEEP_ENABLED
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#endif

#define I2C_ADDR     0x42
#define MOT_PIN      14

#define PS2_CLK      7
#define PS2_DAT      3

#define BURST_ADDR     0x12
#define SPEED_REG      PowerCurve::REG_SENS

#define READ_INTERVAL_MS 20
#define MAX_DELTA 127
#define DEADBAND 3
#define IDLE_TIMEOUT_MS 5000
#define SERIAL_LOG 0 /* 1 = log wake/sleep transitions only, 0 = fully silent */

PS2Trackpoint ps2(PS2_CLK, PS2_DAT);
PowerCurve curve;

static uint8_t cur_addr;
static int8_t  burst_x;
static int8_t  burst_y;
static unsigned long last_motion_ms = 0;
static uint16_t      wake_count = 0;

void requestEvent() {
    if (cur_addr == BURST_ADDR) {
        uint8_t buf[2] = { (uint8_t)burst_x, (uint8_t)burst_y };
        Wire.write(buf, 2);
    } else if (cur_addr == 0x01) {
        uint8_t buf[2] = { (uint8_t)(wake_count >> 8), (uint8_t)wake_count };
        Wire.write(buf, 2);
    } else {
        Wire.write(0x00);
    }
}

void receiveEvent(int len) {
    if (len <= 0) {
        return;
    }
    cur_addr = Wire.read();
    uint8_t b1 = 0, b2 = 0;
    if (len > 1) b1 = Wire.read();
    if (len > 2) b2 = Wire.read();

    if (cur_addr == SPEED_REG) {
        curve.setSens(b1);
    } else if (cur_addr == PowerCurve::REG_RATE
            || cur_addr == PowerCurve::REG_EXP
            || cur_addr == PowerCurve::REG_START) {
        curve.setParam(cur_addr, (uint16_t)b1 | ((uint16_t)b2 << 8));
    }
}

void setup() {
    pinMode(MOT_PIN, OUTPUT);
    digitalWrite(MOT_PIN, HIGH);

    Wire.begin(I2C_ADDR);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);

    ps2.begin();
    curve.begin();

#if SERIAL_LOG
    Serial.begin(9600);
    Serial.println("--- Exp46: I2C slave boot OK (9600) ---");
#endif

    last_motion_ms = millis();
}

void loop() {
    static uint8_t       was_moving = 0;
    static unsigned long last_ps2_ms = 0;
    static unsigned long last_read_ms = 0;
    static unsigned long last_hb_ms = 0;

    int8_t x, y;
    uint8_t buttons;

    unsigned long now = millis();

    curve.update(); /* rebuild LUT if a param landed at init (no-op otherwise) */

#if SERIAL_LOG
    if (now - last_hb_ms >= 500) {
        last_hb_ms = now;
        Serial.println(now);
    }
#endif

    if (now - last_read_ms >= READ_INTERVAL_MS) {
        last_read_ms = now;
        if (ps2.readPacket(x, y, buttons)) {
            last_ps2_ms = millis();

#if SERIAL_LOG
            Serial.print("X:"); Serial.print(x);
            Serial.print(" Y:"); Serial.println(y);
#endif

            if (abs(x) >= 127 || abs(y) >= 127 || abs(x) > MAX_DELTA || abs(y) > MAX_DELTA) {
                burst_x = 0; burst_y = 0;
            } else {
                curve.apply(x, y, burst_x, burst_y);
                if (abs(burst_x) > DEADBAND || abs(burst_y) > DEADBAND) {
                    was_moving = 1;
                    last_motion_ms = millis();
                }
            }
        }
    }

    if (burst_x || burst_y) {
        if (millis() - last_ps2_ms > 100) {
            burst_x = 0;
            burst_y = 0;

#if SERIAL_LOG
            static unsigned long last_hb = 0;
            if (millis() - last_hb >= 1000) {
                last_hb = millis();
                Serial.println("idle");
            }
#endif

            if (was_moving) {
                was_moving = 0;
            }
        }
    }

#if SLEEP_ENABLED
    if (millis() - last_motion_ms >= IDLE_TIMEOUT_MS) {
        burst_x = 0;
        burst_y = 0;
#if SERIAL_LOG
        Serial.println("Sleeping...");
        Serial.flush();
#endif

        pinMode(PS2_CLK, OUTPUT);
        digitalWrite(PS2_CLK, LOW);

        digitalWrite(MOT_PIN, LOW);
        TWCR = 0;

        while (1) {
            MCUSR &= ~(1<<WDRF);
            WDTCSR |= (1<<WDCE) | (1<<WDE);
            WDTCSR = (1<<WDIE) | (1<<WDP0) | (1<<WDP2);
            set_sleep_mode(SLEEP_MODE_IDLE);
            sleep_enable();
            sei();
            sleep_cpu();
            sleep_disable();
            WDTCSR |= (1<<WDCE) | (1<<WDE);
            WDTCSR = 0;
            burst_x = 0;
            burst_y = 0;

            wake_count++;

            pinMode(PS2_CLK, INPUT_PULLUP);

            if (ps2.readPacket(x, y, buttons)) {
                if (!(abs(x) >= 127 || abs(y) >= 127 || abs(x) > MAX_DELTA || abs(y) > MAX_DELTA)) {
                    if (abs(x) > DEADBAND || abs(y) > DEADBAND) {
                        curve.apply(x, y, burst_x, burst_y);
                        was_moving = 1;
                        Wire.begin(I2C_ADDR);
                        digitalWrite(MOT_PIN, HIGH);
                        last_motion_ms = millis();
                        last_ps2_ms = millis();
#if SERIAL_LOG
                        Serial.println("Woke!");
#endif
                        break;
                    }
                }
            }

            pinMode(PS2_CLK, OUTPUT);
            digitalWrite(PS2_CLK, LOW);
        }
    }
#endif
}
