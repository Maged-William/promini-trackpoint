#include <Wire.h>
#include <PS2Trackpoint.h>
#include <LowPower.h>

#define I2C_ADDR     0x42
#define MOT_PIN      14
#define NPN_PIN      4
#define PMOS_PIN     6
#define LED_PIN      13

#define PS2_CLK      2
#define PS2_DAT      3

#define BURST_ADDR    0x12

#define READ_INTERVAL_MS 20
#define MAX_DELTA 25
#define DEADBAND 3
#define IDLE_TIMEOUT_MS 1000

PS2Trackpoint ps2(PS2_CLK, PS2_DAT);

static uint8_t cur_addr;
static int8_t  burst_x;
static int8_t  burst_y;
static bool          calibrated = false;
static int8_t        calib_x = 0, calib_y = 0;
static int32_t       calib_sum_x = 0, calib_sum_y = 0;
static uint16_t      calib_count = 0;
static unsigned long calib_end_ms = 0;
static unsigned long last_motion_ms = 0;


void requestEvent() {
    if (cur_addr == BURST_ADDR) {
        uint8_t buf[2] = { (uint8_t)burst_x, (uint8_t)burst_y };
        Wire.write(buf, 2);
    } else {
        Wire.write(0x00);
    }
}

void receiveEvent(int len) {
    if (len > 0) cur_addr = Wire.read();
}

static void pulse_mot() {
    digitalWrite(MOT_PIN, LOW);
    delayMicroseconds(100);
    digitalWrite(MOT_PIN, HIGH);
}

void setup() {
    pinMode(NPN_PIN, OUTPUT);
    digitalWrite(NPN_PIN, HIGH);
    pinMode(PMOS_PIN, OUTPUT);
    digitalWrite(PMOS_PIN, LOW);
    pinMode(MOT_PIN, OUTPUT);
    digitalWrite(MOT_PIN, HIGH);

    Wire.begin(I2C_ADDR);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);

    ps2.begin();

    Serial.begin(115200);
    Serial.println("--- I2C Slave + PS/2 + CLK-inhibit WDT Sleep ---");

    last_motion_ms = millis();
}

void loop() {
    static uint8_t       was_moving = 0;
    static unsigned long last_ps2_ms = 0;
    static unsigned long last_read_ms = 0;

    int8_t x, y;
    uint8_t buttons;

    unsigned long now = millis();
    if (now - last_read_ms >= READ_INTERVAL_MS) {
        last_read_ms = now;
        if (ps2.readPacket(x, y, buttons)) {
            last_ps2_ms = millis();

            if (!calibrated) {
                if (millis() >= calib_end_ms) {
                    if (calib_count > 0) {
                        calib_x = (int8_t)(calib_sum_x / calib_count);
                        calib_y = (int8_t)(calib_sum_y / calib_count);
                    }
                    calibrated = true;
                } else if (abs(x) < 127 && abs(y) < 127) {
                    calib_sum_x += x;
                    calib_sum_y += y;
                    calib_count++;
                } else {
                    burst_x = 0; burst_y = 0;
                }
            }

            if (calibrated) {
                if (abs(x) >= 127 || abs(y) >= 127 || abs(x) > MAX_DELTA || abs(y) > MAX_DELTA) {
                    burst_x = 0; burst_y = 0;
                } else {
                    int32_t cx = (int32_t)x - calib_x;
                    int32_t cy = (int32_t)y - calib_y;

                    x = (cx < -128) ? -128 : (cx > 127) ? 127 : (int8_t)cx;
                    y = (cy < -128) ? -128 : (cy > 127) ? 127 : (int8_t)cy;

                    burst_x = x;
                    burst_y = y;
                    if (abs(burst_x) > DEADBAND || abs(burst_y) > DEADBAND) {
                        Serial.print(burst_x);
                        Serial.print(",");
                        Serial.println(burst_y);
                        was_moving = 1;
                        pulse_mot();
                        last_motion_ms = millis();
                    }
                }
            }
        }
    }

    if (burst_x || burst_y) {
        if (millis() - last_ps2_ms > 100) {
            burst_x = 0;
            burst_y = 0;
            if (was_moving) {
                Serial.println("0");
                was_moving = 0;
            }
        }
    }

    if (millis() - last_motion_ms >= IDLE_TIMEOUT_MS) {
        Serial.println("Sleeping...");
        Serial.flush();

        pinMode(PS2_CLK, OUTPUT);
        digitalWrite(PS2_CLK, LOW);
        TWCR = 0;

        while (1) {
            LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);

            Wire.begin(I2C_ADDR);
            Wire.onRequest(requestEvent);
            Wire.onReceive(receiveEvent);

            pinMode(PS2_CLK, INPUT_PULLUP);

            if (ps2.readPacket(x, y, buttons)) {
                if (!calibrated) {
                    if (millis() >= calib_end_ms) {
                        if (calib_count > 0) {
                            calib_x = (int8_t)(calib_sum_x / calib_count);
                            calib_y = (int8_t)(calib_sum_y / calib_count);
                        }
                        calibrated = true;
                    } else if (abs(x) < 127 && abs(y) < 127) {
                        calib_sum_x += x;
                        calib_sum_y += y;
                        calib_count++;
                    }
                }

                if (calibrated) {
                    if (!(abs(x) >= 127 || abs(y) >= 127 || abs(x) > MAX_DELTA || abs(y) > MAX_DELTA)) {
                        int32_t cx = (int32_t)x - calib_x;
                        int32_t cy = (int32_t)y - calib_y;
                        x = (cx < -128) ? -128 : (cx > 127) ? 127 : (int8_t)cx;
                        y = (cy < -128) ? -128 : (cy > 127) ? 127 : (int8_t)cy;
                        if (abs(x) > DEADBAND || abs(y) > DEADBAND) {
                            burst_x = x;
                            burst_y = y;
                            Serial.print("W:");
                            Serial.print(burst_x);
                            Serial.print(",");
                            Serial.println(burst_y);
                            was_moving = 1;
                            pulse_mot();
                            last_motion_ms = millis();
                            last_ps2_ms = millis();
                            Serial.println("Woke!");
                            break;
                        }
                    }
                }
            }

            TWCR = 0;
            pinMode(PS2_CLK, OUTPUT);
            digitalWrite(PS2_CLK, LOW);
        }
    }
}
