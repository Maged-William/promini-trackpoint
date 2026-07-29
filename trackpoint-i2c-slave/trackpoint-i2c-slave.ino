#include <Wire.h>
#include <PS2Trackpoint.h>
#include <LowPower.h>

#define I2C_ADDR     0x42
#define MOT_PIN      14
#define NPN_PIN      4
#define PMOS_PIN     6
#define LED_PIN      13

#define PS2_CLK      7
#define PS2_DAT      3

#define BURST_ADDR     0x12
#define SPEED_REG      0x11
#define SPEED_DEFAULT  255

#define READ_INTERVAL_MS 20
#define MAX_DELTA 25
#define DEADBAND 3
#define IDLE_TIMEOUT_MS 5000
#define SERIAL_LOG 1

PS2Trackpoint ps2(PS2_CLK, PS2_DAT);

static uint8_t cur_addr;
static uint8_t speed_scale = SPEED_DEFAULT;
static int8_t  burst_x;
static int8_t  burst_y;
static int16_t rem_x = 0;
static int16_t rem_y = 0;
static bool          calibrated = false;
static int8_t        calib_x = 0, calib_y = 0;
static int32_t       calib_sum_x = 0, calib_sum_y = 0;
static uint16_t      calib_count = 0;
static unsigned long calib_end_ms = 0;
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
    if (len > 0) {
        cur_addr = Wire.read();
        if (len > 1 && cur_addr == SPEED_REG) {
            speed_scale = Wire.read();
        }
    }
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

    if (SERIAL_LOG) {
        Serial.begin(115200);
        Serial.println("--- Exp26 — PS/2 DAT D3 CLK D7 ---");
    }

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

                    int32_t tx = (int32_t)x * speed_scale + rem_x;
                    int32_t ty = (int32_t)y * speed_scale + rem_y;
                    int8_t sx = tx / 256;
                    int8_t sy = ty / 256;
                    rem_x = tx - sx * 256;
                    rem_y = ty - sy * 256;
                    burst_x = sx;
                    burst_y = sy;
                    if (abs(burst_x) > DEADBAND || abs(burst_y) > DEADBAND) {
                        if (SERIAL_LOG) {
                            Serial.print(burst_x);
                            Serial.print(",");
                            Serial.println(burst_y);
                        }
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
                if (SERIAL_LOG) Serial.println("0");
                was_moving = 0;
            }
        }
    }

    if (millis() - last_motion_ms >= IDLE_TIMEOUT_MS) {
        burst_x = 0;
        burst_y = 0;
        if (SERIAL_LOG) {
            Serial.println("Sleeping...");
            Serial.flush();
        }

        pinMode(PS2_CLK, OUTPUT);
        digitalWrite(PS2_CLK, LOW);

        while (1) {
            LowPower.idle(SLEEP_60MS, ADC_OFF);
            burst_x = 0;
            burst_y = 0;

            wake_count++;
            if (SERIAL_LOG && (wake_count % 100 == 0)) {
                Serial.print("SLP:");
                Serial.println(wake_count);
            }

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
                            int32_t tx = (int32_t)x * speed_scale + rem_x;
                            int32_t ty = (int32_t)y * speed_scale + rem_y;
                            int8_t sx = tx / 256;
                            int8_t sy = ty / 256;
                            rem_x = tx - sx * 256;
                            rem_y = ty - sy * 256;
                            burst_x = sx;
                            burst_y = sy;
                            if (SERIAL_LOG) {
                                Serial.print("W:");
                                Serial.print(burst_x);
                                Serial.print(",");
                                Serial.println(burst_y);
                            }
                            was_moving = 1;
                            pulse_mot();
                            last_motion_ms = millis();
                            last_ps2_ms = millis();
                            if (SERIAL_LOG) Serial.println("Woke!");
                            break;
                        }
                    }
                }
            }

            pinMode(PS2_CLK, OUTPUT);
            digitalWrite(PS2_CLK, LOW);
        }
        if (SERIAL_LOG) Serial.println("AWAKE");
    }
}
