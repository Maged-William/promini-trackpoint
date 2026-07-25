#include <Wire.h>
#include <PS2Trackpoint.h>
#include <LowPower.h>

#define I2C_ADDR     0x42
#define MOT_PIN      14
#define TOUCH_PIN    2
#define NPN_PIN      4
#define PMOS_PIN     6
#define LED_PIN      13

#define PS2_CLK      3
#define PS2_DAT      7

#define MOT_PERIOD_MS 10
#define PULSE_US      100
#define BURST_ADDR    0x12

#define IDLE_TIMEOUT_MS 2000
#define BOOT_GRACE_MS   15000

PS2Trackpoint ps2(PS2_CLK, PS2_DAT);

static uint8_t cur_addr;
static int8_t  burst_x;
static int8_t  burst_y;
static uint8_t wake_discard;

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

static void wakeUp() {}

static void enter_sleep() {
    pinMode(PS2_CLK, INPUT);
    pinMode(PS2_DAT, INPUT);
    digitalWrite(NPN_PIN, LOW);
    digitalWrite(PMOS_PIN, HIGH);
    delay(50);

    Serial.println("Sleeping...");
    Serial.flush();
    delay(10);

    digitalWrite(MOT_PIN, HIGH);
    digitalWrite(LED_PIN, LOW);

    TWCR = 0;
    attachInterrupt(digitalPinToInterrupt(TOUCH_PIN), wakeUp, RISING);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    detachInterrupt(digitalPinToInterrupt(TOUCH_PIN));

    digitalWrite(NPN_PIN, HIGH);
    delay(5);
    digitalWrite(PMOS_PIN, LOW);
    delay(200);

    ps2.begin();

    Wire.begin(I2C_ADDR);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);

    burst_x = 0;
    burst_y = 0;
    cur_addr = 0;
    wake_discard = 5;

    Serial.println("Woke!");
}

void setup() {
    pinMode(NPN_PIN, OUTPUT);
    digitalWrite(NPN_PIN, HIGH);
    pinMode(PMOS_PIN, OUTPUT);
    digitalWrite(PMOS_PIN, LOW);
    pinMode(TOUCH_PIN, INPUT_PULLUP);

    pinMode(MOT_PIN, OUTPUT);
    digitalWrite(MOT_PIN, HIGH);

    Wire.begin(I2C_ADDR);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);

    ps2.begin();

    Serial.begin(115200);
    Serial.println("--- I2C Slave + PS/2 + Sleep ---");
}

void loop() {
    static unsigned long last_mot  = 0;
    static bool          pulsed   = false;
    static unsigned long pulse_us = 0;

    static unsigned long idle_start = 0;
    static bool          boot_grace = true;

    int8_t x, y;
    uint8_t buttons;

    if (ps2.readPacket(x, y, buttons)) {
        if (wake_discard) {
            wake_discard--;
        } else {
            x = -x;
            if (abs(x) < 3 && abs(y) < 3) {
                x = 0; y = 0;
            }
            burst_x = y;
            burst_y = x;
            Serial.print("X:"); Serial.print(burst_x);
            Serial.print(" Y:"); Serial.println(burst_y);
            idle_start = 0;
            boot_grace = false;
        }
    }

    if (!pulsed && (millis() - last_mot >= MOT_PERIOD_MS)) {
        digitalWrite(MOT_PIN, LOW);
        pulsed = true;
        pulse_us = micros();
        last_mot = millis();
    } else if (pulsed && (micros() - pulse_us >= PULSE_US)) {
        digitalWrite(MOT_PIN, HIGH);
        burst_x = 0;
        burst_y = 0;
        pulsed = false;
    }

    if (boot_grace && millis() > BOOT_GRACE_MS) {
        boot_grace = false;
        idle_start = millis();
    }

    if (!boot_grace && !pulsed) {
        if (idle_start == 0) {
            idle_start = millis();
        } else if (millis() - idle_start >= IDLE_TIMEOUT_MS) {
            enter_sleep();
            last_mot  = millis();
            pulsed    = false;
            idle_start = 0;
        }
    }
}
