#include <Wire.h>

#define I2C_ADDR      0x42
#define MOT_PIN       14
#define LED_PIN       13
#define MOT_PERIOD_MS 20
#define PULSE_US      100

static uint8_t regs[128];
static uint8_t current_addr;

void requestEvent() {
    Wire.write(&regs[current_addr], 1);
}

void receiveEvent(int len) {
    if (len > 0) {
        current_addr = Wire.read();
    }
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    for (uint8_t i = 0; i < 6; i++) {
        digitalWrite(LED_PIN, i & 1);
        delay(100);
    }

    pinMode(MOT_PIN, OUTPUT);
    digitalWrite(MOT_PIN, HIGH);

    regs[0x00] = 0x3E;
    regs[0x01] = 0x01;
    regs[0x02] = 0x04;
    regs[0x03] = 0x00;
    regs[0x04] = 0x00;
    regs[0x05] = 0x00;
    regs[0x06] = 0x00;
    regs[0x12] = 0x01;
    regs[0x13] = 0x04;
    regs[0x14] = 0x00;
    regs[0x15] = 0x00;
    regs[0x16] = 0x3E;

    Wire.begin(I2C_ADDR);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);

    Serial.begin(115200);
    Serial.print("--- I2C Slave 0x");
    Serial.print(I2C_ADDR, HEX);
    Serial.println(" (Exp17) ---");
}

void loop() {
    static unsigned long last_mot = 0;
    static bool pulsed = false;
    static unsigned long last_pulse_us = 0;
    static unsigned long last_print = 0;

    if (!pulsed && (millis() - last_mot >= MOT_PERIOD_MS)) {
        digitalWrite(MOT_PIN, LOW);
        pulsed = true;
        last_pulse_us = micros();
        last_mot = millis();
    } else if (pulsed && (micros() - last_pulse_us >= PULSE_US)) {
        digitalWrite(MOT_PIN, HIGH);
        pulsed = false;
    }

    if (millis() - last_print >= 3000) {
        Serial.print("regs[0x00]=");
        Serial.print(regs[0x00], HEX);
        Serial.print(" MOT=");
        Serial.print(digitalRead(MOT_PIN));
        Serial.println();
        last_print = millis();
    }
}
