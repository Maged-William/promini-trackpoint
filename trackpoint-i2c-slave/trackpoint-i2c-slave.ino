#include <Wire.h>

#define I2C_ADDR      0x42
#define MOT_PIN       14
#define LED_PIN       13
#define MOT_PERIOD_MS 20
#define PULSE_US      100

#define BURST_ADDR    0x12

static uint8_t regs[128];
static uint8_t current_addr;

// Segments: dx, dy, steps, skip (extra MOT cycles to idle between updates)
// update_interval = (skip + 1) * MOT_PERIOD_MS
// Speed = dx / update_interval
static const int8_t rect_seg[4][4] = {
    {  1,  0, 200, 1  },   // Right 0.1x: +1/40ms = 25 px/s (smooth)
    {  0, 16,  12, 0  },   // Down    4x: +16/20ms = 800 px/s (halved)
    { -4,  0, 50,  0  },   // Left    1x: -4/20ms = 200 px/s
    {  0, -4, 50,  0  },   // Up      1x: -4/20ms = 200 px/s
};

static uint8_t segment = 0;
static uint8_t step_in_seg = 0;
static uint8_t skip_cnt = 0;

static void advance_rect(void) {
    int8_t skip = rect_seg[segment][3];
    if (skip_cnt < skip) {
        skip_cnt++;
        return;
    }
    skip_cnt = 0;

    int8_t dx = rect_seg[segment][0];
    int8_t dy = rect_seg[segment][1];
    regs[0x02] = (uint8_t)(int8_t)dx;
    regs[0x03] = (uint8_t)(int8_t)dy;

    step_in_seg++;
    if (step_in_seg >= rect_seg[segment][2]) {
        segment = (segment + 1) & 3;
        step_in_seg = 0;
        skip_cnt = 0;
    }
}

void requestEvent() {
    if (current_addr == BURST_ADDR) {
        Wire.write(&regs[0x02], 2);
        regs[0x02] = 0;
        regs[0x03] = 0;
    } else {
        Wire.write(&regs[current_addr], 1);
    }
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

    Wire.begin(I2C_ADDR);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);

    Serial.begin(115200);
    Serial.println("--- I2C Slave Exp18 speed variants ---");
}

void loop() {
    static unsigned long last_mot = 0;
    static bool          pulsed  = false;
    static unsigned long last_pulse_us = 0;

    if (!pulsed && (millis() - last_mot >= MOT_PERIOD_MS)) {
        advance_rect();
        digitalWrite(MOT_PIN, LOW);
        pulsed = true;
        last_pulse_us = micros();
        last_mot = millis();
    } else if (pulsed && (micros() - last_pulse_us >= PULSE_US)) {
        digitalWrite(MOT_PIN, HIGH);
        pulsed = false;
    }
}
