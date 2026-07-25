#include <Wire.h>

#define I2C_ADDR      0x42
#define MOT_PIN       14
#define LED_PIN       13
#define MOT_PERIOD_MS 20
#define PULSE_US      100

#define BURST_ADDR    0x12

static uint8_t regs[128];
static uint8_t current_addr;

// Rectangle: uniform +/-1 to test all 4 directions
static const int8_t rect_seg[4][3] = {
    {  1,  0, 200 },   // A->B: Right  (+1,  0) x 200
    {  0,  1, 200 },   // B->C: Down   ( 0, +1) x 200
    { -1,  0, 200 },   // C->D: Left   (-1,  0) x 200
    {  0, -1, 200 },   // D->A: Up     ( 0, -1) x 200
};

static uint8_t segment = 0;
static uint8_t step_in_seg = 0;

static void encode_delta_12(int8_t dx, int8_t dy) {
    uint16_t dx_12 = (dx >= 0) ? (uint16_t)dx : (uint16_t)(4096 + dx);
    uint16_t dy_12 = (dy >= 0) ? (uint16_t)dy : (uint16_t)(4096 + dy);

    regs[0x03] = (uint8_t)(dx_12 & 0xFF);         // X_L
    regs[0x04] = (uint8_t)(dy_12 & 0xFF);         // Y_L
    regs[0x05] = ((uint8_t)(dx_12 >> 4) & 0xF0)   // XY_H
               | ((uint8_t)(dy_12 >> 8) & 0x0F);
    regs[0x02] = (dx || dy) ? 0x01 : 0x00;        // MOTION
}

static void advance_rect(void) {
    int8_t dx = rect_seg[segment][0];
    int8_t dy = rect_seg[segment][1];
    encode_delta_12(dx, dy);

    step_in_seg++;
    if (step_in_seg >= rect_seg[segment][2]) {
        segment = (segment + 1) & 3;
        step_in_seg = 0;
    }
}

void requestEvent() {
    if (current_addr == BURST_ADDR) {
        Wire.write(&regs[0x02], 7);
        regs[0x02] = 0x00;
        regs[0x03] = 0x00;
        regs[0x04] = 0x00;
        regs[0x05] = 0x00;
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

    regs[0x00] = 0x3E;  // Product ID
    regs[0x01] = 0x01;  // Revision ID
    regs[0x06] = 0x00;  // SQUAL
    regs[0x07] = 0x00;  // Shutter_H
    regs[0x08] = 0x00;  // Shutter_L

    advance_rect();

    Wire.begin(I2C_ADDR);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);

    Serial.begin(115200);
    Serial.println("--- I2C Slave 0x42 Exp18 Rectangle ---");
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
