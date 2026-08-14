#define SLEEP_ENABLED 0
#define SERIAL_LOG 0

#include <Wire.h>
#include <PS2Trackpoint.h>
#include <avr/power.h>

#define I2C_ADDR     0x42
#define BURST_ADDR   0x12
#define DEBUG_ADDR   0x03

/*
  ATtiny85 (tinyX5 core) pin map:
    0 (PB0) = SDA  (fixed by USI TWI)   physical pin 5
    2 (PB2) = SCL  (fixed by USI TWI)   physical pin 7
    1 (PB1) = MOT                       physical pin 6
    3 (PB3) = PS2 CLK                   physical pin 2
    4 (PB4) = PS2 DAT                   physical pin 3
*/
#define MOT_PIN      1
#define PS2_CLK      3
#define PS2_DAT      4

PS2Trackpoint ps2(PS2_CLK, PS2_DAT);

static volatile uint8_t cur_addr;
static volatile int8_t  burst_x = 0;
static volatile int8_t  burst_y = 0;
static unsigned long last_good_ms = 0;

void requestEvent() {
    if (cur_addr == BURST_ADDR) {
        /* Exp43 fix: destructive read (PMW3610 burst semantics) — serve the
         * sample once, then zero it. Prevents the 10ms-poll driver from
         * accumulating the same sample 2-3x and from re-serving stale data. */
        uint8_t buf[2] = { (uint8_t)burst_x, (uint8_t)burst_y };
        burst_x = 0;
        burst_y = 0;
        Wire.write(buf, 2);
    } else if (cur_addr == DEBUG_ADDR) {
        /* Exp43 debug: [status, xraw, yraw, timeouts_lo, timeouts_hi] */
        uint8_t buf[5] = { ps2.last_status, ps2.last_xraw, ps2.last_yraw,
                           (uint8_t)ps2.read_timeouts, (uint8_t)(ps2.read_timeouts >> 8) };
        Wire.write(buf, 5);
    } else {
        Wire.write(0x00);
    }
}

void receiveEvent(int len) {
    if (len > 0) {
        cur_addr = Wire.read();
    }
}

void setup() {
    clock_prescale_set(clock_div_1); /* 8 MHz (fuse CKDIV8 stays set; software override only) */
    pinMode(MOT_PIN, OUTPUT);
    digitalWrite(MOT_PIN, HIGH);

    Wire.begin(I2C_ADDR);
    Wire.onRequest(requestEvent);
    Wire.onReceive(receiveEvent);

    ps2.begin();
    ps2.setReadTimeout(2000); /* Exp43: bound PS/2 idle wait */

    last_good_ms = millis();
}

void loop() {
    int8_t x, y;
    uint8_t buttons;

    /* readPacket syncs itself to packet gaps — no fixed interval needed */
    if (ps2.readPacket(x, y, buttons)) {
        burst_x = x;
        burst_y = y;
        last_good_ms = millis();
    }

    /* stop feeding the driver stale motion after 100ms without a packet */
    if ((burst_x || burst_y) && (millis() - last_good_ms > 100)) {
        burst_x = 0;
        burst_y = 0;
    }
}
