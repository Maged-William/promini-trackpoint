#define SLEEP_ENABLED 0 /* Exp43 debug: stay awake, continuous live display */
#define SERIAL_LOG 0 /* ATtiny85 has no UART and all 6 GPIOs are in use */

/*
  Exp43 — OLED debug: show the raw PS/2 packet bytes + parsed X/Y.

  The slave build reads xraw=0 from this trackpoint while the OLED build
  showed X values. This debug displays the raw status/xraw/yraw bytes so
  we can see exactly what the trackpoint sends standalone (no I2C slave).

  Pins:
    0 (PB0) = SDA (USI I2C master) -> OLED SDA      physical pin 5
    2 (PB2) = SCL                  -> OLED SCL      physical pin 7
    3 (PB3) = PS2 CLK                               physical pin 2
    4 (PB4) = PS2 DAT                               physical pin 3
    1 (PB1) = unused in this test build             physical pin 6
*/

#include <Tiny4kOLED.h>
#include <PS2Trackpoint.h>
#include <avr/power.h>
#if SLEEP_ENABLED
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#endif

#define PS2_CLK      3
#define PS2_DAT      4

#define READ_INTERVAL_MS 20
#define MAX_DELTA 25
#define DEADBAND 3
#define IDLE_TIMEOUT_MS 5000

PS2Trackpoint ps2(PS2_CLK, PS2_DAT);

static int8_t  last_x = 0;
static int8_t  last_y = 0;
static uint8_t last_s = 0xFF;
static uint8_t last_xr = 0xFF;
static uint8_t last_yr = 0xFF;

void showPacket() {
    oled.clear();
    oled.setCursor(0, 0);
    oled.print(F("S:"));
    oled.print(ps2.last_status, HEX);
    oled.print(F(" X:"));
    oled.print(ps2.last_xraw, HEX);
    oled.print(F(" Y:"));
    oled.print(ps2.last_yraw, HEX);
    oled.setCursor(0, 2);
    oled.print(F("X:"));
    oled.print(last_x);
    oled.print(F(" Y:"));
    oled.print(last_y);
    oled.switchFrame();
}

void setup() {
    clock_prescale_set(clock_div_1); /* 8 MHz (fuse CKDIV8 stays set; software override only) */
    oled.begin();
    oled.setFont(FONT8X16);
    oled.clear();
    oled.on();

    ps2.begin();

    showPacket();
}

void loop() {
    static unsigned long last_read_ms = 0;

    int8_t x, y;
    uint8_t buttons;

    unsigned long now = millis();
    if (now - last_read_ms >= READ_INTERVAL_MS) {
        last_read_ms = now;
        if (ps2.readPacket(x, y, buttons)) {
            if (abs(x) >= 127 || abs(y) >= 127 || abs(x) > MAX_DELTA || abs(y) > MAX_DELTA) {
                x = 0; y = 0;
            }
            last_x = x;
            last_y = y;
            if (ps2.last_status != last_s || ps2.last_xraw != last_xr || ps2.last_yraw != last_yr) {
                last_s = ps2.last_status;
                last_xr = ps2.last_xraw;
                last_yr = ps2.last_yraw;
                showPacket();
            }
        }
    }
}
