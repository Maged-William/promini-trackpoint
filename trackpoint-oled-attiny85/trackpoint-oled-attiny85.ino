#define SLEEP_ENABLED 1
#define SERIAL_LOG 0 /* ATtiny85 has no UART and all 6 GPIOs are in use */

/*
  Exp42 — OLED isolation test for the ATtiny85 trackpoint.

  Reads the PS/2 trackpoint and shows live X/Y on a 0.91" SSD1306 OLED
  (128x32). The OLED rides the USI I2C pins in MASTER mode — the same
  PB0/PB2 wires that will later go to the NiceNano slave bus.

  Pins:
    0 (PB0) = SDA (USI I2C master) -> OLED SDA      physical pin 5
    2 (PB2) = SCL                  -> OLED SCL      physical pin 7
    3 (PB3) = PS2 CLK                               physical pin 2
    4 (PB4) = PS2 DAT                               physical pin 3
    1 (PB1) = unused in this test build             physical pin 6
*/

#include <Tiny4kOLED.h>
#include <PS2Trackpoint.h>
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

static int8_t        last_x = 0;
static int8_t        last_y = 0;
static unsigned long last_motion_ms = 0;

void showXY(int8_t x, int8_t y) {
    oled.clear();
    oled.setCursor(0, 0);
    oled.print(F("X: "));
    oled.print(x);
    oled.setCursor(0, 2);
    oled.print(F("Y: "));
    oled.print(y);
    oled.switchFrame();
}

void showSleep() {
    oled.clear();
    oled.setCursor(32, 1);
    oled.print(F("SLEEP"));
    oled.switchFrame();
}

void setup() {
    oled.begin();
    oled.setFont(FONT8X16);
    oled.clear();
    oled.on();

    ps2.begin();
    last_motion_ms = millis();

    showXY(0, 0);
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
            if (abs(x) > DEADBAND || abs(y) > DEADBAND) {
                last_motion_ms = millis();
            }
            if (x != last_x || y != last_y) {
                last_x = x;
                last_y = y;
                showXY(x, y);
            }
        }
    }

#if SLEEP_ENABLED
    if (millis() - last_motion_ms >= IDLE_TIMEOUT_MS) {
        showSleep();

        pinMode(PS2_CLK, OUTPUT);
        digitalWrite(PS2_CLK, LOW);

        while (1) {
            MCUSR &= ~(1<<WDRF);
            WDTCR |= (1<<WDCE) | (1<<WDE);
            WDTCR = (1<<WDIE) | (1<<WDP0) | (1<<WDP2);
            set_sleep_mode(SLEEP_MODE_IDLE);
            sleep_enable();
            sei();
            sleep_cpu();
            sleep_disable();
            WDTCR |= (1<<WDCE) | (1<<WDE);
            WDTCR = 0;

            pinMode(PS2_CLK, INPUT_PULLUP);

            if (ps2.readPacket(x, y, buttons)) {
                if (!(abs(x) >= 127 || abs(y) >= 127 || abs(x) > MAX_DELTA || abs(y) > MAX_DELTA)) {
                    if (abs(x) > DEADBAND || abs(y) > DEADBAND) {
                        last_x = x;
                        last_y = y;
                        last_motion_ms = millis();
                        showXY(x, y);
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
