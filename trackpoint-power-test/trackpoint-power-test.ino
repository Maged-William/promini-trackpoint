// Exp09: TrackPoint Power Test — Phase 1 (manual D4 on/off)
// D4 → 1kΩ → NPN Base (HIGH=TrackPoint GND connected, powered ON)
// D7=CLK, D3=DAT — PS/2 to TrackPoint
// D19 ← TTP223 OUT (Phase 2)
// Serial commands: 'on', 'off', 'status'

#include <PS2Trackpoint.h>

#define POWER_PIN    4
#define PS2_CLK      3
#define PS2_DAT      2
#define TOUCH_PIN    19

PS2Trackpoint ps2(PS2_CLK, PS2_DAT);

static bool powered   = true;
static unsigned long pkt_count  = 0;
static unsigned long last_print = 0;
static int8_t  last_x = 0, last_y = 0;
static uint8_t last_buttons = 0;

void power_on() {
    digitalWrite(POWER_PIN, HIGH);
    delay(10);
    ps2.begin();
    delay(100);
    powered = true;
    Serial.println(F("[POWER] ON"));
}

void power_off() {
    powered = false;
    digitalWrite(POWER_PIN, LOW);
    pinMode(PS2_CLK, OUTPUT); digitalWrite(PS2_CLK, LOW);
    pinMode(PS2_DAT, OUTPUT); digitalWrite(PS2_DAT, LOW);
    pkt_count = 0;
    last_x = 0; last_y = 0; last_buttons = 0;
    Serial.println(F("[POWER] OFF"));
}

void print_status() {
    Serial.print(F("[STATUS] powered="));
    Serial.print(powered ? "ON" : "OFF");
    Serial.print(F("  touch="));
    Serial.print(digitalRead(TOUCH_PIN));
    Serial.print(F("  pkts="));
    Serial.println(pkt_count);
}

void setup() {
    pinMode(POWER_PIN, OUTPUT);

    pinMode(TOUCH_PIN, INPUT);

    Serial.begin(38400);

    Serial.println(F("Exp09: on/off/s/status"));

    power_on();
    ps2.begin();
    delay(2000);
}

void loop() {
    // ── Read PS/2 packets ──
    if (powered) {
        int8_t x, y;
        uint8_t buttons;
        if (ps2.readPacket(x, y, buttons)) {
            pkt_count++;
            last_x = x;
            last_y = y;
            last_buttons = buttons;
        }
    }

    // ── Serial commands ──
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() == 0) {
            // ignore empty lines
        } else if (cmd == "on") {
            power_on();
        } else if (cmd == "off") {
            power_off();
        } else if (cmd == "s" || cmd == "status") {
            print_status();
        } // unknown commands silently ignored
    }

    // ── Periodic X/Y log ──
    if (millis() - last_print >= 500) {
        Serial.print(last_x);
        Serial.print('\t');
        Serial.print(last_y);
        Serial.print('\t');
        Serial.println(last_buttons);
        last_print = millis();
    }
}
