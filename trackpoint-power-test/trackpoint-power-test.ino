// Exp09: TrackPoint Power Test — Phase 1 (manual D4 on/off)
// D4 → 1kΩ → NPN Base (HIGH=TrackPoint GND connected, powered ON)
// D7=CLK, D3=DAT — PS/2 to TrackPoint
// D19 ← TTP223 OUT (Phase 2)
// Serial commands: 'on', 'off', 'status'

#include <PS2Trackpoint.h>

#define POWER_PIN    4
#define PS2_CLK      7
#define PS2_DAT      3
#define TOUCH_PIN    19

PS2Trackpoint ps2(PS2_CLK, PS2_DAT);

static bool powered   = true;
static unsigned long pkt_count  = 0;
static unsigned long last_print = 0;

void power_on() {
    digitalWrite(POWER_PIN, HIGH);
    powered = true;
    Serial.println(F("[POWER] ON"));
    delay(100);  // let TrackPoint stabilize
}

void power_off() {
    digitalWrite(POWER_PIN, LOW);
    powered = false;
    pkt_count = 0;
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

    Serial.begin(115200);

    Serial.println();
    Serial.println(F("=== Exp09 Phase 1: Manual Power Control ==="));
    Serial.println(F("Commands: on / off / status"));
    Serial.println();

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
        }
    }

    // ── Serial commands ──
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') continue;

        String cmd;
        cmd += c;
        while (Serial.available()) {
            char n = Serial.peek();
            if (n == '\n' || n == '\r') break;
            cmd += (char)Serial.read();
        }

        if (cmd == "on") {
            power_on();
        } else if (cmd == "off") {
            power_off();
        } else if (cmd == "status") {
            print_status();
        } else {
            Serial.print(F("[CMD] unknown: "));
            Serial.println(cmd);
        }
    }

    // ── Periodic status ──
    if (millis() - last_print >= 2000) {
        print_status();
        last_print = millis();
    }
}
