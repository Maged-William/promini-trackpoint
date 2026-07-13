// Exp06: TrackPoint Serial Reader
// Minimal sketch — just PS/2 → Serial. No SPI, no ZMK, no MOT.
// Wiring: D7=CLK, D3=DAT (per AGENTS.md)

#include <PS2Trackpoint.h>

#define CLK_PIN  7
#define DAT_PIN  3

PS2Trackpoint ps2(CLK_PIN, DAT_PIN);

void setup() {
    Serial.begin(115200);
    Serial.println("--- Exp06: TrackPoint Serial Reader ---");

    ps2.begin();
    delay(2000);

    Serial.print("Resetting...");
    ps2.reset();
    Serial.println(" done");

    Serial.print("Enabling streaming...");
    ps2.enableStreaming();
    Serial.println(" done");

    Serial.println("Ready. Move the TrackPoint nub.");
}

void loop() {
    int8_t x, y;
    uint8_t buttons;

    if (ps2.readPacket(x, y, buttons)) {
        Serial.print("X:");
        Serial.print(x);
        Serial.print(" Y:");
        Serial.print(y);
        Serial.print(" B:");
        Serial.println(buttons);
    }
}
