// Exp45: TrackPoint Serial Reader — sanity re-read
// Reads 3-byte motion packets via PS2Trackpoint library.
// This IC streams on power-up (no reset/enableStreaming needed).
// Wiring: CLK=D7, DAT=D3 (matches production slave Exp44 + AGENTS.md)

#include <PS2Trackpoint.h>

#define CLK_PIN  7
#define DAT_PIN  3

PS2Trackpoint ps2(CLK_PIN, DAT_PIN);

void setup() {
    Serial.begin(115200);
    Serial.println("--- Exp45: TrackPoint Serial Reader ---");

    ps2.begin();
    delay(2000);

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
