// Exp06: TrackPoint Serial Reader — debug mode
// Reads raw PS/2 bytes to diagnose why packets aren't arriving.
// Per the reference (randalea.de/~db7), this IC:
//   - Streams immediately on power-up (no init needed)
//   - Rejects all host commands
// So we skip reset/enableStreaming and just listen.
// Wiring: D7=CLK, D3=DAT (per AGENTS.md)
// NOTE: This IC needs 4.7kΩ pull-ups on CLK and DAT to VCC.

#include <PS2Trackpoint.h>

#define CLK_PIN  7
#define DAT_PIN  3

PS2Trackpoint ps2(CLK_PIN, DAT_PIN);

void setup() {
    Serial.begin(115200);
    Serial.println("--- Exp06 debug: raw PS/2 bytes ---");

    ps2.begin();
    delay(2000);  // let the TrackPoint power up

    Serial.println("Listening for PS/2 bytes...");
}

void loop() {
    // Try to read a byte with 50ms timeout
    uint8_t b = ps2.readByte(50000);
    PS2Trackpoint::Error err = ps2.lastError();

    if (err == PS2Trackpoint::ERR_OK) {
        Serial.print("BYTE: 0x");
        if (b < 0x10) Serial.print("0");
        Serial.println(b, HEX);
    } else {
        // Only print timeout errors occasionally to avoid spam
        static unsigned long last_err_print = 0;
        if (millis() - last_err_print > 2000) {
            Serial.print("ERR: ");
            switch (err) {
                case PS2Trackpoint::ERR_CLK_STUCK_HIGH: Serial.println("CLK_STUCK_HIGH"); break;
                case PS2Trackpoint::ERR_CLK_STUCK_LOW:  Serial.println("CLK_STUCK_LOW");  break;
                case PS2Trackpoint::ERR_DATA_BIT_TIMEOUT: Serial.println("BIT_TIMEOUT"); break;
                case PS2Trackpoint::ERR_PARITY_STOP_TIMEOUT: Serial.println("PARITY_STOP"); break;
                default: Serial.println("UNKNOWN"); break;
            }
            last_err_print = millis();
        }
    }
}
