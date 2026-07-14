#include <PS2Trackpoint.h>

#define PS2_CLK 3
#define PS2_DAT 7

PS2Trackpoint ps2(PS2_CLK, PS2_DAT);

char buf[80];
unsigned long pkt_count = 0;
unsigned long stray_count = 0;
unsigned long last_pkt_ms = 0;

void setup() {
  ps2.begin();

  Serial.begin(38400);

  delay(2000);

  Serial.println(F("=== PS/2 Trackpoint Diagnostic ==="));
  Serial.println(F("Status byte: bit3(1=packet), bit4(Xsign), bit5(Ysign), bit6(Xovf), bit7(Yovf), bit0-2(buttons)"));
  Serial.println(F("Format: PKT N: S=0x%02X X=%4d Y=%4d  raw: s=%02x x=%02x y=%02x  btn=%c%c%c"));
  Serial.println(F("--- START ---"));
  Serial.flush();
}

void loop() {
  uint8_t b = ps2.readByte(80000);

  if (b == 0 && ps2.lastError() == PS2Trackpoint::ERR_CLK_STUCK_HIGH) {
    if (millis() - last_pkt_ms > 5000) {
      Serial.println(F("[IDLE] No clock activity for 5s — check wiring"));
      last_pkt_ms = millis();
    }
    return;
  }
  if (b == 0xFF || b == 0xFE || b == 0xFD || b == 0xFC || b == 0xFB) {
    snprintf(buf, sizeof(buf), "[TIMEOUT] byte=%02x at pkt=%lu stray=%lu",
      b, pkt_count, stray_count);
    Serial.println(buf);
    return;
  }

  if (b & 0x08) {
    uint8_t s = b;
    uint8_t xraw = ps2.readByte(80000);
    uint8_t yraw = ps2.readByte(80000);

    int x = (int)xraw - ((s << 4) & 0x100);
    int y = (int)yraw - ((s << 3) & 0x100);

    if (s & 0x40) x = (x < 0) ? -128 : 127;
    if (s & 0x80) y = (y < 0) ? -128 : 127;
    if (x > 127) x = 127;
    if (x < -128) x = -128;
    if (y > 127) y = 127;
    if (y < -128) y = -128;

    if (pkt_count < 50 || (x != 0 || y != 0) || pkt_count % 100 == 0) {
      snprintf(buf, sizeof(buf),
        "PKT %5lu: S=0x%02X X=%4d Y=%4d  raw: %02x %02x %02x  btn=%c%c%c",
        pkt_count, s, x, y, s, xraw, yraw,
        (s & 1) ? 'L' : '-',
        (s & 2) ? 'R' : '-',
        (s & 4) ? 'M' : '-');
      Serial.println(buf);
    }

    pkt_count++;
    last_pkt_ms = millis();
  } else {
    stray_count++;
    if (stray_count <= 10 || stray_count % 50 == 0) {
      snprintf(buf, sizeof(buf), "[STRAY] byte=%02x (bit3=0) pkt=%lu stray=%lu",
        b, pkt_count, stray_count);
      Serial.println(buf);
    }
  }
}
