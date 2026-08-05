#define PS2_CLK 7
#define PS2_DAT 3

uint8_t readByte() {
  uint8_t out = 0;

  // start bit (always low)
  while (digitalRead(PS2_CLK) == HIGH);
  while (digitalRead(PS2_CLK) == LOW);

  // 8 data bits, LSB first
  for (int i = 0; i < 8; i++) {
    while (digitalRead(PS2_CLK) == HIGH);
    out |= (digitalRead(PS2_DAT) << i);
    while (digitalRead(PS2_CLK) == LOW);
  }

  // parity + stop bit (ignored here)
  for (int i = 0; i < 2; i++) {
    while (digitalRead(PS2_CLK) == HIGH);
    while (digitalRead(PS2_CLK) == LOW);
  }

  return out;
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  pinMode(PS2_CLK, INPUT_PULLUP);
  pinMode(PS2_DAT, INPUT_PULLUP);
  Serial.println("Waiting for bytes...");
}

void loop() {
  uint8_t b = readByte();

  // Serial.print("0x");
  // if (b < 0x10) Serial.print("0");
  // Serial.print(b, HEX);
  // Serial.print("  (");
  // Serial.print(b);          // decimal too, if you want it
  // Serial.println(") ");
  for (int i = 7; i >= 0; i--) Serial.print((b >> i) & 1);
  Serial.print("\n");
}
