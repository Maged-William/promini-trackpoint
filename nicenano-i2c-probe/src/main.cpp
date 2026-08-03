/*
  Exp43 — NiceNano I2C probe (diagnostic).

  Bypasses ZMK/driver/shell entirely: a plain Arduino I2C master on the
  nice!nano reads the ATtiny85 slave (0x42) and prints the exact bytes.

  Reads:
    reg 0x12 (burst)  -> [x, y]              (what the ZMK driver consumes)
    reg 0x03 (debug)  -> [status, xraw, yraw, timeouts_lo, timeouts_hi]

  Combined transaction via endTransmission(false) (repeated START, no STOP)
  then requestFrom() — matches what the ATtiny85 USI slave expects.

  Serial command "BOOTLOADER" drops back into the adafruit bootloader so the
  ZMK firmware can be restored afterwards.
*/

#include <Arduino.h>
#include <Wire.h>

#define I2C_ADDR     0x42
#define BURST_REG    0x12
#define DEBUG_REG    0x03

#define SDA_PIN  PIN_017   /* P0.17 -> ATtiny85 PB0 (SDA) */
#define SCL_PIN  PIN_020   /* P0.20 -> ATtiny85 PB2 (SCL) */

static int i2cReadReg(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;   /* NACK / other error */
  uint8_t got = Wire.requestFrom(I2C_ADDR, (size_t)len, true);
  if (got != len) return -2;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return 0;
}

static void checkSerialCommand() {
  if (!Serial.available()) return;

  static char buf[32];
  static uint8_t i = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || i >= sizeof(buf) - 1) {
      buf[i] = '\0';
      i = 0;
      if (strcmp(buf, "BOOTLOADER") == 0) {
        Serial.println("OK_RESET");
        Serial.flush();
        delay(20);
        enterSerialDfu();
      }
    } else if (c != '\r') {
      buf[i++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  Wire.setClock(100000);
  Serial.println("PROBE: i2c 0x42 burst+debug");
}

void loop() {
  checkSerialCommand();

  uint8_t burst[2];
  uint8_t dbg[5];

  int r1 = i2cReadReg(BURST_REG, burst, 2);
  int r2 = i2cReadReg(DEBUG_REG, dbg, 5);

  if (r1 == 0 && r2 == 0) {
    int8_t x = (int8_t)burst[0];
    int8_t y = (int8_t)burst[1];
    uint16_t tmo = (uint16_t)dbg[3] | ((uint16_t)dbg[4] << 8);

    Serial.print("B x="); Serial.print(x);
    Serial.print(" y="); Serial.print(y);
    Serial.print(" | D s="); Serial.print(dbg[0], HEX);
    Serial.print(" xr="); Serial.print(dbg[1], HEX);
    Serial.print(" yr="); Serial.print(dbg[2], HEX);
    Serial.print(" tmo="); Serial.println(tmo);
  } else {
    Serial.print("ERR r1="); Serial.print(r1);
    Serial.print(" r2="); Serial.println(r2);
  }

  delay(10); /* Exp43 fix attempt: driver-equivalent 10ms poll to reproduce x=0 / always-up */
}
