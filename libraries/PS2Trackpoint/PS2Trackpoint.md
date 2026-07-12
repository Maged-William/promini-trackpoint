# PS2Trackpoint

An Arduino library for reading from a PS/2 TrackPoint module (e.g. the
ThinkPad TrackPoint nub) using direct port manipulation for low latency.

## Hardware Connection

| TrackPoint pin | Arduino pin |
|----------------|-------------|
| CLK            | any digital |
| DAT            | any digital |

Both pins are configured as inputs with internal pull-ups enabled by
`begin()`. No external resistors are required.

## API

### `PS2Trackpoint(clkPin, datPin)`

Constructor. Saves the pin numbers; does not touch the hardware.

**Parameters**

- `clkPin` – Arduino pin number connected to the TrackPoint CLK line.
- `datPin` – Arduino pin number connected to the TrackPoint DAT line.

### `void begin()`

Sets both CLK and DAT pins to input with pull-up. Must be called before any
other methods.

### `uint8_t readByte(uint16_t timeout)`

Reads one byte from the TrackPoint by bit-banging the PS/2 protocol.

- If `timeout` is `0` (default) the call blocks forever until a byte arrives.
- If `timeout` is non-zero it is the maximum number of **microseconds** to
  wait. On timeout the method returns early and `lastError()` will indicate
  the failure.

Returns the byte read.

### `void sendByte(uint8_t data)`

Sends one byte to the TrackPoint, including parity generation and the stop
bit. Blocks until the device acknowledges the byte.

### `bool readPacket(int8_t &x, int8_t &y, uint8_t &buttons)`

Reads a standard 3-byte TrackPoint motion packet.

**Returns** `true` if a valid packet was received, `false` if the first byte
did not have bit 3 set (indicating no valid packet available).

**Output parameters**

- `x` – signed motion delta (−128…+127). Positive = right.
- `y` – signed motion delta (−128…+127). Positive = down.
- `buttons` – bitmask of the three physical buttons:

  | Bit | Button  |
  |-----|---------|
  | 0   | Left    |
  | 1   | Right   |
  | 2   | Middle  |

The values are already clamped to the valid int8_t range.

### `void reset()`

Sends the Reset command (`0xFF`) and discards the three response bytes.
Typically called once during startup.

### `void enableStreaming()`

Sends the Enable Streaming command (`0xF4`). After this, the TrackPoint
will send motion packets whenever the nub is moved.

### `lastError()`

Returns the error code from the last `readByte(...)` operation.

| Error                   | Meaning                                         |
|-------------------------|-------------------------------------------------|
| `ERR_OK`                | No error.                                       |
| `ERR_CLK_STUCK_HIGH`    | CLK stayed high during read timeout.            |
| `ERR_CLK_STUCK_LOW`     | CLK stayed low during read timeout.             |
| `ERR_DATA_BIT_TIMEOUT`  | A data bit did not complete in time.            |
| `ERR_PARITY_STOP_TIMEOUT` | Parity or stop bit handshake timed out.       |

## Usage Example

```cpp
#include <PS2Trackpoint.h>

#define CLK_PIN  7
#define DAT_PIN  3

PS2Trackpoint ps2(CLK_PIN, DAT_PIN);

void setup() {
    Serial.begin(9600);

    ps2.begin();
    delay(2000);       // wait for the device to power up
    ps2.reset();
    ps2.enableStreaming();
}

void loop() {
    int8_t x, y;
    uint8_t buttons;

    if (ps2.readPacket(x, y, buttons)) {
        Serial.print("x: "); Serial.print(x);
        Serial.print("  y: "); Serial.print(y);
        Serial.print("  buttons: "); Serial.println(buttons);
    }
}
```

## Notes

- After `begin()` the TrackPoint needs roughly 1–2 seconds to initialise
  before `reset()` and `enableStreaming()` will work.
- `readByte()` with `timeout = 0` blocks the CPU completely. For non-blocking
  usage pass a non-zero timeout and check `lastError()`.
- The library uses direct port register access (`digitalPinToPort`,
  `portModeRegister`, etc.) and is designed for classic 8-bit AVR Arduinos.
  Porting to other architectures requires replacing the register internals.
