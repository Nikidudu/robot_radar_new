# USB State Packet – PC Receive Guide

The STM32 sends a **state number** (0–20) over USB at 10 Hz. This doc describes how to read it on the PC.

---

## Frame Format

| Byte | Content |
|------|---------|
| 0 | Magic: `0x7F` |
| 1 | Packet ID: `21` (ID_STATE) |
| 2 | **State** (0–20) |
| 3–4 | CRC (2 bytes, little-endian) |

**Total frame size: 5 bytes**

---

## How to Read the State

1. Read bytes from the USB CDC port (e.g. COM3, /dev/ttyACM0).
2. Look for `0x7F` (magic).
3. Next byte must be `21` (packet ID).
4. The byte after that is the **state** (0–20).

```
Frame: [0x7F] [0x15] [state] [crc_lo] [crc_hi]
                ^      ^
                |      +-- state number (0–20)
                +-- packet ID 21 (0x15)
```

---

## State Values

| State | Meaning |
|-------|---------|
| 0 | Game not started |
| 1 | Red, game started, health > 80% |
| 2 | Red, game started, health < 40% |
| 3–10 | Red spare (reserved) |
| 11 | Blue, game started, health > 80% |
| 12 | Blue, game started, health < 40% |
| 13–20 | Blue spare (reserved) |

---

## Example (Python)

```python
import serial

ser = serial.Serial("COM3", 115200)  # or /dev/ttyACM0 on Linux

while True:
    b = ser.read(1)
    if b[0] == 0x7F and ser.in_waiting >= 4:
        frame = b + ser.read(4)
        if frame[1] == 21:  # ID_STATE
            state = frame[2]
            print(f"state = {state}")
```

---

## Example (C/C++)

```c
// After reading 0x7F, next 4 bytes:
// buf[0] = packet_id (21)
// buf[1] = state      <-- READ THIS
// buf[2], buf[3] = crc

uint8_t state = buf[1];
```

---

## Constants (for PC code)

```c
#define USB_MAGIC_BYTE  0x7F
#define ID_STATE        21
#define STATE_OFFSET    2   /* byte index in frame, after magic+id */
```
