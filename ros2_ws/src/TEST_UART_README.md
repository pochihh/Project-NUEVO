# UART TLV Communication Test - Raspberry Pi

This directory contains the Python test script for verifying TLV protocol communication between Raspberry Pi and Arduino Mega 2560.

## Hardware Setup

### Connection (Raspberry Pi 5 ↔ Arduino Mega 2560)

**IMPORTANT: Use a bi-directional level shifter (5V ↔ 3.3V)**

| Raspberry Pi 5 | Level Shifter | Arduino Mega 2560 |
|----------------|---------------|-------------------|
| GPIO 14 (TX)   | → 3.3V → 5V → | Pin 17 (RX2)      |
| GPIO 15 (RX)   | ← 5V → 3.3V ← | Pin 16 (TX2)      |
| GND            | GND           | GND               |

**Baud Rate:** 921600

## Software Setup

### 1. Install Dependencies

```bash
# On Raspberry Pi:
pip3 install pyserial
```

### 2. Enable Serial Port

Edit `/boot/config.txt`:
```bash
sudo nano /boot/config.txt
```

Add these lines:
```
# Enable UART on GPIO 14/15
enable_uart=1
dtoverlay=disable-bt
```

Reboot:
```bash
sudo reboot
```

### 3. Verify Serial Port

Check that `/dev/ttyAMA0` exists:
```bash
ls -l /dev/ttyAMA0
```

## Running the Test

### Basic Usage

```bash
cd /home/pi/ros2_ws/src
python3 test_uart_arduino.py
```

### Custom Port/Baud

```bash
python3 test_uart_arduino.py --port /dev/ttyAMA0 --baud 921600
```

### With Verbose Output

```bash
python3 test_uart_arduino.py 2>&1 | tee uart_test.log
```

## Test Features

The test script provides:

### Automatic Features
- **Periodic Heartbeat**: Sends `SYS_HEARTBEAT` every 500ms
- **Message Decoding**: Automatically decodes and displays incoming messages
- **Statistics Tracking**: Counts messages sent/received

### Interactive Commands

While the test is running, press:

| Key | Action |
|-----|--------|
| `h` | Send heartbeat immediately |
| `s` | Print statistics |
| `e` | Enable DC motor 0 (velocity mode) |
| `d` | Disable DC motor 0 |
| `v` | Set DC motor 0 velocity to 1000 ticks/sec |
| `l` | Set NeoPixel to green (LED test) |
| `q` | Quit test |

## Expected Output

### Successful Connection

```
[UART] Opening /dev/ttyAMA0 @ 921600 baud...
[UART] Connected successfully

============================================================
UART TLV Communication Test
============================================================
Commands:
  h - Send heartbeat
  s - Print statistics
  ...
============================================================

[TX] Heartbeat #1 sent (13 bytes)
[RX] Frame from device 0x01, frame #0, 1 TLVs
  [SYS_STATUS]
    Uptime: 5.2s
    Last heartbeat: 102ms ago
    Battery: 12000mV
    5V rail: 5000mV
    Servo rail: 6000mV
    Error flags: 0x00
    Motor enable mask: 0x00
```

### Decoded Messages

The script automatically decodes and displays:
- **SYS_STATUS**: System uptime, battery voltage, rail voltages, error flags
- **SENSOR_VOLTAGE**: Battery and power rail measurements
- **SENSOR_ENCODER**: Motor positions and velocities (4 motors)

## Troubleshooting

### Error: Permission Denied

Add user to dialout group:
```bash
sudo usermod -a -G dialout $USER
logout  # Log out and back in
```

### Error: Serial Port Not Found

Check if UART is enabled:
```bash
ls -l /dev/ttyAMA0
dmesg | grep tty
```

Verify config.txt settings:
```bash
cat /boot/config.txt | grep uart
```

### Error: No Response from Arduino

1. Check wiring (TX ↔ RX crossover, GND connected)
2. Verify level shifter is working (measure voltages)
3. Confirm Arduino is running test_uart_tlv or main firmware
4. Check baud rate matches (921600 on both sides)
5. Monitor Arduino Serial0 (USB) for debug output

### Decode Errors

If you see CRC errors or decode errors:
1. CRC is **disabled** in Phase 2 (crc=False in both encoder/decoder)
2. Check for electrical noise (shorten wires, add bypass capacitors)
3. Try lower baud rate (460800 or 230400)

## Integration with ROS2

This test script can be adapted into a ROS2 node. See `serial_bridge` package for the production implementation.

Key differences between test script and ROS2 node:
- ROS2 node publishes sensor data to topics
- ROS2 node subscribes to command topics
- Proper error handling and reconnection logic
- Launch file integration

## Files

- `test_uart_arduino.py` - Main test script
- `tlvcodec/` - Python TLV encoder/decoder library
- `TLV_TypeDefs.py` - Message type constants (matches Arduino)

## Next Steps

After successful UART test:
1. Upload main firmware to Arduino (`firmware/arduino/arduino.ino`)
2. Test motor control commands (Phase 3)
3. Integrate into ROS2 serial_bridge node
4. Develop high-level control logic

## Contact

For issues or questions, contact the MAE 162 course staff.
