#!/usr/bin/env python3
"""
I2C Scanner - Detect all I2C devices on the bus

This script scans all possible I2C addresses (0x03 to 0x77)
and reports which devices respond.

Usage:
    python3 i2c_scanner.py [bus_number]

    bus_number: I2C bus to scan (default: 1)
                - Raspberry Pi 5: usually bus 1
                - Check with: ls /dev/i2c-*

Examples:
    python3 i2c_scanner.py        # Scan bus 1
    python3 i2c_scanner.py 0      # Scan bus 0

Requirements:
    pip install smbus2
"""

import sys
import time

try:
    from smbus2 import SMBus
except ImportError:
    print("Error: smbus2 not installed")
    print("Install with: pip install smbus2")
    sys.exit(1)


def scan_i2c_bus(bus_number=1):
    """
    Scan I2C bus for devices

    Args:
        bus_number: I2C bus number (default: 1)

    Returns:
        List of addresses where devices were found
    """
    print(f"Scanning I2C bus {bus_number}...")
    print("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f")

    found_devices = []

    try:
        with SMBus(bus_number) as bus:
            for row in range(0, 0x80, 16):
                print(f"{row:02x}: ", end="")

                for col in range(16):
                    address = row + col

                    # Skip reserved addresses
                    if address < 0x03 or address > 0x77:
                        print("   ", end="")
                        continue

                    try:
                        # Try to read from the device
                        bus.read_byte(address)
                        print(f"{address:02x} ", end="")
                        found_devices.append(address)
                    except OSError:
                        # No device at this address
                        print("-- ", end="")

                print()  # New line after each row

        return found_devices

    except FileNotFoundError:
        print(f"\nError: I2C bus {bus_number} not found!")
        print("Available I2C buses:")
        import os
        for dev in os.listdir('/dev'):
            if dev.startswith('i2c-'):
                print(f"  /dev/{dev}")
        return []

    except PermissionError:
        print(f"\nError: Permission denied for /dev/i2c-{bus_number}")
        print("Run with sudo or add user to i2c group:")
        print(f"  sudo usermod -aG i2c $USER")
        print("Then log out and log back in")
        return []


def identify_common_devices(address):
    """
    Identify common I2C devices by address

    Args:
        address: I2C address (integer)

    Returns:
        String describing possible device
    """
    common_devices = {
        0x20: "PCF8574 I/O Expander / MCP23008",
        0x21: "PCF8574 I/O Expander / MCP23008",
        0x27: "PCF8574 LCD Backpack",
        0x3C: "SSD1306 OLED Display",
        0x3D: "SSD1306 OLED Display",
        0x40: "PCA9685 PWM Driver / Si7021 Humidity",
        0x48: "ADS1115 ADC / TMP102 Temperature",
        0x49: "ADS1115 ADC / TSL2561 Light",
        0x50: "AT24C32 EEPROM",
        0x51: "AT24C32 EEPROM",
        0x52: "Nunchuk controller",
        0x53: "ADXL345 Accelerometer",
        0x68: "MPU6050 / DS1307 RTC / ICM-20948",
        0x69: "MPU6050 / ICM-20948",
        0x70: "PCA9685 PWM Driver (All Call)",
        0x76: "BMP280 / BME280 Pressure",
        0x77: "BMP280 / BME280 Pressure",
    }

    return common_devices.get(address, "Unknown device")


def main():
    """Main function"""
    # Get bus number from command line argument
    bus_number = 1  # Default to bus 1

    if len(sys.argv) > 1:
        try:
            bus_number = int(sys.argv[1])
        except ValueError:
            print(f"Error: Invalid bus number '{sys.argv[1]}'")
            print(f"Usage: {sys.argv[0]} [bus_number]")
            sys.exit(1)

    print("=" * 60)
    print("I2C Bus Scanner")
    print("=" * 60)
    print()

    # Scan the bus
    found_devices = scan_i2c_bus(bus_number)

    print()
    print("=" * 60)

    if found_devices:
        print(f"Found {len(found_devices)} device(s):")
        print()
        for addr in found_devices:
            device_name = identify_common_devices(addr)
            print(f"  0x{addr:02X} ({addr:3d}) - {device_name}")
    else:
        print("No I2C devices found!")
        print()
        print("Troubleshooting:")
        print("  1. Check I2C is enabled: sudo raspi-config")
        print("  2. Check connections (SDA, SCL, VCC, GND)")
        print("  3. Verify pull-up resistors (typically 4.7kΩ)")
        print("  4. Try a different bus number")

    print("=" * 60)


if __name__ == "__main__":
    main()
