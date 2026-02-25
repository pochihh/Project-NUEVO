"""
Configuration for NUEVO Bridge
"""
import os

# Serial port configuration
SERIAL_PORT = os.getenv("NUEVO_SERIAL_PORT", "/dev/ttyAMA0")  # RPi GPIO 14/15
SERIAL_BAUD = int(os.getenv("NUEVO_SERIAL_BAUD", "1000000"))  # 1 Mbps (matches RPI_BAUD_RATE in config.h)
SERIAL_TIMEOUT = 0.1  # seconds (non-blocking)

# Heartbeat configuration
HEARTBEAT_INTERVAL = 0.2  # seconds (200ms, same as test_uart_arduino.py)

# TLV protocol configuration
DEVICE_ID = 0x01  # RPi device ID
ENABLE_CRC = True

# WebSocket configuration
WS_HEARTBEAT_INTERVAL = 1.0  # seconds (ping to keep connection alive)

# Mock mode (for development without hardware)
MOCK_MODE = os.getenv("NUEVO_MOCK", "0") == "1"

# Connection statistics broadcast interval
STATS_INTERVAL = 1.0  # seconds
