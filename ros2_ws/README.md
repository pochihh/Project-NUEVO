# ROS2 Packages

This directory will contain ROS2 packages for high-level robot control.

## Planned Packages

- **robot_bringup**: Launch files and configuration for system startup
- **serial_bridge**: Arduino-ROS2 communication bridge node
- **sensor_drivers**: Camera and GPS driver nodes
- **robot_control**: High-level control algorithms and navigation
- **robot_description**: URDF models and visualization

## Development

Build the workspace from the parent directory:
```bash
cd ros2_ws
colcon build
source install/setup.bash
```

Target ROS2 distribution: Humble or Jazzy


### Raspberry Pi 5 GPIO Allocation

| GPIO | Function | Notes |
|------|----------|-------|
| GPIO2 (SDA1) | I2C Data | Qwiic connector |
| GPIO3 (SCL1) | I2C Clock | Qwiic connector |
| GPIO14 (TXD0) | UART TX to Arduino | Via level shifter |
| GPIO15 (RXD0) | UART RX from Arduino | Via level shifter |
| Other | GPIO Breakout | Unused pins to terminals |
