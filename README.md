# Project NUEVO
![](/assets/NUEVO.png)

Lab project material for the MAE 162 robotics course (Winter/Spring 2026).

## Overview

A modular two-wheeled mobile robot platform designed for hands-on robotics education. Features customizable manipulators and a dual-layer control architecture for teaching embedded systems, ROS2, and mechatronics fundamentals.

## System Architecture

**Low-Level Control (Arduino)**
- Real-time motor control (DC, stepper, servo)
- GPIO, LEDs, and button inputs
- UART communication to Raspberry Pi

**High-Level Control (Raspberry Pi 5 + ROS2)**
- Decision-making and path planning
- Camera and GPS sensor processing
- ROS2 node orchestration

**Custom PCB**
- Integrates Arduino, motor drivers, and power management
- Standardized interface for educational reproducibility

## Repository Structure

```
├── firmware/      Arduino firmware (.ino files)
├── ros2_ws/       ROS2 workspace
├── TLV protocol/  TLV type definitions used for both Arduino and ROS2 communication
├── NUEVO board/   PCB design files (schematics, layouts, BOM)
├── mechanical/    CAD files for chassis and manipulators
├── NUEVO UI/      User interface code (if applicable)
├── notes/         Notes and documentation for development
└── docs/          Documentation and reports
```




## Key Documents

| Document | Purpose |
|----------|---------|
| [DESIGN_GUIDELINES.md](DESIGN_GUIDELINES.md) | Cross-project conventions: numbering, naming, units, coordinate system, workflow |
| [COMMUNICATION_PROTOCOL.md](COMMUNICATION_PROTOCOL.md) | TLV v2.0 wire protocol specification |
| [firmware/README.md](firmware/README.md) | Arduino firmware architecture, build instructions |
| [NUEVO board/SPECIFICATIONS.md](NUEVO%20board/SPECIFICATIONS.md) | PCB hardware specifications |

## Technologies

- **Embedded**: Arduino (C/C++)
- **High-Level**: ROS2 (Python/C++), Raspberry Pi 5
- **Communication**: UART serial protocol
- **Sensors**: Camera, GPS, encoders
- **Hardware**: Custom PCB, stepper/servo motors

