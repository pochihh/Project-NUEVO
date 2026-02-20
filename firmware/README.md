# Arduino Firmware

This directory contains the Arduino Mega 2560 firmware for the MAE 162 robot platform. The firmware handles all real-time control: motor PWM, encoder counting, sensor reading, and communication with the Raspberry Pi over a custom TLV (Type-Length-Value) protocol.

## Directory Structure

```
firmware/
├── arduino/                  # Main firmware sketch (upload this to the robot)
│   ├── arduino.ino           # Entry point (setup/loop)
│   └── src/                  # All firmware source code
│       ├── config.h          # Compile-time configuration (edit this first)
│       ├── pins.h            # All GPIO pin definitions
│       ├── drivers/          # Hardware abstractions
│       │   ├── DCMotor       # H-bridge PWM + direction control
│       │   ├── StepperMotor  # STEP/DIR/ENABLE stepper driver interface
│       │   ├── ServoController  # PCA9685 I2C servo controller
│       │   ├── UARTDriver    # TLV-framed serial comms (RPi link)
│       │   ├── NeoPixelDriver   # WS2812B RGB status LED
│       │   ├── IMUDriver     # ICM-20948 9-DoF IMU
│       │   ├── LidarDriver   # Garmin LIDAR-Lite v4 (I2C)
│       │   └── UltrasonicDriver # SparkFun Qwiic HC-SR04 (I2C)
│       ├── modules/          # Higher-level subsystems
│       │   ├── EncoderCounter   # Interrupt-driven quadrature counting (2x/4x)
│       │   ├── VelocityEstimator # Edge-timing velocity with moving-average filter
│       │   ├── SensorManager    # IMU fusion (Madgwick AHRS) + mag calibration
│       │   ├── PersistentStorage # EEPROM read/write (wheel geometry, mag cal)
│       │   ├── MessageCenter    # TLV packet assembly and dispatch
│       │   ├── StepperManager   # Multi-motor step sequencing (Timer3 ISR)
│       │   └── UserIO           # Buttons, LEDs, NeoPixel patterns
│       └── lib/              # Vendored third-party libraries
│           ├── Fusion/       # Madgwick AHRS (x-io Technologies)
│           ├── tlvcodec      # TLV packet encoder/decoder
│           ├── PCA9685       # I2C PWM driver
│           ├── Adafruit_NeoPixel
│           ├── SparkFun_9DoF_IMU_Breakout (ICM-20948)
│           ├── SparkFun_Garmin_LIDAR-Lite_v4
│           ├── SparkFun_Qwiic_Ultrasonic
│           └── SparkFun_Toolkit  # SparkFun sfTk dependency
├── tests/                    # Standalone test sketches
│   ├── test_scheduler/
│   ├── test_uart_tlv/
│   ├── test_encoder/
│   ├── test_dc_motor_pwm/
│   ├── test_dc_motor_pid/
│   ├── test_current_sensing/
│   ├── test_servo/
│   ├── test_stepper/
│   ├── test_user_io/
│   ├── test_voltage/
│   ├── test_eeprom/
│   └── test_i2c_scanner/
├── notes/                    # Design notes and analysis
│   ├── REV_A_TO_REV_B_CHANGES.md
│   ├── TIMER3_CONFLICT_ANALYSIS.md
│   └── technical_notes.md
├── pin_table_rev_A.md        # Complete GPIO mapping — PCB Rev. A
└── pin_table_rev_B.md        # Complete GPIO mapping — PCB Rev. B (in testing)
```

## Features

### Communication
- **TLV protocol** over UART at 1 Mbps (Serial2, pins 16/17 via level shifter)
- Bidirectional: RPi sends commands, Arduino sends sensor data and status
- CRC-checked frames; hardware safety timeout (500 ms, motors auto-disable)
- Debug output on USB Serial (Serial0) at 115200 baud

### DC Motors (4 channels)
- H-bridge control via PWM (speed) + digital direction signals
- Interrupt-driven quadrature encoder counting: **2x mode** (phase A only) or **4x mode** (both phases)
- Edge-timing velocity estimator with configurable moving-average filter
- **Cascade PID**: Position → Velocity → Torque (inner torque loop requires current sensing)
- All PID gains runtime-configurable via TLV commands
- Per-motor direction inversion (corrects for reversed wiring)

### Stepper Motors (4 channels)
- STEP/DIR/ENABLE interface compatible with A4988 / DRV8825 drivers
- Timer3 ISR at 10 kHz for precise pulse generation (up to 5000 steps/sec)
- Trapezoidal acceleration profiling
- Limit switch homing support
- Individual enable/disable per channel

### Servos (PCA9685)
- Up to 16 servo channels via I2C PWM controller
- Simple position command interface (angle or pulse width)

### IMU — ICM-20948 9-DoF
- 3-axis accelerometer (mg), gyroscope (DPS), magnetometer (µT)
- **Madgwick AHRS** sensor fusion via Fusion library (x-io Technologies)
- Outputs: quaternion, Euler angles, earth-frame linear acceleration
- 9-DoF mode (with magnetometer) or 6-DoF fallback if uncalibrated
- **Magnetometer hard-iron calibration**: interactive calibration command, stored to EEPROM
- IMU update rate: 100 Hz

### Distance Sensors (I2C via Qwiic)
- **Garmin LIDAR-Lite v4**: 5 cm – 10 m, ~1 cm resolution, up to 4 sensors, 50 Hz
- **SparkFun Qwiic Ultrasonic (HC-SR04)**: 2 cm – 400 cm, ~3 mm accuracy, up to 4 sensors, 15 Hz
- Sensor counts and I2C addresses configured in `config.h`

### Voltage Monitoring
- Battery voltage (1:6 divider, 0–24 V range) at 10 Hz
- 5V rail and servo rail monitors
- Per-motor current sense (ADC) for torque feedback

### Persistent Storage (EEPROM)
- Survives power-off; ~100,000 write cycles per byte
- Stores: wheel diameter, wheel base, magnetometer calibration offsets
- API: `PersistentStorage::init()` / `get*` / `set*` (see `src/modules/PersistentStorage.h`)

### User I/O
- 2 on-board push-buttons (INPUT_PULLUP)
- 8 shared limit switch / button inputs (JST XH 3-pin connectors, pins 40–41, 48–53)
- Status RGB LED: WS2812B NeoPixel (pin 42) — system state patterns
- 3 user LEDs: green (pin 44), blue (pin 45), orange (pin 46) — PWM brightness control
- 1 user LED: purple (pin 47) — digital only

### Scheduler
- Timer1-based cooperative scheduler at 1 kHz base tick
- Configurable per-task frequencies (see `config.h`):

| Task | Default Rate |
|------|-------------|
| DC motor PID | 200 Hz |
| UART comms | 100 Hz |
| IMU reading | 100 Hz |
| LIDAR reading | 50 Hz |
| UserIO (LEDs/buttons) | 20 Hz |
| Ultrasonic reading | 15 Hz |
| Voltage monitoring | 10 Hz |

---

## Pin Tables

Two PCB revisions are supported. The firmware selects pins from `pins.h`:

- **[Rev. A](pin_table_rev_A.md)** — current production board (2x encoder mode for M3/M4)
- **[Rev. B](pin_table_rev_B.md)** — in testing; full 4x quadrature on all motors via PCINT

> **Migration note:** Rev. B is not fully validated yet. Do not switch `pins.h` to Rev. B until hardware testing is complete. See [notes/REV_A_TO_REV_B_CHANGES.md](notes/REV_A_TO_REV_B_CHANGES.md) for the complete pin remapping rationale.

---

## Building and Uploading

### Prerequisites

Install **Arduino IDE 2.x** or **arduino-cli**. Required board package:
- `arduino:avr` (Arduino AVR Boards) — install via Board Manager, target: **Arduino Mega 2560**

Required libraries are vendored in `src/lib/` — no Library Manager installs needed.

### Arduino IDE

1. Open `firmware/arduino/arduino.ino`
2. Select **Tools → Board → Arduino Mega 2560**
3. Select the correct COM/serial port
4. Click **Upload** (Ctrl+U / ⌘U)

### arduino-cli

```bash
cd firmware/arduino
arduino-cli compile --fqbn arduino:avr:mega .
arduino-cli upload  --fqbn arduino:avr:mega --port /dev/ttyUSB0 .
```

Replace `/dev/ttyUSB0` with your port (`/dev/tty.usbmodem*` on macOS, `COM3` on Windows).

### Configuration

Edit `src/config.h` before building:
- Enable/disable motors, sensors, servo controller
- Set UART baud rate, heartbeat timeout, PID defaults
- Set `IMU_ENABLED`, `LIDAR_COUNT`, `ULTRASONIC_COUNT`
- Set `ENCODER_N_MODE` per motor (`ENCODER_2X` or `ENCODER_4X`)

---

## Test Sketches

Each test in `firmware/tests/` is a standalone Arduino sketch that exercises one subsystem. Tests share the firmware source tree via a **symbolic link** (`src → ../../arduino/src`) — this is the standard Arduino IDE workaround for sharing source files across sketches.

### The `src` Symlink

The Arduino IDE requires all source files to be inside the sketch folder. Each test directory contains a `src` symlink that points to the shared firmware source at `firmware/arduino/src/`. You must create this symlink after cloning the repo — it is **not stored in git**.

#### Create symlinks — macOS / Linux

```bash
cd firmware/tests
for dir in test_*/; do
    ln -sf ../../arduino/src "$dir/src"
done
```

Verify:
```bash
ls -la firmware/tests/test_scheduler/src
# Should show: src -> ../../arduino/src
```

#### Create symlinks — Windows (PowerShell, run as Administrator)

```powershell
cd firmware\tests
Get-ChildItem -Directory -Filter "test_*" | ForEach-Object {
    $target = Resolve-Path "..\..\arduino\src"
    $link   = Join-Path $_.FullName "src"
    New-Item -ItemType Junction -Path $link -Target $target -Force
}
```

> **Note:** Windows uses a **directory junction** instead of a symlink. Junctions work the same way with the Arduino IDE. You must run PowerShell as Administrator.

#### Verify symlinks exist

```bash
# macOS / Linux
ls -la firmware/tests/*/src

# Windows PowerShell
Get-ChildItem firmware\tests\test_*\src | Select-Object FullName, Target
```

### Building and Running a Test

#### Arduino IDE

1. Open the desired test sketch (e.g., `firmware/tests/test_encoder/test_encoder.ino`)
2. Verify the `src` symlink exists in that folder
3. Set board to **Arduino Mega 2560** and select your port
4. Click **Upload**, then open **Serial Monitor** at 115200 baud

#### arduino-cli

```bash
# macOS / Linux
arduino-cli compile --fqbn arduino:avr:mega firmware/tests/test_encoder
arduino-cli upload  --fqbn arduino:avr:mega --port /dev/ttyUSB0 firmware/tests/test_encoder

# Windows PowerShell
arduino-cli compile --fqbn arduino:avr:mega firmware\tests\test_encoder
arduino-cli upload  --fqbn arduino:avr:mega --port COM3 firmware\tests\test_encoder
```

### Test Index

| Test | What it tests | Serial Monitor |
|------|---------------|----------------|
| `test_scheduler` | Timer1 1kHz scheduler, task timing accuracy | 115200 baud |
| `test_uart_tlv` | TLV packet encode/decode, loopback (TX→RX jumper needed) | 115200 baud |
| `test_encoder` | Quadrature encoder counting (2x/4x), direction, velocity | 115200 baud |
| `test_dc_motor_pwm` | Direct PWM motor control, H-bridge direction | 115200 baud |
| `test_dc_motor_pid` | Closed-loop PID velocity and position control | 115200 baud |
| `test_current_sensing` | ADC current sensor readings per motor | 115200 baud |
| `test_servo` | PCA9685 servo commands, angle sweep | 115200 baud |
| `test_stepper` | Stepper STEP/DIR pulses, acceleration, limit switch homing | 115200 baud |
| `test_user_io` | Buttons, LEDs (PWM/blink/breathing), NeoPixel colors | 115200 baud |
| `test_voltage` | ADC battery/rail voltage readings | 115200 baud |
| `test_eeprom` | EEPROM read/write; power-cycle persistence demo | 115200 baud |
| `test_i2c_scanner` | Scans I2C bus and prints all detected addresses | 115200 baud |

### `test_eeprom` — Power-Cycle Demo

This test teaches EEPROM persistence to students who have never used non-volatile memory:

1. **Run 1**: Writes known values (wheel diameter, wheel base, mag offsets) → verifies immediate read-back → instructs you to power off
2. **Run 2** (after power cycle): Reads values back → confirms data survived → **proves EEPROM is non-volatile**

Serial commands while running:
- `d` — dump raw EEPROM bytes with field annotations
- `r` — erase all stored data (next boot = Run 1)
- `w` — write test values again
- `h` — help

---

## Known Issues

### Rev. B — LED_RED PWM conflict (Timer 3)

Rev. B moves `LED_RED` from pin 11 (Timer 1) to pin 5 (Timer 3). Timer 3 is already used for stepper pulse generation in CTC mode. **PWM brightness/breathing on LED_RED is not available in Rev. B** — only ON/OFF control works. See [notes/TIMER3_CONFLICT_ANALYSIS.md](notes/TIMER3_CONFLICT_ANALYSIS.md) for full analysis.

### EEPROM unused warning

The compiler emits `'EEPROM' defined but not used` from Arduino's `EEPROM.h` when included in translation units that don't call EEPROM functions directly. This is a harmless upstream header issue and does not affect behavior.
