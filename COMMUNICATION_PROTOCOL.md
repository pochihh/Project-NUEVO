# NUEVO System Design Specification
## Communication Protocol & Firmware Architecture

**Version:** 2.0
**Date:** 2026-02-19
**Status:** Design — pending firmware and bridge implementation

> This document supersedes the previous COMMUNICATION_PROTOCOL.md. It defines
> the complete communication protocol and firmware state machine for the NUEVO
> platform. Use this document as the authoritative reference when updating
> firmware and the NUEVO Bridge.

---

## Table of Contents

1. [System Architecture](#1-system-architecture)
2. [Firmware State Machine](#2-firmware-state-machine)
3. [Physical Layer](#3-physical-layer)
4. [TLV Frame Format](#4-tlv-frame-format)
5. [Safety Design](#5-safety-design)
6. [Startup Behavior](#6-startup-behavior)
7. [TLV Message Catalog](#7-tlv-message-catalog)
8. [Data Stream Design](#8-data-stream-design)
9. [System Configuration Parameters](#9-system-configuration-parameters)
10. [Implementation Notes](#10-implementation-notes)

---

## 1. System Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                  User-Facing Control Layer                   │
│                                                              │
│   ┌─────────────────────┐     ┌───────────────────────────┐  │
│   │     NUEVO UI        │     │  ROS2 Topics / Python API │  │
│   │  (real-time monitor,│     │  (programmatic control,   │  │
│   │   manual control)   │     │   autonomous behavior)    │  │
│   └──────────┬──────────┘     └─────────────┬─────────────┘  │
└──────────────┼─────────────────────────────-┼────────────────┘
               │ WebSocket                     │ ROS2 / Python
               ▼                               ▼
┌──────────────────────────────────────────────────────────────┐
│             NUEVO Bridge  (Raspberry Pi 5, Python)           │
│  • Serial Manager: UART ↔ TLV encode/decode                  │
│  • Message Router: fan-out to WebSocket clients + ROS2       │
│  • Liveness Monitor: heartbeat to Arduino                    │
│  • Command Arbiter: merges UI + API commands                 │
└─────────────────────────────┬────────────────────────────────┘
                              │ UART 1 Mbps, TLV protocol
                              │ TXB0104 level shifter (5V ↔ 3.3V)
                              ▼
┌──────────────────────────────────────────────────────────────┐
│          NUEVO Board Firmware  (Arduino Mega 2560)           │
│  • State Machine (Idle / Running / Error / E-STOP)           │
│  • MessageCenter: TLV routing                                │
│  • DC Motor Controllers (PID position + velocity)            │
│  • Stepper Manager (AccelStepper, Timer3)                    │
│  • Servo Manager (PCA9685 via I2C)                           │
│  • Sensor Drivers (ICM-20948 IMU, voltage, range)            │
│  • UserIO (LEDs, NeoPixels, buttons, limit switches)         │
└──────────────────────────────────────────────────────────────┘
```

**Key principles:**
- UI and ROS2/API can **co-exist** and both send commands simultaneously.
  The Bridge arbitrates by last-write-wins per subsystem.
- Commands **do not require acknowledgement.** The next streamed status
  frame shows the applied result.
- The Arduino is **unaware of who is controlling it.** All commands arrive
  identically via UART from the Bridge.

---

## 2. Firmware State Machine

```
                     Power On
                         │
                         ▼
                   ┌──────────┐
                   │   INIT   │  Initialize modules, run self-checks
                   └────┬─────┘
                        │ Ready
                        ▼
              ┌────────────────────┐
         ┌───►│       IDLE         │◄──────────────────────────┐
         │    └────────┬───────────┘                           │
         │             │ SYS_CMD(START)                        │
         │             ▼                                       │ SYS_CMD(STOP)
         │    ┌────────────────────┐                           │
         │    │      RUNNING       │───────────────────────────┘
         │    └────────┬───────────┘
         │             │ Critical error
         │             ▼
         │    ┌────────────────────┐
         │    │       ERROR        │
         │    └────────┬───────────┘
         │             │ SYS_CMD(RESET)
         └─────────────┘

        Any state  ──── SYS_CMD(ESTOP) ────►   ┌────────────────┐
                                               │     E-STOP     │
                                               └───────┬────────┘
                                                       │ SYS_CMD(RESET)
                                                       │ or hardware reset
                                                       ▼
                                                      IDLE
```

### INIT
- Initialize all modules (motors disabled, steppers disabled, servos disabled).
- Run I2C scan (detect PCA9685, ICM-20948).
- If any **critical** initialization fails (e.g., stack overflow, bad flash):
  enter ERROR directly.
- On success: transition to IDLE.

### IDLE
- All actuators remain disabled.
- Stream `SYS_STATUS` at **1 Hz**.
- Accept configuration commands (`SYS_CONFIG`).
- Accept `SYS_CMD(START)` → transition to RUNNING.
- Accept `SYS_SET_PID` (pre-configure gains before running).

### RUNNING
- Stream all data (see [Section 8](#8-data-stream-design)).
- Accept all motor control commands.
- Accept `SYS_SET_PID` (live PID tuning is supported).
- Accept `SYS_CMD(STOP)` → transition to IDLE (disables all actuators).
- On critical error → transition to ERROR.
- Monitor liveness (see [Section 5](#5-safety-design)).
- **Configuration commands (`SYS_CONFIG`) are not accepted in RUNNING.**
  The Bridge must send STOP first.

### ERROR
- All actuators are immediately disabled.
- Stream `SYS_STATUS` at **10 Hz** (includes error flags for UI display).
- Accept `SYS_CMD(RESET)` → transition to IDLE.
- Accept `SYS_CMD(ESTOP)` → transition to E-STOP.
- No motor control commands accepted.

**Conditions that trigger ERROR:**
| Error Flag | Condition |
|---|---|
| `ERR_UNDERVOLTAGE` | Battery voltage below configured minimum (default: 10.0 V) |
| `ERR_OVERVOLTAGE` | Battery voltage above configured maximum (default: 25.0 V) |
| `ERR_ENCODER_FAIL` | Encoder reads 0 for >500 ms while motor PWM > 20% |

### E-STOP
- All actuators immediately disabled and **held disabled**.
- Stream `SYS_STATUS` at **1 Hz** (to indicate the board is still alive).
- Only `SYS_CMD(RESET)` is accepted — all other incoming TLV messages are silently discarded.
- **Exit:** `SYS_CMD(RESET)` → IDLE, or hardware Arduino reset.
- Triggered by `SYS_CMD(ESTOP)` from any state.

> **Educational note:** E-STOP is for genuine safety emergencies (e.g., robot
> about to fall off a table, motor running away). For normal stop, use
> `SYS_CMD(STOP)` which returns to IDLE gracefully.

---

## 3. Physical Layer

| Parameter | Value |
|---|---|
| Interface | UART (Arduino Serial2: pins 16 TX, 17 RX) |
| Baud rate | **1,000,000 bps (1 Mbps)** |
| Data format | 8N1 (8 data, no parity, 1 stop bit) |
| Level shifting | TXB0104 bidirectional (5V Arduino ↔ 3.3V RPi) |
| RPi UART | GPIO14 (TX), GPIO15 (RX) — `/dev/ttyAMA0` or `/dev/serial0` |
| Duplex | **Full duplex** — TX and RX are independent channels |

**On full duplex:** At 1 Mbps, the Arduino can simultaneously transmit up to
1 Mbps of streaming data *and* receive up to 1 Mbps of command data. The two
directions do not interfere. Stream bandwidth and command bandwidth are
independently 1 Mbps each.

**Effective throughput:** At 10 bits/byte (start + 8 data + stop):
1,000,000 / 10 = **100,000 bytes/sec** per direction.

---

## 4. TLV Frame Format

Each transmission is a **frame** containing one or more TLV packets. Frames
are delimited by a magic number header.

```
┌─────────────────────────────────────────────────────────────────────────┐
│  FRAME                                                                  │
│                                                                         │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │  FrameHeader  (28 bytes)                                         │   │
│  │  uint8_t  magicNum[8]    sync bytes — frame boundary detection   │   │
│  │  uint32_t numTotalBytes  total byte count of this frame          │   │
│  │  uint32_t checksum       CRC32 (see below)                       │   │
│  │  uint32_t deviceId       sender ID (Arduino=1, RPi=2)            │   │
│  │  uint32_t frameNum       incrementing frame sequence number      │   │
│  │  uint32_t numTlvs        number of TLV packets that follow       │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                                                         │
│  ┌──────────────────────────────────────┐                               │
│  │  TLV packet 1                        │                               │
│  │  uint32_t tlvType    (4 bytes)       │                               │
│  │  uint32_t tlvLen     (4 bytes)       │                               │
│  │  uint8_t  payload    (tlvLen bytes)  │                               │
│  └──────────────────────────────────────┘                               │
│  ┌──────────────────────────────────────┐                               │
│  │  TLV packet 2  ...                   │                               │
│  └──────────────────────────────────────┘                               │
│  ...  (numTlvs total)                                                   │
└─────────────────────────────────────────────────────────────────────────┘
```

**FrameHeader fields:**
- **`magicNum[8]`:** Fixed sync pattern used by the decoder to locate frame
  boundaries in the byte stream. Defined in `tlvcodec.c` as `FRAME_HEADER_MAGIC_NUM`.
- **`numTotalBytes`:** Total byte count of the frame (header + all TLV packets).
- **`checksum`:** CRC32 computed over bytes 16 onward — i.e., over `deviceId`,
  `frameNum`, `numTlvs`, and all TLV data. The first 16 bytes (magicNum +
  numTotalBytes + checksum itself) are excluded (`CRC32_BYTES2IGNORE = 16`).
- **`deviceId`:** Sender identifier. Defined in `config.h`.
- **`frameNum`:** Monotonically incrementing frame counter. Can be used to
  detect dropped frames.
- **`numTlvs`:** Count of TLV packets in the frame body.

**TLV packet fields:**
- **`tlvType`:** Message type ID (see [Section 7](#7-tlv-message-catalog)).
- **`tlvLen`:** Payload byte count. May be 0 for commands with no data.
- **`payload`:** Packed struct data (`#pragma pack(push, 1)`, no padding).

**Sizes:**
- Frame header: **28 bytes**
- Per TLV overhead: **8 bytes** (type + length fields)
- Minimum frame (header + 1 empty TLV): 28 + 8 = **36 bytes**

**Bundling:** The protocol supports multiple TLVs per frame. The current
`UARTDriver::send()` implementation sends one TLV per frame for simplicity.
Bundling multiple related TLV packets into one frame reduces header overhead
and is worth implementing for the high-rate streaming path.

---

## 5. Safety Design

### 5.1 Liveness Detection (Heartbeat)

The Arduino monitors **time since last received TLV from the RPi** in RUNNING
state. The Bridge is responsible for maintaining continuous communication:

- When commands are being sent: each command resets the liveness timer.
- When no commands are pending: the Bridge sends `SYS_HEARTBEAT` at **200 ms**
  intervals to keep the timer alive.

**Timeout behavior (RUNNING state only):**

| Elapsed since last RX | Action |
|---|---|
| > 500 ms | Disable all motors. Log warning in `SYS_STATUS`. Stay in RUNNING state — re-enable when communication resumes. |
| Communication resumes | Motor commands are accepted again (re-enable manually). |

> **Design note:** The timeout does not force a state transition to IDLE. The
> system simply cuts motor power as a safety measure. When the Bridge
> reconnects and resumes sending, the user can re-enable motors. This avoids
> requiring a full STOP/START cycle on a brief network hiccup.

The timeout threshold is configurable via `SYS_CONFIG` (default: 500 ms).

### 5.2 E-STOP

- Triggered by `SYS_CMD(ESTOP)` command.
- Immediately disables all actuators.
- **Cannot be reversed by software.** Physical Arduino reset required.
- The Bridge should provide a dedicated E-STOP button in the UI (and via API).

### 5.3 Error Conditions

See [Section 2 — ERROR state](#error) for conditions and flags.

All error flags are reported in `SYS_STATUS.errorFlags` as a bitmask.

---

## 6. Startup Behavior

All actuators start in a **safe, disabled state** on every power-on or reset.

| Subsystem | Startup State |
|---|---|
| DC Motors | Mode = disabled. H-bridge coast (PWM = 0). Encoders zeroed. |
| Stepper Motors | Disabled (ENABLE pin high). Step count = 0. |
| Servos (PCA9685) | PCA9685 output disabled (OE pin high). All channels at 0 µs. No servo movement. |
| LEDs | All off. |
| NeoPixels | All off (0, 0, 0). |

The user must explicitly:
1. (Optionally) Send `SYS_CONFIG` to set parameters.
2. Send `SYS_CMD(START)` to enter RUNNING state.
3. Send motor enable commands with desired mode.
4. Send target position/velocity/angle commands.

---

## 7. TLV Message Catalog

### Direction conventions
- **↓ Down:** RPi → Arduino (command)
- **↑ Up:** Arduino → RPi (data / telemetry)

---

### 7.1 System Messages (type 1–19)

#### `SYS_HEARTBEAT = 1` ↓
Sent by RPi to indicate it is alive. Any received TLV resets the liveness
timer; the Bridge sends this when no other messages are pending.

```
Payload: PayloadHeartbeat
  uint32_t  timestamp    // RPi milliseconds since boot
  uint8_t   flags        // reserved
Size: 5 bytes
```

#### `SYS_STATUS = 2` ↑
Periodic system health report.
Rate: 1 Hz (IDLE), 10 Hz (RUNNING, ERROR), 1 Hz (E-STOP).

```
Payload: PayloadSystemStatus
  uint8_t   firmwareMajor    // e.g., 1
  uint8_t   firmwareMinor    // e.g., 0
  uint8_t   firmwarePatch    // e.g., 0
  uint8_t   state            // 0=INIT, 1=IDLE, 2=RUNNING, 3=ERROR, 4=ESTOP
  uint32_t  uptimeMs         // Arduino uptime (ms)
  uint32_t  lastRxMs         // ms since last received TLV from RPi
  uint16_t  batteryMv        // battery voltage (mV)
  uint16_t  rail5vMv         // 5V rail voltage (mV)
  uint8_t   errorFlags       // bitmask (see SystemErrorFlags)
  uint8_t   attachedSensors  // bitmask: bit0=IMU, bit1=Lidar, bit2=Ultrasonic
  uint16_t  freeSram         // free SRAM (bytes)
  uint16_t  loopTimeAvgUs    // avg control loop execution time (µs)
  uint16_t  loopTimeMaxUs    // max control loop time since last status (µs)
  uint16_t  uartRxErrors     // cumulative UART CRC/framing error count
  uint32_t  lastCmdMs        // ms since last received non-heartbeat command
  // System parameters (echo of current config):
  float     wheelDiameterMm  // configured wheel diameter
  float     wheelBaseMm      // configured wheel base (center-to-center)
  uint8_t   motorDirMask     // bit N = direction inversion for motor N (0=normal)
  uint8_t   neoPixelCount    // configured NeoPixel count
  uint16_t  heartbeatTimeoutMs // configured liveness timeout
  // Limit switch / button GPIO configuration (set at compile time in config.h):
  uint16_t  limitSwitchMask  // bitmask: bit N = GPIO N is configured as a limit switch
  uint8_t   stepperHomeLimitGpio[4] // GPIO index used as home limit for each stepper
                             //   (0xFF = no limit switch configured for that stepper)
Size: ~54 bytes
```

> **Limit switch / button GPIO sharing:**
> Button and limit switch inputs share the same physical GPIO pins. A GPIO
> configured as a limit switch via `config.h` appears in `IO_STATUS.buttonMask`
> exactly like a regular button press. The `limitSwitchMask` and
> `stepperHomeLimitGpio` fields tell the UI which bits in `buttonMask`
> represent limit switch events vs. user button presses, so it can display
> them appropriately. The firmware uses the `stepperHomeLimitGpio` mapping
> internally to monitor the correct pin during `STEP_HOME` execution.

**SystemErrorFlags bitmask:**
```
ERR_UNDERVOLTAGE   = 0x01
ERR_OVERVOLTAGE    = 0x02
ERR_ENCODER_FAIL   = 0x04
ERR_I2C_ERROR      = 0x08
ERR_IMU_ERROR      = 0x10
ERR_LIVENESS_LOST  = 0x20  // liveness timeout triggered (motors cut)
ERR_LOOP_OVERRUN   = 0x40  // control loop missed deadline
```

#### `SYS_CMD = 3` ↓
System state control commands.

```
Payload: PayloadSysCmd
  uint8_t   command      // see SysCmdType enum
  uint8_t   reserved[3]
Size: 4 bytes

SysCmdType enum:
  SYS_CMD_START  = 1   // IDLE → RUNNING
  SYS_CMD_STOP   = 2   // RUNNING → IDLE (disables all actuators)
  SYS_CMD_RESET  = 3   // ERROR → IDLE
  SYS_CMD_ESTOP  = 4   // any → E-STOP (hardware reset required to exit)
```

#### `SYS_CONFIG = 4` ↓
Configure system parameters. **IDLE state only.** Ignored if received in
RUNNING. Changes are reflected in the next `SYS_STATUS`.

```
Payload: PayloadSysConfig
  float     wheelDiameterMm    // wheel diameter (mm), 0 = no change
  float     wheelBaseMm        // wheel base (mm), 0 = no change
  uint8_t   motorDirMask       // bit N = invert direction for motor N
  uint8_t   motorDirChangeMask // which motors to apply direction change to
  uint8_t   neoPixelCount      // NeoPixel count (0 = no change)
  uint8_t   attachedSensors    // sensor attachment bitmask (0xFF = no change)
  uint16_t  heartbeatTimeoutMs // liveness timeout (ms), 0 = no change
  uint8_t   resetOdometry      // 1 = reset x, y, theta to (0, 0, 0)
  uint8_t   reserved
Size: 16 bytes
```

#### `SYS_SET_PID = 5` ↓
Set PID gains for a DC motor (position or velocity loop).
Available in **IDLE and RUNNING** states (live tuning supported).

```
Payload: PayloadSetPID  (unchanged from v1)
  uint8_t   motorId      // 0–3
  uint8_t   loopType     // 0 = position loop, 1 = velocity loop
  uint8_t   reserved[2]
  float     kp
  float     ki
  float     kd
  float     maxOutput    // output clamp (default: 255)
  float     maxIntegral  // anti-windup limit
Size: 24 bytes
```

---

### 7.2 DC Motor Messages (type 256–279)

#### `DC_ENABLE = 256` ↓
Enable or disable a DC motor with mode selection.

```
Payload: PayloadDCEnable
  uint8_t   motorId    // 0–3
  uint8_t   mode       // 0=disable, 1=position, 2=velocity, 3=pwm
  uint8_t   reserved[2]
Size: 4 bytes
```

Disabling a motor coasts the H-bridge (PWM = 0). Re-enabling does not move
the motor until a target command is sent.

#### `DC_SET_POSITION = 257` ↓
Command absolute target position (position control mode).

```
Payload: PayloadDCSetPosition
  uint8_t   motorId       // 0–3
  uint8_t   reserved[3]
  int32_t   targetTicks   // absolute encoder ticks
  int32_t   maxVelTicks   // velocity limit during move (ticks/sec), 0 = default
Size: 12 bytes
```

#### `DC_SET_VELOCITY = 258` ↓
Command target velocity (velocity control mode).

```
Payload: PayloadDCSetVelocity
  uint8_t   motorId       // 0–3
  uint8_t   reserved[3]
  int32_t   targetTicks   // target velocity (ticks/sec)
Size: 8 bytes
```

#### `DC_SET_PWM = 259` ↓
Command direct PWM output (PWM control mode, open loop).

```
Payload: PayloadDCSetPWM
  uint8_t   motorId    // 0–3
  uint8_t   reserved
  int16_t   pwm        // -255 to +255 (sign = direction)
Size: 4 bytes
```

#### `DC_STATUS_ALL = 260` ↑
Status for all 4 DC motors in one message.
Rate: **100 Hz** in RUNNING state.

```
Payload: PayloadDCStatusAll
  struct DCMotorStatus motors[4]:
    uint8_t   mode          // 0=disabled, 1=position, 2=velocity, 3=pwm
    uint8_t   faultFlags    // bit0=overcurrent, bit1=stall
    int32_t   position      // current position (encoder ticks)
    int32_t   velocity      // current velocity (ticks/sec)
    int32_t   targetPos     // target position (meaningful in position mode)
    int32_t   targetVel     // target velocity (meaningful in velocity mode)
    int16_t   pwmOutput     // current PWM output (-255 to +255)
    int16_t   currentMa     // motor current (mA); -1 if not measured
    float     posKp, posKi, posKd   // position loop PID gains
    float     velKp, velKi, velKd   // velocity loop PID gains
  // Per-motor size: 2 + 4×4 + 2×2 + 6×4 = 2 + 16 + 4 + 24 = 46 bytes
  // Total payload: 4 × 46 = 184 bytes
Size: 184 bytes
```

> **Note on "commanded count" vs "measured":** DC motor position is measured
> from encoders (closed-loop). This is a true measurement, not a commanded
> value.

---

### 7.3 Stepper Motor Messages (type 512–539)

#### `STEP_ENABLE = 512` ↓
Enable or disable a stepper motor driver.

```
Payload: PayloadStepEnable
  uint8_t   stepperId   // 0–3
  uint8_t   enable      // 0 = disable (release), 1 = enable (hold)
  uint8_t   reserved[2]
Size: 4 bytes
```

#### `STEP_SET_PARAMS = 513` ↓
Set motion parameters (replaces separate SET_ACCEL and SET_VEL messages).

```
Payload: PayloadStepSetParams
  uint8_t   stepperId     // 0–3
  uint8_t   reserved[3]
  uint32_t  maxVelocity   // steps/sec
  uint32_t  acceleration  // steps/sec²
Size: 12 bytes
```

#### `STEP_MOVE = 514` ↓
Move stepper to a target (absolute or relative).

```
Payload: PayloadStepMove
  uint8_t   stepperId   // 0–3
  uint8_t   moveType    // 0 = absolute, 1 = relative
  uint8_t   reserved[2]
  int32_t   target      // target (ticks for absolute, steps for relative)
Size: 8 bytes
```

#### `STEP_HOME = 515` ↓
Run homing sequence (move until limit switch triggers, then zero position).

```
Payload: PayloadStepHome
  uint8_t   stepperId      // 0–3
  int8_t    direction      // -1 = reverse, +1 = forward
  uint8_t   reserved[2]
  uint32_t  homeVelocity   // homing speed (steps/sec)
  int32_t   backoffSteps   // steps to retreat after limit hit
Size: 12 bytes
```

#### `STEP_STATUS_ALL = 516` ↑
Status for all 4 steppers in one message.
Rate: **100 Hz** in RUNNING state.

```
Payload: PayloadStepStatusAll
  struct StepperStatus steppers[4]:
    uint8_t   enabled           // 0 = disabled, 1 = enabled
    uint8_t   motionState       // 0=idle, 1=accel, 2=cruise, 3=decel, 4=homing
    uint8_t   limitHit          // bit0=min limit, bit1=max limit
    uint8_t   reserved
    int32_t   commandedCount    // commanded step count (not measured — open loop)
    int32_t   targetCount       // target step count
    uint32_t  currentSpeed      // current speed (steps/sec)
    uint32_t  maxSpeed          // configured max speed
    uint32_t  acceleration      // configured acceleration
  // Per-stepper size: 4 + 3×4 + 2×4 = 4 + 12 + 8 = 24 bytes
  // Total payload: 4 × 24 = 96 bytes
Size: 96 bytes
```

> **Open-loop note:** `commandedCount` is the firmware's internal step counter.
> It assumes no missed steps. It is **not** a measured position.

---

### 7.4 Servo Messages (type 768–789)

#### `SERVO_ENABLE = 768` ↓
Enable or disable individual servo channels (per-servo, not the whole PCA9685).

```
Payload: PayloadServoEnable
  uint8_t   channel      // 0–15 servo channel, 0xFF = all channels
  uint8_t   enable       // 0 = disable (pulse off), 1 = enable
  uint8_t   reserved[2]
Size: 4 bytes
```

Disabling a servo cuts the PWM signal (servo goes limp / relaxes).
On startup, all channels are disabled.

#### `SERVO_SET = 769` ↓
Set target angle for one servo or bulk-set multiple consecutive servos.

```
// Single servo:
Payload: PayloadServoSetSingle
  uint8_t   channel    // 0–15
  uint8_t   reserved
  uint16_t  pulseUs    // pulse width (µs), typically 500–2500

// Bulk update (consecutive channels):
Payload: PayloadServoSetBulk
  uint8_t   startChannel   // first channel to update
  uint8_t   count          // number of channels (1–16)
  uint16_t  pulseUs[16]    // pulse widths; only first 'count' values used
```

Use `PayloadServoSetBulk` with `count=1` for single servo, or set multiple
servos synchronously for coordinated motion.

#### `SERVO_STATUS_ALL = 770` ↑
PCA9685 controller status and all 16 servo commanded positions.
Rate: **50 Hz** in RUNNING state (servo feedback is not available; this
reports the commanded state only).

```
Payload: PayloadServoStatusAll
  uint8_t   pca9685Connected  // 0 = not detected, 1 = connected
  uint8_t   pca9685Error      // error flags from PCA9685
  uint16_t  enabledMask       // bitmask of enabled channels (bit N = channel N)
  uint16_t  pulseUs[16]       // commanded pulse width per channel (0 = disabled)
  // Total payload: 4 + 32 = 36 bytes
Size: 36 bytes
```

> On firmware reset, all `pulseUs` values are 0 (disabled). The user must
> enable and set each servo after startup. This prevents unpredictable servo
> movement when the firmware initializes.

---

### 7.5 Sensor Messages (type 1024–1059)

#### `SENSOR_IMU = 1024` ↑
Full ICM-20948 data.
Rate: **100 Hz** in RUNNING state (if IMU attached).

```
Payload: PayloadSensorIMU
  // Orientation from DMP (if DMP enabled):
  float     quatW, quatX, quatY, quatZ    // unit quaternion (16 bytes)
  // Global-frame linear acceleration (gravity removed, world frame):
  float     globalAccX, globalAccY, globalAccZ   // m/s² (12 bytes)
  // Raw sensor data (sensor/robot frame):
  int16_t   rawAccX, rawAccY, rawAccZ     // raw accelerometer LSBs (6 bytes)
  int16_t   rawGyroX, rawGyroY, rawGyroZ  // raw gyroscope LSBs (6 bytes)
  int16_t   magX, magY, magZ             // magnetometer LSBs (6 bytes)
  uint32_t  timestamp                     // Arduino micros() at sample time
  // Total: 16 + 12 + 6 + 6 + 6 + 4 = 50 bytes
Size: 50 bytes
```

> Global-frame acceleration is computed on the Arduino using the DMP
> quaternion: `global_acc = R(q) × raw_acc - g`. If DMP is unavailable,
> quaternion fields are set to (1, 0, 0, 0) and global_acc equals raw_acc
> rotated by identity (i.e., uncorrected).

#### `SENSOR_KINEMATICS = 1025` ↑
Estimated robot pose and velocity from wheel odometry.
Rate: **100 Hz** in RUNNING state.

```
Payload: PayloadSensorKinematics
  float     x            // position X (mm from start)
  float     y            // position Y (mm from start)
  float     theta        // heading (radians, CCW positive)
  float     vx           // velocity X (mm/s, robot frame)
  float     vy           // velocity Y (mm/s, robot frame)
  float     vTheta       // angular velocity (rad/s)
  uint32_t  timestamp    // Arduino micros() at compute time
  // Total: 6×4 + 4 = 28 bytes
Size: 28 bytes
```

Kinematics are calculated from encoder counts and the configured
`wheelDiameterMm` and `wheelBaseMm`. Position resets to (0, 0, 0) on:
- Firmware reset
- `SYS_CONFIG` with `resetOdometry = 1`

#### `SENSOR_VOLTAGE = 1026` ↑
Battery and rail voltages.
Rate: **10 Hz** in RUNNING state.

```
Payload: PayloadSensorVoltage
  uint16_t  batteryMv     // battery input voltage (mV)
  uint16_t  rail5vMv      // 5V regulated rail (mV)
  uint16_t  servoRailMv   // servo power rail (mV)
  uint16_t  reserved
Size: 8 bytes
```

#### `SENSOR_RANGE = 1027` ↑
Distance measurement from a range sensor (ultrasonic or lidar).
One message per sensor reading. Rate depends on sensor polling frequency.

```
Payload: PayloadSensorRange
  uint8_t   sensorId      // sensor index (0-based)
  uint8_t   sensorType    // 0 = ultrasonic, 1 = lidar
  uint8_t   status        // 0 = valid, 1 = out of range, 2 = error
  uint8_t   reserved
  uint16_t  distanceMm    // measured distance (mm)
  uint16_t  reserved2
  uint32_t  timestamp     // Arduino micros()
Size: 12 bytes
```

---

### 7.6 User I/O Messages (type 1280–1299)

#### `IO_SET_LED = 1280` ↓
Set LED state (user LEDs and status LED).

```
Payload: PayloadSetLED  (unchanged from v1)
  uint8_t   ledId        // LED index (see pin config)
  uint8_t   mode         // 0=off, 1=on, 2=blink, 3=breathe*, 4=pwm
  uint8_t   brightness   // 0–255
  uint8_t   reserved
  uint16_t  periodMs     // blink/breathe period
  uint16_t  dutyCycle    // blink duty cycle (0–1000 = 0–100.0%)
Size: 8 bytes
```

> *Note: LED_RED (pin 5, Rev. B) cannot breathe — Timer3 conflict with
> steppers. Breathe mode falls back to blink for this LED.
> See `firmware/notes/TIMER3_CONFLICT_ANALYSIS.md`.

#### `IO_SET_NEOPIXEL = 1281` ↓
Set NeoPixel color for one pixel or all pixels.

```
Payload: PayloadSetNeoPixel
  uint8_t   index    // pixel index (0xFF = all pixels)
  uint8_t   red
  uint8_t   green
  uint8_t   blue
Size: 4 bytes
```

#### `IO_STATUS = 1282` ↑
All UserIO state in one message.
Rate: **100 Hz** in RUNNING state.

```
Payload: PayloadIOStatus
  uint16_t  buttonMask     // bitmask of all digital input GPIOs (bit N = GPIO N)
                           //   covers both regular buttons AND limit switches
                           //   use SYS_STATUS.limitSwitchMask to distinguish them
  // LED states (one per configured LED):
  uint8_t   ledBrightness[3]   // brightness 0–255 (0 = off) for each LED
  uint8_t   reserved
  // NeoPixel colors (variable length based on neoPixelCount):
  // uint8_t rgb[neoPixelCount × 3]  — appended after fixed fields
  // For neoPixelCount=2: 6 bytes appended → total payload = 10 bytes
  uint32_t  timestamp      // Arduino millis()
Size: variable (10 bytes base + 3×neoPixelCount bytes)
```

> **Button and limit switch sharing:** All digital input GPIOs report in the
> single `buttonMask`. The Bridge/UI uses `SYS_STATUS.limitSwitchMask` to
> know which bits are limit switches and `stepperHomeLimitGpio[4]` to map
> each limit to its stepper. This means students can test limit switch behavior
> by pressing the physical button — no actual limit switch hardware required.
> The NeoPixel RGB array length is determined by `SYS_STATUS.neoPixelCount`.

---

## 8. Data Stream Design

### 8.1 Streaming Rates by State

| Message | IDLE | RUNNING | ERROR | E-STOP |
|---|---|---|---|---|
| `SYS_STATUS` | 1 Hz | 10 Hz | 10 Hz | 1 Hz |
| `DC_STATUS_ALL` | — | 100 Hz | — | — |
| `STEP_STATUS_ALL` | — | 100 Hz | — | — |
| `SERVO_STATUS_ALL` | — | 50 Hz | — | — |
| `IO_STATUS` | — | 100 Hz | — | — |
| `SENSOR_IMU` | — | 100 Hz | — | — |
| `SENSOR_KINEMATICS` | — | 100 Hz | — | — |
| `SENSOR_VOLTAGE` | — | 10 Hz | 10 Hz | — |
| `SENSOR_RANGE` | — | sensor rate | — | — |

### 8.2 Bandwidth Budget (RUNNING state)

Frame overhead per message: 28-byte FrameHeader + 8-byte TlvHeader = **36 bytes**.

| Message | Payload (bytes) | Frame total | Rate (Hz) | Avg bytes/sec |
|---|---|---|---|---|
| `DC_STATUS_ALL` | 184 | 220 | 100 | 22,000 |
| `STEP_STATUS_ALL` | 96 | 132 | 100 | 13,200 |
| `SENSOR_IMU` | 50 | 86 | 100 | 8,600 |
| `SENSOR_KINEMATICS` | 28 | 64 | 100 | 6,400 |
| `IO_STATUS` | 12 | 48 | 100 | 4,800 |
| `SERVO_STATUS_ALL` | 36 | 72 | 50 | 3,600 |
| `SYS_STATUS` | 44 | 80 | 10 | 800 |
| `SENSOR_VOLTAGE` | 8 | 44 | 10 | 440 |
| **Total TX (1 TLV/frame)** | | | | **~59,840 bytes/sec** |

At 1 Mbps (100,000 bytes/sec effective): **59.8% TX utilization.**

**Bundling optimization:** The protocol supports multiple TLVs per frame.
Bundling all 100 Hz messages into one frame per cycle replaces five separate
28-byte FrameHeaders with one, saving ~11,200 bytes/sec and dropping to
**~48% TX utilization.** Worth implementing once basic streaming is stable.

Command traffic from RPi→Arduino on the independent RX channel:

| Command | Payload | Frame total | Rate | bytes/sec |
|---|---|---|---|---|
| `SYS_HEARTBEAT` | 5 | 41 | 5 Hz | 205 |
| `DC_SET_VELOCITY × 4` | 8 each | 44 each | 100 Hz | 17,600 |
| **Total RX** | | | | **~17,805 bytes/sec** |

RX utilization at 100 Hz of motor commands: **17.8%.** Full duplex, so this
does not reduce TX bandwidth.

---

## 9. System Configuration Parameters

These parameters are set via `SYS_CONFIG` in IDLE state (except `resetOdometry`
which can be sent at any time).

| Parameter | Default | Description |
|---|---|---|
| `wheelDiameterMm` | 65.0 | Wheel outer diameter (mm) |
| `wheelBaseMm` | 150.0 | Center-to-center wheel distance (mm) |
| `motorDirMask` | 0x00 | Per-motor direction inversion (bit N = motor N) |
| `neoPixelCount` | 1 | Number of WS2812B NeoPixels on the strip |
| `attachedSensors` | 0x01 (IMU) | Sensor attachment bitmask |
| `heartbeatTimeoutMs` | 500 | Liveness timeout for RUNNING state |

**`attachedSensors` bitmask:**
```
BIT_IMU        = 0x01   // ICM-20948 (I2C)
BIT_LIDAR      = 0x02   // lidar sensor
BIT_ULTRASONIC = 0x04   // ultrasonic range sensor(s)
```

Configuration does **not persist across firmware reset.** Defaults reload on
every power-on. The Bridge should re-send `SYS_CONFIG` after connecting if
non-default parameters are needed.

---

## 10. Implementation Notes

### 10.1 Free SRAM Measurement (Arduino)

The Arduino Mega 2560 has 8 KB of SRAM. Free SRAM can be measured at runtime:

```cpp
extern int __heap_start, *__brkval;
uint16_t freeMemory() {
    int v;
    return (uint16_t)&v - (__brkval == 0
        ? (uint16_t)&__heap_start
        : (uint16_t)__brkval);
}
```

Call this in the `SYS_STATUS` build routine. If free SRAM drops below ~500
bytes, set `ERR_LOOP_OVERRUN` as a warning (not currently a defined error, but
worth monitoring).

### 10.2 Control Loop Timing Measurement

Track loop execution time in the main control ISR (or the `loop()` function
if running as a cooperative scheduler):

```cpp
// At the start of each control cycle:
uint32_t loopStart = micros();

// ... control loop body ...

uint32_t elapsed = micros() - loopStart;
loopTimeMax = max(loopTimeMax, elapsed);
loopTimeSum += elapsed;
loopCount++;
```

Report `loopTimeAvgUs = loopTimeSum / loopCount` and `loopTimeMaxUs` in
`SYS_STATUS`. Reset max and sum every 1 second. Set `ERR_LOOP_OVERRUN` if
`loopTimeMaxUs` exceeds the control period (e.g., >10,000 µs for a 100 Hz loop).

### 10.3 UART Error Counting

Increment `uartRxErrors` in the TLV decoder whenever:
- CRC32 mismatch is detected
- Framing error occurs (invalid start bytes or out-of-sync)
- A message is discarded due to buffer overflow

Report the cumulative count in `SYS_STATUS`. The Bridge UI can show this to
help students diagnose cable issues.

### 10.4 TLV Frame Bundling (Streaming Path)

The `tlvcodec` library supports multiple TLV packets per frame via
`addTlvPacket()` called multiple times before `wrapupBuffer()`. The current
`UARTDriver::send()` sends one TLV per frame. For the RUNNING state 100 Hz
streaming path, implement a bundled send in `MessageCenter`:

```cpp
// In MessageCenter::sendTelemetry():
resetDescriptor(&encodeDesc_);                          // start frame
addTlvPacket(&encodeDesc_, DC_STATUS_ALL,    ...);      // add DC status
addTlvPacket(&encodeDesc_, STEP_STATUS_ALL,  ...);      // add stepper status
addTlvPacket(&encodeDesc_, SENSOR_IMU,       ...);      // add IMU
addTlvPacket(&encodeDesc_, SENSOR_KINEMATICS,...);      // add kinematics
addTlvPacket(&encodeDesc_, IO_STATUS,        ...);      // add IO
int totalBytes = wrapupBuffer(&encodeDesc_);            // finalize + CRC
serial_.write(txBuffer_, totalBytes);                   // one UART write
```

This replaces five separate 28-byte FrameHeaders with one, reducing streaming
overhead from ~59,840 to ~48,640 bytes/sec.

Lower-rate messages (`SYS_STATUS`, `SERVO_STATUS_ALL`, `SENSOR_VOLTAGE`) can
be bundled into the same frame when their update interval expires, or sent as
separate frames on their own schedule — whichever is simpler to implement.

### 10.5 Limit Switch Configuration (Firmware `config.h`)

Limit switch → stepper mappings are **compile-time constants** in `config.h`.
The runtime cannot reassign them. Example configuration:

```cpp
// config.h — Limit switch GPIO assignments
// Each entry is the index into the button GPIO array (0-based).
// Set to 0xFF if no limit switch is used for that stepper.
#define STEPPER0_HOME_LIMIT_GPIO   2    // button index 2 → stepper 0 home limit
#define STEPPER1_HOME_LIMIT_GPIO   3    // button index 3 → stepper 1 home limit
#define STEPPER2_HOME_LIMIT_GPIO   0xFF // no limit switch
#define STEPPER3_HOME_LIMIT_GPIO   0xFF // no limit switch
```

The firmware populates `SYS_STATUS.stepperHomeLimitGpio[]` from these defines
so the Bridge and UI always have the correct mapping at runtime.

During `STEP_HOME` execution, the firmware monitors the configured GPIO pin
and stops + zeros the stepper when the pin goes active (assumes active-low
with pull-up). The `backoffSteps` in `STEP_HOME` payload controls how far the
stepper retreats after the limit triggers.

### 10.6 Bridge-Side Liveness

The Bridge maintains its own 200 ms heartbeat timer. Logic:

```python
async def maintain_liveness():
    while running:
        time_since_last_tx = now() - last_tx_time
        if time_since_last_tx > 0.15:  # 150ms threshold → send before 200ms
            await send_tlv(SYS_HEARTBEAT, PayloadHeartbeat(timestamp=now_ms()))
        await asyncio.sleep(0.05)  # check every 50ms
```

Any outgoing TLV resets `last_tx_time`, so heartbeats are only sent when the
channel is otherwise idle.

### 10.5 TLV Type ID Summary

| Type ID | Name | Direction | Changed in v2 |
|---|---|---|---|
| 1 | `SYS_HEARTBEAT` | ↓ RPi→Arduino | Direction clarified (Arduino never sends) |
| 2 | `SYS_STATUS` | ↑ Arduino→RPi | Payload expanded |
| 3 | `SYS_CMD` | ↓ RPi→Arduino | **New** (replaces implicit state control) |
| 4 | `SYS_CONFIG` | ↓ RPi→Arduino | **New** |
| 5 | `SYS_SET_PID` | ↓ RPi→Arduino | Now accepted in RUNNING state |
| 256 | `DC_ENABLE` | ↓ | Unchanged |
| 257 | `DC_SET_POSITION` | ↓ | Unchanged |
| 258 | `DC_SET_VELOCITY` | ↓ | Unchanged |
| 259 | `DC_SET_PWM` | ↓ | **New** (was missing) |
| 260 | `DC_STATUS_ALL` | ↑ | **New** (replaces per-motor DC_STATUS=259) |
| 512 | `STEP_ENABLE` | ↓ | Unchanged |
| 513 | `STEP_SET_PARAMS` | ↓ | **Changed** (was two separate messages) |
| 514 | `STEP_MOVE` | ↓ | Renumbered |
| 515 | `STEP_HOME` | ↓ | Renumbered |
| 516 | `STEP_STATUS_ALL` | ↑ | **New** (replaces per-stepper STEP_STATUS=517) |
| 768 | `SERVO_ENABLE` | ↓ | Changed: now per-servo, not per-controller |
| 769 | `SERVO_SET` | ↓ | Unchanged |
| 770 | `SERVO_STATUS_ALL` | ↑ | **New** |
| 1024 | `SENSOR_IMU` | ↑ | Payload expanded (quaternion + global accel) |
| 1025 | `SENSOR_KINEMATICS` | ↑ | **New** |
| 1026 | `SENSOR_VOLTAGE` | ↑ | Renumbered |
| 1027 | `SENSOR_RANGE` | ↑ | Renumbered |
| 1280 | `IO_SET_LED` | ↓ | Unchanged |
| 1281 | `IO_SET_NEOPIXEL` | ↓ | Unchanged |
| 1282 | `IO_STATUS` | ↑ | **New** (replaces separate button/limit messages) |

**Removed from v1:**
- `SYS_GET_PID (4)`, `SYS_RES_PID (5)` — PID gains included in `DC_STATUS_ALL`
- `SENSOR_ENCODER (1025)`, `SENSOR_CURRENT (1026)` — folded into `DC_STATUS_ALL`
- `IO_BUTTON_STATE (1282)`, `IO_LIMIT_STATE (1283)` — folded into `IO_STATUS`

---

*End of NUEVO System Design Specification v2.0*
