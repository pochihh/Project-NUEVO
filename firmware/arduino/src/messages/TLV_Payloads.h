/**
 * @file TLV_Payloads.h
 * @brief Packed struct definitions for TLV message payloads (v2.0)
 *
 * All structs use #pragma pack(push, 1) for wire format compatibility.
 * These payloads are used for communication between Raspberry Pi and Arduino
 * via the TLV protocol over UART.
 *
 * IMPORTANT: Struct layout must match exactly on both sides (Arduino + Python).
 * Do not add padding, virtual functions, or inheritance to these structs.
 *
 * Source of truth: tlv_protocol/TLV_Payloads.md
 * Type IDs:        tlv_protocol/TLV_TypeDefs.json (auto-generated to TLV_TypeDefs.h)
 */

#ifndef TLV_PAYLOADS_H
#define TLV_PAYLOADS_H

#include <stdint.h>

// Ensure structs are tightly packed with no padding
#pragma pack(push, 1)

// ============================================================================
// SYSTEM PAYLOADS
// ============================================================================

/**
 * @brief Heartbeat payload (RPi → Arduino) — TLV type SYS_HEARTBEAT (1)
 *
 * Sent by the Bridge when no other message is pending (~200 ms interval).
 * Any received TLV resets the firmware's liveness timer.
 */
struct PayloadHeartbeat {
    uint32_t timestamp;     // RPi milliseconds since boot
    uint8_t  flags;         // Reserved (set to 0)
};
// 5 bytes

/**
 * @brief Firmware / system state enum (reported in PayloadSystemStatus.state)
 */
enum SystemState : uint8_t {
    SYS_STATE_INIT    = 0,  // Initializing
    SYS_STATE_IDLE    = 1,  // Idle — actuators disabled, config accepted
    SYS_STATE_RUNNING = 2,  // Running — full telemetry, motor commands accepted
    SYS_STATE_ERROR   = 3,  // Error — all actuators disabled, awaiting RESET
    SYS_STATE_ESTOP   = 4,  // E-STOP — all actuators disabled, hardware reset required
};

/**
 * @brief Error flags bitmask (used in PayloadSystemStatus.errorFlags)
 */
enum SystemErrorFlags : uint8_t {
    ERR_NONE           = 0x00,
    ERR_UNDERVOLTAGE   = 0x01,  // Battery below configured minimum
    ERR_OVERVOLTAGE    = 0x02,  // Battery above configured maximum
    ERR_ENCODER_FAIL   = 0x04,  // Encoder reads 0 for >500 ms while PWM > 20%
    ERR_I2C_ERROR      = 0x08,  // I2C bus error (PCA9685 or IMU)
    ERR_IMU_ERROR      = 0x10,  // IMU specifically not responding
    ERR_LIVENESS_LOST  = 0x20,  // Liveness timeout — motors cut, not in ERROR state
    ERR_LOOP_OVERRUN   = 0x40,  // Control loop exceeded deadline
};

/**
 * @brief System status payload (Arduino → RPi) — TLV type SYS_STATUS (2)
 *
 * Rate: 1 Hz (IDLE/ESTOP), 10 Hz (RUNNING/ERROR).
 */
struct PayloadSystemStatus {
    uint8_t  firmwareMajor;         // Firmware major version
    uint8_t  firmwareMinor;         // Firmware minor version
    uint8_t  firmwarePatch;         // Firmware patch version
    uint8_t  state;                 // Current SystemState enum value
    uint32_t uptimeMs;              // Arduino uptime (ms)
    uint32_t lastRxMs;              // ms since last TLV received from RPi
    uint32_t lastCmdMs;             // ms since last non-heartbeat command
    uint16_t batteryMv;             // Battery voltage (mV)
    uint16_t rail5vMv;              // 5V regulated rail (mV)
    uint8_t  errorFlags;            // SystemErrorFlags bitmask
    uint8_t  attachedSensors;       // bit0=IMU, bit1=Lidar, bit2=Ultrasonic
    uint16_t freeSram;              // Free SRAM (bytes)
    uint16_t loopTimeAvgUs;         // Average control loop time (µs)
    uint16_t loopTimeMaxUs;         // Max control loop time since last status (µs)
    uint16_t uartRxErrors;          // Cumulative UART CRC/framing error count
    float    wheelDiameterMm;       // Configured wheel diameter (mm)
    float    wheelBaseMm;           // Configured wheel base center-to-center (mm)
    uint8_t  motorDirMask;          // Direction inversion bitmask (bit N = motor N)
    uint8_t  neoPixelCount;         // Configured NeoPixel count
    uint16_t heartbeatTimeoutMs;    // Configured liveness timeout (ms)
    uint16_t limitSwitchMask;       // Bitmask: bit N = GPIO N is a limit switch
    uint8_t  stepperHomeLimitGpio[4]; // GPIO index used as home limit per stepper; 0xFF = none
};
// 48 bytes: 4×uint8 + 3×uint32 + 2×uint16 + 2×uint8 + 4×uint16 + 2×float + 2×uint8 + 2×uint16 + uint8[4]
//         = 4 + 12 + 4 + 2 + 8 + 8 + 2 + 4 + 4 = 48

/**
 * @brief System command payload (RPi → Arduino) — TLV type SYS_CMD (3)
 */
enum SysCmdType : uint8_t {
    SYS_CMD_START  = 1,  // IDLE → RUNNING
    SYS_CMD_STOP   = 2,  // RUNNING → IDLE (graceful stop, disables actuators)
    SYS_CMD_RESET  = 3,  // ERROR or ESTOP → IDLE
    SYS_CMD_ESTOP  = 4,  // Any state → ESTOP (all actuators off, stays until RESET)
};

struct PayloadSysCmd {
    uint8_t command;        // SysCmdType
    uint8_t reserved[3];    // Set to 0
};
// 4 bytes

/**
 * @brief System configuration payload (RPi → Arduino) — TLV type SYS_CONFIG (4)
 *
 * IDLE state only. Fields at sentinel value (0 or 0xFF) are not changed.
 */
struct PayloadSysConfig {
    float    wheelDiameterMm;       // Wheel diameter (mm); 0.0 = no change
    float    wheelBaseMm;           // Wheel base (mm); 0.0 = no change
    uint8_t  motorDirMask;          // Direction inversion bitmask to apply
    uint8_t  motorDirChangeMask;    // Which motors to update (bitmask)
    uint8_t  neoPixelCount;         // NeoPixel count; 0 = no change
    uint8_t  attachedSensors;       // Sensor bitmask; 0xFF = no change
    uint16_t heartbeatTimeoutMs;    // Liveness timeout (ms); 0 = no change
    uint8_t  resetOdometry;         // 1 = reset x, y, theta to (0, 0, 0)
    uint8_t  reserved;              // Set to 0
};
// 16 bytes

/**
 * @brief Set PID gains payload (RPi → Arduino) — TLV type SYS_SET_PID (5)
 *
 * Accepted in IDLE and RUNNING states. Applies to DC motors only.
 */
struct PayloadSetPID {
    uint8_t motorId;        // DC motor index (0–3)
    uint8_t loopType;       // 0 = position loop, 1 = velocity loop
    uint8_t reserved[2];    // Set to 0
    float   kp;             // Proportional gain
    float   ki;             // Integral gain
    float   kd;             // Derivative gain
    float   maxOutput;      // Output clamp (default: 255.0)
    float   maxIntegral;    // Anti-windup integral limit
};
// 24 bytes

// ============================================================================
// DC MOTOR PAYLOADS
// ============================================================================

/**
 * @brief DC motor control modes
 */
enum DCMotorMode : uint8_t {
    DC_MODE_DISABLED = 0,   // H-bridge off, motor coasts
    DC_MODE_POSITION = 1,   // Position cascade PID (outer pos, inner vel)
    DC_MODE_VELOCITY = 2,   // Velocity PID loop
    DC_MODE_PWM      = 3,   // Direct PWM (open loop)
};

/**
 * @brief DC motor enable/disable (RPi → Arduino) — TLV type DC_ENABLE (256)
 *
 * Set mode=0 to disable. Set mode=1/2/3 to enable in selected mode.
 */
struct PayloadDCEnable {
    uint8_t motorId;        // Motor index (0–3)
    uint8_t mode;           // DCMotorMode: 0=disable, 1=position, 2=velocity, 3=pwm
    uint8_t reserved[2];    // Set to 0
};
// 4 bytes

/**
 * @brief DC motor set position (RPi → Arduino) — TLV type DC_SET_POSITION (257)
 *
 * Only effective in position mode (mode=1).
 */
struct PayloadDCSetPosition {
    uint8_t  motorId;       // Motor index (0–3)
    uint8_t  reserved[3];   // Padding
    int32_t  targetTicks;   // Target position (encoder ticks)
    int32_t  maxVelTicks;   // Velocity cap during move (ticks/sec); 0 = default
};
// 12 bytes

/**
 * @brief DC motor set velocity (RPi → Arduino) — TLV type DC_SET_VELOCITY (258)
 *
 * Only effective in velocity mode (mode=2).
 */
struct PayloadDCSetVelocity {
    uint8_t  motorId;       // Motor index (0–3)
    uint8_t  reserved[3];   // Padding
    int32_t  targetTicks;   // Target velocity (ticks/sec); negative = reverse
};
// 8 bytes

/**
 * @brief DC motor set direct PWM (RPi → Arduino) — TLV type DC_SET_PWM (259)
 *
 * Only effective in PWM mode (mode=3).
 */
struct PayloadDCSetPWM {
    uint8_t  motorId;       // Motor index (0–3)
    uint8_t  reserved;      // Set to 0
    int16_t  pwm;           // PWM value: -255 to +255 (sign = direction)
};
// 4 bytes

/**
 * @brief Per-motor status sub-struct for DC_STATUS_ALL
 */
struct DCMotorStatus {
    uint8_t  mode;          // Current DCMotorMode (0=disabled, 1=pos, 2=vel, 3=pwm)
    uint8_t  faultFlags;    // bit0=overcurrent, bit1=stall
    int32_t  position;      // Current position (encoder ticks, measured)
    int32_t  velocity;      // Current velocity (ticks/sec, measured)
    int32_t  targetPos;     // Current target position
    int32_t  targetVel;     // Current target velocity
    int16_t  pwmOutput;     // Actual PWM output (-255 to +255)
    int16_t  currentMa;     // Motor current (mA); -1 if not measured
    float    posKp;         // Position loop Kp
    float    posKi;         // Position loop Ki
    float    posKd;         // Position loop Kd
    float    velKp;         // Velocity loop Kp
    float    velKi;         // Velocity loop Ki
    float    velKd;         // Velocity loop Kd
};
// 2 + 4*4 + 2*2 + 6*4 = 2 + 16 + 4 + 24 = 46 bytes

/**
 * @brief DC motor status for all motors (Arduino → RPi) — TLV type DC_STATUS_ALL (260)
 *
 * Rate: 100 Hz in RUNNING state.
 */
struct PayloadDCStatusAll {
    DCMotorStatus motors[4];    // Motors 0–3 in order
};
// 184 bytes

// ============================================================================
// STEPPER MOTOR PAYLOADS
// ============================================================================

/**
 * @brief Stepper motor enable/disable (RPi → Arduino) — TLV type STEP_ENABLE (512)
 */
struct PayloadStepEnable {
    uint8_t stepperId;      // Stepper index (0–3)
    uint8_t enable;         // 0 = disable (coil off), 1 = enable (coil on, holds)
    uint8_t reserved[2];    // Set to 0
};
// 4 bytes

/**
 * @brief Stepper set motion parameters (RPi → Arduino) — TLV type STEP_SET_PARAMS (513)
 *
 * Replaces the old STEP_SET_ACCEL and STEP_SET_VEL messages.
 */
struct PayloadStepSetParams {
    uint8_t  stepperId;     // Stepper index (0–3)
    uint8_t  reserved[3];   // Set to 0
    uint32_t maxVelocity;   // Maximum speed (steps/sec)
    uint32_t acceleration;  // Acceleration and deceleration (steps/sec²)
};
// 12 bytes

/**
 * @brief Stepper move command (RPi → Arduino) — TLV type STEP_MOVE (514)
 */
enum StepMoveType : uint8_t {
    STEP_MOVE_ABSOLUTE = 0, // Move to absolute step count
    STEP_MOVE_RELATIVE = 1, // Move by relative steps
};

struct PayloadStepMove {
    uint8_t  stepperId;     // Stepper index (0–3)
    uint8_t  moveType;      // StepMoveType
    uint8_t  reserved[2];   // Set to 0
    int32_t  target;        // Target (absolute ticks or relative steps)
};
// 8 bytes

/**
 * @brief Stepper homing command (RPi → Arduino) — TLV type STEP_HOME (515)
 *
 * Move stepper toward limit switch, then zero position and back off.
 */
struct PayloadStepHome {
    uint8_t  stepperId;     // Stepper index (0–3)
    int8_t   direction;     // -1 = reverse, +1 = forward
    uint8_t  reserved[2];   // Set to 0
    uint32_t homeVelocity;  // Homing speed (steps/sec); should be slow
    int32_t  backoffSteps;  // Steps to retreat after limit triggered
};
// 12 bytes

/**
 * @brief Stepper motion states (used in StepperStatus.motionState)
 */
enum StepperState : uint8_t {
    STEPPER_IDLE    = 0,    // Not moving
    STEPPER_ACCEL   = 1,    // Accelerating
    STEPPER_CRUISE  = 2,    // Constant velocity
    STEPPER_DECEL   = 3,    // Decelerating
    STEPPER_HOMING  = 4,    // Homing sequence active
    STEPPER_FAULT   = 5,    // Fault condition
};

/**
 * @brief Per-stepper status sub-struct for STEP_STATUS_ALL
 *
 * commandedCount is the firmware's internal step counter (open-loop, not measured).
 */
struct StepperStatus {
    uint8_t  enabled;           // 0 = disabled, 1 = enabled
    uint8_t  motionState;       // StepperState enum value
    uint8_t  limitHit;          // bit0 = min limit, bit1 = max limit
    uint8_t  reserved;
    int32_t  commandedCount;    // Commanded step count (open-loop)
    int32_t  targetCount;       // Current move target
    uint32_t currentSpeed;      // Current speed (steps/sec)
    uint32_t maxSpeed;          // Configured max speed
    uint32_t acceleration;      // Configured acceleration
};
// 4 + 3*4 + 2*4 = 4 + 12 + 8 = 24 bytes

/**
 * @brief Stepper status for all steppers (Arduino → RPi) — TLV type STEP_STATUS_ALL (516)
 *
 * Rate: 100 Hz in RUNNING state.
 */
struct PayloadStepStatusAll {
    StepperStatus steppers[4];  // Steppers 0–3 in order
};
// 96 bytes

// ============================================================================
// SERVO PAYLOADS
// ============================================================================

/**
 * @brief Servo channel enable/disable (RPi → Arduino) — TLV type SERVO_ENABLE (768)
 *
 * Per-channel control. All channels start disabled on reset.
 * channel=0xFF enables/disables all 16 channels at once.
 */
struct PayloadServoEnable {
    uint8_t channel;        // Servo channel (0–15); 0xFF = all channels
    uint8_t enable;         // 0 = disable (no pulse), 1 = enable
    uint8_t reserved[2];    // Set to 0
};
// 4 bytes

/**
 * @brief Servo set position — single channel (RPi → Arduino) — TLV type SERVO_SET (769)
 *
 * Use count=1 for a single servo or count>1 for consecutive bulk update.
 * Dispatcher reads 'count' to select single (4 bytes) or bulk (34 bytes).
 */
struct PayloadServoSetSingle {
    uint8_t  channel;       // Servo channel (0–15)
    uint8_t  count;         // 1 for single-channel update
    uint16_t pulseUs;       // Pulse width (µs), typically 500–2500
};
// 4 bytes

/**
 * @brief Servo bulk update — consecutive channels (RPi → Arduino) — TLV type SERVO_SET (769)
 */
struct PayloadServoSetBulk {
    uint8_t  startChannel;  // First channel to update
    uint8_t  count;         // Number of consecutive channels (1–16)
    uint16_t pulseUs[16];   // Pulse widths; only first 'count' values used
};
// 34 bytes

/**
 * @brief Servo status all channels (Arduino → RPi) — TLV type SERVO_STATUS_ALL (770)
 *
 * Rate: 50 Hz in RUNNING state. Reports commanded state only (no feedback).
 */
struct PayloadServoStatusAll {
    uint8_t  pca9685Connected;  // 0 = not detected on I2C, 1 = connected
    uint8_t  pca9685Error;      // Error flags (0 = no error)
    uint16_t enabledMask;       // Bitmask of enabled channels (bit N = channel N)
    uint16_t pulseUs[16];       // Commanded pulse width per channel (0 = disabled)
};
// 36 bytes

// ============================================================================
// SENSOR PAYLOADS (Arduino → RPi)
// ============================================================================

/**
 * @brief IMU sensor data (Arduino → RPi) — TLV type SENSOR_IMU (1024)
 *
 * Rate: 100 Hz in RUNNING state (if IMU attached).
 * Quaternion and earth-frame acceleration from Fusion AHRS (Madgwick).
 * In 9-DOF mode (mag calibrated) yaw is north-referenced; 6-DOF = yaw drifts.
 */
struct PayloadSensorIMU {
    float    quatW;         // Orientation quaternion W
    float    quatX;         // Orientation quaternion X
    float    quatY;         // Orientation quaternion Y
    float    quatZ;         // Orientation quaternion Z
    float    earthAccX;     // Earth-frame linear accel X (g, gravity removed)
    float    earthAccY;     // Earth-frame linear accel Y (g)
    float    earthAccZ;     // Earth-frame linear accel Z (g)
    int16_t  rawAccX;       // Accelerometer X (mg, cast of accX())
    int16_t  rawAccY;       // Accelerometer Y (mg)
    int16_t  rawAccZ;       // Accelerometer Z (mg)
    int16_t  rawGyroX;      // Gyroscope X (0.1 DPS units, gyrX()*10 cast)
    int16_t  rawGyroY;      // Gyroscope Y (0.1 DPS units)
    int16_t  rawGyroZ;      // Gyroscope Z (0.1 DPS units)
    int16_t  magX;          // Magnetometer X (µT, calibration offset applied)
    int16_t  magY;          // Magnetometer Y (µT)
    int16_t  magZ;          // Magnetometer Z (µT)
    uint8_t  magCalibrated; // 0 = 6-DOF (no mag cal), 1 = 9-DOF (mag cal active)
    uint8_t  reserved;      // Set to 0
    uint32_t timestamp;     // Arduino micros() at sample time
};
// 7*4 + 9*2 + 2*1 + 4 = 28 + 18 + 2 + 4 = 52 bytes

/**
 * @brief Robot kinematics from wheel odometry (Arduino → RPi) — TLV type SENSOR_KINEMATICS (1025)
 *
 * Rate: 100 Hz in RUNNING state.
 * Computed from encoder counts, wheelDiameterMm, wheelBaseMm.
 * Resets to (0, 0, 0) on firmware reset or SYS_CONFIG(resetOdometry=1).
 */
struct PayloadSensorKinematics {
    float    x;             // Position X from start (mm)
    float    y;             // Position Y from start (mm)
    float    theta;         // Heading (radians, CCW positive from start)
    float    vx;            // Velocity X in robot frame (mm/s)
    float    vy;            // Velocity Y in robot frame (mm/s, always 0 for diff drive)
    float    vTheta;        // Angular velocity (rad/s, CCW positive)
    uint32_t timestamp;     // Arduino micros() at compute time
};
// 6*4 + 4 = 28 bytes

/**
 * @brief Battery and rail voltages (Arduino → RPi) — TLV type SENSOR_VOLTAGE (1026)
 *
 * Rate: 10 Hz in RUNNING and ERROR states.
 */
struct PayloadSensorVoltage {
    uint16_t batteryMv;     // Battery input voltage (mV)
    uint16_t rail5vMv;      // 5V regulated rail (mV)
    uint16_t servoRailMv;   // Servo power rail (mV); 0 if not measured
    uint16_t reserved;      // Reserved (set to 0)
};
// 8 bytes

/**
 * @brief Range sensor reading (Arduino → RPi) — TLV type SENSOR_RANGE (1027)
 *
 * One message per sensor reading. Rate depends on sensor polling frequency.
 */
struct PayloadSensorRange {
    uint8_t  sensorId;      // Sensor index (0-based)
    uint8_t  sensorType;    // 0 = ultrasonic, 1 = lidar
    uint8_t  status;        // 0 = valid, 1 = out of range, 2 = sensor error
    uint8_t  reserved;      // Set to 0
    uint16_t distanceMm;    // Distance (mm); 0 if status ≠ 0
    uint16_t reserved2;     // Set to 0
    uint32_t timestamp;     // Arduino micros() at measurement time
};
// 12 bytes

/**
 * @brief Magnetometer calibration command (RPi → Arduino) — TLV type SENSOR_MAG_CAL_CMD (1028)
 *
 * IDLE state only. Ignored in RUNNING.
 */
enum MagCalCmdType : uint8_t {
    MAG_CAL_START = 1,  // Start sampling; Arduino streams SENSOR_MAG_CAL_STATUS at ~10 Hz
    MAG_CAL_STOP  = 2,  // Stop sampling without saving
    MAG_CAL_SAVE  = 3,  // Save computed offsets to EEPROM and activate 9-DOF
    MAG_CAL_APPLY = 4,  // Apply user-provided offsets (from payload) and save
    MAG_CAL_CLEAR = 5,  // Clear EEPROM calibration; revert to 6-DOF mode
};

struct PayloadMagCalCmd {
    uint8_t  command;       // MagCalCmdType
    uint8_t  reserved[3];   // Set to 0
    float    offsetX;       // Hard-iron offset X (µT); used only with MAG_CAL_APPLY
    float    offsetY;       // Hard-iron offset Y (µT)
    float    offsetZ;       // Hard-iron offset Z (µT)
};
// 16 bytes

/**
 * @brief Magnetometer calibration status (Arduino → RPi) — TLV type SENSOR_MAG_CAL_STATUS (1029)
 *
 * Sent at ~10 Hz while calibration is active. Also sent once on complete/cancel.
 */
struct PayloadMagCalStatus {
    uint8_t  state;         // MagCalState: 0=idle, 1=sampling, 2=complete, 3=saved, 4=error
    uint16_t sampleCount;   // Number of magnetometer samples collected
    uint8_t  reserved;      // Set to 0
    float    minX;          // Current minimum X seen (µT)
    float    maxX;          // Current maximum X seen (µT)
    float    minY;          // Current minimum Y seen (µT)
    float    maxY;          // Current maximum Y seen (µT)
    float    minZ;          // Current minimum Z seen (µT)
    float    maxZ;          // Current maximum Z seen (µT)
    float    offsetX;       // Computed hard-iron offset X = (maxX+minX)/2
    float    offsetY;       // Computed hard-iron offset Y
    float    offsetZ;       // Computed hard-iron offset Z
    uint8_t  savedToEeprom; // 1 = offsets loaded from / saved to EEPROM
    uint8_t  reserved2[3];  // Set to 0
};
// 4 + 9*4 + 4 = 44 bytes

// ============================================================================
// USER I/O PAYLOADS
// ============================================================================

/**
 * @brief Set LED state (RPi → Arduino) — TLV type IO_SET_LED (1280)
 */
enum LEDMode : uint8_t {
    LED_OFF     = 0,    // LED off
    LED_ON      = 1,    // LED on (full brightness)
    LED_BLINK   = 2,    // Blinking (square wave)
    LED_BREATHE = 3,    // Breathing (sine wave fade) — falls back to blink for Timer3 LEDs
    LED_PWM     = 4,    // Constant PWM (dimming)
};

struct PayloadSetLED {
    uint8_t  ledId;         // LED index (see pins.h)
    uint8_t  mode;          // LEDMode
    uint8_t  brightness;    // Brightness 0–255 (for PWM/breathe modes)
    uint8_t  reserved;      // Set to 0
    uint16_t periodMs;      // Blink/breathe period (ms)
    uint16_t dutyCycle;     // Blink on-time: 0–1000 represents 0.0–100.0%
};
// 8 bytes

/**
 * @brief Set NeoPixel color (RPi → Arduino) — TLV type IO_SET_NEOPIXEL (1281)
 *
 * index=0xFF sets all pixels simultaneously.
 */
struct PayloadSetNeoPixel {
    uint8_t index;          // Pixel index (0-based); 0xFF = all pixels
    uint8_t red;            // Red component (0–255)
    uint8_t green;          // Green component (0–255)
    uint8_t blue;           // Blue component (0–255)
};
// 4 bytes

/**
 * @brief All UserIO state (Arduino → RPi) — TLV type IO_STATUS (1282)
 *
 * Rate: 100 Hz in RUNNING state.
 *
 * buttonMask covers all digital input GPIOs (buttons AND limit switches).
 * Use SYS_STATUS.limitSwitchMask and stepperHomeLimitGpio[] to distinguish them.
 *
 * NeoPixel RGB data (neoPixelCount×3 bytes) is appended after the fixed fields.
 * Total payload = 10 + (neoPixelCount × 3) bytes.
 */
struct PayloadIOStatus {
    uint16_t buttonMask;        // All digital input GPIOs (bit N = GPIO N pressed)
    uint8_t  ledBrightness[3];  // Brightness of each user LED (0=off, 255=full)
    uint8_t  reserved;          // Set to 0
    uint32_t timestamp;         // Arduino millis()
    // uint8_t neoPixelRGB[neoPixelCount * 3]; — appended after fixed fields
};
// 10 bytes fixed (+ 3 × neoPixelCount appended)

// ============================================================================
// END OF PACKED STRUCTS
// ============================================================================

#pragma pack(pop)

// ============================================================================
// PAYLOAD SIZE VALIDATION
// ============================================================================

#define STATIC_ASSERT_SIZE(type, expected) \
    static_assert(sizeof(type) == expected, #type " size mismatch")

// System payloads
STATIC_ASSERT_SIZE(PayloadHeartbeat,    5);
STATIC_ASSERT_SIZE(PayloadSystemStatus, 48);
STATIC_ASSERT_SIZE(PayloadSysCmd,       4);
STATIC_ASSERT_SIZE(PayloadSysConfig,    16);
STATIC_ASSERT_SIZE(PayloadSetPID,       24);

// DC Motor payloads
STATIC_ASSERT_SIZE(PayloadDCEnable,     4);
STATIC_ASSERT_SIZE(PayloadDCSetPosition, 12);
STATIC_ASSERT_SIZE(PayloadDCSetVelocity, 8);
STATIC_ASSERT_SIZE(PayloadDCSetPWM,     4);
STATIC_ASSERT_SIZE(DCMotorStatus,       46);
STATIC_ASSERT_SIZE(PayloadDCStatusAll,  184);

// Stepper payloads
STATIC_ASSERT_SIZE(PayloadStepEnable,   4);
STATIC_ASSERT_SIZE(PayloadStepSetParams, 12);
STATIC_ASSERT_SIZE(PayloadStepMove,     8);
STATIC_ASSERT_SIZE(PayloadStepHome,     12);
STATIC_ASSERT_SIZE(StepperStatus,       24);
STATIC_ASSERT_SIZE(PayloadStepStatusAll, 96);

// Servo payloads
STATIC_ASSERT_SIZE(PayloadServoEnable,  4);
STATIC_ASSERT_SIZE(PayloadServoSetSingle, 4);
STATIC_ASSERT_SIZE(PayloadServoSetBulk, 34);
STATIC_ASSERT_SIZE(PayloadServoStatusAll, 36);

// Sensor payloads
STATIC_ASSERT_SIZE(PayloadSensorIMU,    52);
STATIC_ASSERT_SIZE(PayloadSensorKinematics, 28);
STATIC_ASSERT_SIZE(PayloadSensorVoltage, 8);
STATIC_ASSERT_SIZE(PayloadSensorRange,  12);
STATIC_ASSERT_SIZE(PayloadMagCalCmd,    16);
STATIC_ASSERT_SIZE(PayloadMagCalStatus, 44);

// I/O payloads
STATIC_ASSERT_SIZE(PayloadSetLED,       8);
STATIC_ASSERT_SIZE(PayloadSetNeoPixel,  4);
STATIC_ASSERT_SIZE(PayloadIOStatus,     10);

#endif // TLV_PAYLOADS_H
