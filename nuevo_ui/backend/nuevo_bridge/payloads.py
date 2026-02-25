"""
TLV Payload Structures (ctypes)

Exact mirror of firmware/arduino/src/messages/TLV_Payloads.h.
All struct sizes are verified against STATIC_ASSERT_SIZE values in the firmware.

_pack_ = 1 ensures no padding, matching #pragma pack(push, 1) on the Arduino side.

Source of truth: tlv_protocol/TLV_Payloads.md
"""
import ctypes


# ============================================================================
# SYSTEM PAYLOADS
# ============================================================================

class PayloadHeartbeat(ctypes.Structure):
    """
    TLV Type: SYS_HEARTBEAT (1)  Direction: RPi → Arduino
    Size: 5 bytes
    """
    _pack_ = 1
    _fields_ = [
        ("timestamp", ctypes.c_uint32),  # RPi ms since boot
        ("flags",     ctypes.c_uint8),   # Reserved, set to 0
    ]


class PayloadSystemStatus(ctypes.Structure):
    """
    TLV Type: SYS_STATUS (2)  Direction: Arduino → RPi
    Size: 48 bytes
    Rate: 1 Hz (IDLE/ESTOP), 10 Hz (RUNNING/ERROR)
    """
    _pack_ = 1
    _fields_ = [
        ("firmwareMajor",          ctypes.c_uint8),
        ("firmwareMinor",          ctypes.c_uint8),
        ("firmwarePatch",          ctypes.c_uint8),
        ("state",                  ctypes.c_uint8),    # SystemState enum
        ("uptimeMs",               ctypes.c_uint32),
        ("lastRxMs",               ctypes.c_uint32),   # ms since last TLV rx
        ("lastCmdMs",              ctypes.c_uint32),   # ms since last non-HB cmd
        ("batteryMv",              ctypes.c_uint16),
        ("rail5vMv",               ctypes.c_uint16),
        ("errorFlags",             ctypes.c_uint8),    # SystemErrorFlags bitmask
        ("attachedSensors",        ctypes.c_uint8),    # bit0=IMU, bit1=Lidar, bit2=US
        ("freeSram",               ctypes.c_uint16),
        ("loopTimeAvgUs",          ctypes.c_uint16),
        ("loopTimeMaxUs",          ctypes.c_uint16),
        ("uartRxErrors",           ctypes.c_uint16),
        ("wheelDiameterMm",        ctypes.c_float),
        ("wheelBaseMm",            ctypes.c_float),
        ("motorDirMask",           ctypes.c_uint8),    # Direction inversion bitmask
        ("neoPixelCount",          ctypes.c_uint8),
        ("heartbeatTimeoutMs",     ctypes.c_uint16),
        ("limitSwitchMask",        ctypes.c_uint16),
        ("stepperHomeLimitGpio",   ctypes.c_uint8 * 4),  # 0xFF = none
    ]


class PayloadSysCmd(ctypes.Structure):
    """
    TLV Type: SYS_CMD (3)  Direction: RPi → Arduino
    Size: 4 bytes
    SysCmdType: 1=START, 2=STOP, 3=RESET, 4=ESTOP
    """
    _pack_ = 1
    _fields_ = [
        ("command",  ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 3),
    ]


class PayloadSysConfig(ctypes.Structure):
    """
    TLV Type: SYS_CONFIG (4)  Direction: RPi → Arduino
    Size: 16 bytes
    IDLE state only. Fields at sentinel (0 / 0xFF) are not changed.
    """
    _pack_ = 1
    _fields_ = [
        ("wheelDiameterMm",    ctypes.c_float),   # 0.0 = no change
        ("wheelBaseMm",        ctypes.c_float),   # 0.0 = no change
        ("motorDirMask",       ctypes.c_uint8),
        ("motorDirChangeMask", ctypes.c_uint8),   # Which motors to update
        ("neoPixelCount",      ctypes.c_uint8),   # 0 = no change
        ("attachedSensors",    ctypes.c_uint8),   # 0xFF = no change
        ("heartbeatTimeoutMs", ctypes.c_uint16),  # 0 = no change
        ("resetOdometry",      ctypes.c_uint8),   # 1 = reset x/y/theta to 0
        ("reserved",           ctypes.c_uint8),
    ]


class PayloadSetPID(ctypes.Structure):
    """
    TLV Type: SYS_SET_PID (5)  Direction: RPi → Arduino
    Size: 24 bytes
    Applies to DC motors only. motorId is 0-based wire index.
    loopType: 0=position, 1=velocity
    """
    _pack_ = 1
    _fields_ = [
        ("motorId",    ctypes.c_uint8),
        ("loopType",   ctypes.c_uint8),
        ("reserved",   ctypes.c_uint8 * 2),
        ("kp",         ctypes.c_float),
        ("ki",         ctypes.c_float),
        ("kd",         ctypes.c_float),
        ("maxOutput",  ctypes.c_float),
        ("maxIntegral", ctypes.c_float),
    ]


# ============================================================================
# DC MOTOR PAYLOADS
# ============================================================================

class PayloadDCEnable(ctypes.Structure):
    """
    TLV Type: DC_ENABLE (256)  Direction: RPi → Arduino
    Size: 4 bytes
    mode: 0=disable, 1=position, 2=velocity, 3=pwm
    """
    _pack_ = 1
    _fields_ = [
        ("motorId",  ctypes.c_uint8),
        ("mode",     ctypes.c_uint8),   # DCMotorMode enum
        ("reserved", ctypes.c_uint8 * 2),
    ]


class PayloadDCSetPosition(ctypes.Structure):
    """
    TLV Type: DC_SET_POSITION (257)  Direction: RPi → Arduino
    Size: 12 bytes
    Only effective in position mode (mode=1).
    """
    _pack_ = 1
    _fields_ = [
        ("motorId",     ctypes.c_uint8),
        ("reserved",    ctypes.c_uint8 * 3),
        ("targetTicks", ctypes.c_int32),   # Target position (encoder ticks)
        ("maxVelTicks", ctypes.c_int32),   # Velocity cap; 0 = default
    ]


class PayloadDCSetVelocity(ctypes.Structure):
    """
    TLV Type: DC_SET_VELOCITY (258)  Direction: RPi → Arduino
    Size: 8 bytes
    Only effective in velocity mode (mode=2).
    """
    _pack_ = 1
    _fields_ = [
        ("motorId",     ctypes.c_uint8),
        ("reserved",    ctypes.c_uint8 * 3),
        ("targetTicks", ctypes.c_int32),   # Target velocity (ticks/sec); neg = reverse
    ]


class PayloadDCSetPWM(ctypes.Structure):
    """
    TLV Type: DC_SET_PWM (259)  Direction: RPi → Arduino
    Size: 4 bytes
    Only effective in PWM mode (mode=3).
    """
    _pack_ = 1
    _fields_ = [
        ("motorId",  ctypes.c_uint8),
        ("reserved", ctypes.c_uint8),
        ("pwm",      ctypes.c_int16),   # -255 to +255 (sign = direction)
    ]


class DCMotorStatus(ctypes.Structure):
    """
    Sub-struct for PayloadDCStatusAll.
    Size: 46 bytes per motor.
    """
    _pack_ = 1
    _fields_ = [
        ("mode",       ctypes.c_uint8),   # DCMotorMode
        ("faultFlags", ctypes.c_uint8),   # bit0=overcurrent, bit1=stall
        ("position",   ctypes.c_int32),   # Current position (ticks, measured)
        ("velocity",   ctypes.c_int32),   # Current velocity (ticks/sec, measured)
        ("targetPos",  ctypes.c_int32),   # Current target position
        ("targetVel",  ctypes.c_int32),   # Current target velocity
        ("pwmOutput",  ctypes.c_int16),   # Actual PWM output (-255 to +255)
        ("currentMa",  ctypes.c_int16),   # Motor current (mA); -1 if not measured
        ("posKp",      ctypes.c_float),
        ("posKi",      ctypes.c_float),
        ("posKd",      ctypes.c_float),
        ("velKp",      ctypes.c_float),
        ("velKi",      ctypes.c_float),
        ("velKd",      ctypes.c_float),
    ]


class PayloadDCStatusAll(ctypes.Structure):
    """
    TLV Type: DC_STATUS_ALL (260)  Direction: Arduino → RPi
    Size: 184 bytes (4 × 46)
    Rate: 100 Hz in RUNNING state.
    """
    _pack_ = 1
    _fields_ = [
        ("motors", DCMotorStatus * 4),
    ]


# ============================================================================
# STEPPER MOTOR PAYLOADS
# ============================================================================

class PayloadStepEnable(ctypes.Structure):
    """
    TLV Type: STEP_ENABLE (512)  Direction: RPi → Arduino
    Size: 4 bytes
    enable: 0=disable (coil off), 1=enable (coil on, holds)
    """
    _pack_ = 1
    _fields_ = [
        ("stepperId", ctypes.c_uint8),
        ("enable",    ctypes.c_uint8),
        ("reserved",  ctypes.c_uint8 * 2),
    ]


class PayloadStepSetParams(ctypes.Structure):
    """
    TLV Type: STEP_SET_PARAMS (513)  Direction: RPi → Arduino
    Size: 12 bytes
    Replaces old STEP_SET_ACCEL + STEP_SET_VEL messages.
    """
    _pack_ = 1
    _fields_ = [
        ("stepperId",    ctypes.c_uint8),
        ("reserved",     ctypes.c_uint8 * 3),
        ("maxVelocity",  ctypes.c_uint32),   # steps/sec
        ("acceleration", ctypes.c_uint32),   # steps/sec²
    ]


class PayloadStepMove(ctypes.Structure):
    """
    TLV Type: STEP_MOVE (514)  Direction: RPi → Arduino
    Size: 8 bytes
    moveType: 0=absolute, 1=relative
    """
    _pack_ = 1
    _fields_ = [
        ("stepperId", ctypes.c_uint8),
        ("moveType",  ctypes.c_uint8),
        ("reserved",  ctypes.c_uint8 * 2),
        ("target",    ctypes.c_int32),
    ]


class PayloadStepHome(ctypes.Structure):
    """
    TLV Type: STEP_HOME (515)  Direction: RPi → Arduino
    Size: 12 bytes
    """
    _pack_ = 1
    _fields_ = [
        ("stepperId",    ctypes.c_uint8),
        ("direction",    ctypes.c_int8),    # -1=reverse, +1=forward
        ("reserved",     ctypes.c_uint8 * 2),
        ("homeVelocity", ctypes.c_uint32),  # steps/sec (slow)
        ("backoffSteps", ctypes.c_int32),   # Steps to retreat after limit hit
    ]


class StepperStatus(ctypes.Structure):
    """
    Sub-struct for PayloadStepStatusAll.
    Size: 24 bytes per stepper.
    StepperState: 0=idle, 1=accel, 2=cruise, 3=decel, 4=homing, 5=fault
    """
    _pack_ = 1
    _fields_ = [
        ("enabled",        ctypes.c_uint8),
        ("motionState",    ctypes.c_uint8),   # StepperState enum
        ("limitHit",       ctypes.c_uint8),   # bit0=min, bit1=max
        ("reserved",       ctypes.c_uint8),
        ("commandedCount", ctypes.c_int32),   # Commanded step count (open-loop)
        ("targetCount",    ctypes.c_int32),   # Current move target
        ("currentSpeed",   ctypes.c_uint32),  # steps/sec
        ("maxSpeed",       ctypes.c_uint32),  # Configured max speed
        ("acceleration",   ctypes.c_uint32),  # Configured acceleration
    ]


class PayloadStepStatusAll(ctypes.Structure):
    """
    TLV Type: STEP_STATUS_ALL (516)  Direction: Arduino → RPi
    Size: 96 bytes (4 × 24)
    Rate: 100 Hz in RUNNING state.
    """
    _pack_ = 1
    _fields_ = [
        ("steppers", StepperStatus * 4),
    ]


# ============================================================================
# SERVO PAYLOADS
# ============================================================================

class PayloadServoEnable(ctypes.Structure):
    """
    TLV Type: SERVO_ENABLE (768)  Direction: RPi → Arduino
    Size: 4 bytes
    channel=0xFF enables/disables all 16 channels at once.
    """
    _pack_ = 1
    _fields_ = [
        ("channel",  ctypes.c_uint8),   # 0–15; 0xFF = all
        ("enable",   ctypes.c_uint8),   # 0=disable, 1=enable
        ("reserved", ctypes.c_uint8 * 2),
    ]


class PayloadServoSetSingle(ctypes.Structure):
    """
    TLV Type: SERVO_SET (769)  Direction: RPi → Arduino
    Size: 4 bytes  (count must be 1)
    """
    _pack_ = 1
    _fields_ = [
        ("channel",  ctypes.c_uint8),
        ("count",    ctypes.c_uint8),   # Must be 1 for this variant
        ("pulseUs",  ctypes.c_uint16),  # 500–2500 µs typical
    ]


class PayloadServoSetBulk(ctypes.Structure):
    """
    TLV Type: SERVO_SET (769)  Direction: RPi → Arduino
    Size: 34 bytes  (count > 1)
    Bulk consecutive channel update.
    """
    _pack_ = 1
    _fields_ = [
        ("startChannel", ctypes.c_uint8),
        ("count",        ctypes.c_uint8),    # 1–16 channels
        ("pulseUs",      ctypes.c_uint16 * 16),  # Only first 'count' values used
    ]


class PayloadServoStatusAll(ctypes.Structure):
    """
    TLV Type: SERVO_STATUS_ALL (770)  Direction: Arduino → RPi
    Size: 36 bytes
    Rate: 50 Hz in RUNNING state.
    """
    _pack_ = 1
    _fields_ = [
        ("pca9685Connected", ctypes.c_uint8),
        ("pca9685Error",     ctypes.c_uint8),
        ("enabledMask",      ctypes.c_uint16),      # Bit N = channel N enabled
        ("pulseUs",          ctypes.c_uint16 * 16), # Commanded pulse per channel
    ]


# ============================================================================
# SENSOR PAYLOADS (Arduino → RPi)
# ============================================================================

class PayloadSensorIMU(ctypes.Structure):
    """
    TLV Type: SENSOR_IMU (1024)  Direction: Arduino → RPi
    Size: 52 bytes
    Rate: 100 Hz in RUNNING state (if IMU attached).
    Quaternion and earth-frame acceleration from Fusion AHRS (Madgwick).
    """
    _pack_ = 1
    _fields_ = [
        ("quatW",        ctypes.c_float),   # Orientation quaternion W
        ("quatX",        ctypes.c_float),   # Orientation quaternion X
        ("quatY",        ctypes.c_float),   # Orientation quaternion Y
        ("quatZ",        ctypes.c_float),   # Orientation quaternion Z
        ("earthAccX",    ctypes.c_float),   # Earth-frame linear accel X (g)
        ("earthAccY",    ctypes.c_float),   # Earth-frame linear accel Y (g)
        ("earthAccZ",    ctypes.c_float),   # Earth-frame linear accel Z (g)
        ("rawAccX",      ctypes.c_int16),   # Accelerometer X (mg)
        ("rawAccY",      ctypes.c_int16),   # Accelerometer Y (mg)
        ("rawAccZ",      ctypes.c_int16),   # Accelerometer Z (mg)
        ("rawGyroX",     ctypes.c_int16),   # Gyroscope X (0.1 DPS units)
        ("rawGyroY",     ctypes.c_int16),   # Gyroscope Y (0.1 DPS units)
        ("rawGyroZ",     ctypes.c_int16),   # Gyroscope Z (0.1 DPS units)
        ("magX",         ctypes.c_int16),   # Magnetometer X (µT)
        ("magY",         ctypes.c_int16),   # Magnetometer Y (µT)
        ("magZ",         ctypes.c_int16),   # Magnetometer Z (µT)
        ("magCalibrated", ctypes.c_uint8),  # 0=6-DOF, 1=9-DOF (mag cal active)
        ("reserved",     ctypes.c_uint8),
        ("timestamp",    ctypes.c_uint32),  # Arduino micros() at sample time
    ]


class PayloadSensorKinematics(ctypes.Structure):
    """
    TLV Type: SENSOR_KINEMATICS (1025)  Direction: Arduino → RPi
    Size: 28 bytes
    Rate: 100 Hz in RUNNING state.
    Differential-drive wheel odometry. Resets on SYS_CONFIG(resetOdometry=1).
    """
    _pack_ = 1
    _fields_ = [
        ("x",         ctypes.c_float),    # Position X from start (mm)
        ("y",         ctypes.c_float),    # Position Y from start (mm)
        ("theta",     ctypes.c_float),    # Heading (rad, CCW positive)
        ("vx",        ctypes.c_float),    # Linear velocity in robot frame (mm/s)
        ("vy",        ctypes.c_float),    # Lateral velocity (mm/s, always ≈0 for diff drive)
        ("vTheta",    ctypes.c_float),    # Angular velocity (rad/s, CCW positive)
        ("timestamp", ctypes.c_uint32),   # Arduino micros()
    ]


class PayloadSensorVoltage(ctypes.Structure):
    """
    TLV Type: SENSOR_VOLTAGE (1026)  Direction: Arduino → RPi
    Size: 8 bytes
    Rate: 10 Hz in RUNNING and ERROR states.
    """
    _pack_ = 1
    _fields_ = [
        ("batteryMv",   ctypes.c_uint16),
        ("rail5vMv",    ctypes.c_uint16),
        ("servoRailMv", ctypes.c_uint16),
        ("reserved",    ctypes.c_uint16),
    ]


class PayloadSensorRange(ctypes.Structure):
    """
    TLV Type: SENSOR_RANGE (1027)  Direction: Arduino → RPi
    Size: 12 bytes
    sensorType: 0=ultrasonic, 1=lidar
    status: 0=valid, 1=out of range, 2=sensor error
    """
    _pack_ = 1
    _fields_ = [
        ("sensorId",   ctypes.c_uint8),
        ("sensorType", ctypes.c_uint8),
        ("status",     ctypes.c_uint8),
        ("reserved",   ctypes.c_uint8),
        ("distanceMm", ctypes.c_uint16),
        ("reserved2",  ctypes.c_uint16),
        ("timestamp",  ctypes.c_uint32),
    ]


class PayloadMagCalCmd(ctypes.Structure):
    """
    TLV Type: SENSOR_MAG_CAL_CMD (1028)  Direction: RPi → Arduino
    Size: 16 bytes
    command: 1=START, 2=STOP, 3=SAVE, 4=APPLY (use offsetX/Y/Z), 5=CLEAR
    IDLE state only. Ignored in RUNNING.
    """
    _pack_ = 1
    _fields_ = [
        ("command",  ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 3),
        ("offsetX",  ctypes.c_float),   # Hard-iron offset X (µT); MAG_CAL_APPLY only
        ("offsetY",  ctypes.c_float),   # Hard-iron offset Y (µT)
        ("offsetZ",  ctypes.c_float),   # Hard-iron offset Z (µT)
    ]


class PayloadMagCalStatus(ctypes.Structure):
    """
    TLV Type: SENSOR_MAG_CAL_STATUS (1029)  Direction: Arduino → RPi
    Size: 44 bytes
    state: 0=idle, 1=sampling, 2=complete, 3=saved, 4=error
    Sent at ~10 Hz while calibration is active.
    """
    _pack_ = 1
    _fields_ = [
        ("state",        ctypes.c_uint8),
        ("sampleCount",  ctypes.c_uint16),
        ("reserved",     ctypes.c_uint8),
        ("minX",         ctypes.c_float),
        ("maxX",         ctypes.c_float),
        ("minY",         ctypes.c_float),
        ("maxY",         ctypes.c_float),
        ("minZ",         ctypes.c_float),
        ("maxZ",         ctypes.c_float),
        ("offsetX",      ctypes.c_float),   # (maxX+minX)/2
        ("offsetY",      ctypes.c_float),
        ("offsetZ",      ctypes.c_float),
        ("savedToEeprom", ctypes.c_uint8),
        ("reserved2",    ctypes.c_uint8 * 3),
    ]


# ============================================================================
# USER I/O PAYLOADS
# ============================================================================

class PayloadSetLED(ctypes.Structure):
    """
    TLV Type: IO_SET_LED (1280)  Direction: RPi → Arduino
    Size: 8 bytes
    mode: 0=off, 1=on, 2=blink, 3=breathe, 4=pwm
    """
    _pack_ = 1
    _fields_ = [
        ("ledId",      ctypes.c_uint8),
        ("mode",       ctypes.c_uint8),
        ("brightness", ctypes.c_uint8),
        ("reserved",   ctypes.c_uint8),
        ("periodMs",   ctypes.c_uint16),
        ("dutyCycle",  ctypes.c_uint16),  # 0–1000 = 0.0–100.0%
    ]


class PayloadSetNeoPixel(ctypes.Structure):
    """
    TLV Type: IO_SET_NEOPIXEL (1281)  Direction: RPi → Arduino
    Size: 4 bytes
    index=0xFF sets all pixels simultaneously.
    """
    _pack_ = 1
    _fields_ = [
        ("index", ctypes.c_uint8),
        ("red",   ctypes.c_uint8),
        ("green", ctypes.c_uint8),
        ("blue",  ctypes.c_uint8),
    ]


class PayloadIOStatus(ctypes.Structure):
    """
    TLV Type: IO_STATUS (1282)  Direction: Arduino → RPi
    Size: 10 bytes fixed  (+ 3 × neoPixelCount bytes appended)
    Rate: 100 Hz in RUNNING state.

    NeoPixel RGB data (neoPixelCount×3 bytes) is appended after the fixed fields
    and must be read separately from the raw TLV bytes.
    """
    _pack_ = 1
    _fields_ = [
        ("buttonMask",     ctypes.c_uint16),      # Digital input GPIO bitmask
        ("ledBrightness",  ctypes.c_uint8 * 3),   # Brightness of user LEDs 0–2
        ("reserved",       ctypes.c_uint8),
        ("timestamp",      ctypes.c_uint32),
    ]


# ============================================================================
# SIZE VERIFICATION (matches firmware STATIC_ASSERT_SIZE values)
# ============================================================================

def verify_payload_sizes():
    """Verify all payload sizes match firmware STATIC_ASSERT_SIZE expectations."""
    expected_sizes = {
        # System
        PayloadHeartbeat:         5,
        PayloadSystemStatus:      48,
        PayloadSysCmd:            4,
        PayloadSysConfig:         16,
        PayloadSetPID:            24,
        # DC Motor
        PayloadDCEnable:          4,
        PayloadDCSetPosition:     12,
        PayloadDCSetVelocity:     8,
        PayloadDCSetPWM:          4,
        DCMotorStatus:            46,
        PayloadDCStatusAll:       184,
        # Stepper
        PayloadStepEnable:        4,
        PayloadStepSetParams:     12,
        PayloadStepMove:          8,
        PayloadStepHome:          12,
        StepperStatus:            24,
        PayloadStepStatusAll:     96,
        # Servo
        PayloadServoEnable:       4,
        PayloadServoSetSingle:    4,
        PayloadServoSetBulk:      34,
        PayloadServoStatusAll:    36,
        # Sensor
        PayloadSensorIMU:         52,
        PayloadSensorKinematics:  28,
        PayloadSensorVoltage:     8,
        PayloadSensorRange:       12,
        PayloadMagCalCmd:         16,
        PayloadMagCalStatus:      44,
        # I/O
        PayloadSetLED:            8,
        PayloadSetNeoPixel:       4,
        PayloadIOStatus:          10,
    }

    errors = []
    for payload_class, expected in expected_sizes.items():
        actual = ctypes.sizeof(payload_class)
        if actual != expected:
            errors.append(
                f"  {payload_class.__name__}: expected {expected}, got {actual}"
            )

    if errors:
        raise AssertionError("Payload size mismatches:\n" + "\n".join(errors))

    print(f"✓ All {len(expected_sizes)} payload sizes verified")


if __name__ == "__main__":
    verify_payload_sizes()
