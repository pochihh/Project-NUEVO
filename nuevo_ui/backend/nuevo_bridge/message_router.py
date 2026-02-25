"""
Message Router — TLV ↔ JSON Translation

Bidirectional translation between TLV binary payloads and JSON WebSocket messages.

Numbering convention (DESIGN_GUIDELINES.md §3):
  Wire (TLV):  0-based  (motorId 0–3, stepperId 0–3, channel 0–15)
  User (JSON): 1-based  (motorNumber 1–4, stepperNumber 1–4, channel 1–16)

The bridge is the ONLY place that performs this conversion.
  Outgoing (UI → wire): motorNumber - 1 → motorId
  Incoming (wire → UI): motorId + 1 → motorNumber

Servo channel special case: 0xFF ("all channels") passes through unchanged.
"""
import time
import ctypes
import asyncio
import struct as _struct
from typing import Any, Dict, List, Optional, Tuple

from .TLV_TypeDefs import *
from .payloads import *


# ============================================================================
# INCOMING: DECODE FUNCTIONS (wire bytes → JSON dict)
# ============================================================================
#
# Each function receives raw TLV payload bytes and returns a dict (or None on
# error). They are responsible for struct sizing, field extraction, and
# applying 0-based → 1-based number conversion for user-facing fields.

def _decode_fixed(payload_class, tlv_data: bytes) -> Optional[dict]:
    """Decode a fixed-size struct into a flat dict. Skips 'reserved*' fields."""
    expected = ctypes.sizeof(payload_class)
    if len(tlv_data) != expected:
        return None
    s = payload_class.from_buffer_copy(tlv_data)
    return _struct_to_dict(s)


def _struct_to_dict(s: ctypes.Structure) -> dict:
    """Recursively convert a ctypes Structure to a plain dict."""
    result = {}
    for field_name, _ in s._fields_:
        if field_name.startswith("reserved"):
            continue
        value = getattr(s, field_name)
        if isinstance(value, ctypes.Structure):
            result[field_name] = _struct_to_dict(value)
        elif hasattr(value, "__len__") and not isinstance(value, (str, bytes)):
            # Array — may contain structs or primitives
            items = []
            for item in value:
                if isinstance(item, ctypes.Structure):
                    items.append(_struct_to_dict(item))
                else:
                    items.append(item)
            result[field_name] = items
        else:
            result[field_name] = value
    return result


def decode_system_status(tlv_data: bytes) -> Optional[dict]:
    d = _decode_fixed(PayloadSystemStatus, tlv_data)
    if d is None:
        return None
    # stepperHomeLimitGpio is already a list from _struct_to_dict
    return d


_dc_frame_counter: int = 0


def decode_dc_status_all(tlv_data: bytes) -> Optional[dict]:
    """Expand PayloadDCStatusAll.motors[4] — add motorNumber (1-based) and frameIndex."""
    global _dc_frame_counter
    if len(tlv_data) != ctypes.sizeof(PayloadDCStatusAll):
        return None
    p = PayloadDCStatusAll.from_buffer_copy(tlv_data)
    frame_idx = _dc_frame_counter
    _dc_frame_counter += 1
    motors = []
    for i in range(4):
        m = p.motors[i]
        motors.append({
            "motorNumber": i + 1,   # 0-based → 1-based (DESIGN_GUIDELINES §3)
            "frameIndex":  frame_idx,  # sequential counter, same for all 4 motors in packet
            "mode":        m.mode,
            "faultFlags":  m.faultFlags,
            "position":    m.position,
            "velocity":    m.velocity,
            "targetPos":   m.targetPos,
            "targetVel":   m.targetVel,
            "pwmOutput":   m.pwmOutput,
            "currentMa":   m.currentMa,
            "posKp":       m.posKp,
            "posKi":       m.posKi,
            "posKd":       m.posKd,
            "velKp":       m.velKp,
            "velKi":       m.velKi,
            "velKd":       m.velKd,
        })
    return {"motors": motors}


def decode_step_status_all(tlv_data: bytes) -> Optional[dict]:
    """Expand PayloadStepStatusAll.steppers[4] — add stepperNumber (1-based)."""
    if len(tlv_data) != ctypes.sizeof(PayloadStepStatusAll):
        return None
    p = PayloadStepStatusAll.from_buffer_copy(tlv_data)
    steppers = []
    for i in range(4):
        s = p.steppers[i]
        steppers.append({
            "stepperNumber":  i + 1,    # 0-based → 1-based
            "enabled":        s.enabled,
            "motionState":    s.motionState,
            "limitHit":       s.limitHit,
            "commandedCount": s.commandedCount,
            "targetCount":    s.targetCount,
            "currentSpeed":   s.currentSpeed,
            "maxSpeed":       s.maxSpeed,
            "acceleration":   s.acceleration,
        })
    return {"steppers": steppers}


def decode_servo_status_all(tlv_data: bytes) -> Optional[dict]:
    """Decode SERVO_STATUS_ALL — convert channel indices to 1-based (DESIGN_GUIDELINES §3)."""
    if len(tlv_data) != ctypes.sizeof(PayloadServoStatusAll):
        return None
    p = PayloadServoStatusAll.from_buffer_copy(tlv_data)
    channels = []
    for i in range(16):
        channels.append({
            "channelNumber": i + 1,              # 0-based → 1-based
            "enabled":       bool(p.enabledMask & (1 << i)),
            "pulseUs":       p.pulseUs[i],
        })
    return {
        "pca9685Connected": p.pca9685Connected,
        "pca9685Error":     p.pca9685Error,
        "channels":         channels,
    }


def decode_sensor_imu(tlv_data: bytes) -> Optional[dict]:
    return _decode_fixed(PayloadSensorIMU, tlv_data)


def decode_sensor_kinematics(tlv_data: bytes) -> Optional[dict]:
    return _decode_fixed(PayloadSensorKinematics, tlv_data)


def decode_sensor_voltage(tlv_data: bytes) -> Optional[dict]:
    d = _decode_fixed(PayloadSensorVoltage, tlv_data)
    return d


def decode_sensor_range(tlv_data: bytes) -> Optional[dict]:
    return _decode_fixed(PayloadSensorRange, tlv_data)


def decode_mag_cal_status(tlv_data: bytes) -> Optional[dict]:
    return _decode_fixed(PayloadMagCalStatus, tlv_data)


def decode_io_status(tlv_data: bytes) -> Optional[dict]:
    """
    Variable-length payload: 10 bytes fixed + 3 × neoPixelCount bytes appended.
    Always parse the fixed header; append neoPixelRGB if extra bytes are present.
    """
    fixed_size = ctypes.sizeof(PayloadIOStatus)
    if len(tlv_data) < fixed_size:
        return None
    p = PayloadIOStatus.from_buffer_copy(tlv_data[:fixed_size])
    result = {
        "buttonMask":     p.buttonMask,
        "ledBrightness":  list(p.ledBrightness),
        "timestamp":      p.timestamp,
    }
    # Parse appended NeoPixel RGB triples
    extra = tlv_data[fixed_size:]
    neo_pixels = []
    for i in range(0, len(extra) - 2, 3):
        neo_pixels.append({
            "r": extra[i],
            "g": extra[i + 1],
            "b": extra[i + 2],
        })
    if neo_pixels:
        result["neoPixels"] = neo_pixels
    return result


# ============================================================================
# INCOMING REGISTRY
# ============================================================================
#
# Maps TLV type ID → (WebSocket topic, decode_fn)

INCOMING_REGISTRY: Dict[int, Tuple[str, Any]] = {
    SYS_STATUS:          ("system_status", decode_system_status),
    DC_STATUS_ALL:       ("dc_status_all",  decode_dc_status_all),
    STEP_STATUS_ALL:     ("step_status_all", decode_step_status_all),
    SERVO_STATUS_ALL:    ("servo_status_all", decode_servo_status_all),
    SENSOR_IMU:          ("imu",            decode_sensor_imu),
    SENSOR_KINEMATICS:   ("kinematics",     decode_sensor_kinematics),
    SENSOR_VOLTAGE:      ("voltage",        decode_sensor_voltage),
    SENSOR_RANGE:        ("range",          decode_sensor_range),
    SENSOR_MAG_CAL_STATUS: ("mag_cal_status", decode_mag_cal_status),
    IO_STATUS:           ("io_status",      decode_io_status),
}


# ============================================================================
# OUTGOING: ENCODE FUNCTIONS (JSON dict → wire ctypes struct)
# ============================================================================
#
# Each function receives the user-facing JSON dict (1-based numbering) and
# returns a ctypes struct with 0-based wire-format values, or None on error.
#
# Range validation happens here — invalid commands are rejected before the wire.

def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


def encode_sys_cmd(data: dict) -> Optional[ctypes.Structure]:
    p = PayloadSysCmd()
    p.command = int(data["command"])   # 1=START, 2=STOP, 3=RESET, 4=ESTOP
    return p


def encode_sys_config(data: dict) -> Optional[ctypes.Structure]:
    p = PayloadSysConfig()
    p.wheelDiameterMm    = float(data.get("wheelDiameterMm", 0.0))
    p.wheelBaseMm        = float(data.get("wheelBaseMm", 0.0))
    p.motorDirMask       = int(data.get("motorDirMask", 0))
    p.motorDirChangeMask = int(data.get("motorDirChangeMask", 0))
    p.neoPixelCount      = int(data.get("neoPixelCount", 0))
    p.attachedSensors    = int(data.get("attachedSensors", 0xFF))
    p.heartbeatTimeoutMs = int(data.get("heartbeatTimeoutMs", 0))
    p.resetOdometry      = int(data.get("resetOdometry", 0))
    return p


def encode_set_pid(data: dict) -> Optional[ctypes.Structure]:
    """motorNumber (1-based) → motorId (0-based)."""
    motor_number = int(data["motorNumber"])
    if not 1 <= motor_number <= 4:
        return None
    p = PayloadSetPID()
    p.motorId    = motor_number - 1
    p.loopType   = int(data.get("loopType", 1))   # 0=position, 1=velocity
    p.kp         = float(data.get("kp", 0.0))
    p.ki         = float(data.get("ki", 0.0))
    p.kd         = float(data.get("kd", 0.0))
    p.maxOutput  = float(data.get("maxOutput", 255.0))
    p.maxIntegral = float(data.get("maxIntegral", 100.0))
    return p


def encode_dc_enable(data: dict) -> Optional[ctypes.Structure]:
    """motorNumber (1-based) → motorId (0-based). mode: 0=off, 1=pos, 2=vel, 3=pwm."""
    motor_number = int(data["motorNumber"])
    if not 1 <= motor_number <= 4:
        return None
    p = PayloadDCEnable()
    p.motorId = motor_number - 1
    p.mode    = int(data.get("mode", 0))
    return p


def encode_dc_set_position(data: dict) -> Optional[ctypes.Structure]:
    motor_number = int(data["motorNumber"])
    if not 1 <= motor_number <= 4:
        return None
    p = PayloadDCSetPosition()
    p.motorId     = motor_number - 1
    p.targetTicks = int(data["targetTicks"])
    p.maxVelTicks = int(data.get("maxVelTicks", 0))
    return p


def encode_dc_set_velocity(data: dict) -> Optional[ctypes.Structure]:
    motor_number = int(data["motorNumber"])
    if not 1 <= motor_number <= 4:
        return None
    p = PayloadDCSetVelocity()
    p.motorId     = motor_number - 1
    p.targetTicks = int(data["targetTicks"])
    return p


def encode_dc_set_pwm(data: dict) -> Optional[ctypes.Structure]:
    motor_number = int(data["motorNumber"])
    if not 1 <= motor_number <= 4:
        return None
    pwm = _clamp(int(data["pwm"]), -255, 255)
    p = PayloadDCSetPWM()
    p.motorId = motor_number - 1
    p.pwm     = pwm
    return p


def encode_step_enable(data: dict) -> Optional[ctypes.Structure]:
    """stepperNumber (1-based) → stepperId (0-based)."""
    stepper_number = int(data["stepperNumber"])
    if not 1 <= stepper_number <= 4:
        return None
    p = PayloadStepEnable()
    p.stepperId = stepper_number - 1
    p.enable    = int(data.get("enable", 0))
    return p


def encode_step_set_params(data: dict) -> Optional[ctypes.Structure]:
    stepper_number = int(data["stepperNumber"])
    if not 1 <= stepper_number <= 4:
        return None
    p = PayloadStepSetParams()
    p.stepperId    = stepper_number - 1
    p.maxVelocity  = int(data.get("maxVelocity", 1000))
    p.acceleration = int(data.get("acceleration", 500))
    return p


def encode_step_move(data: dict) -> Optional[ctypes.Structure]:
    stepper_number = int(data["stepperNumber"])
    if not 1 <= stepper_number <= 4:
        return None
    p = PayloadStepMove()
    p.stepperId = stepper_number - 1
    p.moveType  = int(data.get("moveType", 0))   # 0=absolute, 1=relative
    p.target    = int(data["target"])
    return p


def encode_step_home(data: dict) -> Optional[ctypes.Structure]:
    stepper_number = int(data["stepperNumber"])
    if not 1 <= stepper_number <= 4:
        return None
    p = PayloadStepHome()
    p.stepperId    = stepper_number - 1
    p.direction    = int(data.get("direction", -1))
    p.homeVelocity = int(data.get("homeVelocity", 200))
    p.backoffSteps = int(data.get("backoffSteps", 100))
    return p


def encode_servo_enable(data: dict) -> Optional[ctypes.Structure]:
    """
    channel: 1-based (1–16) in UI, or 255 for all channels.
    255 passes through as 0xFF unchanged.
    """
    ch = int(data["channel"])
    if ch != 255 and not 1 <= ch <= 16:
        return None
    p = PayloadServoEnable()
    p.channel = 0xFF if ch == 255 else (ch - 1)
    p.enable  = int(data.get("enable", 0))
    return p


def encode_servo_set(data: dict) -> Optional[ctypes.Structure]:
    """Single servo set. channel: 1-based in UI."""
    ch = int(data["channel"])
    if not 1 <= ch <= 16:
        return None
    pulse_us = _clamp(int(data["pulseUs"]), 500, 2500)
    p = PayloadServoSetSingle()
    p.channel = ch - 1
    p.count   = 1
    p.pulseUs = pulse_us
    return p


def encode_set_led(data: dict) -> Optional[ctypes.Structure]:
    """LED IDs are 0-based (named, not numbered) — no conversion needed."""
    p = PayloadSetLED()
    p.ledId      = int(data.get("ledId", 0))
    p.mode       = int(data.get("mode", 0))
    p.brightness = _clamp(int(data.get("brightness", 255)), 0, 255)
    p.periodMs   = int(data.get("periodMs", 1000))
    p.dutyCycle  = _clamp(int(data.get("dutyCycle", 500)), 0, 1000)
    return p


def encode_set_neopixel(data: dict) -> Optional[ctypes.Structure]:
    """index: 0-based (or 0xFF for all pixels) — pass through unchanged."""
    p = PayloadSetNeoPixel()
    p.index = int(data.get("index", 0))
    p.red   = _clamp(int(data.get("red", 0)), 0, 255)
    p.green = _clamp(int(data.get("green", 0)), 0, 255)
    p.blue  = _clamp(int(data.get("blue", 0)), 0, 255)
    return p


def encode_mag_cal_cmd(data: dict) -> Optional[ctypes.Structure]:
    p = PayloadMagCalCmd()
    p.command = int(data["command"])   # 1=START,2=STOP,3=SAVE,4=APPLY,5=CLEAR
    p.offsetX = float(data.get("offsetX", 0.0))
    p.offsetY = float(data.get("offsetY", 0.0))
    p.offsetZ = float(data.get("offsetZ", 0.0))
    return p


# ============================================================================
# OUTGOING REGISTRY
# ============================================================================
#
# Maps WebSocket command name → (TLV type ID, encode_fn)

OUTGOING_REGISTRY: Dict[str, Tuple[int, Any]] = {
    # System
    "sys_cmd":         (SYS_CMD,        encode_sys_cmd),
    "sys_config":      (SYS_CONFIG,     encode_sys_config),
    "set_pid":         (SYS_SET_PID,    encode_set_pid),
    # DC motors
    "dc_enable":       (DC_ENABLE,      encode_dc_enable),
    "dc_set_position": (DC_SET_POSITION, encode_dc_set_position),
    "dc_set_velocity": (DC_SET_VELOCITY, encode_dc_set_velocity),
    "dc_set_pwm":      (DC_SET_PWM,     encode_dc_set_pwm),
    # Steppers
    "step_enable":     (STEP_ENABLE,    encode_step_enable),
    "step_set_params": (STEP_SET_PARAMS, encode_step_set_params),
    "step_move":       (STEP_MOVE,      encode_step_move),
    "step_home":       (STEP_HOME,      encode_step_home),
    # Servos
    "servo_enable":    (SERVO_ENABLE,   encode_servo_enable),
    "servo_set":       (SERVO_SET,      encode_servo_set),
    # I/O
    "set_led":         (IO_SET_LED,     encode_set_led),
    "set_neopixel":    (IO_SET_NEOPIXEL, encode_set_neopixel),
    # Sensors
    "mag_cal_cmd":     (SENSOR_MAG_CAL_CMD, encode_mag_cal_cmd),
}


# ============================================================================
# MESSAGE ROUTER CLASS
# ============================================================================

class MessageRouter:
    """Bidirectional TLV ↔ JSON router with numbering conversion."""

    def __init__(self, ws_manager):
        self.ws_manager = ws_manager

    def handle_incoming(self, tlv_type: int, tlv_data: bytes):
        """
        Handle incoming TLV message from Arduino.
        Decodes to JSON dict and broadcasts over WebSocket.
        """
        if tlv_type not in INCOMING_REGISTRY:
            print(f"[Router] Unknown TLV type: {tlv_type:#06x}")
            return

        topic, decode_fn = INCOMING_REGISTRY[tlv_type]

        try:
            data_dict = decode_fn(tlv_data)
        except Exception as e:
            print(f"[Router] Decode error for {topic}: {e}")
            return

        if data_dict is None:
            print(f"[Router] Size mismatch for {topic}: got {len(tlv_data)} bytes")
            return

        message = {
            "topic": topic,
            "data":  data_dict,
            "ts":    time.time(),
        }

        asyncio.create_task(self.ws_manager.broadcast(message))

    def handle_outgoing(self, cmd: str, data: Dict[str, Any]) -> Optional[Tuple[int, ctypes.Structure]]:
        """
        Handle outgoing command from frontend.
        Encodes JSON to TLV payload.
        Returns (tlv_type, payload_ctypes) or None on unknown/invalid command.
        """
        if cmd not in OUTGOING_REGISTRY:
            print(f"[Router] Unknown command: {cmd!r}")
            return None

        tlv_type, encode_fn = OUTGOING_REGISTRY[cmd]

        try:
            payload = encode_fn(data)
        except Exception as e:
            print(f"[Router] Encode error for {cmd!r}: {e}")
            return None

        if payload is None:
            print(f"[Router] Rejected command {cmd!r}: validation failed (data={data})")
            return None

        return (tlv_type, payload)
