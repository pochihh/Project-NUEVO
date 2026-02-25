"""
Serial Manager — Serial Port Owner + Mock Simulator

SerialManager: manages UART to Arduino.
  - Auto-reconnect on disconnect
  - Sends SYS_HEARTBEAT every 200 ms (safety: firmware cuts motors at 500 ms timeout)
  - Feeds received bytes to TLV decoder → message_router.handle_incoming()
  - Provides send(tlv_type, payload) for outgoing commands

MockSerialManager: physics-based Arduino simulator for development.
  - Full Arduino state machine: INIT → IDLE → RUNNING / ERROR / ESTOP
  - DC motor PID with first-order velocity response (tau = 150 ms)
  - Stepper trapezoidal motion profile (accel / cruise / decel)
  - Differential-drive kinematics and 2D odometry
  - IMU: quaternion derived from integrated yaw, gravity + noise
  - Battery discharge, 5 V rail with noise
  - IO: button state simulation, NeoPixel tracking

All mock telemetry is fed through message_router.handle_incoming() (i.e. through
the real decode path) so 1-based conversion and field naming are exercised.
"""
import math
import time
import random
import asyncio
import ctypes
from typing import Optional

from .config import (
    SERIAL_PORT, SERIAL_BAUD, SERIAL_TIMEOUT,
    HEARTBEAT_INTERVAL, DEVICE_ID, ENABLE_CRC, STATS_INTERVAL,
)
from .payloads import (
    PayloadHeartbeat, PayloadSysCmd,
    PayloadSystemStatus, PayloadDCStatusAll, PayloadStepStatusAll,
    PayloadServoStatusAll, PayloadSensorIMU, PayloadSensorKinematics,
    PayloadSensorVoltage, PayloadIOStatus,
    DCMotorStatus, StepperStatus,
)
from .TLV_TypeDefs import (
    SYS_HEARTBEAT, SYS_CMD, SYS_STATUS, SYS_CONFIG, SYS_SET_PID,
    DC_ENABLE, DC_SET_VELOCITY, DC_SET_POSITION, DC_SET_PWM,
    DC_STATUS_ALL, STEP_ENABLE, STEP_SET_PARAMS, STEP_MOVE, STEP_HOME,
    STEP_STATUS_ALL, SERVO_ENABLE, SERVO_SET, SERVO_STATUS_ALL,
    SENSOR_IMU, SENSOR_KINEMATICS, SENSOR_VOLTAGE, IO_SET_LED,
    IO_SET_NEOPIXEL, IO_STATUS,
)

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from tlvcodec import Encoder, Decoder, DecodeErrorCode


# ============================================================================
# REAL SERIAL MANAGER
# ============================================================================

class SerialManager:
    """UART serial manager for Arduino communication."""

    def __init__(self, message_router, ws_manager):
        self.message_router = message_router
        self.ws_manager = ws_manager

        self.ser = None
        self.connected = False

        self.encoder = Encoder(deviceId=DEVICE_ID, bufferSize=4096, crc=ENABLE_CRC)
        self.decoder = Decoder(callback=self._decode_callback, crc=ENABLE_CRC)

        self.last_heartbeat_time = 0.0
        self.last_stats_time = 0.0

        self.stats = {
            "connected": False,
            "port": SERIAL_PORT,
            "baud": SERIAL_BAUD,
            "rx_count": 0,
            "tx_count": 0,
            "crc_errors": 0,
            "decode_errors_by_type": {},
        }

        self._running = False

    def _try_connect(self):
        try:
            import serial
            self.ser = serial.Serial(
                port=SERIAL_PORT,
                baudrate=SERIAL_BAUD,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=SERIAL_TIMEOUT,
            )
            self.connected = True
            self.stats["connected"] = True
            print(f"[Serial] Connected to {SERIAL_PORT} @ {SERIAL_BAUD} baud")
            return True
        except Exception as e:
            self.connected = False
            self.stats["connected"] = False
            print(f"[Serial] Failed to connect: {e}")
            return False

    def _decode_callback(self, error_code, frame_header, tlv_list):
        if error_code != DecodeErrorCode.NoError:
            self.stats["crc_errors"] += 1
            name = error_code.name if hasattr(error_code, "name") else str(error_code)
            self.stats["decode_errors_by_type"][name] = \
                self.stats["decode_errors_by_type"].get(name, 0) + 1
            return

        self.stats["rx_count"] += 1
        for tlv_type, tlv_len, tlv_data in tlv_list:
            self.message_router.handle_incoming(tlv_type, tlv_data)

    def _send_heartbeat(self):
        p = PayloadHeartbeat()
        p.timestamp = int(time.time() * 1000) & 0xFFFFFFFF
        p.flags = 0
        self.send(SYS_HEARTBEAT, p)

    def send(self, tlv_type: int, payload: ctypes.Structure):
        if not self.connected or not self.ser:
            return
        try:
            self.encoder.reset()
            self.encoder.addPacket(tlv_type, ctypes.sizeof(payload), payload)
            length, buffer = self.encoder.wrapupBuffer()
            self.ser.write(buffer[:length])
            self.stats["tx_count"] += 1
        except Exception as e:
            print(f"[Serial] Send error: {e}")
            self.connected = False
            self.stats["connected"] = False

    async def _broadcast_stats(self):
        await self.ws_manager.broadcast({
            "topic": "connection",
            "data": {
                "serialConnected": self.stats["connected"],
                "port":       self.stats["port"],
                "baud":       self.stats["baud"],
                "rxCount":    self.stats["rx_count"],
                "txCount":    self.stats["tx_count"],
                "crcErrors":  self.stats["crc_errors"],
            },
            "ts": time.time(),
        })

    async def run(self):
        self._running = True
        print("[Serial] Starting serial manager...")

        while self._running:
            if not self.connected:
                self._try_connect()
                await asyncio.sleep(1.0)
                continue

            now = time.time()

            if now - self.last_heartbeat_time >= HEARTBEAT_INTERVAL:
                self._send_heartbeat()
                self.last_heartbeat_time = now

            try:
                if self.ser.in_waiting > 0:
                    data = await asyncio.get_event_loop().run_in_executor(
                        None, self.ser.read, self.ser.in_waiting
                    )
                    self.decoder.decode(data)
            except Exception as e:
                print(f"[Serial] Read error: {e}")
                self.connected = False
                self.stats["connected"] = False
                if self.ser:
                    self.ser.close()
                await asyncio.sleep(1.0)
                continue

            if now - self.last_stats_time >= STATS_INTERVAL:
                await self._broadcast_stats()
                self.last_stats_time = now

            await asyncio.sleep(0.01)

    def stop(self):
        self._running = False
        if self.connected:
            p = PayloadSysCmd()
            p.command = 2   # STOP
            self.send(SYS_CMD, p)
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("[Serial] Stopped")


# ============================================================================
# ARDUINO SIMULATION CORE
# ============================================================================

_SYS_INIT    = 0
_SYS_IDLE    = 1
_SYS_RUNNING = 2
_SYS_ERROR   = 3
_SYS_ESTOP   = 4

_DC_DISABLED = 0
_DC_POSITION = 1
_DC_VELOCITY = 2
_DC_PWM      = 3

_STEP_IDLE    = 0
_STEP_ACCEL   = 1
_STEP_CRUISE  = 2
_STEP_DECEL   = 3
_STEP_HOMING  = 4
_STEP_FAULT   = 5

# Simulated wheel geometry (matches default firmware config)
_WHEEL_DIAMETER_MM = 65.0
_WHEEL_BASE_MM     = 150.0
_TICKS_PER_REV     = 1440           # encoder PPR × gear × 4 (quadrature)
_MM_PER_TICK       = (_WHEEL_DIAMETER_MM * math.pi) / _TICKS_PER_REV


class _DC:
    """State for one simulated DC motor."""
    __slots__ = (
        "mode", "position", "velocity", "target_vel", "target_pos", "pwm",
        "vel_integral", "fault_flags", "current_ma",
        "kp_vel", "ki_vel", "kd_vel",
        "kp_pos", "ki_pos", "kd_pos",
    )

    def __init__(self):
        self.mode         = _DC_DISABLED
        self.position     = 0.0     # encoder ticks (float for smooth sim)
        self.velocity     = 0.0     # ticks/s
        self.target_vel   = 0
        self.target_pos   = 0
        self.pwm          = 0
        self.vel_integral = 0.0
        self.fault_flags  = 0
        self.current_ma   = 0.0
        # Default PID gains
        self.kp_vel = 2.0;  self.ki_vel = 0.5;  self.kd_vel = 0.05
        self.kp_pos = 1.5;  self.ki_pos = 0.05; self.kd_pos = 0.1

    def update(self, dt: float):
        if dt <= 0:
            return

        if self.mode == _DC_DISABLED:
            # Coast to stop with friction
            self.velocity *= math.exp(-dt / 0.08)
            self.pwm = 0
            self.current_ma = 0.0

        elif self.mode == _DC_VELOCITY:
            # First-order velocity response (tau = 150 ms) + integral windup guard
            target = float(self.target_vel)
            tau = 0.15
            alpha = 1.0 - math.exp(-dt / tau)
            self.velocity += (target - self.velocity) * alpha
            self.velocity += random.gauss(0, 1.5)          # sensor noise
            self.pwm = int(_clamp(self.velocity / 10.0, -255, 255))
            self.current_ma = abs(self.velocity) * 0.28 + random.gauss(45, 8)

        elif self.mode == _DC_POSITION:
            # Proportional position → velocity setpoint (simple cascade)
            pos_error = float(self.target_pos) - self.position
            vel_setpoint = _clamp(pos_error * 8.0, -900, 900)
            tau = 0.15
            alpha = 1.0 - math.exp(-dt / tau)
            self.velocity += (vel_setpoint - self.velocity) * alpha
            self.velocity += random.gauss(0, 1.0)
            self.pwm = int(_clamp(self.velocity / 10.0, -255, 255))
            self.current_ma = abs(self.velocity) * 0.28 + random.gauss(55, 10)

        elif self.mode == _DC_PWM:
            # Velocity proportional to PWM with low-pass filter
            target_vel = float(self.pwm) * 9.0
            tau = 0.10
            alpha = 1.0 - math.exp(-dt / tau)
            self.velocity += (target_vel - self.velocity) * alpha
            self.velocity += random.gauss(0, 1.5)
            self.current_ma = abs(self.velocity) * 0.25 + random.gauss(40, 8)

        # Integrate position
        self.position += self.velocity * dt


class _Stepper:
    """State for one simulated stepper motor."""
    __slots__ = (
        "enabled", "state", "position", "target",
        "speed", "max_speed", "accel", "limit_hit",
    )

    def __init__(self):
        self.enabled   = False
        self.state     = _STEP_IDLE
        self.position  = 0      # commanded step count (integer)
        self.target    = 0
        self.speed     = 0.0    # current speed (steps/s)
        self.max_speed = 1000
        self.accel     = 500
        self.limit_hit = 0

    def update(self, dt: float):
        if not self.enabled or dt <= 0:
            self.state = _STEP_IDLE
            self.speed = 0.0
            return

        steps_to_go = self.target - self.position
        if steps_to_go == 0:
            self.state = _STEP_IDLE
            self.speed = 0.0
            return

        direction = 1 if steps_to_go > 0 else -1
        dist = abs(steps_to_go)

        # Deceleration distance: v²/(2a)
        decel_dist = (self.speed ** 2) / (2.0 * self.accel) if self.accel > 0 else 0.0

        if self.state == _STEP_HOMING:
            # Homing: just move slowly toward 0
            self.speed = min(self.speed + self.accel * dt, 200.0)
            self.state = _STEP_HOMING
        elif dist > decel_dist + 1:
            new_speed = min(self.speed + self.accel * dt, float(self.max_speed))
            self.state = _STEP_CRUISE if new_speed >= self.max_speed else _STEP_ACCEL
            self.speed = new_speed
        else:
            new_speed = max(self.speed - self.accel * dt, 50.0)
            self.state = _STEP_DECEL
            self.speed = new_speed

        move = self.speed * dt * direction
        self.position = int(self.position + move)

        if (direction > 0 and self.position >= self.target) or \
           (direction < 0 and self.position <= self.target):
            self.position = self.target
            self.state = _STEP_IDLE
            self.speed = 0.0


class _ArduinoSim:
    """
    Simulated Arduino firmware.
    Call update(dt) each tick, then read state to generate telemetry.
    """

    FIRMWARE_VERSION = (0, 8, 0)

    def __init__(self):
        self.state      = _SYS_INIT
        self.uptime_us  = 0     # micros() equivalent
        self.last_rx_ms = 9999  # ms since last TLV received (starts large)
        self.last_cmd_ms = 9999
        self.error_flags = 0
        self.loop_avg_us = 420
        self.loop_max_us = 680
        self.free_sram   = 3800

        self.wheel_diameter_mm = _WHEEL_DIAMETER_MM
        self.wheel_base_mm     = _WHEEL_BASE_MM
        self.motor_dir_mask    = 0
        self.neopixel_count    = 1
        self.heartbeat_timeout_ms = 500
        self.limit_switch_mask = 0
        self.stepper_home_limit = [0xFF, 0xFF, 0xFF, 0xFF]
        self.attached_sensors  = 0x01   # IMU present

        # DC motors (0–3)
        self.dc = [_DC() for _ in range(4)]

        # Steppers (0–3)
        self.steppers = [_Stepper() for _ in range(4)]

        # Servos (PCA9685, 16 channels)
        self.servo_connected  = True
        self.servo_error      = 0
        self.servo_enabled_mask = 0
        self.servo_pulses     = [1500] * 16

        # IMU state
        self.imu_yaw   = 0.0    # radians
        self.imu_pitch = 0.0
        self.imu_roll  = 0.0

        # Kinematics (differential drive)
        self.odom_x     = 0.0   # mm
        self.odom_y     = 0.0   # mm
        self.odom_theta = 0.0   # rad

        # Voltage
        self.battery_mv  = float(random.randint(12400, 12800))
        self.rail5v_mv   = float(random.randint(4990, 5020))
        self.servo_rail_mv = 0.0

        # IO
        self.button_mask   = 0
        self.led_brightness = [0, 0, 0]   # LEDs 0–2
        self.neopixel_rgb  = [0, 0, 30]   # dim blue (INIT color)

        # Timing helpers
        self._init_timer = 0.0   # time since boot (seconds)
        self._last_update = time.time()

    def receive_command(self):
        """Called whenever any TLV is received — resets liveness counters."""
        self.last_rx_ms = 0
        self.last_cmd_ms = 0

    def update(self, dt: float):
        """Advance the simulation by dt seconds."""
        self._init_timer += dt
        self.uptime_us += int(dt * 1_000_000)

        # Increment liveness timers
        self.last_rx_ms  = min(self.last_rx_ms  + int(dt * 1000), 9999)
        self.last_cmd_ms = min(self.last_cmd_ms + int(dt * 1000), 9999)

        # INIT → IDLE after 2 s
        if self.state == _SYS_INIT and self._init_timer >= 2.0:
            self.state = _SYS_IDLE
            self.neopixel_rgb = [0, 20, 0]   # dim green (IDLE)

        if self.state == _SYS_RUNNING:
            self._update_motors(dt)
            self._update_steppers(dt)
            self._update_kinematics(dt)
            self._update_imu(dt)

        # Slow battery discharge (realistic: ~1 mV / 2 s when motors active)
        active_motors = sum(1 for m in self.dc if m.mode != _DC_DISABLED)
        drain = (0.3 + active_motors * 0.15) * dt
        self.battery_mv = max(9000.0, self.battery_mv - drain)
        self.rail5v_mv  = 5000.0 + random.gauss(5, 2)

        # Random button noise (simulate occasional press at ~0.5 Hz each)
        if random.random() < dt * 0.5:
            btn = random.randint(0, 9)
            self.button_mask ^= (1 << btn)

        # Slowly fluctuate loop timing
        self.loop_avg_us = int(420 + random.gauss(0, 15))
        self.loop_max_us = max(self.loop_avg_us, int(680 + random.gauss(0, 40)))

    def _update_motors(self, dt: float):
        for m in self.dc:
            m.update(dt)

    def _update_steppers(self, dt: float):
        for s in self.steppers:
            s.update(dt)

    def _update_kinematics(self, dt: float):
        """Differential drive odometry from motors 0 (left) and 1 (right)."""
        mm_per_tick = _MM_PER_TICK
        v_left  = self.dc[0].velocity * mm_per_tick    # mm/s
        v_right = self.dc[1].velocity * mm_per_tick

        v_linear  = (v_left + v_right) * 0.5
        v_angular = (v_right - v_left) / self.wheel_base_mm

        self.odom_theta += v_angular * dt
        self.odom_x     += v_linear * math.cos(self.odom_theta) * dt
        self.odom_y     += v_linear * math.sin(self.odom_theta) * dt

    def _update_imu(self, dt: float):
        """Integrate angular velocity from kinematics into IMU yaw."""
        mm_per_tick = _MM_PER_TICK
        v_left  = self.dc[0].velocity * mm_per_tick
        v_right = self.dc[1].velocity * mm_per_tick
        v_angular = (v_right - v_left) / self.wheel_base_mm
        self.imu_yaw += v_angular * dt + random.gauss(0, 0.0005)   # gyro drift

    def _euler_to_quat(self, yaw, pitch, roll):
        """ZYX Euler → quaternion (w, x, y, z)."""
        cy = math.cos(yaw * 0.5);   sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5);  sr = math.sin(roll * 0.5)
        return (
            cr*cp*cy + sr*sp*sy,
            sr*cp*cy - cr*sp*sy,
            cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy,
        )


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


# ============================================================================
# MOCK SERIAL MANAGER
# ============================================================================

class MockSerialManager:
    """
    Mock serial manager for development without hardware.

    Simulates Arduino firmware at 20 Hz:
    - Realistic DC motor physics (first-order velocity, PID position)
    - Stepper trapezoidal motion profile
    - Differential-drive kinematics and odometry
    - IMU quaternion (integrated yaw) with sensor noise
    - Battery voltage with realistic discharge
    - IO button state simulation

    All telemetry is pushed through message_router.handle_incoming() so the
    full decode / 1-based conversion pipeline is exercised during development.
    """

    # Telemetry tick divisors (at 100 Hz base rate)
    _TICK_SYS_STATUS_IDLE    = 100  # 1 Hz
    _TICK_SYS_STATUS_RUNNING = 10   # 10 Hz
    _TICK_VOLTAGE            = 10   # 10 Hz
    _TICK_FULL               = 1    # 100 Hz (DC, step, servo, IMU, kin, IO)

    def __init__(self, message_router, ws_manager):
        self.message_router = message_router
        self.ws_manager     = ws_manager

        self.arduino = _ArduinoSim()

        self.stats = {
            "connected": True,
            "port": "MOCK",
            "baud": SERIAL_BAUD,
            "rx_count": 0,
            "tx_count": 0,
            "crc_errors": 0,
        }

        self._running       = False
        self._tick          = 0
        self.last_stats_time = 0.0

    # ------------------------------------------------------------------
    # Command handler (called from app.py when UI sends a command)
    # ------------------------------------------------------------------

    def send(self, tlv_type: int, payload: ctypes.Structure):
        """
        Receive outgoing TLV from bridge — update Arduino simulation state.
        payload already contains 0-based wire-format IDs (converted by router).
        """
        self.stats["tx_count"] += 1
        self.arduino.receive_command()

        try:
            self._handle_command(tlv_type, payload)
        except Exception as e:
            print(f"[Mock] Command handling error (type={tlv_type:#06x}): {e}")

    def _handle_command(self, tlv_type: int, payload):
        a = self.arduino

        if tlv_type == SYS_HEARTBEAT:
            pass   # liveness already reset in receive_command()

        elif tlv_type == SYS_CMD:
            cmd = payload.command
            if cmd == 1 and a.state == _SYS_IDLE:       # START
                a.state = _SYS_RUNNING
                a.neopixel_rgb = [0, 60, 0]
                print("[Mock] Arduino → RUNNING")
            elif cmd == 2 and a.state == _SYS_RUNNING:  # STOP
                a.state = _SYS_IDLE
                for m in a.dc:
                    m.mode = _DC_DISABLED
                a.neopixel_rgb = [0, 20, 0]
                print("[Mock] Arduino → IDLE")
            elif cmd == 3 and a.state in (_SYS_ERROR, _SYS_ESTOP):  # RESET
                a.state = _SYS_IDLE
                a.error_flags = 0
                a.neopixel_rgb = [0, 20, 0]
                print("[Mock] Arduino → IDLE (reset)")
            elif cmd == 4:                              # ESTOP
                a.state = _SYS_ESTOP
                for m in a.dc:
                    m.mode = _DC_DISABLED
                a.neopixel_rgb = [60, 0, 0]
                print("[Mock] Arduino → ESTOP")

        elif tlv_type == SYS_CONFIG:
            if a.state == _SYS_IDLE:
                if payload.wheelDiameterMm > 0:
                    a.wheel_diameter_mm = payload.wheelDiameterMm
                if payload.wheelBaseMm > 0:
                    a.wheel_base_mm = payload.wheelBaseMm
                if payload.heartbeatTimeoutMs:
                    a.heartbeat_timeout_ms = payload.heartbeatTimeoutMs
                if payload.resetOdometry:
                    a.odom_x = a.odom_y = a.odom_theta = 0.0
                    print("[Mock] Odometry reset")

        elif tlv_type == SYS_SET_PID:
            mid = payload.motorId
            if 0 <= mid < 4:
                m = a.dc[mid]
                if payload.loopType == 0:   # position
                    m.kp_pos = payload.kp
                    m.ki_pos = payload.ki
                    m.kd_pos = payload.kd
                else:                       # velocity
                    m.kp_vel = payload.kp
                    m.ki_vel = payload.ki
                    m.kd_vel = payload.kd

        elif tlv_type == DC_ENABLE:
            mid = payload.motorId
            if 0 <= mid < 4:
                prev_mode = a.dc[mid].mode
                a.dc[mid].mode = payload.mode
                if payload.mode == _DC_DISABLED and prev_mode != _DC_DISABLED:
                    a.dc[mid].velocity = 0.0
                    a.dc[mid].pwm = 0

        elif tlv_type == DC_SET_VELOCITY:
            mid = payload.motorId
            if 0 <= mid < 4:
                a.dc[mid].target_vel = payload.targetTicks

        elif tlv_type == DC_SET_POSITION:
            mid = payload.motorId
            if 0 <= mid < 4:
                a.dc[mid].target_pos = payload.targetTicks

        elif tlv_type == DC_SET_PWM:
            mid = payload.motorId
            if 0 <= mid < 4:
                a.dc[mid].pwm = payload.pwm

        elif tlv_type == STEP_ENABLE:
            sid = payload.stepperId
            if 0 <= sid < 4:
                a.steppers[sid].enabled = bool(payload.enable)
                if not payload.enable:
                    a.steppers[sid].speed = 0.0
                    a.steppers[sid].state = _STEP_IDLE

        elif tlv_type == STEP_SET_PARAMS:
            sid = payload.stepperId
            if 0 <= sid < 4:
                a.steppers[sid].max_speed = payload.maxVelocity
                a.steppers[sid].accel     = payload.acceleration

        elif tlv_type == STEP_MOVE:
            sid = payload.stepperId
            if 0 <= sid < 4:
                if payload.moveType == 0:   # absolute
                    a.steppers[sid].target = payload.target
                else:                       # relative
                    a.steppers[sid].target = int(a.steppers[sid].position) + payload.target

        elif tlv_type == STEP_HOME:
            sid = payload.stepperId
            if 0 <= sid < 4:
                a.steppers[sid].target = 0
                a.steppers[sid].state  = _STEP_HOMING

        elif tlv_type == SERVO_ENABLE:
            ch = payload.channel
            if ch == 0xFF:
                a.servo_enabled_mask = 0xFFFF if payload.enable else 0
            elif 0 <= ch < 16:
                if payload.enable:
                    a.servo_enabled_mask |= (1 << ch)
                else:
                    a.servo_enabled_mask &= ~(1 << ch)

        elif tlv_type == SERVO_SET:
            ch = payload.channel
            if 0 <= ch < 16:
                a.servo_pulses[ch] = payload.pulseUs

        elif tlv_type == IO_SET_LED:
            lid = payload.ledId
            if 0 <= lid < 3:
                a.led_brightness[lid] = payload.brightness

        elif tlv_type == IO_SET_NEOPIXEL:
            a.neopixel_rgb = [payload.red, payload.green, payload.blue]

    # ------------------------------------------------------------------
    # Telemetry generation helpers
    # ------------------------------------------------------------------

    def _emit(self, tlv_type: int, payload: ctypes.Structure):
        """Feed a ctypes struct through the message router as real Arduino data."""
        data = bytes(payload)
        self.message_router.handle_incoming(tlv_type, data)
        self.stats["rx_count"] += 1

    def _emit_raw(self, tlv_type: int, raw_bytes: bytes):
        """Feed raw bytes (for variable-length payloads like IO_STATUS)."""
        self.message_router.handle_incoming(tlv_type, raw_bytes)
        self.stats["rx_count"] += 1

    def _gen_sys_status(self):
        a = self.arduino
        p = PayloadSystemStatus()
        p.firmwareMajor = a.FIRMWARE_VERSION[0]
        p.firmwareMinor = a.FIRMWARE_VERSION[1]
        p.firmwarePatch = a.FIRMWARE_VERSION[2]
        p.state           = a.state
        p.uptimeMs        = (a.uptime_us // 1000) & 0xFFFFFFFF
        p.lastRxMs        = a.last_rx_ms
        p.lastCmdMs       = a.last_cmd_ms
        p.batteryMv       = int(_clamp(a.battery_mv, 0, 65535))
        p.rail5vMv        = int(_clamp(a.rail5v_mv, 0, 65535))
        p.errorFlags      = a.error_flags
        p.attachedSensors = a.attached_sensors
        p.freeSram        = a.free_sram
        p.loopTimeAvgUs   = a.loop_avg_us
        p.loopTimeMaxUs   = a.loop_max_us
        p.uartRxErrors    = 0
        p.wheelDiameterMm = a.wheel_diameter_mm
        p.wheelBaseMm     = a.wheel_base_mm
        p.motorDirMask    = a.motor_dir_mask
        p.neoPixelCount   = a.neopixel_count
        p.heartbeatTimeoutMs = a.heartbeat_timeout_ms
        p.limitSwitchMask = a.limit_switch_mask
        for i in range(4):
            p.stepperHomeLimitGpio[i] = a.stepper_home_limit[i]
        self._emit(SYS_STATUS, p)

    def _gen_dc_status_all(self):
        a = self.arduino
        p = PayloadDCStatusAll()
        for i in range(4):
            m  = a.dc[i]
            ms = p.motors[i]
            ms.mode      = m.mode
            ms.faultFlags = m.fault_flags
            ms.position  = int(m.position)
            ms.velocity  = int(m.velocity)
            ms.targetPos = m.target_pos if m.mode == _DC_POSITION else 0
            ms.targetVel = m.target_vel if m.mode == _DC_VELOCITY else 0
            ms.pwmOutput = m.pwm
            ms.currentMa = int(_clamp(m.current_ma, 0, 32767))
            ms.posKp = m.kp_pos;  ms.posKi = m.ki_pos;  ms.posKd = m.kd_pos
            ms.velKp = m.kp_vel;  ms.velKi = m.ki_vel;  ms.velKd = m.kd_vel
        self._emit(DC_STATUS_ALL, p)

    def _gen_step_status_all(self):
        a = self.arduino
        p = PayloadStepStatusAll()
        for i in range(4):
            s  = a.steppers[i]
            ss = p.steppers[i]
            ss.enabled        = 1 if s.enabled else 0
            ss.motionState    = s.state
            ss.limitHit       = s.limit_hit
            ss.commandedCount = int(s.position)
            ss.targetCount    = int(s.target)
            ss.currentSpeed   = int(s.speed)
            ss.maxSpeed       = s.max_speed
            ss.acceleration   = s.accel
        self._emit(STEP_STATUS_ALL, p)

    def _gen_servo_status_all(self):
        a = self.arduino
        p = PayloadServoStatusAll()
        p.pca9685Connected = 1 if a.servo_connected else 0
        p.pca9685Error     = a.servo_error
        p.enabledMask      = a.servo_enabled_mask
        for i in range(16):
            if a.servo_enabled_mask & (1 << i):
                p.pulseUs[i] = a.servo_pulses[i]
            else:
                p.pulseUs[i] = 0
        self._emit(SERVO_STATUS_ALL, p)

    def _gen_sensor_imu(self):
        a = self.arduino
        qw, qx, qy, qz = a._euler_to_quat(a.imu_yaw, a.imu_pitch, a.imu_roll)

        # Linear acceleration in body frame (gravity + robot acceleration)
        mm_per_tick = _MM_PER_TICK
        v_left  = a.dc[0].velocity * mm_per_tick
        v_right = a.dc[1].velocity * mm_per_tick
        v_lin   = (v_left + v_right) * 0.5   # mm/s

        p = PayloadSensorIMU()
        p.quatW = qw;  p.quatX = qx;  p.quatY = qy;  p.quatZ = qz

        # Earth-frame: gravity already removed; add small linear accel noise
        p.earthAccX = float(random.gauss(0, 0.005))
        p.earthAccY = float(random.gauss(0, 0.003))
        p.earthAccZ = float(random.gauss(0, 0.002))

        # Body-frame raw accel: gravity on Z, robot forward on X
        p.rawAccX = int(_clamp(random.gauss(0, 5), -32768, 32767))
        p.rawAccY = int(_clamp(random.gauss(0, 3), -32768, 32767))
        p.rawAccZ = int(_clamp(-9810 + random.gauss(0, 15), -32768, 32767))  # -1 g

        # Gyro (0.1 DPS units): Z axis from angular velocity
        v_angular = (v_right - v_left) / a.wheel_base_mm   # rad/s
        gyro_z_dps = math.degrees(v_angular) * 10.0        # 0.1 DPS
        p.rawGyroX = int(_clamp(random.gauss(0, 2), -32768, 32767))
        p.rawGyroY = int(_clamp(random.gauss(0, 2), -32768, 32767))
        p.rawGyroZ = int(_clamp(gyro_z_dps + random.gauss(0, 3), -32768, 32767))

        # Magnetometer (µT) — roughly north-pointing
        p.magX = int(_clamp(25 + random.gauss(0, 2), -32768, 32767))
        p.magY = int(_clamp(-5 + random.gauss(0, 2), -32768, 32767))
        p.magZ = int(_clamp(-42 + random.gauss(0, 2), -32768, 32767))

        p.magCalibrated = 0
        p.timestamp     = a.uptime_us & 0xFFFFFFFF
        self._emit(SENSOR_IMU, p)

    def _gen_sensor_kinematics(self):
        a = self.arduino
        mm_per_tick = _MM_PER_TICK
        v_left  = a.dc[0].velocity * mm_per_tick
        v_right = a.dc[1].velocity * mm_per_tick
        v_lin   = (v_left + v_right) * 0.5
        v_ang   = (v_right - v_left) / a.wheel_base_mm

        p = PayloadSensorKinematics()
        p.x         = a.odom_x
        p.y         = a.odom_y
        p.theta     = a.odom_theta
        p.vx        = v_lin
        p.vy        = 0.0
        p.vTheta    = v_ang
        p.timestamp = a.uptime_us & 0xFFFFFFFF
        self._emit(SENSOR_KINEMATICS, p)

    def _gen_sensor_voltage(self):
        a = self.arduino
        p = PayloadSensorVoltage()
        p.batteryMv   = int(_clamp(a.battery_mv + random.gauss(0, 8), 0, 65535))
        p.rail5vMv    = int(_clamp(a.rail5v_mv  + random.gauss(0, 3), 0, 65535))
        p.servoRailMv = 0
        self._emit(SENSOR_VOLTAGE, p)

    def _gen_io_status(self):
        a = self.arduino
        p = PayloadIOStatus()
        p.buttonMask = a.button_mask
        for i in range(3):
            p.ledBrightness[i] = a.led_brightness[i]
        p.timestamp = (a.uptime_us // 1000) & 0xFFFFFFFF

        # Append NeoPixel RGB (neoPixelCount × 3 bytes)
        fixed_bytes = bytes(p)
        neo_bytes = bytes(a.neopixel_rgb[:3])   # 1 pixel × 3 bytes
        self._emit_raw(IO_STATUS, fixed_bytes + neo_bytes)

    # ------------------------------------------------------------------
    # Stats broadcast
    # ------------------------------------------------------------------

    async def _broadcast_stats(self):
        await self.ws_manager.broadcast({
            "topic": "connection",
            "data": {
                "serialConnected": True,
                "port":      "MOCK",
                "baud":      SERIAL_BAUD,
                "rxCount":   self.stats["rx_count"],
                "txCount":   self.stats["tx_count"],
                "crcErrors": 0,
            },
            "ts": time.time(),
        })

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    async def run(self):
        self._running = True
        print("[Mock] Starting mock serial manager (100 Hz physics simulation)...")

        TARGET_DT  = 0.01   # 100 Hz
        last_tick  = time.monotonic()

        while self._running:
            now = time.monotonic()
            dt  = now - last_tick
            last_tick = now

            # Advance physics
            self.arduino.update(dt)
            self._tick += 1

            a     = self.arduino
            state = a.state

            # System status (always)
            sys_div = self._TICK_SYS_STATUS_RUNNING if state == _SYS_RUNNING \
                      else self._TICK_SYS_STATUS_IDLE
            if self._tick % sys_div == 0:
                self._gen_sys_status()

            # Full telemetry only in RUNNING state
            if state == _SYS_RUNNING:
                if self._tick % self._TICK_FULL == 0:
                    self._gen_dc_status_all()
                    self._gen_step_status_all()
                    self._gen_servo_status_all()
                    self._gen_sensor_imu()
                    self._gen_sensor_kinematics()
                    self._gen_io_status()

                if self._tick % self._TICK_VOLTAGE == 0:
                    self._gen_sensor_voltage()

            # Connection stats broadcast
            real_now = time.time()
            if real_now - self.last_stats_time >= STATS_INTERVAL:
                await self._broadcast_stats()
                self.last_stats_time = real_now

            # Sleep to maintain ~20 Hz
            elapsed = time.monotonic() - now
            sleep_time = max(0.0, TARGET_DT - elapsed)
            await asyncio.sleep(sleep_time)

    def stop(self):
        self._running = False
        stop_cmd = PayloadSysCmd()
        stop_cmd.command = 2   # STOP
        self._handle_command(SYS_CMD, stop_cmd)
        print("[Mock] Stopped")
