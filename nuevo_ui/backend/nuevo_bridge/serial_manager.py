"""
Serial Manager — Serial Port Owner + Mock Simulator

SerialManager: manages UART to Arduino.
  - Dedicated blocking reader thread: ser.read() returns the instant bytes arrive
    (10 ms timeout allows clean shutdown check without busy-looping)
  - asyncio event loop handles heartbeat / stats / WebSocket — never blocked by I/O
  - threading.Lock on ser.write() so asyncio heartbeat and rclpy callbacks coexist safely
  - Feeds received bytes to TLV decoder → message_router.decode_incoming()
  - Flushes decoded batch to WebSocket (via asyncio.run_coroutine_threadsafe) and
    optionally to a ROS2 node (publisher calls are thread-safe in rclpy)

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
import threading
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
    """
    UART serial manager for Arduino communication.

    Threading model:
      - _reader_loop()  runs in a dedicated daemon thread (blocking serial reads)
      - run()           is an asyncio coroutine (heartbeat + stats + WS)
      - send()          is thread-safe via _write_lock (called from both)
    """

    def __init__(self, message_router, ws_manager):
        self.message_router = message_router
        self.ws_manager = ws_manager

        self.ser = None
        self.connected = False

        self.encoder = Encoder(deviceId=DEVICE_ID, bufferSize=4096, crc=ENABLE_CRC)
        self.decoder = Decoder(callback=self._decode_callback, crc=ENABLE_CRC)

        # Protects ser.write() — called from asyncio thread (heartbeat) and
        # rclpy spin thread (/cmd_vel callbacks) simultaneously in ROS2 mode.
        self._write_lock = threading.Lock()

        # Set by app.py when NUEVO_ROS2=1; publish_decoded() called from reader thread.
        self._ros2_node = None

        # Captured in run() so the reader thread can schedule coroutines on it.
        self._asyncio_loop: Optional[asyncio.AbstractEventLoop] = None

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
        self._pending_messages: list = []  # accumulated within one decoder.decode() call

    # ------------------------------------------------------------------
    # ROS2 integration hook
    # ------------------------------------------------------------------

    def set_ros2_node(self, node) -> None:
        """Called from app.py after the ROS2 node is constructed."""
        self._ros2_node = node

    # ------------------------------------------------------------------
    # Serial connection
    # ------------------------------------------------------------------

    def _try_connect(self) -> bool:
        """Open the serial port. Returns True on success. Blocking — run in reader thread."""
        try:
            import serial
            self.ser = serial.Serial(
                port=SERIAL_PORT,
                baudrate=SERIAL_BAUD,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                # 10 ms read timeout: ser.read() returns when bytes arrive OR after 10 ms.
                # This lets _reader_loop check _running without busy-looping.
                timeout=0.01,
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

    # ------------------------------------------------------------------
    # Decoder callback — runs inside _reader_loop thread
    # ------------------------------------------------------------------

    def _decode_callback(self, error_code, frame_header, tlv_list):
        if error_code != DecodeErrorCode.NoError:
            self.stats["crc_errors"] += 1
            name = error_code.name if hasattr(error_code, "name") else str(error_code)
            self.stats["decode_errors_by_type"][name] = \
                self.stats["decode_errors_by_type"].get(name, 0) + 1
            return

        self.stats["rx_count"] += 1
        # Accumulate decoded messages; flushed together after decoder.decode() returns,
        # producing one asyncio schedule + one ROS2 publish loop per serial read batch.
        for tlv_type, tlv_len, tlv_data in tlv_list:
            msg = self.message_router.decode_incoming(tlv_type, tlv_data)
            if msg is not None:
                self._pending_messages.append(msg)

    def _flush_pending(self) -> None:
        """
        Distribute accumulated messages to WebSocket and ROS2.
        Called from the reader thread after each decoder.decode() call.
        """
        if not self._pending_messages:
            return

        msgs = self._pending_messages[:]
        self._pending_messages.clear()

        # → WebSocket: schedule coroutine on the asyncio event loop (thread-safe)
        if self.ws_manager.connections and self._asyncio_loop:
            asyncio.run_coroutine_threadsafe(
                self.message_router.flush_to_ws(msgs),
                self._asyncio_loop,
            )

        # → ROS2: rclpy publishers are thread-safe for concurrent publish() calls
        if self._ros2_node is not None:
            for msg in msgs:
                self._ros2_node.publish_decoded(msg)

    # ------------------------------------------------------------------
    # Blocking reader thread
    # ------------------------------------------------------------------

    def _reader_loop(self) -> None:
        """
        Dedicated UART reader thread.

        ser.read(4096) blocks until bytes arrive or 10 ms elapses — hardware-driven
        timing with zero CPU burn while waiting. This gives <1 ms read latency
        regardless of asyncio event loop activity.
        """
        print("[Serial] Reader thread started.")
        while self._running:
            if not self.connected:
                if not self._try_connect():
                    time.sleep(1.0)
                continue

            try:
                data = self.ser.read(4096)
                if data:
                    self.decoder.decode(data)
                    self._flush_pending()
            except Exception as e:
                print(f"[Serial] Read error: {e}")
                self.connected = False
                self.stats["connected"] = False
                if self.ser:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                time.sleep(1.0)

        print("[Serial] Reader thread stopped.")

    # ------------------------------------------------------------------
    # Write path — thread-safe
    # ------------------------------------------------------------------

    def send(self, tlv_type: int, payload: ctypes.Structure) -> None:
        """
        Thread-safe TLV send. Called from:
          - asyncio event loop (heartbeat, UI WebSocket commands)
          - rclpy spin thread (/cmd_vel, /nuevo/step_cmd, etc.) in ROS2 mode
        ser.write() to the OS TX buffer is sub-millisecond; lock hold time is negligible.
        """
        if not self.connected or not self.ser:
            return
        with self._write_lock:
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

    def _send_heartbeat(self) -> None:
        p = PayloadHeartbeat()
        p.timestamp = int(time.time() * 1000) & 0xFFFFFFFF
        p.flags = 0
        self.send(SYS_HEARTBEAT, p)

    # ------------------------------------------------------------------
    # Stats broadcast
    # ------------------------------------------------------------------

    async def _broadcast_stats(self) -> None:
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

    # ------------------------------------------------------------------
    # Asyncio entry point
    # ------------------------------------------------------------------

    async def run(self) -> None:
        """
        Asyncio entry point. Captures the event loop reference, starts the
        blocking reader thread, then manages heartbeat and stats broadcasts.
        """
        self._asyncio_loop = asyncio.get_event_loop()
        self._running = True

        reader_thread = threading.Thread(
            target=self._reader_loop,
            daemon=True,
            name="serial-reader",
        )
        reader_thread.start()

        print("[Serial] Manager started (reader thread is hardware-driven).")

        try:
            while self._running:
                now = time.time()

                # Heartbeat: send() uses _write_lock; hold time is sub-ms
                if self.connected and now - self.last_heartbeat_time >= HEARTBEAT_INTERVAL:
                    self._send_heartbeat()
                    self.last_heartbeat_time = now

                if now - self.last_stats_time >= STATS_INTERVAL:
                    await self._broadcast_stats()
                    self.last_stats_time = now

                # Sleep 50 ms — heartbeat threshold is checked 4× per 200 ms window
                await asyncio.sleep(0.05)
        finally:
            reader_thread.join(timeout=2.0)

    def stop(self) -> None:
        self._running = False
        # Send STOP before closing so Arduino disables actuators gracefully
        if self.connected:
            p = PayloadSysCmd()
            p.command = 2  # SYS_CMD_STOP
            self.send(SYS_CMD, p)
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("[Serial] Stopped.")


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
        self.position     = 0.0
        self.velocity     = 0.0
        self.target_vel   = 0
        self.target_pos   = 0
        self.pwm          = 0
        self.vel_integral = 0.0
        self.fault_flags  = 0
        self.current_ma   = 0.0
        self.kp_vel = 2.0;  self.ki_vel = 0.5;  self.kd_vel = 0.05
        self.kp_pos = 1.5;  self.ki_pos = 0.05; self.kd_pos = 0.1

    def update(self, dt: float):
        if dt <= 0:
            return

        if self.mode == _DC_DISABLED:
            self.velocity *= math.exp(-dt / 0.08)
            self.pwm = 0
            self.current_ma = 0.0

        elif self.mode == _DC_VELOCITY:
            target = float(self.target_vel)
            tau = 0.15
            alpha = 1.0 - math.exp(-dt / tau)
            self.velocity += (target - self.velocity) * alpha
            self.velocity += random.gauss(0, 1.5)
            self.pwm = int(_clamp(self.velocity / 10.0, -255, 255))
            self.current_ma = abs(self.velocity) * 0.28 + random.gauss(45, 8)

        elif self.mode == _DC_POSITION:
            pos_error = float(self.target_pos) - self.position
            vel_setpoint = _clamp(pos_error * 8.0, -900, 900)
            tau = 0.15
            alpha = 1.0 - math.exp(-dt / tau)
            self.velocity += (vel_setpoint - self.velocity) * alpha
            self.velocity += random.gauss(0, 1.0)
            self.pwm = int(_clamp(self.velocity / 10.0, -255, 255))
            self.current_ma = abs(self.velocity) * 0.28 + random.gauss(55, 10)

        elif self.mode == _DC_PWM:
            target_vel = float(self.pwm) * 9.0
            tau = 0.10
            alpha = 1.0 - math.exp(-dt / tau)
            self.velocity += (target_vel - self.velocity) * alpha
            self.velocity += random.gauss(0, 1.5)
            self.current_ma = abs(self.velocity) * 0.25 + random.gauss(40, 8)

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
        self.position  = 0
        self.target    = 0
        self.speed     = 0.0
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
        decel_dist = (self.speed ** 2) / (2.0 * self.accel) if self.accel > 0 else 0.0

        if self.state == _STEP_HOMING:
            self.speed = min(self.speed + self.accel * dt, 200.0)
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
        self.uptime_us  = 0
        self.last_rx_ms = 9999
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

        self.dc = [_DC() for _ in range(4)]
        self.steppers = [_Stepper() for _ in range(4)]

        self.servo_connected  = True
        self.servo_error      = 0
        self.servo_enabled_mask = 0
        self.servo_pulses     = [1500] * 16

        self.imu_yaw   = 0.0
        self.imu_pitch = 0.0
        self.imu_roll  = 0.0

        self.odom_x     = 0.0
        self.odom_y     = 0.0
        self.odom_theta = 0.0

        self.battery_mv  = float(random.randint(12400, 12800))
        self.rail5v_mv   = float(random.randint(4990, 5020))
        self.servo_rail_mv = 0.0

        self.button_mask   = 0
        self.led_brightness = [0, 0, 0]
        self.neopixel_rgb  = [0, 0, 30]

        self._init_timer = 0.0
        self._last_update = time.time()

    def receive_command(self):
        self.last_rx_ms = 0
        self.last_cmd_ms = 0

    def update(self, dt: float):
        self._init_timer += dt
        self.uptime_us += int(dt * 1_000_000)

        self.last_rx_ms  = min(self.last_rx_ms  + int(dt * 1000), 9999)
        self.last_cmd_ms = min(self.last_cmd_ms + int(dt * 1000), 9999)

        if self.state == _SYS_INIT and self._init_timer >= 2.0:
            self.state = _SYS_IDLE
            self.neopixel_rgb = [0, 20, 0]

        if self.state == _SYS_RUNNING:
            self._update_motors(dt)
            self._update_steppers(dt)
            self._update_kinematics(dt)
            self._update_imu(dt)

        active_motors = sum(1 for m in self.dc if m.mode != _DC_DISABLED)
        drain = (0.3 + active_motors * 0.15) * dt
        self.battery_mv = max(9000.0, self.battery_mv - drain)
        self.rail5v_mv  = 5000.0 + random.gauss(5, 2)

        if random.random() < dt * 0.5:
            btn = random.randint(0, 9)
            self.button_mask ^= (1 << btn)

        self.loop_avg_us = int(420 + random.gauss(0, 15))
        self.loop_max_us = max(self.loop_avg_us, int(680 + random.gauss(0, 40)))

    def _update_motors(self, dt):
        for m in self.dc:
            m.update(dt)

    def _update_steppers(self, dt):
        for s in self.steppers:
            s.update(dt)

    def _update_kinematics(self, dt):
        mm_per_tick = _MM_PER_TICK
        v_left  = self.dc[0].velocity * mm_per_tick
        v_right = self.dc[1].velocity * mm_per_tick
        v_linear  = (v_left + v_right) * 0.5
        v_angular = (v_right - v_left) / self.wheel_base_mm
        self.odom_theta += v_angular * dt
        self.odom_x     += v_linear * math.cos(self.odom_theta) * dt
        self.odom_y     += v_linear * math.sin(self.odom_theta) * dt

    def _update_imu(self, dt):
        mm_per_tick = _MM_PER_TICK
        v_left  = self.dc[0].velocity * mm_per_tick
        v_right = self.dc[1].velocity * mm_per_tick
        v_angular = (v_right - v_left) / self.wheel_base_mm
        self.imu_yaw += v_angular * dt + random.gauss(0, 0.0005)

    def _euler_to_quat(self, yaw, pitch, roll):
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

    Runs entirely within the asyncio event loop (no threads needed) — the mock
    generates telemetry at a fixed rate without any blocking I/O. handle_incoming()
    is called synchronously from the async loop, so asyncio.create_task() is safe.

    The real SerialManager uses a blocking reader thread and
    asyncio.run_coroutine_threadsafe() instead — see above.
    """

    _TICK_SYS_STATUS_IDLE    = 100  # 1 Hz  (base rate 100 Hz)
    _TICK_SYS_STATUS_RUNNING = 10   # 10 Hz
    _TICK_VOLTAGE            = 10   # 10 Hz
    _TICK_FULL               = 5    # 20 Hz (DC, step, servo, IMU, kin, IO)

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
    # ROS2 stub — mock doesn't publish ROS2 topics (no hardware to mirror)
    # ------------------------------------------------------------------

    def set_ros2_node(self, node) -> None:
        pass  # intentionally a no-op for the mock

    # ------------------------------------------------------------------
    # Command handler
    # ------------------------------------------------------------------

    def send(self, tlv_type: int, payload: ctypes.Structure):
        self.stats["tx_count"] += 1
        self.arduino.receive_command()
        try:
            self._handle_command(tlv_type, payload)
        except Exception as e:
            print(f"[Mock] Command handling error (type={tlv_type:#06x}): {e}")

    def _handle_command(self, tlv_type: int, payload):
        a = self.arduino

        if tlv_type == SYS_HEARTBEAT:
            pass

        elif tlv_type == SYS_CMD:
            cmd = payload.command
            if cmd == 1 and a.state == _SYS_IDLE:
                a.state = _SYS_RUNNING
                a.neopixel_rgb = [0, 60, 0]
                print("[Mock] Arduino → RUNNING")
            elif cmd == 2 and a.state == _SYS_RUNNING:
                a.state = _SYS_IDLE
                for m in a.dc:
                    m.mode = _DC_DISABLED
                a.neopixel_rgb = [0, 20, 0]
                print("[Mock] Arduino → IDLE")
            elif cmd == 3 and a.state in (_SYS_ERROR, _SYS_ESTOP):
                a.state = _SYS_IDLE
                a.error_flags = 0
                a.neopixel_rgb = [0, 20, 0]
                print("[Mock] Arduino → IDLE (reset)")
            elif cmd == 4:
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
                if payload.loopType == 0:
                    m.kp_pos = payload.kp
                    m.ki_pos = payload.ki
                    m.kd_pos = payload.kd
                else:
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
                if payload.moveType == 0:
                    a.steppers[sid].target = payload.target
                else:
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
    # Telemetry generation
    # ------------------------------------------------------------------

    def _emit(self, tlv_type: int, payload: ctypes.Structure):
        data = bytes(payload)
        self.message_router.handle_incoming(tlv_type, data)
        self.stats["rx_count"] += 1

    def _emit_raw(self, tlv_type: int, raw_bytes: bytes):
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
            p.pulseUs[i] = a.servo_pulses[i] if (a.servo_enabled_mask & (1 << i)) else 0
        self._emit(SERVO_STATUS_ALL, p)

    def _gen_sensor_imu(self):
        a = self.arduino
        qw, qx, qy, qz = a._euler_to_quat(a.imu_yaw, a.imu_pitch, a.imu_roll)

        mm_per_tick = _MM_PER_TICK
        v_left  = a.dc[0].velocity * mm_per_tick
        v_right = a.dc[1].velocity * mm_per_tick

        p = PayloadSensorIMU()
        p.quatW = qw;  p.quatX = qx;  p.quatY = qy;  p.quatZ = qz

        p.earthAccX = float(random.gauss(0, 0.005))
        p.earthAccY = float(random.gauss(0, 0.003))
        p.earthAccZ = float(random.gauss(0, 0.002))

        p.rawAccX = int(_clamp(random.gauss(0, 5), -32768, 32767))
        p.rawAccY = int(_clamp(random.gauss(0, 3), -32768, 32767))
        p.rawAccZ = int(_clamp(-9810 + random.gauss(0, 15), -32768, 32767))

        v_angular = (v_right - v_left) / a.wheel_base_mm
        gyro_z_dps = math.degrees(v_angular) * 10.0
        p.rawGyroX = int(_clamp(random.gauss(0, 2), -32768, 32767))
        p.rawGyroY = int(_clamp(random.gauss(0, 2), -32768, 32767))
        p.rawGyroZ = int(_clamp(gyro_z_dps + random.gauss(0, 3), -32768, 32767))

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
        fixed_bytes = bytes(p)
        neo_bytes = bytes(a.neopixel_rgb[:3])
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

        TARGET_DT = 0.01
        last_tick = time.monotonic()

        while self._running:
            now = time.monotonic()
            dt  = now - last_tick
            last_tick = now

            self.arduino.update(dt)
            self._tick += 1

            a     = self.arduino
            state = a.state

            sys_div = self._TICK_SYS_STATUS_RUNNING if state == _SYS_RUNNING \
                      else self._TICK_SYS_STATUS_IDLE
            if self._tick % sys_div == 0:
                self._gen_sys_status()

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

            real_now = time.time()
            if real_now - self.last_stats_time >= STATS_INTERVAL:
                await self._broadcast_stats()
                self.last_stats_time = real_now

            elapsed = time.monotonic() - now
            sleep_time = max(0.0, TARGET_DT - elapsed)
            await asyncio.sleep(sleep_time)

    def stop(self):
        self._running = False
        stop_cmd = PayloadSysCmd()
        stop_cmd.command = 2
        self._handle_command(SYS_CMD, stop_cmd)
        print("[Mock] Stopped.")
