"""
robot_impl/hardware.py — HardwareMixin

Low-level hardware access: system state, DC motors, steppers, servos,
buttons, limit switches, and LEDs.  All state lives in Robot.__init__;
this mixin only defines methods.
"""
from __future__ import annotations

import math
import threading
import time
import warnings
from enum import IntEnum

from bridge_interfaces.msg import (
    DCEnable,
    DCHome,
    DCPid,
    DCPidReq,
    DCPidSet,
    DCResetPosition,
    DCSetPosition,
    DCSetPwm,
    DCSetVelocity,
    DCStateAll,
    IOSetLed,
    IOSetNeopixel,
    IOInputState,
    IOOutputState,
    ServoEnable,
    ServoSet,
    ServoStateAll,
    StepConfig,
    StepConfigReq,
    StepConfigSet,
    StepEnable,
    StepHome,
    StepMove,
    StepStateAll,
    SystemConfig,
    SystemDiag,
    SystemInfo,
    SystemPower,
    SystemState,
)
from bridge_interfaces.srv import SetFirmwareState

from robot.hardware_map import (
    BUTTON_COUNT,
    DCMotorMode,
    DCPidLoop,
    LEDMode,
    LIMIT_COUNT,
    StepMoveType,
    StepperMotionState,
)


class FirmwareState(IntEnum):
    IDLE    = 1
    RUNNING = 2
    ERROR   = 3
    ESTOP   = 4


class HardwareMixin:
    """Hardware callbacks and low-level actuator/IO methods."""

    # =========================================================================
    # Subscription callbacks — system
    # =========================================================================

    def _on_sys_state(self, msg: SystemState) -> None:
        with self._lock:
            self._sys_state = msg.state

    def _on_sys_power(self, msg: SystemPower) -> None:
        with self._lock:
            self._sys_power = msg

    def _on_sys_info(self, msg: SystemInfo) -> None:
        with self._lock:
            self._sys_info = msg

    def _on_sys_config(self, msg: SystemConfig) -> None:
        with self._lock:
            self._sys_config = msg

    def _on_sys_diag(self, msg: SystemDiag) -> None:
        with self._lock:
            self._sys_diag = msg

    def _on_dc_pid(self, msg: DCPid) -> None:
        with self._lock:
            self._dc_pid_cache[(int(msg.motor_number), int(msg.loop_type))] = msg

    def _on_dc_state(self, msg: DCStateAll) -> None:
        with self._lock:
            self._dc_state = msg

    def _on_step_config(self, msg: StepConfig) -> None:
        with self._lock:
            self._step_config_cache[int(msg.stepper_number)] = msg

    def _on_step_state(self, msg: StepStateAll) -> None:
        with self._lock:
            self._step_state = msg

    def _on_servo_state(self, msg: ServoStateAll) -> None:
        with self._lock:
            self._servo_state = msg

    def _on_io_input(self, msg: IOInputState) -> None:
        with self._lock:
            prev_btn = self._buttons if self._have_io_input else msg.button_mask
            prev_lim = self._limits if self._have_io_input else msg.limit_mask
            self._buttons = msg.button_mask
            self._limits  = msg.limit_mask
            self._button_edges |= msg.button_mask & ~prev_btn
            self._limit_edges |= msg.limit_mask & ~prev_lim
            self._have_io_input = True

        for bit in range(16):
            bid = bit + 1
            if ((msg.button_mask >> bit) & 1) and not ((prev_btn >> bit) & 1):
                ev = self._button_events.get(bid)
                if ev:
                    ev.set()
            if ((msg.limit_mask >> bit) & 1) and not ((prev_lim >> bit) & 1):
                ev = self._limit_events.get(bid)
                if ev:
                    ev.set()

    def _on_io_output(self, msg: IOOutputState) -> None:
        with self._lock:
            self._io_output_state = msg

    # =========================================================================
    # System
    # =========================================================================

    def get_state(self) -> int:
        """Return cached firmware state. Compare to FirmwareState enum."""
        with self._lock:
            return self._sys_state

    def set_state(self, state: FirmwareState, timeout: float = 5.0) -> bool:
        """
        Request a firmware state transition via /set_firmware_state.
        Blocks until the transition completes or timeout expires.
        """
        if not self._set_state_client.wait_for_service(timeout_sec=timeout):
            self._node.get_logger().error('set_firmware_state service not available')
            return False
        req = SetFirmwareState.Request()
        req.target_state = int(state)
        req.timeout_sec  = float(timeout)
        future = self._set_state_client.call_async(req)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        done.wait(timeout=timeout + 1.0)
        if not future.done():
            return False
        result = future.result()
        return result is not None and result.success

    def estop(self) -> bool:
        """Immediately transition firmware to ESTOP."""
        return self.set_state(FirmwareState.ESTOP)

    def reset_estop(self) -> bool:
        """Clear ESTOP and return to IDLE."""
        return self.set_state(FirmwareState.IDLE)

    def get_power(self) -> SystemPower:
        """Return cached power state (battery_mv, rail_5v_mv, servo_rail_mv)."""
        with self._lock:
            return self._sys_power

    def get_system_info(self) -> SystemInfo:
        """Return cached system information from /sys_info_rsp."""
        with self._lock:
            return self._sys_info

    def get_system_config(self) -> SystemConfig:
        """Return cached system configuration from /sys_config_rsp."""
        with self._lock:
            return self._sys_config

    def get_system_diag(self) -> SystemDiag:
        """Return cached system diagnostics from /sys_diag_rsp."""
        with self._lock:
            return self._sys_diag

    # =========================================================================
    # DC motors — low-level
    # =========================================================================

    def set_motor_pwm(self, motor_id: int, pwm: int) -> None:
        """Raw PWM command, −255 … 255."""
        motor_id = self._require_id("motor_id", motor_id, 1, 4)
        msg = DCSetPwm()
        msg.motor_number = motor_id
        msg.pwm = int(pwm)
        self._dc_pwm_pub.publish(msg)

    def set_motor_position(
        self,
        motor_id: int,
        ticks: int,
        max_vel_ticks: int = 200,
        blocking: bool = True,
        tolerance_ticks: int = 20,
        timeout: float = None,
    ) -> bool:
        """
        Move a DC motor to an absolute encoder position.
        Returns True when within tolerance, False on timeout.
        """
        motor_id = self._require_id("motor_id", motor_id, 1, 4)
        msg = DCSetPosition()
        msg.motor_number  = motor_id
        msg.target_ticks  = int(ticks)
        msg.max_vel_ticks = int(max_vel_ticks)
        self._dc_pos_pub.publish(msg)
        if not blocking:
            return True
        return self._wait_dc_position(motor_id, ticks, tolerance_ticks, timeout)

    def enable_motor(
        self,
        motor_id: int,
        mode: DCMotorMode | int = DCMotorMode.VELOCITY,
    ) -> None:
        """Enable a DC motor in the requested firmware mode."""
        motor_id = self._require_id("motor_id", motor_id, 1, 4)
        mode = self._require_enum("mode", mode, DCMotorMode)
        msg = DCEnable()
        msg.motor_number = motor_id
        msg.mode = mode
        self._dc_en_pub.publish(msg)

    def disable_motor(self, motor_id: int) -> None:
        """Disable a DC motor (mode 0)."""
        motor_id = self._require_id("motor_id", motor_id, 1, 4)
        msg = DCEnable()
        msg.motor_number = motor_id
        msg.mode = int(DCMotorMode.DISABLED)
        self._dc_en_pub.publish(msg)

    def home_motor(
        self,
        motor_id: int,
        direction: int = 1,
        home_velocity: int = 100,
        blocking: bool = True,
        timeout: float = None,
    ) -> bool:
        """
        Run a DC motor toward its limit switch for homing.
        direction: +1 or -1
        Returns True when homing completes, False on timeout.
        """
        motor_id = self._require_id("motor_id", motor_id, 1, 4)
        msg = DCHome()
        msg.motor_number  = motor_id
        msg.direction     = direction
        msg.home_velocity = home_velocity
        self._dc_home_pub.publish(msg)
        if not blocking:
            return True
        return self._wait_dc_not_homing(motor_id, timeout)

    def reset_motor_position(self, motor_id: int) -> None:
        """Zero the encoder position for a DC motor."""
        motor_id = self._require_id("motor_id", motor_id, 1, 4)
        msg = DCResetPosition()
        msg.motor_number = motor_id
        self._dc_rst_pub.publish(msg)

    def set_pid_gains(
        self,
        motor_id: int,
        loop_type: DCPidLoop | int,
        kp: float,
        ki: float,
        kd: float,
        max_output: float = 255.0,
        max_integral: float = 1000.0,
    ) -> None:
        """Set PID gains for a DC motor. loop_type: 0=position, 1=velocity."""
        motor_id = self._require_id("motor_id", motor_id, 1, 4)
        loop_type = self._require_enum("loop_type", loop_type, DCPidLoop)
        msg = DCPidSet()
        msg.motor_number  = motor_id
        msg.loop_type     = loop_type
        msg.kp            = float(kp)
        msg.ki            = float(ki)
        msg.kd            = float(kd)
        msg.max_output    = float(max_output)
        msg.max_integral  = float(max_integral)
        self._dc_pid_set.publish(msg)

    def request_pid(self, motor_id: int, loop_type: DCPidLoop | int) -> None:
        """Request PID parameters from firmware. Response arrives on /dc_pid_rsp."""
        motor_id = self._require_id("motor_id", motor_id, 1, 4)
        loop_type = self._require_enum("loop_type", loop_type, DCPidLoop)
        msg = DCPidReq()
        msg.motor_number = motor_id
        msg.loop_type    = loop_type
        self._dc_pid_req.publish(msg)

    def get_pid(self, motor_id: int, loop_type: DCPidLoop | int) -> DCPid | None:
        """Return cached PID parameters for one motor/loop pair, or None if unknown."""
        motor_id = self._require_id("motor_id", motor_id, 1, 4)
        loop_type = self._require_enum("loop_type", loop_type, DCPidLoop)
        with self._lock:
            return self._dc_pid_cache.get((motor_id, loop_type))

    def get_dc_state(self) -> DCStateAll:
        """Return cached DC motor state (position, velocity, mode, etc.)."""
        with self._lock:
            return self._dc_state

    # =========================================================================
    # Stepper motors
    # =========================================================================

    def enable_stepper(self, stepper_id: int) -> None:
        """Enable a stepper motor."""
        stepper_id = self._require_id("stepper_id", stepper_id, 1, 4)
        msg = StepEnable()
        msg.stepper_number = stepper_id
        msg.enable = True
        self._step_en_pub.publish(msg)

    def disable_stepper(self, stepper_id: int) -> None:
        """Disable a stepper motor."""
        stepper_id = self._require_id("stepper_id", stepper_id, 1, 4)
        msg = StepEnable()
        msg.stepper_number = stepper_id
        msg.enable = False
        self._step_en_pub.publish(msg)

    def move_stepper(
        self,
        stepper_id: int,
        steps: int,
        move_type: StepMoveType | int = StepMoveType.RELATIVE,
        blocking: bool = True,
        timeout: float = None,
    ) -> bool:
        """
        Move a stepper motor.
        move_type: 0=absolute, 1=relative.
        Returns True when motion completes, False on timeout.
        """
        stepper_id = self._require_id("stepper_id", stepper_id, 1, 4)
        move_type = self._require_enum("move_type", move_type, StepMoveType)
        msg = StepMove()
        msg.stepper_number = stepper_id
        msg.move_type      = move_type
        msg.target         = int(steps)
        self._step_mv_pub.publish(msg)
        if not blocking:
            return True
        return self._wait_stepper_idle(stepper_id, timeout)

    def home_stepper(
        self,
        stepper_id: int,
        direction: int = -1,
        home_velocity: int = 500,
        backoff_steps: int = 50,
        blocking: bool = True,
        timeout: float = None,
    ) -> bool:
        """
        Home a stepper motor against its limit switch.
        Returns True when complete, False on timeout.
        """
        stepper_id = self._require_id("stepper_id", stepper_id, 1, 4)
        msg = StepHome()
        msg.stepper_number = stepper_id
        msg.direction      = direction
        msg.home_velocity  = home_velocity
        msg.backoff_steps  = backoff_steps
        self._step_hm_pub.publish(msg)
        if not blocking:
            return True
        return self._wait_stepper_idle(stepper_id, timeout)

    def set_stepper_config(
        self,
        stepper_id: int,
        max_velocity: int,
        acceleration: int,
    ) -> None:
        """Set speed and acceleration for a stepper motor."""
        stepper_id = self._require_id("stepper_id", stepper_id, 1, 4)
        msg = StepConfigSet()
        msg.stepper_number = stepper_id
        msg.max_velocity   = int(max_velocity)
        msg.acceleration   = int(acceleration)
        self._step_cfg_pub.publish(msg)

    def request_stepper_config(self, stepper_id: int) -> None:
        """Request current speed/acceleration config for one stepper."""
        stepper_id = self._require_id("stepper_id", stepper_id, 1, 4)
        msg = StepConfigReq()
        msg.stepper_number = stepper_id
        self._step_cfg_req_pub.publish(msg)

    def get_stepper_config(self, stepper_id: int) -> StepConfig | None:
        """Return cached config for one stepper, or None if not yet seen."""
        stepper_id = self._require_id("stepper_id", stepper_id, 1, 4)
        with self._lock:
            return self._step_config_cache.get(stepper_id)

    def get_stepper_state(self) -> StepStateAll:
        """Return cached stepper motor state."""
        with self._lock:
            return self._step_state

    # =========================================================================
    # Deprecated step_* compatibility aliases
    # =========================================================================

    def step_enable(self, stepper_id: int) -> None:
        """Deprecated; use enable_stepper()."""
        warnings.warn(
            "step_enable() is deprecated; use enable_stepper()",
            DeprecationWarning,
            stacklevel=2,
        )
        self.enable_stepper(stepper_id)

    def step_disable(self, stepper_id: int) -> None:
        """Deprecated; use disable_stepper()."""
        warnings.warn(
            "step_disable() is deprecated; use disable_stepper()",
            DeprecationWarning,
            stacklevel=2,
        )
        self.disable_stepper(stepper_id)

    def step_move(
        self,
        stepper_id: int,
        steps: int,
        move_type: StepMoveType | int = StepMoveType.RELATIVE,
        blocking: bool = True,
        timeout: float = None,
    ) -> bool:
        """Deprecated; use move_stepper()."""
        warnings.warn(
            "step_move() is deprecated; use move_stepper()",
            DeprecationWarning,
            stacklevel=2,
        )
        return self.move_stepper(stepper_id, steps, move_type, blocking, timeout)

    def step_home(
        self,
        stepper_id: int,
        direction: int = -1,
        home_velocity: int = 500,
        backoff_steps: int = 50,
        blocking: bool = True,
        timeout: float = None,
    ) -> bool:
        """Deprecated; use home_stepper()."""
        warnings.warn(
            "step_home() is deprecated; use home_stepper()",
            DeprecationWarning,
            stacklevel=2,
        )
        return self.home_stepper(
            stepper_id,
            direction,
            home_velocity,
            backoff_steps,
            blocking,
            timeout,
        )

    def step_set_config(
        self,
        stepper_id: int,
        max_velocity: int,
        acceleration: int,
    ) -> None:
        """Deprecated; use set_stepper_config()."""
        warnings.warn(
            "step_set_config() is deprecated; use set_stepper_config()",
            DeprecationWarning,
            stacklevel=2,
        )
        self.set_stepper_config(stepper_id, max_velocity, acceleration)

    def request_step_config(self, stepper_id: int) -> None:
        """Deprecated; use request_stepper_config()."""
        warnings.warn(
            "request_step_config() is deprecated; use request_stepper_config()",
            DeprecationWarning,
            stacklevel=2,
        )
        self.request_stepper_config(stepper_id)

    def get_step_config(self, stepper_id: int) -> StepConfig | None:
        """Deprecated; use get_stepper_config()."""
        warnings.warn(
            "get_step_config() is deprecated; use get_stepper_config()",
            DeprecationWarning,
            stacklevel=2,
        )
        return self.get_stepper_config(stepper_id)

    def get_step_state(self) -> StepStateAll:
        """Deprecated; use get_stepper_state()."""
        warnings.warn(
            "get_step_state() is deprecated; use get_stepper_state()",
            DeprecationWarning,
            stacklevel=2,
        )
        return self.get_stepper_state()

    # =========================================================================
    # Servos
    # =========================================================================

    def set_servo(self, channel: int, angle_deg: float) -> None:
        """
        Set a servo to angle_deg (0–180°).
        Maps 0° → 1000 µs, 180° → 2000 µs.
        """
        channel = self._require_id("channel", channel, 1, 16)
        angle_clamped = max(0.0, min(180.0, angle_deg))
        pulse_us = int(self._SERVO_MIN_US + (angle_clamped / 180.0) * (self._SERVO_MAX_US - self._SERVO_MIN_US))
        msg = ServoSet()
        msg.channel  = channel
        msg.pulse_us = pulse_us
        self._srv_set_pub.publish(msg)

    def set_servo_pulse(self, channel: int, pulse_us: int) -> None:
        """Set a servo directly by pulse width in microseconds."""
        channel = self._require_id("channel", channel, 1, 16)
        msg = ServoSet()
        msg.channel  = channel
        msg.pulse_us = int(pulse_us)
        self._srv_set_pub.publish(msg)

    def enable_servo(self, channel: int) -> None:
        channel = self._require_id("channel", channel, 1, 16)
        msg = ServoEnable()
        msg.channel = channel
        msg.enable  = True
        self._srv_en_pub.publish(msg)

    def disable_servo(self, channel: int) -> None:
        channel = self._require_id("channel", channel, 1, 16)
        msg = ServoEnable()
        msg.channel = channel
        msg.enable  = False
        self._srv_en_pub.publish(msg)

    def get_servo_state(self) -> ServoStateAll:
        """Return cached servo state."""
        with self._lock:
            return self._servo_state

    # =========================================================================
    # IO — buttons, limit switches, LEDs, NeoPixel
    # =========================================================================

    def get_button(self, button_id: int) -> bool:
        """Non-blocking: current state of button button_id (1–10)."""
        button_id = self._require_id("button_id", button_id, 1, BUTTON_COUNT)
        with self._lock:
            return bool((self._buttons >> (button_id - 1)) & 1)

    def was_button_pressed(self, button_id: int, consume: bool = True) -> bool:
        """Return True once per rising edge seen on button_id."""
        button_id = self._require_id("button_id", button_id, 1, BUTTON_COUNT)
        mask = 1 << (button_id - 1)
        with self._lock:
            pressed = bool(self._button_edges & mask)
            if pressed and consume:
                self._button_edges &= ~mask
            return pressed

    def wait_for_button(self, button_id: int, timeout: float = None) -> bool:
        """Blocking: wait until button_id transitions from not-pressed to pressed."""
        button_id = self._require_id("button_id", button_id, 1, BUTTON_COUNT)
        with self._lock:
            if (self._buttons >> (button_id - 1)) & 1:
                return True
            ev = threading.Event()
            self._button_events[button_id] = ev

        pressed = ev.wait(timeout=timeout)
        with self._lock:
            self._button_events.pop(button_id, None)
        return pressed

    def get_limit(self, limit_id: int) -> bool:
        """Non-blocking: current state of limit switch limit_id (1–8)."""
        limit_id = self._require_id("limit_id", limit_id, 1, LIMIT_COUNT)
        with self._lock:
            return bool((self._limits >> (limit_id - 1)) & 1)

    def get_io_output_state(self) -> IOOutputState:
        """Return cached LED/NeoPixel output state from /io_output_state."""
        with self._lock:
            return self._io_output_state

    def was_limit_triggered(self, limit_id: int, consume: bool = True) -> bool:
        """Return True once per rising edge seen on limit_id."""
        limit_id = self._require_id("limit_id", limit_id, 1, LIMIT_COUNT)
        mask = 1 << (limit_id - 1)
        with self._lock:
            triggered = bool(self._limit_edges & mask)
            if triggered and consume:
                self._limit_edges &= ~mask
            return triggered

    def wait_for_limit(self, limit_id: int, timeout: float = None) -> bool:
        """Blocking: wait until limit switch limit_id is triggered."""
        limit_id = self._require_id("limit_id", limit_id, 1, LIMIT_COUNT)
        with self._lock:
            if (self._limits >> (limit_id - 1)) & 1:
                return True
            ev = threading.Event()
            self._limit_events[limit_id] = ev

        triggered = ev.wait(timeout=timeout)
        with self._lock:
            self._limit_events.pop(limit_id, None)
        return triggered

    def set_led(
        self,
        led_id: int,
        brightness: int,
        mode: LEDMode | int | None = None,
        period_ms: int | None = None,
        duty_cycle: int = 500,
    ) -> None:
        """
        Control an onboard LED.
        When mode is omitted, brightness 0 maps to OFF and non-zero maps to steady ON.
        Explicit modes: OFF=0, ON=1, BLINK=2, BREATHE=3, PWM=4.
        BLINK and BREATHE default to 1000 ms period when none is supplied.
        duty_cycle is in permille (0–1000).
        """
        led_id = self._require_id("led_id", led_id, 0, 4)
        clamped_brightness = max(0, min(255, int(brightness)))
        resolved_mode = (
            int(LEDMode.OFF if clamped_brightness == 0 else LEDMode.ON)
            if mode is None
            else self._require_enum("mode", mode, LEDMode)
        )
        if period_ms is None:
            period_ms = 1000 if resolved_mode in (int(LEDMode.BLINK), int(LEDMode.BREATHE)) else 0
        msg = IOSetLed()
        msg.led_id     = led_id
        msg.brightness = clamped_brightness
        msg.mode       = resolved_mode
        msg.period_ms  = max(0, int(period_ms))
        msg.duty_cycle = max(0, min(1000, int(duty_cycle)))
        self._led_pub.publish(msg)

    def set_neopixel(self, index: int, red: int, green: int, blue: int) -> None:
        """Set a NeoPixel LED color. index is 0-based."""
        msg = IOSetNeopixel()
        msg.index = index
        msg.red   = red
        msg.green = green
        msg.blue  = blue
        self._neo_pub.publish(msg)

    # =========================================================================
    # Internal — blocking waits for actuator completion
    # =========================================================================

    def _wait_dc_position(
        self, motor_id: int, target: int, tolerance: int, timeout: float
    ) -> bool:
        motor_idx = motor_id - 1
        deadline = time.monotonic() + timeout if timeout else None
        while True:
            with self._lock:
                dc = self._dc_state
            if dc is not None and abs(dc.motors[motor_idx].position - target) <= tolerance:
                return True
            if deadline and time.monotonic() > deadline:
                return False
            time.sleep(0.02)

    def _wait_dc_not_homing(self, motor_id: int, timeout: float) -> bool:
        """Wait until a DC motor's mode is no longer 4 (homing)."""
        motor_idx = motor_id - 1
        deadline = time.monotonic() + timeout if timeout else None
        time.sleep(0.1)
        while True:
            with self._lock:
                dc = self._dc_state
            if dc is not None and dc.motors[motor_idx].mode != int(DCMotorMode.HOMING):
                return True
            if deadline and time.monotonic() > deadline:
                return False
            time.sleep(0.02)

    def _wait_stepper_idle(self, stepper_id: int, timeout: float) -> bool:
        """
        Wait for a stepper command to start, then return to idle.

        This avoids the false-success case where the cached state is already
        idle when the command is sent, but the first non-idle telemetry update
        has not arrived yet.
        """
        stepper_idx = stepper_id - 1
        deadline = time.monotonic() + timeout if timeout else None
        saw_active = False
        time.sleep(0.1)
        while True:
            with self._lock:
                step = self._step_state
            if step is not None:
                motion_state = step.steppers[stepper_idx].motion_state
                if motion_state != int(StepperMotionState.IDLE):
                    saw_active = True
                elif saw_active:
                    return True
            if deadline and time.monotonic() > deadline:
                return False
            time.sleep(0.02)

    # =========================================================================
    # Internal — low-level motor helpers
    # =========================================================================

    def _send_motor_velocity_mm(self, motor_id: int, velocity_mm_s: float) -> None:
        msg = DCSetVelocity()
        msg.motor_number = motor_id
        msg.target_ticks = int(velocity_mm_s * self._ticks_per_mm)
        self._dc_vel_pub.publish(msg)

    def _ensure_drive_motors_enabled(self) -> None:
        """Enable the configured drive motors in velocity mode when needed."""
        with self._lock:
            drive_motors = tuple(dict.fromkeys((self._left_wheel_motor, self._right_wheel_motor)))
            dc_state = self._dc_state

        if dc_state is None or not hasattr(dc_state, "motors"):
            for motor_id in drive_motors:
                self.enable_motor(motor_id, DCMotorMode.VELOCITY)
            return

        for motor_id in drive_motors:
            motor_index = motor_id - 1
            try:
                current_mode = dc_state.motors[motor_index].mode
            except (AttributeError, IndexError, TypeError):
                current_mode = None
            if current_mode != int(DCMotorMode.VELOCITY):
                self.enable_motor(motor_id, DCMotorMode.VELOCITY)

    # =========================================================================
    # Internal — validation helpers
    # =========================================================================

    @staticmethod
    def _require_id(name: str, value: int, low: int, high: int) -> int:
        value = int(value)
        if not low <= value <= high:
            raise ValueError(f"{name} must be between {low} and {high}, got {value}")
        return value

    @staticmethod
    def _require_enum(name: str, value: int, enum_type: type[IntEnum]) -> int:
        try:
            return int(enum_type(int(value)))
        except (TypeError, ValueError) as exc:
            valid = ", ".join(f"{member.name}={int(member)}" for member in enum_type)
            raise ValueError(f"{name} must be one of: {valid}") from exc

    @staticmethod
    def _require_positive_float(name: str, value: float) -> float:
        value = float(value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be a finite positive number, got {value}")
        return value

    @staticmethod
    def _require_finite_float(name: str, value: float) -> float:
        value = float(value)
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite, got {value}")
        return value
