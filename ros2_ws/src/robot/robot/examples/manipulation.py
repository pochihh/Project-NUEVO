"""
manipulation.py — blocking pick sequence example
================================================
Demonstrates one simple manipulator sequence using:

1. Servo 1 as a gripper
2. Stepper 1 as a horizontal arm
3. DC Motor 3 in POSITION mode as a vertical lift

HOW TO RUN
----------
Copy this file over main.py, then restart the robot node:

    cp examples/manipulation.py main.py
    ros2 run robot robot

WHAT THE ROBOT DOES
-------------------
Press BTN_1 to run one pick sequence:

  1. Raise the lift
  2. Extend the arm
  3. Open the gripper
  4. Lower the lift
  5. Close the gripper
  6. Raise the lift
  7. Retract the arm

This example uses a blocking sequence on purpose, so the code is easy to read.
During the sequence, the FSM stays inside one state until the whole sequence
finishes.

WHAT THIS TEACHES
-----------------
1. `home_stepper()` for stepper homing at startup
2. `set_servo()` for gripper angle control
3. `enable_motor(..., DCMotorMode.POSITION)` for M3 position control
4. `set_motor_position(..., max_vel_ticks=...)` to apply a velocity limit
5. Writing a simple blocking actuator sequence with `time.sleep(...)`
"""

from __future__ import annotations

import time

from robot.hardware_map import (
    Button,
    DCMotorMode,
    DEFAULT_FSM_HZ,
    LED,
    Motor,
    POSITION_UNIT,
    ServoChannel,
    StepMoveType,
    Stepper,
)
from robot.robot import FirmwareState, Robot


# ---------------------------------------------------------------------------
# Actuator configuration — edit these to match your build
# ---------------------------------------------------------------------------

# Servo 1 — gripper jaw
GRIPPER_SERVO = ServoChannel.CH_1
GRIPPER_OPEN_DEG = 15.0
GRIPPER_CLOSE_DEG = 120.0
GRIPPER_SETTLE_S = 1.0

# Stepper 1 — horizontal arm extension
ARM_STEPPER = Stepper.STEPPER_1
ARM_EXTEND_STEPS = 2000
ARM_MAX_VELOCITY = 800
ARM_ACCELERATION = 400
ARM_HOME_VELOCITY = 300
ARM_MOVE_TIMEOUT_S = 10.0

# DC Motor 3 — vertical lift in position mode
LIFT_MOTOR = Motor.DC_M3
LIFT_UP_TICKS = 1500
LIFT_DOWN_TICKS = 0
LIFT_MAX_VEL_TICKS = 600
LIFT_TOLERANCE_TICKS = 30
LIFT_MOVE_TIMEOUT_S = 5.0


def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


def home_arm(robot: Robot) -> bool:
    """Home the arm stepper against its limit switch."""
    print("[FSM] HOMING — press BTN_3 to trigger the shared LIM1 input for stepper 1")
    robot.enable_stepper(ARM_STEPPER)
    ok = robot.home_stepper(
        ARM_STEPPER,
        direction=-1,
        home_velocity=ARM_HOME_VELOCITY,
        backoff_steps=50,
        blocking=True,
        timeout=15.0,
    )
    if not ok:
        print("[warn] arm homing timed out — check LIM1 or use BTN_3 to simulate it")
        robot.disable_stepper(ARM_STEPPER)
        return False
    robot.disable_stepper(ARM_STEPPER)
    return True


def restore_idle_lift(robot: Robot) -> bool:
    """
    Return M3 to the idle position and zero its encoder there.

    This keeps the low position as the encoder reference for the next run.
    """
    robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
    ok = robot.set_motor_position(
        LIFT_MOTOR,
        LIFT_DOWN_TICKS,
        max_vel_ticks=LIFT_MAX_VEL_TICKS,
        tolerance_ticks=LIFT_TOLERANCE_TICKS,
        blocking=True,
        timeout=LIFT_MOVE_TIMEOUT_S,
    )
    if not ok:
        print("[warn] lift failed to reach idle position")
        robot.disable_motor(LIFT_MOTOR)
        return False
    robot.reset_motor_position(LIFT_MOTOR)
    robot.disable_motor(LIFT_MOTOR)
    return True


def run_pick_sequence(robot: Robot) -> bool:
    """Run one blocking pick sequence from top to bottom."""
    robot.set_stepper_config(
        ARM_STEPPER,
        max_velocity=ARM_MAX_VELOCITY,
        acceleration=ARM_ACCELERATION,
    )
    robot.enable_servo(GRIPPER_SERVO)
    robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
    robot.enable_stepper(ARM_STEPPER)

    print("[SEQ] raise lift")
    if not robot.set_motor_position(
        LIFT_MOTOR,
        LIFT_UP_TICKS,
        max_vel_ticks=LIFT_MAX_VEL_TICKS,
        tolerance_ticks=LIFT_TOLERANCE_TICKS,
        blocking=True,
        timeout=LIFT_MOVE_TIMEOUT_S,
    ):
        print("[warn] lift failed to reach the raised position")
        robot.disable_stepper(ARM_STEPPER)
        robot.disable_servo(GRIPPER_SERVO)
        robot.disable_motor(LIFT_MOTOR)
        return False

    print("[SEQ] extend arm")
    if not robot.move_stepper(
        ARM_STEPPER,
        steps=ARM_EXTEND_STEPS,
        move_type=StepMoveType.RELATIVE,
        blocking=True,
        timeout=ARM_MOVE_TIMEOUT_S,
    ):
        print("[warn] arm failed to extend — check stepper enable or home limit wiring")
        robot.disable_stepper(ARM_STEPPER)
        robot.disable_servo(GRIPPER_SERVO)
        robot.disable_motor(LIFT_MOTOR)
        return False

    print("[SEQ] open gripper")
    robot.set_servo(GRIPPER_SERVO, GRIPPER_OPEN_DEG)
    time.sleep(GRIPPER_SETTLE_S)

    print("[SEQ] lower lift")
    if not robot.set_motor_position(
        LIFT_MOTOR,
        LIFT_DOWN_TICKS,
        max_vel_ticks=LIFT_MAX_VEL_TICKS,
        tolerance_ticks=LIFT_TOLERANCE_TICKS,
        blocking=True,
        timeout=LIFT_MOVE_TIMEOUT_S,
    ):
        print("[warn] lift failed to lower")
        robot.disable_stepper(ARM_STEPPER)
        robot.disable_servo(GRIPPER_SERVO)
        robot.disable_motor(LIFT_MOTOR)
        return False

    print("[SEQ] close gripper")
    robot.set_servo(GRIPPER_SERVO, GRIPPER_CLOSE_DEG)
    time.sleep(GRIPPER_SETTLE_S)

    print("[SEQ] raise lift")
    if not robot.set_motor_position(
        LIFT_MOTOR,
        LIFT_UP_TICKS,
        max_vel_ticks=LIFT_MAX_VEL_TICKS,
        tolerance_ticks=LIFT_TOLERANCE_TICKS,
        blocking=True,
        timeout=LIFT_MOVE_TIMEOUT_S,
    ):
        print("[warn] lift failed to raise with the part")
        robot.disable_stepper(ARM_STEPPER)
        robot.disable_servo(GRIPPER_SERVO)
        robot.disable_motor(LIFT_MOTOR)
        return False

    print("[SEQ] retract arm")
    if not robot.move_stepper(
        ARM_STEPPER,
        steps=ARM_EXTEND_STEPS,
        move_type=StepMoveType.RELATIVE,
        blocking=True,
        timeout=ARM_MOVE_TIMEOUT_S,
    ):
        print("[warn] arm failed to retract")
        robot.disable_stepper(ARM_STEPPER)
        robot.disable_servo(GRIPPER_SERVO)
        robot.disable_motor(LIFT_MOTOR)
        return False

    robot.disable_stepper(ARM_STEPPER)
    robot.disable_servo(GRIPPER_SERVO)

    print("[SEQ] return lift to idle")
    return restore_idle_lift(robot)


def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:
        if state == "INIT":
            start_robot(robot)
            home_arm(robot)
            restore_idle_lift(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — press BTN_1 to run the pick sequence")
            print(
                f"[CFG] gripper open={GRIPPER_OPEN_DEG:.0f}° close={GRIPPER_CLOSE_DEG:.0f}°"
            )
            print(
                f"[CFG] lift low={LIFT_DOWN_TICKS} high={LIFT_UP_TICKS} ticks "
                f"max_vel={LIFT_MAX_VEL_TICKS} ticks/s"
            )
            print(
                f"[CFG] arm extend={ARM_EXTEND_STEPS} steps "
                f"home_vel={ARM_HOME_VELOCITY} steps/s"
            )
            state = "IDLE"

        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                show_running_leds(robot)
                print("[FSM] RUN_SEQUENCE")
                state = "RUN_SEQUENCE"

        elif state == "RUN_SEQUENCE":
            ok = run_pick_sequence(robot)
            show_idle_leds(robot)
            if ok:
                print("[FSM] IDLE — sequence complete")
            else:
                print("[FSM] IDLE — sequence stopped due to actuator timeout")
            state = "IDLE"

        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
