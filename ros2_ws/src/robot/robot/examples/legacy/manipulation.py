"""
manipulation.py — pick-and-place with servo, stepper, and DC motor
===================================================================
This example coordinates three actuator types to perform a pick-and-place
task: pick a disk from a fixed station and move it to a drop station.

HOW TO RUN
----------
Copy this file over main.py, then restart the robot node:

    cp examples/manipulation.py main.py
    ros2 run robot robot

ASSUMED HARDWARE
----------------
Edit the constants below to match your actual setup.

    Servo CH_1     — gripper jaw
                     GRIPPER_OPEN_DEG  (e.g. 20°) = jaw open
                     GRIPPER_CLOSE_DEG (e.g. 90°) = gripping

    Stepper 1      — horizontal arm extension
                     Home = fully retracted (use home_stepper() before running)
                     ARM_EXTEND_STEPS forward from home = over pick station

    DC Motor 3     — vertical lift (POSITION mode)
                     LIFT_UP_TICKS   = raised position
                     LIFT_DOWN_TICKS = lowered to disk level

PICK SEQUENCE (triggered by BTN_1)
-----------------------------------
  1. Raise lift           (DC Motor 3, position)
  2. Extend arm           (Stepper 1, steps forward)
  3. Open gripper         (Servo 1)
  4. Lower lift           (DC Motor 3, position)
  5. Close gripper        (Servo 1)
  6. Raise lift           (DC Motor 3, position)
  7. Retract arm          (Stepper 1, steps back)

BTN_2 cancels the active sequence at any time.

WHAT THIS TEACHES
-----------------
1. Coordinating servo, stepper, and DC motor in one sequence
2. run_task() / TaskHandle  — background sequence with cancellation support
3. task.sleep()             — cancellable delay inside a sequence
4. home_stepper()           — homing against a limit switch before moving
5. set_motor_position()     — DC motor position control with blocking
6. Separating sequence logic (worker functions) from FSM state logic
"""

from __future__ import annotations
import time

from robot.hardware_map import (
    Button,
    DEFAULT_FSM_HZ,
    DCMotorMode,
    LED,
    Motor,
    Stepper,
    ServoChannel,
    StepMoveType,
)
from robot.robot import FirmwareState, Robot, Unit
from robot.util import TaskHandle, run_task


# ---------------------------------------------------------------------------
# Robot hardware configuration
# ---------------------------------------------------------------------------

POSITION_UNIT = Unit.MM


# ---------------------------------------------------------------------------
# Actuator configuration — edit to match your build
# ---------------------------------------------------------------------------

# Servo — gripper jaw
GRIPPER_CHANNEL   = ServoChannel.CH_1
GRIPPER_OPEN_DEG  = 20.0   # degrees — jaw fully open
GRIPPER_CLOSE_DEG = 90.0   # degrees — jaw gripping

# Stepper — horizontal arm extension
ARM_STEPPER        = Stepper.STEPPER_1
ARM_EXTEND_STEPS   = 2000   # steps from home to over the pick station
ARM_MAX_VELOCITY   = 800    # steps/s
ARM_ACCELERATION   = 400    # steps/s²
ARM_HOME_VELOCITY  = 300    # steps/s for homing

# DC Motor — vertical lift (position control mode)
LIFT_MOTOR       = Motor.DC_M3
LIFT_UP_TICKS    = 3000   # encoder ticks — raised/travel position
LIFT_DOWN_TICKS  = 0      # encoder ticks — lowered to disk height
LIFT_MAX_VEL     = 250    # ticks/s
LIFT_TOLERANCE   = 30     # ticks


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def configure_robot(robot: Robot) -> None:
    robot.set_unit(POSITION_UNIT)


def start_robot(robot: Robot) -> None:
    current = robot.get_state()
    if current in (FirmwareState.ESTOP, FirmwareState.ERROR):
        robot.reset_estop()
    robot.set_state(FirmwareState.RUNNING)


def home_arm(robot: Robot) -> None:
    """Home the stepper arm against its limit switch. Blocks until complete."""
    robot.set_stepper_config(
        ARM_STEPPER,
        max_velocity=ARM_MAX_VELOCITY,
        acceleration=ARM_ACCELERATION,
    )
    ok = robot.home_stepper(
        ARM_STEPPER,
        direction=-1,
        home_velocity=ARM_HOME_VELOCITY,
        backoff_steps=50,
        blocking=True,
        timeout=15.0,
    )
    if not ok:
        print("[WARN] arm homing timed out — check limit switch")


def show_idle_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 200)
    robot.set_led(LED.GREEN, 0)


def show_running_leds(robot: Robot) -> None:
    robot.set_led(LED.ORANGE, 0)
    robot.set_led(LED.GREEN, 200)


# ---------------------------------------------------------------------------
# Sequence — pick disk
# ---------------------------------------------------------------------------

def pick_disk(robot: Robot, blocking: bool = True, timeout: float = None):
    """
    Pick a disk from the fixed pick station.

    Returns a TaskHandle when blocking=False so the FSM can poll
    is_finished() while staying responsive to button presses.
    """
    def worker(task: TaskHandle) -> None:
        # 1. Raise lift so the arm clears any obstacles while extending
        print("[SEQ] raising lift")
        robot.enable_motor(LIFT_MOTOR, DCMotorMode.POSITION)
        robot.set_motor_position(
            LIFT_MOTOR, LIFT_UP_TICKS,
            max_vel_ticks=LIFT_MAX_VEL,
            tolerance_ticks=LIFT_TOLERANCE,
            blocking=True,
            timeout=5.0,
        )
        if task.cancelled():
            return

        # 2. Extend arm over pick station
        print("[SEQ] extending arm")
        robot.enable_stepper(ARM_STEPPER)
        robot.move_stepper(
            ARM_STEPPER,
            steps=ARM_EXTEND_STEPS,
            move_type=StepMoveType.RELATIVE,
            blocking=True,
            timeout=10.0,
        )
        if task.cancelled():
            return

        # 3. Open gripper before lowering
        print("[SEQ] opening gripper")
        robot.enable_servo(GRIPPER_CHANNEL)
        robot.set_servo(GRIPPER_CHANNEL, GRIPPER_OPEN_DEG)
        if not task.sleep(0.4):   # wait for servo to reach position
            return

        # 4. Lower lift to disk
        print("[SEQ] lowering to disk")
        robot.set_motor_position(
            LIFT_MOTOR, LIFT_DOWN_TICKS,
            max_vel_ticks=LIFT_MAX_VEL,
            tolerance_ticks=LIFT_TOLERANCE,
            blocking=True,
            timeout=5.0,
        )
        if task.cancelled():
            return

        # 5. Close gripper to grip disk
        print("[SEQ] gripping disk")
        robot.set_servo(GRIPPER_CHANNEL, GRIPPER_CLOSE_DEG)
        if not task.sleep(0.5):
            return

        # 6. Raise lift with disk
        print("[SEQ] raising with disk")
        robot.set_motor_position(
            LIFT_MOTOR, LIFT_UP_TICKS,
            max_vel_ticks=LIFT_MAX_VEL,
            tolerance_ticks=LIFT_TOLERANCE,
            blocking=True,
            timeout=5.0,
        )
        if task.cancelled():
            return

        # 7. Retract arm
        print("[SEQ] retracting arm")
        robot.move_stepper(
            ARM_STEPPER,
            steps=ARM_EXTEND_STEPS,
            move_type=StepMoveType.RELATIVE,
            blocking=True,
            timeout=10.0,
        )
        robot.disable_stepper(ARM_STEPPER)

        print("[SEQ] pick complete")

    return run_task(worker, blocking=blocking, timeout=timeout)


# ---------------------------------------------------------------------------
# run() — entry point called by the robot node
# ---------------------------------------------------------------------------

def run(robot: Robot) -> None:
    configure_robot(robot)

    state = "INIT"
    task_handle = None

    period = 1.0 / float(DEFAULT_FSM_HZ)
    next_tick = time.monotonic()

    while True:

        # ── INIT ──────────────────────────────────────────────────────────
        if state == "INIT":
            start_robot(robot)
            print("[FSM] HOMING arm — do not obstruct the arm")
            home_arm(robot)
            show_idle_leds(robot)
            print("[FSM] IDLE — press BTN_1 to pick, BTN_2 to cancel")
            state = "IDLE"

        # ── IDLE ──────────────────────────────────────────────────────────
        elif state == "IDLE":
            if robot.was_button_pressed(Button.BTN_1):
                show_running_leds(robot)
                print("[FSM] PICKING")
                task_handle = pick_disk(robot, blocking=False)
                state = "PICKING"

        # ── PICKING ───────────────────────────────────────────────────────
        elif state == "PICKING":
            # BTN_2 cancels mid-sequence. task.sleep() in the worker
            # checks task.cancelled() so it exits cleanly.
            if robot.get_button(Button.BTN_2):
                if task_handle is not None:
                    task_handle.cancel()
                    task_handle.wait(timeout=3.0)
                    task_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE — cancelled")
                state = "IDLE"

            elif task_handle is not None and task_handle.is_finished():
                task_handle = None
                show_idle_leds(robot)
                print("[FSM] IDLE — pick done, press BTN_1 to pick again")
                state = "IDLE"

        # ── Tick-rate control ─────────────────────────────────────────────
        next_tick += period
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.monotonic()
