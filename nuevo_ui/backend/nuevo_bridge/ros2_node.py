"""
NuevoBridgeNode — ROS2 integration for nuevo_bridge.

This node lives in the same process as the FastAPI / WebSocket bridge and
shares the SerialManager instance.  It is only instantiated when the
NUEVO_ROS2=1 environment variable is set.

Thread model
────────────
  Serial reader thread  →  publish_decoded()  →  rclpy publisher.publish()
                                                  (rcl layer is thread-safe)
  rclpy spin thread     →  subscriber callbacks  →  SerialManager.send()
                                                  (protected by _write_lock)
  asyncio event loop    →  ws commands / heartbeat → SerialManager.send()
                                                  (also protected by _write_lock)

Topics published
────────────────
  Standard :  /odom  /imu/data  /imu/mag
  Custom   :  /nuevo/voltage  /nuevo/sys_status  /nuevo/dc_status
              /nuevo/step_status  /nuevo/servo_status  /nuevo/io_status
              /nuevo/mag_cal

Topics subscribed
─────────────────
  /nuevo/cmd_vel       (nuevo_msgs/MotorVelocities)
  /nuevo/step_cmd      (nuevo_msgs/StepCommand)
  /nuevo/servo_cmd     (nuevo_msgs/ServoCommand)
  /nuevo/sys_cmd       (nuevo_msgs/SysCommand)
  /nuevo/mag_cal_cmd   (nuevo_msgs/MagCalCmd)
"""
import threading

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField
from nav_msgs.msg import Odometry

from nuevo_msgs.msg import (
    DCStatusAll,
    IOStatus,
    MagCalCmd,
    MagCalStatus,
    MotorVelocities,
    ServoCommand,
    ServoStatusAll,
    StepCommand,
    StepStatusAll,
    SysCommand,
    SystemStatus,
    Voltage,
)

from . import ros2_converters as conv


class NuevoBridgeNode(Node):
    """ROS2 node that bridges Arduino telemetry ↔ ROS2 topics.

    Parameters
    ----------
    serial_manager : SerialManager
        The shared serial manager used to send TLV commands to the Arduino.
    message_router : MessageRouter
        The shared message router used to encode outgoing commands.
    """

    def __init__(self, serial_manager, message_router):
        super().__init__('nuevo_bridge')
        self._serial = serial_manager
        self._router = message_router
        self._clock  = self.get_clock()

        # ── QoS ───────────────────────────────────────────────────────────────
        # Depth 10 is sufficient for sensor data; no durability/reliability
        # guarantees needed (latest data always wins for control loops).
        _qos = 10

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_imu      = self.create_publisher(Imu,           '/imu/data',           _qos)
        self._pub_mag      = self.create_publisher(MagneticField, '/imu/mag',            _qos)
        self._pub_odom     = self.create_publisher(Odometry,      '/odom',               _qos)
        self._pub_voltage  = self.create_publisher(Voltage,       '/nuevo/voltage',      _qos)
        self._pub_sys      = self.create_publisher(SystemStatus,  '/nuevo/sys_status',   _qos)
        self._pub_dc       = self.create_publisher(DCStatusAll,   '/nuevo/dc_status',    _qos)
        self._pub_step     = self.create_publisher(StepStatusAll, '/nuevo/step_status',  _qos)
        self._pub_servo    = self.create_publisher(ServoStatusAll,'/nuevo/servo_status', _qos)
        self._pub_io       = self.create_publisher(IOStatus,      '/nuevo/io_status',    _qos)
        self._pub_mag_cal  = self.create_publisher(MagCalStatus,  '/nuevo/mag_cal',      _qos)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            MotorVelocities, '/nuevo/cmd_vel', self._on_cmd_vel, _qos)
        self.create_subscription(
            StepCommand, '/nuevo/step_cmd', self._on_step_cmd, _qos)
        self.create_subscription(
            ServoCommand, '/nuevo/servo_cmd', self._on_servo_cmd, _qos)
        self.create_subscription(
            SysCommand, '/nuevo/sys_cmd', self._on_sys_cmd, _qos)
        self.create_subscription(
            MagCalCmd, '/nuevo/mag_cal_cmd', self._on_mag_cal_cmd, _qos)

        # ── Topic → (publisher, converter) dispatch table ─────────────────────
        # Topics with multiple publishers (imu, kinematics) use a callable
        # handler instead of a (pub, conv) tuple.
        self._handlers = {
            'imu':              self._handle_imu,
            'kinematics':       self._handle_odom,
            'voltage':          (self._pub_voltage, conv.to_voltage),
            'system_status':    (self._pub_sys,     conv.to_sys_status),
            'dc_status_all':    (self._pub_dc,      conv.to_dc_status_all),
            'step_status_all':  (self._pub_step,    conv.to_step_status_all),
            'servo_status_all': (self._pub_servo,   conv.to_servo_status_all),
            'io_status':        (self._pub_io,       conv.to_io_status),
            'mag_cal_status':   (self._pub_mag_cal,  conv.to_mag_cal_status),
        }

        self.get_logger().info('NuevoBridgeNode started — publishing to /odom, /imu/*, /nuevo/*')

    # ── Publish dispatcher ────────────────────────────────────────────────────

    def publish_decoded(self, msg_dict: dict) -> None:
        """Publish a decoded TLV dict to the appropriate ROS2 topic(s).

        Called from the blocking serial reader thread.  Safe to call because
        rclpy publisher.publish() is thread-safe at the underlying rcl layer.
        """
        topic   = msg_dict['topic']
        handler = self._handlers.get(topic)
        if handler is None:
            return
        stamp = self._clock.now().to_msg()
        if callable(handler):
            handler(msg_dict['data'], stamp)
        else:
            pub, converter = handler
            pub.publish(converter(msg_dict['data'], stamp))

    def _handle_imu(self, data: dict, stamp) -> None:
        """IMU data → /imu/data AND /imu/mag (two topics from one TLV)."""
        self._pub_imu.publish(conv.to_imu(data, stamp))
        self._pub_mag.publish(conv.to_mag(data, stamp))

    def _handle_odom(self, data: dict, stamp) -> None:
        self._pub_odom.publish(conv.to_odom(data, stamp))

    # ── Subscriber callbacks (run in rclpy spin thread) ───────────────────────

    def _send(self, cmd: str, data: dict) -> None:
        """Encode a command and forward to the Arduino via UART."""
        result = self._router.handle_outgoing(cmd, data)
        if result:
            tlv_type, payload = result
            self._serial.send(tlv_type, payload)

    def _on_cmd_vel(self, msg: MotorVelocities) -> None:
        """Forward per-motor velocity / PWM targets to the Arduino.

        The kinematics/trajectory node is responsible for computing individual
        motor commands — the bridge just forwards them verbatim.

        mode[i]:  0 = skip  2 = velocity (ticks/sec)  3 = pwm (-255..+255)
        """
        for i in range(4):
            mode = msg.mode[i]
            if mode == 0:
                continue
            motor_number = i + 1
            val = int(msg.velocity_ticks_per_sec[i])
            if mode == 2:
                self._send('dc_set_velocity', {
                    'motorNumber': motor_number,
                    'targetTicks': val,
                })
            elif mode == 3:
                self._send('dc_set_pwm', {
                    'motorNumber': motor_number,
                    'pwm': val,
                })

    def _on_step_cmd(self, msg: StepCommand) -> None:
        """Dispatch a stepper command by command_type field.

        command_type:
          0 = STEP_MOVE       1 = STEP_HOME
          2 = STEP_ENABLE     3 = STEP_DISABLE    4 = STEP_SET_PARAMS
        """
        n  = msg.stepper_number
        ct = msg.command_type
        if ct == 0:
            self._send('step_move', {
                'stepperNumber': n,
                'moveType':      msg.move_type,
                'target':        msg.target,
            })
        elif ct == 1:
            self._send('step_home', {
                'stepperNumber': n,
                'direction':     msg.direction,
                'homeVelocity':  msg.home_velocity,
                'backoffSteps':  msg.backoff_steps,
            })
        elif ct == 2:
            self._send('step_enable', {'stepperNumber': n, 'enable': 1})
        elif ct == 3:
            self._send('step_enable', {'stepperNumber': n, 'enable': 0})
        elif ct == 4:
            self._send('step_set_params', {
                'stepperNumber': n,
                'maxVelocity':   msg.max_velocity,
                'acceleration':  msg.acceleration,
            })

    def _on_servo_cmd(self, msg: ServoCommand) -> None:
        """Enable/disable a servo channel or set its pulse width.

        If pulse_us > 0: set pulse (implies channel is enabled).
        If pulse_us == 0: send enable/disable command.
        channel == 255 (all channels) is valid for enable/disable only.
        """
        ch = msg.channel
        if msg.pulse_us > 0:
            self._send('servo_set', {'channel': ch, 'pulseUs': msg.pulse_us})
        else:
            self._send('servo_enable', {'channel': ch, 'enable': int(msg.enable)})

    def _on_sys_cmd(self, msg: SysCommand) -> None:
        """Forward system state-machine command (START/STOP/RESET/ESTOP)."""
        self._send('sys_cmd', {'command': msg.command})

    def _on_mag_cal_cmd(self, msg: MagCalCmd) -> None:
        """Forward magnetometer calibration command."""
        self._send('mag_cal_cmd', {
            'command': msg.command,
            'offsetX': float(msg.offset_x),
            'offsetY': float(msg.offset_y),
            'offsetZ': float(msg.offset_z),
        })

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def spin_in_thread(self) -> threading.Thread:
        """Start rclpy.spin(self) in a daemon thread and return it.

        The thread is started immediately.  Call destroy_node() to stop the
        spin before joining (or let the daemon thread exit on process shutdown).
        """
        t = threading.Thread(
            target=rclpy.spin,
            args=(self,),
            daemon=True,
            name='ros2-spin',
        )
        t.start()
        return t
