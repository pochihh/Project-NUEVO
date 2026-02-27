"""
ROS2 message converters — decoded TLV dicts → ROS2 message objects.

Called from NuevoBridgeNode.publish_decoded(), which is invoked from the
serial reader thread.  All functions are pure (no side effects, no I/O).

Unit conversions applied:
  rawAcc   (mg)       → m/s²  :  × 9.80665e-3
  rawGyro  (0.1 DPS)  → rad/s :  × π/1800
  mag      (µT)       → T     :  × 1e-6
  kinematics x/y (mm) → m     :  ÷ 1000
  kinematics vx/vy (mm/s) → m/s : ÷ 1000

See COMMUNICATION_PROTOCOL.md for TLV payload field definitions.
"""
import math

from builtin_interfaces.msg import Time
from std_msgs.msg import Header
from sensor_msgs.msg import Imu, MagneticField
from nav_msgs.msg import Odometry

from nuevo_msgs.msg import (
    DCMotorStatus,
    DCStatusAll,
    IOStatus,
    MagCalStatus,
    ServoChannelStatus,
    ServoStatusAll,
    StepperStatus,
    StepStatusAll,
    SystemStatus,
    Voltage,
)

# ── Unit conversion constants ─────────────────────────────────────────────────
_MG_TO_MS2    = 9.80665e-3         # millig → m/s²
_DPS01_TO_RAD = math.pi / 1800.0  # 0.1 DPS/LSB → rad/s
_UT_TO_T      = 1.0e-6             # µT → T

# MagCal payload state → MagCalStatus.msg status field
#   payload: 0=idle  1=sampling  2=complete  3=saved  4=error
#   msg:     0=uncalibrated  1=in_progress  2=calibrated
_STATE_TO_STATUS = {0: 0, 1: 1, 2: 2, 3: 2, 4: 0}


# ── Helpers ───────────────────────────────────────────────────────────────────

def _header(stamp: Time, frame_id: str = '') -> Header:
    h = Header()
    h.stamp = stamp
    h.frame_id = frame_id
    return h


# ── Incoming: Arduino → ROS2 ─────────────────────────────────────────────────

def to_imu(data: dict, stamp: Time) -> Imu:
    """'imu' topic dict → sensor_msgs/Imu.

    orientation  : AHRS quaternion (Madgwick filter output)
    linear_accel : body-frame raw accelerometer including gravity (m/s²)
    angular_vel  : body-frame gyroscope (rad/s)
    Covariance matrices are left as zeros (unknown / not characterised).
    """
    msg = Imu()
    msg.header = _header(stamp, 'imu_link')

    msg.orientation.w = float(data['quatW'])
    msg.orientation.x = float(data['quatX'])
    msg.orientation.y = float(data['quatY'])
    msg.orientation.z = float(data['quatZ'])

    msg.linear_acceleration.x = data['rawAccX'] * _MG_TO_MS2
    msg.linear_acceleration.y = data['rawAccY'] * _MG_TO_MS2
    msg.linear_acceleration.z = data['rawAccZ'] * _MG_TO_MS2

    msg.angular_velocity.x = data['rawGyroX'] * _DPS01_TO_RAD
    msg.angular_velocity.y = data['rawGyroY'] * _DPS01_TO_RAD
    msg.angular_velocity.z = data['rawGyroZ'] * _DPS01_TO_RAD

    # Covariance unknown — use all-zero 3×3 matrices (9 elements each)
    msg.orientation_covariance         = [0.0] * 9
    msg.angular_velocity_covariance    = [0.0] * 9
    msg.linear_acceleration_covariance = [0.0] * 9
    return msg


def to_mag(data: dict, stamp: Time) -> MagneticField:
    """'imu' topic dict → sensor_msgs/MagneticField (Tesla)."""
    msg = MagneticField()
    msg.header = _header(stamp, 'imu_link')
    msg.magnetic_field.x = data['magX'] * _UT_TO_T
    msg.magnetic_field.y = data['magY'] * _UT_TO_T
    msg.magnetic_field.z = data['magZ'] * _UT_TO_T
    msg.magnetic_field_covariance = [0.0] * 9
    return msg


def to_odom(data: dict, stamp: Time) -> Odometry:
    """'kinematics' topic dict → nav_msgs/Odometry.

    Converts firmware units (mm, mm/s, rad) to ROS SI units (m, m/s, rad).
    For a differential-drive robot theta is purely a Z-axis rotation, so the
    quaternion is (0, 0, sin(θ/2), cos(θ/2)).
    """
    msg = Odometry()
    msg.header = _header(stamp, 'odom')
    msg.child_frame_id = 'base_link'

    theta = float(data['theta'])
    msg.pose.pose.position.x = data['x'] / 1000.0
    msg.pose.pose.position.y = data['y'] / 1000.0
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = math.sin(theta / 2.0)
    msg.pose.pose.orientation.w = math.cos(theta / 2.0)
    msg.pose.covariance = [0.0] * 36

    msg.twist.twist.linear.x  = data['vx'] / 1000.0
    msg.twist.twist.linear.y  = data['vy'] / 1000.0
    msg.twist.twist.linear.z  = 0.0
    msg.twist.twist.angular.x = 0.0
    msg.twist.twist.angular.y = 0.0
    msg.twist.twist.angular.z = float(data['vTheta'])
    msg.twist.covariance = [0.0] * 36
    return msg


def to_voltage(data: dict, stamp: Time) -> Voltage:
    """'voltage' topic dict → nuevo_msgs/Voltage."""
    msg = Voltage()
    msg.header = _header(stamp)
    msg.battery_mv    = int(data['batteryMv'])
    msg.rail_5v_mv    = int(data['rail5vMv'])
    msg.servo_rail_mv = int(data['servoRailMv'])
    return msg


def to_sys_status(data: dict, stamp: Time) -> SystemStatus:
    """'system_status' topic dict → nuevo_msgs/SystemStatus."""
    msg = SystemStatus()
    msg.header = _header(stamp)
    msg.state              = data['state']
    msg.firmware_major     = data['firmwareMajor']
    msg.firmware_minor     = data['firmwareMinor']
    msg.firmware_patch     = data['firmwarePatch']
    msg.uptime_ms          = data['uptimeMs']
    msg.error_flags        = data['errorFlags']
    msg.attached_sensors   = data['attachedSensors']
    msg.free_sram          = data['freeSram']
    msg.loop_time_avg_us   = data['loopTimeAvgUs']
    msg.loop_time_max_us   = data['loopTimeMaxUs']
    msg.uart_rx_errors     = data['uartRxErrors']
    msg.last_rx_ms         = data['lastRxMs']
    msg.last_cmd_ms        = data['lastCmdMs']
    msg.wheel_diameter_mm  = float(data['wheelDiameterMm'])
    msg.wheel_base_mm      = float(data['wheelBaseMm'])
    msg.motor_dir_mask     = data['motorDirMask']
    msg.neopixel_count     = data['neoPixelCount']
    msg.heartbeat_timeout_ms     = data['heartbeatTimeoutMs']
    msg.limit_switch_mask        = data['limitSwitchMask']
    msg.stepper_home_limit_gpio  = list(data['stepperHomeLimitGpio'])
    return msg


def to_dc_status_all(data: dict, stamp: Time) -> DCStatusAll:
    """'dc_status_all' topic dict → nuevo_msgs/DCStatusAll."""
    msg = DCStatusAll()
    msg.header = _header(stamp)
    motors = data['motors']
    msg.frame_index = motors[0]['frameIndex'] if motors else 0
    for m_data in motors:
        m = DCMotorStatus()
        m.motor_number = m_data['motorNumber']
        m.mode         = m_data['mode']
        m.fault_flags  = m_data['faultFlags']
        m.position     = m_data['position']
        m.velocity     = m_data['velocity']
        m.target_pos   = m_data['targetPos']
        m.target_vel   = m_data['targetVel']
        m.pwm_output   = m_data['pwmOutput']
        m.current_ma   = m_data['currentMa']
        m.pos_kp       = float(m_data['posKp'])
        m.pos_ki       = float(m_data['posKi'])
        m.pos_kd       = float(m_data['posKd'])
        m.vel_kp       = float(m_data['velKp'])
        m.vel_ki       = float(m_data['velKi'])
        m.vel_kd       = float(m_data['velKd'])
        msg.motors[m_data['motorNumber'] - 1] = m
    return msg


def to_step_status_all(data: dict, stamp: Time) -> StepStatusAll:
    """'step_status_all' topic dict → nuevo_msgs/StepStatusAll."""
    msg = StepStatusAll()
    msg.header = _header(stamp)
    for s_data in data['steppers']:
        s = StepperStatus()
        s.stepper_number  = s_data['stepperNumber']
        s.enabled         = s_data['enabled']
        s.motion_state    = s_data['motionState']
        s.limit_hit       = s_data['limitHit']
        s.commanded_count = s_data['commandedCount']
        s.target_count    = s_data['targetCount']
        s.current_speed   = s_data['currentSpeed']
        s.max_speed       = s_data['maxSpeed']
        s.acceleration    = s_data['acceleration']
        msg.steppers[s_data['stepperNumber'] - 1] = s
    return msg


def to_servo_status_all(data: dict, stamp: Time) -> ServoStatusAll:
    """'servo_status_all' topic dict → nuevo_msgs/ServoStatusAll."""
    msg = ServoStatusAll()
    msg.header = _header(stamp)
    msg.pca9685_connected = bool(data['pca9685Connected'])
    msg.pca9685_error     = data['pca9685Error']
    for ch_data in data['channels']:
        ch = ServoChannelStatus()
        ch.channel_number = ch_data['channelNumber']
        ch.enabled        = bool(ch_data['enabled'])
        ch.pulse_us       = ch_data['pulseUs']
        msg.channels[ch_data['channelNumber'] - 1] = ch
    return msg


def to_io_status(data: dict, stamp: Time) -> IOStatus:
    """'io_status' topic dict → nuevo_msgs/IOStatus."""
    msg = IOStatus()
    msg.header = _header(stamp)
    msg.button_mask    = data['buttonMask']
    msg.led_brightness = list(data['ledBrightness'])
    # Flatten [{r, g, b}, ...] → [R0, G0, B0, R1, G1, B1, ...]
    rgb = []
    for pixel in data.get('neoPixels', []):
        rgb.extend([pixel['r'], pixel['g'], pixel['b']])
    msg.neo_pixels_rgb = rgb
    return msg


def to_mag_cal_status(data: dict, stamp: Time) -> MagCalStatus:
    """'mag_cal_status' topic dict → nuevo_msgs/MagCalStatus.

    field_strength is estimated from the half-range of each calibration axis
    (radius of the calibration sphere), which approximates the local field
    magnitude when the calibration is symmetric.
    """
    msg = MagCalStatus()
    msg.header   = _header(stamp)
    msg.status   = _STATE_TO_STATUS.get(data['state'], 0)
    msg.offset_x = float(data['offsetX'])
    msg.offset_y = float(data['offsetY'])
    msg.offset_z = float(data['offsetZ'])
    rx = (data['maxX'] - data['minX']) / 2.0
    ry = (data['maxY'] - data['minY']) / 2.0
    rz = (data['maxZ'] - data['minZ']) / 2.0
    msg.field_strength = math.sqrt(rx * rx + ry * ry + rz * rz)
    return msg
