cmake_minimum_required(VERSION 3.8)
project(nuevo_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  # Telemetry (Arduino → RPi)
  "msg/Voltage.msg"
  "msg/SystemStatus.msg"
  "msg/DCMotorStatus.msg"
  "msg/DCStatusAll.msg"
  "msg/StepperStatus.msg"
  "msg/StepStatusAll.msg"
  "msg/ServoChannelStatus.msg"
  "msg/ServoStatusAll.msg"
  "msg/IOStatus.msg"
  "msg/MagCalStatus.msg"
  # Commands (RPi → Arduino)
  "msg/MotorVelocities.msg"
  "msg/StepCommand.msg"
  "msg/ServoCommand.msg"
  "msg/SysCommand.msg"
  "msg/MagCalCmd.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
