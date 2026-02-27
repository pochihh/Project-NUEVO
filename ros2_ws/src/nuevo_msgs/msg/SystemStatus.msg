std_msgs/Header header

# Firmware state machine (see COMMUNICATION_PROTOCOL.md §2)
# 0=INIT  1=IDLE  2=RUNNING  3=ERROR  4=ESTOP
uint8 state

uint8  firmware_major
uint8  firmware_minor
uint8  firmware_patch
uint32 uptime_ms

# Error flags bitmask (see SystemErrorFlags in protocol)
# bit0=undervoltage  bit1=overvoltage  bit2=encoder_fail
# bit3=i2c_error     bit4=imu_error    bit5=liveness_lost  bit6=loop_overrun
uint8  error_flags

# Attached sensor bitmask: bit0=IMU  bit1=lidar  bit2=ultrasonic
uint8  attached_sensors

uint16 free_sram           # Free Arduino SRAM (bytes)
uint16 loop_time_avg_us    # Average control loop time (µs)
uint16 loop_time_max_us    # Peak control loop time since last report (µs)
uint16 uart_rx_errors      # Cumulative CRC/framing error count
uint32 last_rx_ms          # ms since last received TLV from RPi
uint32 last_cmd_ms         # ms since last received non-heartbeat command

# Configuration echo (current values set via SYS_CONFIG)
float32 wheel_diameter_mm
float32 wheel_base_mm
uint8   motor_dir_mask         # bit N = direction inversion for motor N
uint8   neopixel_count
uint16  heartbeat_timeout_ms

# Limit switch configuration
uint16 limit_switch_mask           # bit N = GPIO N is a limit switch
uint8[4] stepper_home_limit_gpio   # GPIO index for each stepper's home limit (0xFF = none)
