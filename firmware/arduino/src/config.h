/**
 * @file config.h
 * @brief Central configuration for Arduino Mega 2560 firmware
 *
 * This file contains all compile-time parameters for hardware configuration,
 * timing, communication, and debug settings.
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// FIRMWARE VERSION
// ============================================================================

#define FIRMWARE_VERSION        0x00070000  // Version 0.7.0 (Phase 7 - sensor refactor)

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// DC Motors
#define NUM_DC_MOTORS           4       // Total DC motor channels

// All DC motor channels are always initialized. Use DC_ENABLE TLV command at
// runtime to activate specific motors. Set NUM_DC_MOTORS to reduce channel count.

// DC Motor direction inversion (H-bridge wiring correction)
// Set to 1 to invert motor direction (swaps forward/reverse)
#define DC_MOTOR_1_DIR_INVERTED 1       // 0=normal, 1=inverted
#define DC_MOTOR_2_DIR_INVERTED 1       // 0=normal, 1=inverted
#define DC_MOTOR_3_DIR_INVERTED 1       // 0=normal, 1=inverted
#define DC_MOTOR_4_DIR_INVERTED 1       // 0=normal, 1=inverted

// Stepper Motors
#define NUM_STEPPERS            4       // Total stepper channels

// All stepper channels are always initialized. Use STEP_ENABLE TLV command at
// runtime to activate specific steppers. Set NUM_STEPPERS to reduce channel count.

// Servos (via PCA9685)
#define NUM_SERVO_CHANNELS      16      // PCA9685 provides 16 channels

#define SERVO_CONTROLLER_ENABLED 0      // Enable PCA9685 servo driver

// ============================================================================
// ENCODER CONFIGURATION
// ============================================================================

#define ENCODER_PPR             1440    // Pulses per revolution (manufacturer spec)
#define ENCODER_MAX_RPM         100     // Maximum expected motor RPM

// Encoder resolution modes
#define ENCODER_2X              2       // 2x counting (phase A only)
#define ENCODER_4X              4       // 4x counting (both phases)

// Per-motor encoder mode (use ENCODER_2X or ENCODER_4X)
#define ENCODER_1_MODE          ENCODER_2X
#define ENCODER_2_MODE          ENCODER_2X
#define ENCODER_3_MODE          ENCODER_2X
#define ENCODER_4_MODE          ENCODER_2X

// Encoder direction inversion (polarity correction)
// Set to 1 to invert encoder count direction (flips positive/negative)
#define ENCODER_1_DIR_INVERTED  1       // 0=normal, 1=inverted
#define ENCODER_2_DIR_INVERTED  1       // 0=normal, 1=inverted
#define ENCODER_3_DIR_INVERTED  1       // 0=normal, 1=inverted
#define ENCODER_4_DIR_INVERTED  1       // 0=normal, 1=inverted

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================

// Scheduler base tick (DO NOT CHANGE - Timer1 configured for 1kHz)
#define SCHEDULER_TICK_FREQ_HZ  1000    // 1ms period

// Task update frequencies
#define DC_PID_FREQ_HZ          200     // DC motor PID loop (5ms period)
#define UART_COMMS_FREQ_HZ      100     // UART communication (10ms period)
#define SENSOR_UPDATE_FREQ_HZ   100     // IMU sensor reading (10ms period)
#define SENSOR_LIDAR_FREQ_HZ    50      // Lidar reading (20ms period)
#define SENSOR_ULTRASONIC_FREQ_HZ 15   // Ultrasonic reading (67ms period)
#define SENSOR_VOLTAGE_FREQ_HZ  10      // Voltage monitoring (100ms period)
#define USER_IO_FREQ_HZ         20      // LED/button update (50ms period)

// Stepper pulse generation (Timer3)
#define STEPPER_TIMER_FREQ_HZ   10000   // 10kHz interrupt rate (100µs period)
#define STEPPER_MAX_RATE_SPS    5000    // Maximum steps per second per motor

// Safety timeout
#define HEARTBEAT_TIMEOUT_MS    500      // Disable motors if no heartbeat

// ============================================================================
// COMMUNICATION SETTINGS
// ============================================================================

// UART to Raspberry Pi (Serial2)
#define RPI_BAUD_RATE           1000000 // 1 Mbps UART (full duplex, ~48% TX utilization at 100Hz)
#define RPI_SERIAL              Serial2 // Hardware serial port

// Debug serial (Serial0 - USB)
#define DEBUG_BAUD_RATE         115200
#define DEBUG_SERIAL            Serial

// Device identification
#define DEVICE_ID               0x01    // Arduino device ID for TLV protocol
#define ENABLE_CRC_CHECK      1       // Enable CRC checks on TLV packets

// ============================================================================
// VELOCITY ESTIMATION CONFIGURATION
// ============================================================================

// Velocity estimator settings
#define VELOCITY_FILTER_SIZE    4       // Moving average filter size (2-8 samples)
#define VELOCITY_ZERO_TIMEOUT   50      // Zero velocity timeout (milliseconds)

// ============================================================================
// PID CONTROLLER DEFAULTS
// ============================================================================

// Default PID gains for DC motors (runtime configurable via TLV)
// Position PID (outer loop)
#define DEFAULT_POS_KP          1.8f
#define DEFAULT_POS_KI          0.0f
#define DEFAULT_POS_KD          1.0f

// Velocity PID (middle loop)
#define DEFAULT_VEL_KP          0.5f
#define DEFAULT_VEL_KI          0.1f
#define DEFAULT_VEL_KD          0.0f

// Torque PID (inner loop - optional, requires current sensing)
#define DEFAULT_TRQ_KP          0.2f
#define DEFAULT_TRQ_KI          0.05f
#define DEFAULT_TRQ_KD          0.0f

// PID output limits
#define PID_OUTPUT_MIN          -255    // Minimum PWM value
#define PID_OUTPUT_MAX          255     // Maximum PWM value

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================

// IMU (ICM-20948 via SparkFun library + Fusion AHRS)
#define IMU_ENABLED             1
// AD0_VAL: 0 = I2C addr 0x68 (AD0 pin LOW), 1 = I2C addr 0x69 (AD0 pin HIGH)
#define IMU_AD0_VAL             1       // SparkFun breakout default: AD0 high = 0x69

// Fusion AHRS settings (Madgwick-based sensor fusion)
// gain: 0.5 default. Higher = faster convergence, more susceptible to disturbance.
#define FUSION_GAIN             0.5f
// Rejection thresholds: measurements outside these bounds are rejected during init
#define FUSION_ACCEL_REJECTION  10.0f   // g (acceleration rejection threshold)
#define FUSION_MAG_REJECTION    10.0f   // µT (magnetic rejection threshold)
// Recovery trigger period: seconds before algorithm exits recovery mode
#define FUSION_RECOVERY_PERIOD  5       // seconds

// Lidar (Garmin LIDAR-Lite v4, via I2C)
#define LIDAR_COUNT             0       // Number of attached lidar sensors (0 to disable)
// Up to 4 lidar sensors at different I2C addresses (change with address jumper)
#define LIDAR_0_I2C_ADDR        0x62    // Default LIDAR-Lite v4 address
#define LIDAR_1_I2C_ADDR        0x63
#define LIDAR_2_I2C_ADDR        0x64
#define LIDAR_3_I2C_ADDR        0x65

// Ultrasonic (SparkFun Qwiic HC-SR04, via I2C)
#define ULTRASONIC_COUNT        0       // Number of attached ultrasonic sensors (0 to disable)
// Up to 4 ultrasonic sensors at different I2C addresses (configurable via Example 2)
#define ULTRASONIC_0_I2C_ADDR   0x2F    // Default Qwiic Ultrasonic address
#define ULTRASONIC_1_I2C_ADDR   0x2E
#define ULTRASONIC_2_I2C_ADDR   0x2D
#define ULTRASONIC_3_I2C_ADDR   0x2C

// ============================================================================
// MAGNETOMETER CALIBRATION
// ============================================================================

// Minimum samples required before calibration can be saved.
// The robot must be rotated through all orientations during this time.
// At 100 Hz IMU rate, 50 samples = 0.5 s minimum; collect for 10-20 s in practice.
#define MAG_CAL_MIN_SAMPLES     50

// EEPROM layout and addressing are managed by PersistentStorage.
// See firmware/arduino/src/modules/PersistentStorage.h for layout details.

// Voltage monitoring
#define VBAT_ENABLED            1       // Battery voltage monitoring
#define V5_ENABLED              1       // 5V rail monitoring
#define VSERVO_ENABLED          1       // Servo rail monitoring

// ============================================================================
// VOLTAGE DIVIDER RATIOS (ADC INPUT SCALING)
// ============================================================================

// Battery voltage divider (VBAT_SENSE on A0)
// Hardware: 50kΩ + 10kΩ divider = 1:6 ratio
#define VBAT_DIVIDER_R1         50000.0f  // Upper resistor (Ω)
#define VBAT_DIVIDER_R2         10000.0f  // Lower resistor (Ω)
#define VBAT_DIVIDER_RATIO      ((VBAT_DIVIDER_R1 + VBAT_DIVIDER_R2) / VBAT_DIVIDER_R2)

// 5V rail divider (V5_SENSE on A1)
// Hardware: 1:2 ratio
#define V5_DIVIDER_RATIO        2.0f

// Servo rail divider (VSERVO_SENSE on A2)
// Hardware: 1:3 ratio
#define VSERVO_DIVIDER_RATIO    3.0f

// ADC reference voltage (Arduino Mega 2560)
#define ADC_VREF                5.0f    // 5V reference
#define ADC_RESOLUTION          1024    // 10-bit ADC

// Low battery threshold (volts)
#define VBAT_LOW_THRESHOLD      10.5f   // Warn below this voltage

// ============================================================================
// DC MOTOR CURRENT SENSING
// ============================================================================

// Current sensor configuration
// Hardware: CT Output voltage (V) = Current (A) × 0.155
// Scaling: 0.155 V/A = 155 mV/A = 0.155 mV/mA
// Conversion: Current (mA) = Voltage (V) × (1000 / 0.155) = Voltage × 6451.6
#define CURRENT_SENSE_MA_PER_VOLT   6451.6f  // milliamps per volt (1/0.155 × 1000)

// ============================================================================
// NEOPIXEL CONFIGURATION
// ============================================================================

#define NEOPIXEL_COUNT          1       // Number of WS2812B LEDs
#define NEOPIXEL_BRIGHTNESS     64      // Default brightness (0-255)

// System status colors (first pixel reserved for status indication)
#define STATUS_COLOR_OK         0x00FF00  // Green - normal operation
#define STATUS_COLOR_LOW_BAT    0xFF0000  // Red - low battery
#define STATUS_COLOR_ERROR      0xFF8800  // Orange - error state
#define STATUS_COLOR_DISABLED   0x000000  // Off - motors disabled

// ============================================================================
// LIMIT SWITCH CONFIGURATION
// ============================================================================

// Stepper limit switch assignments (maps to limit switch pins)
// Uncomment and update these definitions if limit switches are connected for homing.
// If no limit switches are used, leave these undefined
// Homing is disabled if no limit switch pins are defined.

// #define PIN_ST1_LIMIT           PIN_LIM1  // Stepper 1 limit (40)
// #define PIN_ST2_LIMIT           PIN_LIM2  // Stepper 2 limit (41)
// #define PIN_ST3_LIMIT           PIN_LIM3  // Stepper 3 limit (48)
// #define PIN_ST4_LIMIT           PIN_LIM4  // Stepper 4 limit (49)

// Limit switch active state
#define LIMIT_ACTIVE_LOW        1       // 1 = active low, 0 = active high

// ============================================================================
// DEBUG CONFIGURATION
// ============================================================================

// Uncomment to enable debug features (increases code size and reduces performance)
// #define DEBUG_MOTOR_PID         // Print PID debug info to Serial
// #define DEBUG_ENCODER           // Print encoder counts to Serial
// #define DEBUG_TLV_PACKETS       // Print TLV packet info to Serial
// #define DEBUG_VELOCITY          // Print velocity estimation debug info
// #define DEBUG_SCHEDULER         // Print scheduler task execution info

// Debug pins for oscilloscope timing measurement
// These pins toggle on entry/exit of critical sections for timing analysis
#define DEBUG_PINS_ENABLED      1

#if DEBUG_PINS_ENABLED
  #define DEBUG_PIN_ENCODER_ISR   A7    // Toggle on encoder ISR entry/exit
  #define DEBUG_PIN_STEPPER_ISR   A8    // Toggle on stepper timer ISR entry/exit
  #define DEBUG_PIN_SCHEDULER     A9    // Toggle on scheduler tick ISR entry/exit
  #define DEBUG_PIN_PID_LOOP      A10   // Toggle during PID computation
#endif

// ============================================================================
// COMPILE-TIME VALIDATION
// ============================================================================

// Ensure timer frequencies are valid
#if (SCHEDULER_TICK_FREQ_HZ != 1000)
  #error "SCHEDULER_TICK_FREQ_HZ must be 1000 Hz (Timer1 is hardcoded for 1ms)"
#endif

#if (DC_PID_FREQ_HZ > SCHEDULER_TICK_FREQ_HZ)
  #error "DC_PID_FREQ_HZ cannot exceed SCHEDULER_TICK_FREQ_HZ"
#endif

#if (UART_COMMS_FREQ_HZ > SCHEDULER_TICK_FREQ_HZ)
  #error "UART_COMMS_FREQ_HZ cannot exceed SCHEDULER_TICK_FREQ_HZ"
#endif

// Ensure encoder modes are valid
#if (ENCODER_1_MODE != ENCODER_2X && ENCODER_1_MODE != ENCODER_4X)
  #error "ENCODER_1_MODE must be ENCODER_2X or ENCODER_4X"
#endif

#if (ENCODER_2_MODE != ENCODER_2X && ENCODER_2_MODE != ENCODER_4X)
  #error "ENCODER_2_MODE must be ENCODER_2X or ENCODER_4X"
#endif

#if (ENCODER_3_MODE != ENCODER_2X && ENCODER_3_MODE != ENCODER_4X)
  #error "ENCODER_3_MODE must be ENCODER_2X or ENCODER_4X"
#endif

#if (ENCODER_4_MODE != ENCODER_2X && ENCODER_4_MODE != ENCODER_4X)
  #error "ENCODER_4_MODE must be ENCODER_2X or ENCODER_4X"
#endif

#endif // CONFIG_H
