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

#define FIRMWARE_VERSION        0x00080000  // Version 0.8.0 (Phase 8 - hard real-time ISR scheduler)

// ============================================================================
// HARDWARE CONFIGURATION
// ============================================================================

// DC Motors
#define NUM_DC_MOTORS           4       // Total DC motor channels
#define DC_MOTOR_1              0       // DC motor index 1 (0-3)
#define DC_MOTOR_2              1       // DC motor index 2 (0-3)
#define DC_MOTOR_3              2       // DC motor index 3 (0-3)
#define DC_MOTOR_4              3       // DC motor index 4 (0-3)

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
#define STEPPER_1               0       // Stepper index 1 (0-3)
#define STEPPER_2               1       // Stepper index 2 (0-3)
#define STEPPER_3               2       // Stepper index 3 (0-3)
#define STEPPER_4               3       // Stepper index 4 (0-3)

// All stepper channels are always initialized. Use STEP_ENABLE TLV command at
// runtime to activate specific steppers. Set NUM_STEPPERS to reduce channel count.

// Servos (via PCA9685)
#define NUM_SERVO_CHANNELS      16      // PCA9685 provides 16 channels

#define SERVO_CONTROLLER_ENABLED 1      // Enable PCA9685 servo driver

// ============================================================================
// ENCODER CONFIGURATION
// ============================================================================

#define ENCODER_PPR             1440    // Pulses per revolution (manufacturer spec)
#define ENCODER_MAX_RPM         100     // Maximum expected motor RPM

// Encoder resolution modes
#define ENCODER_2X              2       // 2x counting (phase A only)
#define ENCODER_4X              4       // 4x counting (both phases)

// Per-motor encoder mode (use ENCODER_2X or ENCODER_4X)
#define ENCODER_1_MODE          ENCODER_4X
#define ENCODER_2_MODE          ENCODER_4X
#define ENCODER_3_MODE          ENCODER_4X
#define ENCODER_4_MODE          ENCODER_4X

// Encoder direction inversion (polarity correction)
// Set to 1 to invert encoder count direction (flips positive/negative)
#define ENCODER_1_DIR_INVERTED  1       // 0=normal, 1=inverted
#define ENCODER_2_DIR_INVERTED  1       // 0=normal, 1=inverted
#define ENCODER_3_DIR_INVERTED  1       // 0=normal, 1=inverted
#define ENCODER_4_DIR_INVERTED  1       // 0=normal, 1=inverted

// ============================================================================
// DIFFERENTIAL DRIVE — WHEEL GEOMETRY AND MOTOR ASSIGNMENT
// ============================================================================

// Which DC motor index drives the left and right wheels.
// Must be in range 0 to NUM_DC_MOTORS-1, and must be different from each other.
// Positive encoder ticks for each wheel must mean "wheel moving forward".
// If the count direction is wrong, set the corresponding ENCODER_N_DIR_INVERTED above.
#define ODOM_LEFT_MOTOR             DC_MOTOR_1       // DC motor index for the left drive wheel (0-3)
#define ODOM_RIGHT_MOTOR            DC_MOTOR_2       // DC motor index for the right drive wheel (0-3)

// Default wheel geometry — loaded at boot before SYS_CONFIG arrives from the RPi.
// Set either value to 0.0f to disable odometry until SYS_CONFIG is received.
// Both values can be overridden at runtime via the SYS_CONFIG TLV command.
#define DEFAULT_WHEEL_DIAMETER_MM   65.0f   // Outer diameter of each drive wheel (mm)
#define DEFAULT_WHEEL_BASE_MM       150.0f  // Centre-to-centre axle distance (mm)

// ============================================================================
// TIMING CONFIGURATION
// ============================================================================

// Hard real-time ISR frequencies (Timer1 and Timer4)
#define DC_PID_FREQ_HZ          200     // DC motor PID ISR — Timer1 OVF (5 ms)
#define UART_COMMS_FREQ_HZ      100     // UART comms ISR   — Timer1 OVF every 2nd tick (10 ms)
#define SENSOR_UPDATE_FREQ_HZ   100     // IMU + Fusion ISR — Timer4 OVF /100 (10 ms)
#define SENSOR_LIDAR_FREQ_HZ    50      // Lidar ISR        — every 2nd sensor tick (20 ms)
#define SENSOR_ULTRASONIC_FREQ_HZ 10   // Ultrasonic ISR   — every 10th sensor tick (100 ms)
#define SENSOR_VOLTAGE_FREQ_HZ  10      // Voltage ISR      — every 10th sensor tick (100 ms)

// Soft real-time frequencies (millis-based, loop-driven)
#define USER_IO_FREQ_HZ         20      // LED/button update (50 ms)

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
#define ENABLE_CRC_CHECK        1       // Enable CRC checks on TLV packets. Set to 0 to disable.

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
// BATTERY TYPE CONFIGURATION
// ============================================================================
//
// Set BATTERY_TYPE to the battery you have installed.
// This automatically configures three voltage thresholds:
//
//   VBAT_WARN_V       — warn below this (LED blink, NeoPixel yellow, ERR flag)
//   VBAT_CUTOFF_V     — disable motors below this (hard safety, same as heartbeat loss)
//   VBAT_OVERVOLTAGE_V — warn above this (wrong charger / wrong chemistry)
//
// ── NiMH ────────────────────────────────────────────────────────────────────
#define BATTERY_NIMH_8CELL    1   //  8-cell NiMH,  9.6 V nominal
#define BATTERY_NIMH_10CELL   2   // 10-cell NiMH, 12.0 V nominal  ← DEFAULT
// ── LiPo ────────────────────────────────────────────────────────────────────
#define BATTERY_LIPO_2S       3   //  2S LiPo,  7.4 V nominal
#define BATTERY_LIPO_3S       4   //  3S LiPo, 11.1 V nominal
#define BATTERY_LIPO_4S       5   //  4S LiPo, 14.8 V nominal
#define BATTERY_LIPO_5S       6   //  5S LiPo, 18.5 V nominal
#define BATTERY_LIPO_6S       7   //  6S LiPo, 22.2 V nominal (⚠ near 24 V hw limit)
// ── Custom ───────────────────────────────────────────────────────────────────
#define BATTERY_CUSTOM        99  // Set VBAT_WARN_V/CUTOFF_V/OVERVOLTAGE_V manually

// ── Selection ────────────────────────────────────────────────────────────────
#define BATTERY_TYPE    BATTERY_NIMH_10CELL   // ← change this line to switch battery

// ── Per-type thresholds ───────────────────────────────────────────────────────
// NiMH thresholds use 1.05 V/cell warn, 1.00 V/cell cutoff, 1.55 V/cell over.
// LiPo thresholds use 3.5 V/cell warn,  3.3 V/cell cutoff, 4.3 V/cell over.
#if   BATTERY_TYPE == BATTERY_NIMH_8CELL
  #define VBAT_WARN_V           8.4f   // 8 × 1.05 V
  #define VBAT_CUTOFF_V         8.0f   // 8 × 1.00 V — disable motors (prevents over-discharge)
  #define VBAT_OVERVOLTAGE_V   12.5f   // above max charge (~8 × 1.45 V)
#elif BATTERY_TYPE == BATTERY_NIMH_10CELL
  #define VBAT_WARN_V          10.5f   // 10 × 1.05 V
  #define VBAT_CUTOFF_V        10.0f   // 10 × 1.00 V
  #define VBAT_OVERVOLTAGE_V   15.5f   // above max charge (~10 × 1.45 V)
#elif BATTERY_TYPE == BATTERY_LIPO_2S
  #define VBAT_WARN_V           7.0f   // 2 × 3.5 V
  #define VBAT_CUTOFF_V         6.6f   // 2 × 3.3 V — LiPo damage threshold
  #define VBAT_OVERVOLTAGE_V    8.6f   // above 2 × 4.3 V
#elif BATTERY_TYPE == BATTERY_LIPO_3S
  #define VBAT_WARN_V          10.5f   // 3 × 3.5 V
  #define VBAT_CUTOFF_V         9.9f   // 3 × 3.3 V
  #define VBAT_OVERVOLTAGE_V   12.9f   // above 3 × 4.3 V
#elif BATTERY_TYPE == BATTERY_LIPO_4S
  #define VBAT_WARN_V          14.0f   // 4 × 3.5 V
  #define VBAT_CUTOFF_V        13.2f   // 4 × 3.3 V
  #define VBAT_OVERVOLTAGE_V   17.2f   // above 4 × 4.3 V
#elif BATTERY_TYPE == BATTERY_LIPO_5S
  #define VBAT_WARN_V          17.5f   // 5 × 3.5 V
  #define VBAT_CUTOFF_V        16.5f   // 5 × 3.3 V
  #define VBAT_OVERVOLTAGE_V   21.5f   // above 5 × 4.3 V
#elif BATTERY_TYPE == BATTERY_LIPO_6S
  #define VBAT_WARN_V          21.0f   // 6 × 3.5 V
  #define VBAT_CUTOFF_V        19.8f   // 6 × 3.3 V
  #define VBAT_OVERVOLTAGE_V   24.0f   // hw limit; 6S full charge = 25.2 V (not measurable)
#elif BATTERY_TYPE == BATTERY_CUSTOM
  // Define your own thresholds:
  #define VBAT_WARN_V          10.5f
  #define VBAT_CUTOFF_V        10.0f
  #define VBAT_OVERVOLTAGE_V   16.0f
#else
  #error "Unknown BATTERY_TYPE — see BATTERY_TYPE options in config.h"
#endif

// Backward-compatible alias used by SensorManager::isBatteryLow()
#define VBAT_LOW_THRESHOLD  VBAT_WARN_V

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
#define DEBUG_PINS_ENABLED      0

#if DEBUG_PINS_ENABLED
  #define DEBUG_PIN_ENCODER_ISR   A7    // Toggle on encoder ISR entry/exit
  #define DEBUG_PIN_STEPPER_ISR   A8    // Toggle on stepper timer ISR entry/exit
  #define DEBUG_PIN_SCHEDULER     A9    // Toggle on scheduler tick ISR entry/exit
  #define DEBUG_PIN_PID_LOOP      A10   // Toggle during PID computation
#endif

// ============================================================================
// COMPILE-TIME VALIDATION
// ============================================================================

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

// Odometry motor assignment validation
#if (ODOM_LEFT_MOTOR >= NUM_DC_MOTORS)
  #error "ODOM_LEFT_MOTOR must be less than NUM_DC_MOTORS"
#endif

#if (ODOM_RIGHT_MOTOR >= NUM_DC_MOTORS)
  #error "ODOM_RIGHT_MOTOR must be less than NUM_DC_MOTORS"
#endif

#if (ODOM_LEFT_MOTOR == ODOM_RIGHT_MOTOR)
  #error "ODOM_LEFT_MOTOR and ODOM_RIGHT_MOTOR must be different motors"
#endif

#endif // CONFIG_H
