/**
 * @file arduino.ino
 * @brief Main entry point for Arduino Mega 2560 firmware
 *
 * Educational robotics platform firmware for MAE 162 course.
 * Provides real-time motor control, sensor integration, and
 * communication with Raspberry Pi 5 via TLV protocol over UART.
 *
 * Architecture:
 * - Cooperative scheduler using Timer1 @ 1kHz
 * - Hardware interrupts for encoders (INT0-INT5)
 * - Timer3 @ 10kHz for stepper pulse generation
 * - Priority-based task execution in main loop
 *
 * Initialization Order (setup):
 * 1. Debug serial (Serial0)
 * 2. Scheduler (Timer1)
 * 3. MessageCenter (Serial2 + TLV codec)
 * 4. SensorManager (I2C, ADC)
 * 5. UserIO (GPIO, NeoPixel)
 * 6. ServoController (PCA9685 via I2C)
 * 7. StepperManager (Timer3)
 * 8. DC Motors (PWM pins, encoder counters)
 * 9. Attach encoder ISRs
 * 10. Register scheduler tasks
 *
 * Main Loop:
 * - Scheduler::tick() executes highest-priority ready task
 * - Task priorities: 0=DC PID, 1=UART Comms, 2=Sensors, 3=User I/O
 */

// ============================================================================
// INCLUDES
// ============================================================================

#include "src/config.h"
#include "src/pins.h"
#include "src/Scheduler.h"

// Phase 2+ includes
#include "src/messages/MessageCenter.h"
#include "src/messages/TLV_Payloads.h"

// Phase 3+ includes (commented out until implemented)
// #include "src/modules/EncoderCounter.h"
// #include "src/modules/VelocityEstimator.h"
// #include "src/drivers/DCMotor.h"

// Phase 4+ includes (commented out until implemented)
// #include "src/modules/StepperManager.h"
// #include "src/drivers/StepperMotor.h"
// #include "src/drivers/ServoController.h"

// Phase 5+ includes (commented out until implemented)
// #include "src/modules/SensorManager.h"
// #include "src/modules/UserIO.h"
// #include "src/drivers/IMUDriver.h"
// #include "src/drivers/NeoPixelDriver.h"

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

// Phase 1: Scheduler only
// Future phases will add motor, sensor, and communication objects here

// ============================================================================
// TASK CALLBACKS
// ============================================================================

/**
 * @brief DC Motor PID control loop (200 Hz, Priority 0)
 *
 * Updates all enabled DC motor PID controllers.
 * Runs every 5ms (200Hz) with highest priority for smooth control.
 */
void taskDCMotorPID() {
  // Phase 3: DC motor PID update will be implemented here
  // For now, this is a placeholder

#ifdef DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_PID_LOOP, HIGH);
#endif

  // Future implementation:
  // for (uint8_t i = 0; i < NUM_DC_MOTORS; i++) {
  //   if (DC_MOTOR_X_ENABLED) {
  //     dcMotors[i].update();
  //   }
  // }

#ifdef DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_PID_LOOP, LOW);
#endif
}

/**
 * @brief UART communication and safety timeout (100 Hz, Priority 1)
 *
 * Processes incoming TLV messages from Raspberry Pi.
 * Checks heartbeat timeout and disables motors if expired.
 * Runs every 10ms (100Hz).
 */
void taskUARTComms() {
  // Process incoming/outgoing TLV messages
  MessageCenter::processingTick();

  // Safety timeout check
  if (!MessageCenter::isHeartbeatValid()) {
    // TODO Phase 3: Disable all motors when heartbeat timeout occurs
    // for (uint8_t i = 0; i < NUM_DC_MOTORS; i++) {
    //   dcMotors[i].disable();
    // }
  }
}

/**
 * @brief Sensor reading and data transmission (50 Hz, Priority 2)
 *
 * Reads all enabled sensors (IMU, voltage, encoders, etc.).
 * Queues outgoing sensor data packets to Raspberry Pi.
 * Runs every 20ms (50Hz).
 */
void taskSensorRead() {
  // Phase 5: Sensor reading will be implemented here
  // For now, this is a placeholder

  // Future implementation:
  // SensorManager::update();
  //
  // // Queue outgoing sensor data
  // MessageCenter::sendEncoderData();
  // MessageCenter::sendVoltageData();
  // if (IMU_ENABLED) {
  //   MessageCenter::sendIMUData();
  // }
}

/**
 * @brief User I/O updates (20 Hz, Priority 3)
 *
 * Updates LED animations (blink, breathe patterns).
 * Reads button states and limit switches.
 * Updates NeoPixel system status indicator.
 * Runs every 50ms (20Hz).
 */
void taskUserIO() {
  // Phase 5: User I/O will be implemented here
  // For now, this is a placeholder

  // Future implementation:
  // UserIO::update();
}

// ============================================================================
// ARDUINO SETUP
// ============================================================================

void setup() {
  // ------------------------------------------------------------------------
  // 1. Initialize Debug Serial (USB)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.begin(DEBUG_BAUD_RATE);
  while (!DEBUG_SERIAL && millis() < 2000) {
    ; // Wait for serial port to connect (up to 2 seconds)
  }

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println(F("  MAE 162 Robot Firmware v0.1.0"));
  DEBUG_SERIAL.println(F("  Phase 1: Scheduler Foundation"));
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println();

  // ------------------------------------------------------------------------
  // 2. Initialize Scheduler (Timer1 @ 1kHz)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing scheduler..."));
  Scheduler::init();

#ifdef DEBUG_PINS_ENABLED
  // Configure debug pins for oscilloscope timing measurement
  pinMode(DEBUG_PIN_ENCODER_ISR, OUTPUT);
  pinMode(DEBUG_PIN_STEPPER_ISR, OUTPUT);
  pinMode(DEBUG_PIN_PID_LOOP, OUTPUT);
  digitalWrite(DEBUG_PIN_ENCODER_ISR, LOW);
  digitalWrite(DEBUG_PIN_STEPPER_ISR, LOW);
  digitalWrite(DEBUG_PIN_PID_LOOP, LOW);
  DEBUG_SERIAL.println(F("[Setup] Debug pins configured (A7-A10)"));
#endif

  // ------------------------------------------------------------------------
  // 3. Initialize MessageCenter (UART + TLV)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing UART communication..."));
  MessageCenter::init();

  // ------------------------------------------------------------------------
  // Phase 5: Initialize SensorManager
  // ------------------------------------------------------------------------
  // DEBUG_SERIAL.println(F("[Setup] Initializing sensors..."));
  // SensorManager::init();

  // ------------------------------------------------------------------------
  // Phase 5: Initialize UserIO (LEDs, Buttons, NeoPixels)
  // ------------------------------------------------------------------------
  // DEBUG_SERIAL.println(F("[Setup] Initializing user I/O..."));
  // UserIO::init();

  // ------------------------------------------------------------------------
  // Phase 4: Initialize ServoController (PCA9685)
  // ------------------------------------------------------------------------
  // if (SERVO_CONTROLLER_ENABLED) {
  //   DEBUG_SERIAL.println(F("[Setup] Initializing servo controller..."));
  //   ServoController::init();
  // }

  // ------------------------------------------------------------------------
  // Phase 4: Initialize StepperManager (Timer3 @ 10kHz)
  // ------------------------------------------------------------------------
  // DEBUG_SERIAL.println(F("[Setup] Initializing stepper motors..."));
  // StepperManager::init();

  // ------------------------------------------------------------------------
  // Phase 3: Initialize DC Motors and Encoders
  // ------------------------------------------------------------------------
  // DEBUG_SERIAL.println(F("[Setup] Initializing DC motors..."));
  // for (uint8_t i = 0; i < NUM_DC_MOTORS; i++) {
  //   if (DC_MOTOR_X_ENABLED) {
  //     dcMotors[i].init();
  //   }
  // }

  // ------------------------------------------------------------------------
  // Phase 3: Attach Encoder ISRs
  // ------------------------------------------------------------------------
  // DEBUG_SERIAL.println(F("[Setup] Attaching encoder interrupts..."));
  // attachInterrupt(digitalPinToInterrupt(PIN_M1_ENC_A), encoderISR_M1, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(PIN_M2_ENC_A), encoderISR_M2, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(PIN_M3_ENC_A), encoderISR_M3, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(PIN_M4_ENC_A), encoderISR_M4, CHANGE);

  // ------------------------------------------------------------------------
  // 3. Register Scheduler Tasks
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Registering scheduler tasks..."));

  // Task registration:
  //   Scheduler::registerTask(callback, periodMs, priority)
  //
  // Priority levels (lower number = higher priority):
  //   0 - DC Motor PID (200 Hz, 5ms period)
  //   1 - UART Communication (100 Hz, 10ms period)
  //   2 - Sensor Reading (50 Hz, 20ms period)
  //   3 - User I/O (20 Hz, 50ms period)

  int8_t taskId;

  taskId = Scheduler::registerTask(taskDCMotorPID, 1000 / DC_PID_FREQ_HZ, 0);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - DC Motor PID: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / DC_PID_FREQ_HZ);
    DEBUG_SERIAL.println(F("ms (Priority 0)"));
  }

  taskId = Scheduler::registerTask(taskUARTComms, 1000 / UART_COMMS_FREQ_HZ, 1);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - UART Comms: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / UART_COMMS_FREQ_HZ);
    DEBUG_SERIAL.println(F("ms (Priority 1)"));
  }

  taskId = Scheduler::registerTask(taskSensorRead, 1000 / SENSOR_UPDATE_FREQ_HZ, 2);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - Sensor Read: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / SENSOR_UPDATE_FREQ_HZ);
    DEBUG_SERIAL.println(F("ms (Priority 2)"));
  }

  taskId = Scheduler::registerTask(taskUserIO, 1000 / USER_IO_FREQ_HZ, 3);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - User I/O: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / USER_IO_FREQ_HZ);
    DEBUG_SERIAL.println(F("ms (Priority 3)"));
  }

  // ------------------------------------------------------------------------
  // Setup Complete
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("[Setup] Initialization complete!"));
  DEBUG_SERIAL.println(F("[Setup] Entering main loop..."));
  DEBUG_SERIAL.println();
}

// ============================================================================
// ARDUINO MAIN LOOP
// ============================================================================

void loop() {
  // Execute highest-priority ready task
  Scheduler::tick();

  // Future: Add any non-time-critical housekeeping here
  // (e.g., debug output, watchdog reset, etc.)
}

// ============================================================================
// ENCODER ISR TRAMPOLINES (Phase 3)
// ============================================================================

// These are minimal ISR wrappers that forward calls to encoder objects.
// Encoder ISRs must be global functions (not class methods) to use with
// attachInterrupt().

// Uncomment when Phase 3 encoders are implemented:

// void encoderISR_M1() {
//   #ifdef DEBUG_PINS_ENABLED
//     digitalWrite(DEBUG_PIN_ENCODER_ISR, HIGH);
//   #endif
//
//   dcMotors[0].onEncoderInterrupt();
//
//   #ifdef DEBUG_PINS_ENABLED
//     digitalWrite(DEBUG_PIN_ENCODER_ISR, LOW);
//   #endif
// }
//
// void encoderISR_M2() {
//   dcMotors[1].onEncoderInterrupt();
// }
//
// void encoderISR_M3() {
//   dcMotors[2].onEncoderInterrupt();
// }
//
// void encoderISR_M4() {
//   dcMotors[3].onEncoderInterrupt();
// }
