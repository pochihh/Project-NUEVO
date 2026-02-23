/**
 * @file arduino.ino
 * @brief Main firmware for Arduino Mega 2560 educational robotics platform
 * @version 0.8.0
 *
 * Educational robotics platform firmware for MAE 162 course.
 * Provides real-time motor control, sensor integration, and
 * communication with Raspberry Pi 5 via TLV protocol over UART.
 *
 * Two-tier scheduling architecture:
 *
 *   Hard real-time (ISR-driven — unaffected by loop() blocking):
 *     TIMER1_OVF_vect  200 Hz  DC motor PID always
 *                      100 Hz  UART comms + heartbeat safety (every 2nd tick)
 *     TIMER3_OVF_vect  10 kHz  Stepper pulse generation (StepperManager)
 *     TIMER4_OVF_vect  100 Hz  IMU/Lidar/Voltage dispatch (SensorManager)
 *
 *   Soft real-time (millis-based, runs in loop()):
 *     taskUserIO        20 Hz  LED animations, buttons, NeoPixel status
 *
 * Initialization Order (setup):
 *  1. Debug serial (Serial0)
 *  2. Scheduler (millis-based soft scheduler)
 *  3. MessageCenter (Serial2 + TLV codec)
 *  4. SensorManager (I2C, ADC)
 *  5. UserIO (GPIO, NeoPixel)
 *  6. ServoController (PCA9685 via I2C)
 *  7. StepperManager (Timer3 — also starts stepper ISR)
 *  8. DC Motors (PWM pins, encoder counters)
 *  9. Attach encoder ISRs
 * 10. Register soft-scheduler tasks
 * 11. ISRScheduler::init() — LAST: starts Timer1 and Timer4 ISRs
 *
 * Main Loop:
 * - Scheduler::tick() executes highest-priority ready soft task
 */

// ============================================================================
// INCLUDES
// ============================================================================

// Core configuration
#include "src/config.h"
#include "src/pins.h"
#include "src/Scheduler.h"
#include "src/ISRScheduler.h"
#include "src/SystemManager.h"

// Communication
#include "src/modules/MessageCenter.h"
#include "src/modules/SafetyManager.h"
#include "src/messages/TLV_Payloads.h"

// DC motor control
#include "src/modules/EncoderCounter.h"
#include "src/modules/VelocityEstimator.h"
#include "src/drivers/DCMotor.h"

// Stepper and servo control
#include "src/modules/StepperManager.h"
#include "src/drivers/StepperMotor.h"
#include "src/drivers/ServoController.h"

// Sensors and user I/O
#include "src/modules/SensorManager.h"
#include "src/modules/UserIO.h"
#include "src/drivers/IMUDriver.h"
#include "src/drivers/NeoPixelDriver.h"

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

// Encoder instances (2x or 4x mode per config.h)
#if ENCODER_1_MODE == ENCODER_2X
EncoderCounter2x encoder1;
#else
EncoderCounter4x encoder1;
#endif

#if ENCODER_2_MODE == ENCODER_2X
EncoderCounter2x encoder2;
#else
EncoderCounter4x encoder2;
#endif

#if ENCODER_3_MODE == ENCODER_2X
EncoderCounter2x encoder3;
#else
EncoderCounter4x encoder3;
#endif

#if ENCODER_4_MODE == ENCODER_2X
EncoderCounter2x encoder4;
#else
EncoderCounter4x encoder4;
#endif

// Velocity estimators (edge-time algorithm)
EdgeTimeVelocityEstimator velocityEst1;
EdgeTimeVelocityEstimator velocityEst2;
EdgeTimeVelocityEstimator velocityEst3;
EdgeTimeVelocityEstimator velocityEst4;

// Motor controller arrays
DCMotor dcMotors[NUM_DC_MOTORS];
StepperMotor steppers[NUM_STEPPERS];

// ============================================================================
// HARD REAL-TIME ISR — TIMER1_OVF_vect (200 Hz / 100 Hz)
// ============================================================================

/**
 * @brief Timer1 Overflow ISR — DC Motor PID (200 Hz) + UART (100 Hz)
 *
 * Fires at 200 Hz (5 ms period).
 *   - Every tick:       DC motor PID update for all enabled motors.
 *   - Every other tick: UART comms (process incoming, send telemetry) and
 *                       heartbeat safety check (disables actuators if expired).
 *
 * Timer1 is configured in Fast PWM mode 14 by ISRScheduler::init().
 * OC1A (pin 11) simultaneously drives LED_RED hardware PWM in Rev A.
 */
ISR(TIMER1_OVF_vect) {
#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_PID_LOOP, HIGH);
#endif

  // ── 200 Hz: DC motor PID ────────────────────────────────────────────────
  for (uint8_t i = 0; i < NUM_DC_MOTORS; i++) {
    dcMotors[i].update();
  }

  // ── 200 Hz: Green LED heartbeat (1 Hz toggle = 2 s blink period) ────────
  // Toggles every 200 PID ticks, giving a visible "scheduler alive" indicator.
  static uint8_t greenCounter = 0;
  static bool    greenState   = false;
  if (++greenCounter >= 200) {
    greenCounter = 0;
    greenState   = !greenState;
    digitalWrite(PIN_LED_GREEN, greenState ? HIGH : LOW);
  }

  // ── 100 Hz: buttons + UART comms + safety check ─────────────────────────
  static bool uartTick = false;
  uartTick = !uartTick;
  if (uartTick) {
    // Button reads at 100 Hz — fast GPIO only, no millis/delay
    UserIO::readButtons();

    // Process incoming TLV messages and send telemetry
    MessageCenter::processIncoming();
    MessageCenter::sendTelemetry();

    // Centralised hard RT safety check (heartbeat + battery faults → ERROR)
    SafetyManager::check();
  }

#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_PID_LOOP, LOW);
#endif
}

// ============================================================================
// SOFT TASK — taskUserIO (20 Hz, millis-based)
// ============================================================================

/**
 * @brief User I/O updates (20 Hz, soft scheduler)
 *
 * Runs LED animations (blink, breathe, RED battery warnings).
 * Reads limit switches.
 * Drives NeoPixel state animations via UserIO::updateNeoPixelAnimation()
 * (INIT=yellow, IDLE=breathing emerald, RUNNING=rainbow, ERROR=flashing red).
 * Button reads are handled at 100 Hz inside TIMER1_OVF_vect via readButtons().
 */
void taskUserIO() {
  // LED animations + limit switches + NeoPixel state animation
  UserIO::update();

  // RED LED battery warnings (independent of NeoPixel system-state animation)
  if (SensorManager::isBatteryOvervoltage()) {
    UserIO::setLED(LED_RED, LED_BLINK, 255, 250);   // Fast blink — wrong battery/charger
  } else if (SensorManager::isBatteryCritical()) {
    UserIO::setLED(LED_RED, LED_BLINK, 255, 250);   // Fast blink — shutdown imminent
  } else if (SensorManager::isBatteryLow()) {
    UserIO::setLED(LED_RED, LED_BLINK, 255, 1000);  // Slow blink — low warning
  }
}

// ============================================================================
// ENCODER ISR TRAMPOLINES
// ============================================================================

/**
 * @brief Encoder ISR wrappers
 *
 * These are minimal ISR wrappers that forward calls to encoder objects.
 * Encoder ISRs must be global functions (not class methods) to use with
 * attachInterrupt().
 */

void encoderISR_M1() {
#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_ENCODER_ISR, HIGH);
#endif

  encoder1.onInterruptA();

#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_ENCODER_ISR, LOW);
#endif
}

void encoderISR_M2() {
  encoder2.onInterruptA();
}

void encoderISR_M3() {
  encoder3.onInterruptA();
}

void encoderISR_M4() {
  encoder4.onInterruptA();
}

// ============================================================================
// ARDUINO SETUP
// ============================================================================

void setup() {
  // ------------------------------------------------------------------------
  // Initialize SystemManager (sets INIT state before any module starts)
  // ------------------------------------------------------------------------
  SystemManager::init();

  // ------------------------------------------------------------------------
  // Initialize Debug Serial (USB)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.begin(DEBUG_BAUD_RATE);
  while (!DEBUG_SERIAL && millis() < 2000) {
    ; // Wait for serial port to connect (up to 2 seconds)
  }

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println(F("  MAE 162 Robot Firmware v0.8.0"));
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println();

  // ------------------------------------------------------------------------
  // Initialize Soft Scheduler (millis-based)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing soft scheduler..."));
  Scheduler::init();

#if DEBUG_PINS_ENABLED
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
  // Initialize Communication (UART + TLV Protocol)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing UART communication..."));
  MessageCenter::init();

  // ------------------------------------------------------------------------
  // Initialize Sensors (I2C, ADC)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing sensors..."));
  SensorManager::init();

  // ------------------------------------------------------------------------
  // Initialize User I/O (LEDs, Buttons, NeoPixels)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing user I/O..."));
  UserIO::init();

  // ------------------------------------------------------------------------
  // Initialize Servo Controller (PCA9685)
  // ------------------------------------------------------------------------
#if SERVO_CONTROLLER_ENABLED
  DEBUG_SERIAL.println(F("[Setup] Initializing servo controller..."));
  ServoController::init();
  DEBUG_SERIAL.println(F("  - PCA9685 initialized (50Hz PWM)"));
#endif

  // ------------------------------------------------------------------------
  // Initialize Stepper Motors (Timer3 @ 10kHz)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing stepper motors..."));
  StepperManager::init();

  // ------------------------------------------------------------------------
  // Initialize DC Motors and Encoders
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing DC motors and encoders..."));

  // Calculate counts per revolution based on encoder mode
  uint16_t countsPerRev = ENCODER_PPR * encoder1.getResolutionMultiplier();
  DEBUG_SERIAL.print(F("  - Encoder resolution: "));
  DEBUG_SERIAL.print(countsPerRev);
  DEBUG_SERIAL.println(F(" counts/rev"));

  // Motor 1
  encoder1.init(PIN_M1_ENC_A, PIN_M1_ENC_B, ENCODER_1_DIR_INVERTED);
  velocityEst1.init(countsPerRev);
  velocityEst1.setFilterSize(VELOCITY_FILTER_SIZE);
  velocityEst1.setZeroTimeout(VELOCITY_ZERO_TIMEOUT);
  dcMotors[0].init(0, &encoder1, &velocityEst1, DC_MOTOR_1_DIR_INVERTED);
  dcMotors[0].setPins(PIN_M1_EN, PIN_M1_IN1, PIN_M1_IN2);
  dcMotors[0].setPositionPID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD);
  dcMotors[0].setVelocityPID(DEFAULT_VEL_KP, DEFAULT_VEL_KI, DEFAULT_VEL_KD);
  DEBUG_SERIAL.println(F("  - Motor 1 initialized"));

  // Motor 2
  encoder2.init(PIN_M2_ENC_A, PIN_M2_ENC_B, ENCODER_2_DIR_INVERTED);
  velocityEst2.init(countsPerRev);
  velocityEst2.setFilterSize(VELOCITY_FILTER_SIZE);
  velocityEst2.setZeroTimeout(VELOCITY_ZERO_TIMEOUT);
  dcMotors[1].init(1, &encoder2, &velocityEst2, DC_MOTOR_2_DIR_INVERTED);
  dcMotors[1].setPins(PIN_M2_EN, PIN_M2_IN1, PIN_M2_IN2);
  dcMotors[1].setPositionPID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD);
  dcMotors[1].setVelocityPID(DEFAULT_VEL_KP, DEFAULT_VEL_KI, DEFAULT_VEL_KD);
  DEBUG_SERIAL.println(F("  - Motor 2 initialized"));

  // Motor 3
  encoder3.init(PIN_M3_ENC_A, PIN_M3_ENC_B, ENCODER_3_DIR_INVERTED);
  velocityEst3.init(countsPerRev);
  velocityEst3.setFilterSize(VELOCITY_FILTER_SIZE);
  velocityEst3.setZeroTimeout(VELOCITY_ZERO_TIMEOUT);
  dcMotors[2].init(2, &encoder3, &velocityEst3, DC_MOTOR_3_DIR_INVERTED);
  dcMotors[2].setPins(PIN_M3_EN, PIN_M3_IN1, PIN_M3_IN2);
  dcMotors[2].setPositionPID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD);
  dcMotors[2].setVelocityPID(DEFAULT_VEL_KP, DEFAULT_VEL_KI, DEFAULT_VEL_KD);
  DEBUG_SERIAL.println(F("  - Motor 3 initialized"));

  // Motor 4
  encoder4.init(PIN_M4_ENC_A, PIN_M4_ENC_B, ENCODER_4_DIR_INVERTED);
  velocityEst4.init(countsPerRev);
  velocityEst4.setFilterSize(VELOCITY_FILTER_SIZE);
  velocityEst4.setZeroTimeout(VELOCITY_ZERO_TIMEOUT);
  dcMotors[3].init(3, &encoder4, &velocityEst4, DC_MOTOR_4_DIR_INVERTED);
  dcMotors[3].setPins(PIN_M4_EN, PIN_M4_IN1, PIN_M4_IN2);
  dcMotors[3].setPositionPID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD);
  dcMotors[3].setVelocityPID(DEFAULT_VEL_KP, DEFAULT_VEL_KI, DEFAULT_VEL_KD);
  DEBUG_SERIAL.println(F("  - Motor 4 initialized"));

  // ------------------------------------------------------------------------
  // Attach Encoder Interrupts
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Attaching encoder interrupts..."));

  attachInterrupt(digitalPinToInterrupt(PIN_M1_ENC_A), encoderISR_M1, CHANGE);
  DEBUG_SERIAL.println(F("  - Motor 1 encoder ISR attached"));

  attachInterrupt(digitalPinToInterrupt(PIN_M2_ENC_A), encoderISR_M2, CHANGE);
  DEBUG_SERIAL.println(F("  - Motor 2 encoder ISR attached"));

  attachInterrupt(digitalPinToInterrupt(PIN_M3_ENC_A), encoderISR_M3, CHANGE);
  DEBUG_SERIAL.println(F("  - Motor 3 encoder ISR attached"));

  attachInterrupt(digitalPinToInterrupt(PIN_M4_ENC_A), encoderISR_M4, CHANGE);
  DEBUG_SERIAL.println(F("  - Motor 4 encoder ISR attached"));

  // ------------------------------------------------------------------------
  // Register Soft Scheduler Tasks
  // DC Motor PID, UART comms, and sensors are now hard real-time ISR tasks
  // (Timer1 @ 200/100 Hz and Timer4 @ 100 Hz).  Only UserIO stays soft.
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Registering soft scheduler tasks..."));

  int8_t taskId = Scheduler::registerTask(taskUserIO, 1000 / USER_IO_FREQ_HZ, 0);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - User I/O: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / USER_IO_FREQ_HZ);
    DEBUG_SERIAL.println(F("ms"));
  }

  // ------------------------------------------------------------------------
  // Transition to IDLE (all hardware initialized, ready to receive commands)
  // ------------------------------------------------------------------------
  SystemManager::requestTransition(SYS_STATE_IDLE);
  DEBUG_SERIAL.println(F("[Setup] System state → IDLE"));

  // ------------------------------------------------------------------------
  // Start Hard Real-Time ISRs (MUST be last — ISRs fire immediately)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Starting hard real-time ISRs (Timer1 + Timer4)..."));
  ISRScheduler::init();

  // ------------------------------------------------------------------------
  // Setup Complete
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("[Setup] Initialization complete!"));
  DEBUG_SERIAL.println(F("  Hard RT: PID@200Hz, UART@100Hz, Sensors@100Hz (Timer1/3/4)"));
  DEBUG_SERIAL.println(F("  Soft RT: UserIO@20Hz (millis-based)"));
  DEBUG_SERIAL.println(F("[Setup] Entering main loop..."));
  DEBUG_SERIAL.println();
}

// ============================================================================
// ARDUINO MAIN LOOP
// ============================================================================

void loop() {
  // Execute highest-priority ready task
  Scheduler::tick();

  // Note: Non-time-critical housekeeping can be added here
  // (e.g., watchdog reset, debug output, etc.)
}
