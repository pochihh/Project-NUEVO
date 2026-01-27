/**
 * @file test_scheduler.ino
 * @brief Test sketch for Scheduler module (Phase 1)
 *
 * This test verifies the Timer1-based cooperative scheduler by blinking
 * multiple LEDs at different frequencies using scheduled tasks.
 *
 * Expected Behavior:
 * - LED_GREEN: Blinks at 2 Hz (500ms on, 500ms off) - Highest priority (0)
 * - LED_BLUE: Blinks at 1 Hz (1000ms on, 1000ms off) - Medium priority (1)
 * - LED_ORANGE: Blinks at 0.5 Hz (2000ms on, 2000ms off) - Low priority (2)
 * - LED_RED: Heartbeat indicator (100ms pulse every 5 seconds) - Lowest priority (3)
 * - Debug Pin A9: Toggles on every scheduler tick (should show 1kHz square wave)
 *
 * Verification:
 * 1. Visual: All four LEDs should blink at their designated rates
 * 2. Serial Monitor: Task execution debug output (if DEBUG_SCHEDULER enabled)
 * 3. Oscilloscope on A9: Should measure 1kHz (1ms period) square wave
 * 4. Oscilloscope on A10: Should show LED task execution timing
 *
 * How to Test:
 * 1. Upload this sketch to Arduino Mega 2560
 * 2. Open Serial Monitor at 115200 baud
 * 3. Observe LED blink patterns
 * 4. (Optional) Connect oscilloscope to debug pins A9 and A10
 *
 * Notes:
 * - This test uses only the Scheduler module (no motors, sensors, or UART)
 * - LED pins are defined in pins.h
 * - Timer1 is configured for 1kHz (1ms) base tick
 * - All tasks are cooperative (non-preemptive)
 */

// ============================================================================
// INCLUDES
// ============================================================================

// Include scheduler and configuration
#include "config.h"
#include "pins.h"
#include "Scheduler.h"

// ============================================================================
// TEST CONFIGURATION
// ============================================================================

// Uncomment to enable detailed debug output
// #define DEBUG_SCHEDULER

// Task periods (milliseconds)
#define TASK_LED_GREEN_PERIOD   500    // 2 Hz blink
#define TASK_LED_BLUE_PERIOD    1000   // 1 Hz blink
#define TASK_LED_ORANGE_PERIOD  2000   // 0.5 Hz blink
#define TASK_LED_RED_PERIOD     5000   // Heartbeat every 5 seconds

// LED pulse width for heartbeat
#define HEARTBEAT_PULSE_MS      100

// ============================================================================
// GLOBAL STATE
// ============================================================================

// LED states
bool ledGreenState = false;
bool ledBlueState = false;
bool ledOrangeState = false;
uint32_t heartbeatStartTime = 0;

// Task execution counters (for debug)
uint32_t taskGreenCount = 0;
uint32_t taskBlueCount = 0;
uint32_t taskOrangeCount = 0;
uint32_t taskRedCount = 0;

// ============================================================================
// TASK CALLBACKS
// ============================================================================

/**
 * @brief Toggle green LED at 2 Hz (Priority 0 - Highest)
 */
void taskToggleLEDGreen() {
  ledGreenState = !ledGreenState;
  digitalWrite(PIN_LED_GREEN, ledGreenState ? HIGH : LOW);
  taskGreenCount++;

#ifdef DEBUG_SCHEDULER
  DEBUG_SERIAL.print(F("[Task] LED_GREEN toggled: "));
  DEBUG_SERIAL.print(ledGreenState ? F("ON") : F("OFF"));
  DEBUG_SERIAL.print(F(" (count: "));
  DEBUG_SERIAL.print(taskGreenCount);
  DEBUG_SERIAL.println(F(")"));
#endif
}

/**
 * @brief Toggle blue LED at 1 Hz (Priority 1)
 */
void taskToggleLEDBlue() {
  ledBlueState = !ledBlueState;
  digitalWrite(PIN_LED_BLUE, ledBlueState ? HIGH : LOW);
  taskBlueCount++;

#ifdef DEBUG_SCHEDULER
  DEBUG_SERIAL.print(F("[Task] LED_BLUE toggled: "));
  DEBUG_SERIAL.print(ledBlueState ? F("ON") : F("OFF"));
  DEBUG_SERIAL.print(F(" (count: "));
  DEBUG_SERIAL.print(taskBlueCount);
  DEBUG_SERIAL.println(F(")"));
#endif
}

/**
 * @brief Toggle orange LED at 0.5 Hz (Priority 2)
 */
void taskToggleLEDOrange() {
  ledOrangeState = !ledOrangeState;
  digitalWrite(PIN_LED_ORANGE, ledOrangeState ? HIGH : LOW);
  taskOrangeCount++;

#ifdef DEBUG_SCHEDULER
  DEBUG_SERIAL.print(F("[Task] LED_ORANGE toggled: "));
  DEBUG_SERIAL.print(ledOrangeState ? F("ON") : F("OFF"));
  DEBUG_SERIAL.print(F(" (count: "));
  DEBUG_SERIAL.print(taskOrangeCount);
  DEBUG_SERIAL.println(F(")"));
#endif
}

/**
 * @brief Heartbeat pulse on red LED (Priority 3 - Lowest)
 *
 * Generates a 100ms pulse every 5 seconds to indicate system is alive.
 */
void taskHeartbeat() {
  // Start pulse
  digitalWrite(PIN_LED_RED, HIGH);
  heartbeatStartTime = millis();
  taskRedCount++;

#ifdef DEBUG_SCHEDULER
  DEBUG_SERIAL.print(F("[Task] Heartbeat pulse #"));
  DEBUG_SERIAL.println(taskRedCount);
#endif
}

// ============================================================================
// ARDUINO SETUP
// ============================================================================

void setup() {
  // Initialize debug serial
  DEBUG_SERIAL.begin(DEBUG_BAUD_RATE);
  while (!DEBUG_SERIAL && millis() < 2000) {
    ; // Wait for serial port to connect (up to 2 seconds)
  }

  // Print test header
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println(F("  Scheduler Test - Phase 1"));
  DEBUG_SERIAL.println(F("  Multi-LED Blink Pattern"));
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println();

  // Configure LED pins as outputs
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_LED_ORANGE, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);

  // Initialize all LEDs to OFF
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_BLUE, LOW);
  digitalWrite(PIN_LED_ORANGE, LOW);
  digitalWrite(PIN_LED_RED, LOW);

  DEBUG_SERIAL.println(F("[Setup] LED pins configured"));

#ifdef DEBUG_PINS_ENABLED
  // Configure debug pins for oscilloscope timing measurement
  pinMode(DEBUG_PIN_SCHEDULER, OUTPUT);
  pinMode(DEBUG_PIN_PID_LOOP, OUTPUT);  // Reuse as LED task timing pin
  digitalWrite(DEBUG_PIN_SCHEDULER, LOW);
  digitalWrite(DEBUG_PIN_PID_LOOP, LOW);
  DEBUG_SERIAL.println(F("[Setup] Debug pins configured:"));
  DEBUG_SERIAL.println(F("  - A9: Scheduler tick (1kHz)"));
  DEBUG_SERIAL.println(F("  - A10: Task execution timing"));
#endif

  // Initialize scheduler
  DEBUG_SERIAL.println(F("[Setup] Initializing scheduler..."));
  Scheduler::init();

  // Register tasks
  DEBUG_SERIAL.println(F("[Setup] Registering tasks..."));

  int8_t taskId;

  taskId = Scheduler::registerTask(taskToggleLEDGreen, TASK_LED_GREEN_PERIOD, 0);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - LED_GREEN: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(TASK_LED_GREEN_PERIOD);
    DEBUG_SERIAL.println(F("ms (Priority 0)"));
  }

  taskId = Scheduler::registerTask(taskToggleLEDBlue, TASK_LED_BLUE_PERIOD, 1);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - LED_BLUE: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(TASK_LED_BLUE_PERIOD);
    DEBUG_SERIAL.println(F("ms (Priority 1)"));
  }

  taskId = Scheduler::registerTask(taskToggleLEDOrange, TASK_LED_ORANGE_PERIOD, 2);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - LED_ORANGE: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(TASK_LED_ORANGE_PERIOD);
    DEBUG_SERIAL.println(F("ms (Priority 2)"));
  }

  taskId = Scheduler::registerTask(taskHeartbeat, TASK_LED_RED_PERIOD, 3);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - Heartbeat: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(TASK_LED_RED_PERIOD);
    DEBUG_SERIAL.println(F("ms (Priority 3)"));
  }

  // Print expected behavior
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("[Setup] Expected Behavior:"));
  DEBUG_SERIAL.println(F("  - LED_GREEN: Blinks at 2 Hz (500ms period)"));
  DEBUG_SERIAL.println(F("  - LED_BLUE: Blinks at 1 Hz (1000ms period)"));
  DEBUG_SERIAL.println(F("  - LED_ORANGE: Blinks at 0.5 Hz (2000ms period)"));
  DEBUG_SERIAL.println(F("  - LED_RED: Heartbeat pulse every 5 seconds"));
  DEBUG_SERIAL.println();

  DEBUG_SERIAL.println(F("[Setup] Test started. Observe LED patterns."));
  DEBUG_SERIAL.println(F("[Setup] Press Ctrl+C to stop."));
  DEBUG_SERIAL.println();
}

// ============================================================================
// ARDUINO MAIN LOOP
// ============================================================================

void loop() {
  // Execute scheduler
  Scheduler::tick();

  // Handle heartbeat pulse duration (turn off LED after 100ms)
  if (heartbeatStartTime > 0 && (millis() - heartbeatStartTime) >= HEARTBEAT_PULSE_MS) {
    digitalWrite(PIN_LED_RED, LOW);
    heartbeatStartTime = 0;  // Clear start time
  }

  // Optional: Print status every 10 seconds
  static uint32_t lastStatusTime = 0;
  if (millis() - lastStatusTime >= 10000) {
    lastStatusTime = millis();

    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(F("========== Status Report =========="));
    DEBUG_SERIAL.print(F("Uptime: "));
    DEBUG_SERIAL.print(millis() / 1000);
    DEBUG_SERIAL.println(F(" seconds"));
    DEBUG_SERIAL.print(F("LED_GREEN executions: "));
    DEBUG_SERIAL.println(taskGreenCount);
    DEBUG_SERIAL.print(F("LED_BLUE executions: "));
    DEBUG_SERIAL.println(taskBlueCount);
    DEBUG_SERIAL.print(F("LED_ORANGE executions: "));
    DEBUG_SERIAL.println(taskOrangeCount);
    DEBUG_SERIAL.print(F("Heartbeat pulses: "));
    DEBUG_SERIAL.println(taskRedCount);
    DEBUG_SERIAL.println(F("==================================="));
    DEBUG_SERIAL.println();
  }
}
