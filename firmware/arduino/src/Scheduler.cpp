/**
 * @file Scheduler.cpp
 * @brief Implementation of cooperative multitasking scheduler
 */

#include "Scheduler.h"
#include "config.h"

#ifdef DEBUG_PINS_ENABLED
#include "pins.h"
#endif

// Static member initialization
Task Scheduler::tasks[MAX_TASKS];
uint8_t Scheduler::taskCount = 0;

/**
 * @brief Initialize Timer1 for 1kHz interrupts (1ms period)
 *
 * Timer1 Configuration:
 * - Mode: CTC (Clear Timer on Compare Match)
 * - Prescaler: 8
 * - OCR1A: 1999 (for 1ms period at 16MHz / 8)
 * - Interrupt: TIMER1_COMPA_vect enabled
 *
 * Calculation:
 *   f_timer = f_cpu / prescaler = 16MHz / 8 = 2MHz
 *   period = 1ms = 0.001s
 *   OCR1A = (f_timer * period) - 1 = (2MHz * 0.001s) - 1 = 1999
 */
void Scheduler::init() {
  // Initialize all task slots as disabled
  for (uint8_t i = 0; i < MAX_TASKS; i++) {
    tasks[i].enabled = false;
    tasks[i].ready = false;
  }
  taskCount = 0;

  // Configure Timer1 for CTC mode with OCR1A
  noInterrupts();  // Disable interrupts during configuration

  TCCR1A = 0;      // Clear control register A
  TCCR1B = 0;      // Clear control register B
  TCNT1 = 0;       // Reset counter

  // Set CTC mode (Mode 4: WGM13:0 = 0b0100)
  // WGM12 = 1 (in TCCR1B), others = 0
  TCCR1B |= (1 << WGM12);

  // Set prescaler to 8 (CS11 = 1)
  TCCR1B |= (1 << CS11);

  // Set compare match value for 1ms period
  // OCR1A = (F_CPU / prescaler / frequency) - 1
  // OCR1A = (16000000 / 8 / 1000) - 1 = 1999
  OCR1A = 1999;

  // Enable Timer1 Compare A interrupt
  TIMSK1 |= (1 << OCIE1A);

  interrupts();  // Re-enable interrupts

#ifdef DEBUG_PINS_ENABLED
  // Configure debug pin for oscilloscope timing measurement
  pinMode(DEBUG_PIN_SCHEDULER, OUTPUT);
  digitalWrite(DEBUG_PIN_SCHEDULER, LOW);
#endif

#ifdef DEBUG_SCHEDULER
  DEBUG_SERIAL.println(F("[Scheduler] Timer1 initialized @ 1kHz"));
#endif
}

/**
 * @brief Register a periodic task
 */
int8_t Scheduler::registerTask(TaskCallback callback, uint16_t periodMs, uint8_t priority) {
  // Validate parameters
  if (callback == nullptr) {
#ifdef DEBUG_SCHEDULER
    DEBUG_SERIAL.println(F("[Scheduler] ERROR: Null callback"));
#endif
    return -1;
  }

  if (periodMs < 1) {
#ifdef DEBUG_SCHEDULER
    DEBUG_SERIAL.println(F("[Scheduler] ERROR: Period must be >= 1ms"));
#endif
    return -1;
  }

  if (priority > 7) {
#ifdef DEBUG_SCHEDULER
    DEBUG_SERIAL.println(F("[Scheduler] ERROR: Priority must be 0-7"));
#endif
    return -1;
  }

  if (taskCount >= MAX_TASKS) {
#ifdef DEBUG_SCHEDULER
    DEBUG_SERIAL.println(F("[Scheduler] ERROR: No task slots available"));
#endif
    return -1;
  }

  // Find first available slot
  int8_t taskId = -1;
  for (uint8_t i = 0; i < MAX_TASKS; i++) {
    if (!tasks[i].enabled) {
      taskId = i;
      break;
    }
  }

  if (taskId < 0) {
    return -1;  // Should never happen if taskCount is accurate
  }

  // Configure task
  tasks[taskId].callback = callback;
  tasks[taskId].period = periodMs;
  tasks[taskId].countdown = periodMs;  // Start with full period
  tasks[taskId].priority = priority;
  tasks[taskId].enabled = true;
  tasks[taskId].ready = false;

  taskCount++;

#ifdef DEBUG_SCHEDULER
  DEBUG_SERIAL.print(F("[Scheduler] Registered task #"));
  DEBUG_SERIAL.print(taskId);
  DEBUG_SERIAL.print(F(" @ "));
  DEBUG_SERIAL.print(periodMs);
  DEBUG_SERIAL.print(F("ms, priority "));
  DEBUG_SERIAL.println(priority);
#endif

  return taskId;
}

/**
 * @brief Enable or disable a registered task
 */
bool Scheduler::setTaskEnabled(int8_t taskId, bool enabled) {
  if (taskId < 0 || taskId >= MAX_TASKS) {
    return false;
  }

  if (tasks[taskId].enabled == enabled) {
    return true;  // Already in desired state
  }

  tasks[taskId].enabled = enabled;

  if (enabled) {
    // Reset countdown when re-enabling
    tasks[taskId].countdown = tasks[taskId].period;
    tasks[taskId].ready = false;
    taskCount++;
  } else {
    taskCount--;
  }

#ifdef DEBUG_SCHEDULER
  DEBUG_SERIAL.print(F("[Scheduler] Task #"));
  DEBUG_SERIAL.print(taskId);
  DEBUG_SERIAL.println(enabled ? F(" enabled") : F(" disabled"));
#endif

  return true;
}

/**
 * @brief Execute the highest-priority ready task
 *
 * This function is called from loop(). It searches for the highest-priority
 * ready task (lowest priority number) and executes it.
 */
void Scheduler::tick() {
  // Find highest-priority ready task
  int8_t highestPriorityTask = -1;
  uint8_t highestPriority = 255;  // Lower number = higher priority

  for (uint8_t i = 0; i < MAX_TASKS; i++) {
    if (tasks[i].enabled && tasks[i].ready) {
      if (tasks[i].priority < highestPriority) {
        highestPriority = tasks[i].priority;
        highestPriorityTask = i;
      }
    }
  }

  // Execute task if found
  if (highestPriorityTask >= 0) {
    // Clear ready flag
    tasks[highestPriorityTask].ready = false;

#ifdef DEBUG_SCHEDULER
    DEBUG_SERIAL.print(F("[Scheduler] Executing task #"));
    DEBUG_SERIAL.print(highestPriorityTask);
    DEBUG_SERIAL.print(F(" (priority "));
    DEBUG_SERIAL.print(tasks[highestPriorityTask].priority);
    DEBUG_SERIAL.println(F(")"));
#endif

    // Execute task callback
    tasks[highestPriorityTask].callback();
  }
}

/**
 * @brief Timer1 ISR handler (called at 1kHz)
 *
 * This function updates task countdown timers and sets ready flags.
 * It is called from ISR(TIMER1_COMPA_vect) at 1ms intervals.
 *
 * CRITICAL: Keep this ISR minimal and fast (<50µs execution time)
 */
void Scheduler::timerISR() {
#ifdef DEBUG_PINS_ENABLED
  // Toggle debug pin on ISR entry
  digitalWrite(DEBUG_PIN_SCHEDULER, HIGH);
#endif

  // Update all enabled tasks
  for (uint8_t i = 0; i < MAX_TASKS; i++) {
    if (tasks[i].enabled) {
      // Decrement countdown
      if (tasks[i].countdown > 0) {
        tasks[i].countdown--;
      }

      // Check if task period expired
      if (tasks[i].countdown == 0) {
        tasks[i].ready = true;            // Mark task as ready
        tasks[i].countdown = tasks[i].period;  // Reload period
      }
    }
  }

#ifdef DEBUG_PINS_ENABLED
  // Toggle debug pin on ISR exit
  digitalWrite(DEBUG_PIN_SCHEDULER, LOW);
#endif
}

// ============================================================================
// TIMER1 INTERRUPT SERVICE ROUTINE
// ============================================================================

/**
 * @brief Timer1 Compare A interrupt vector
 *
 * This ISR is triggered at 1kHz (every 1ms) by Timer1 OCR1A compare match.
 * It calls Scheduler::timerISR() to update task timers.
 */
ISR(TIMER1_COMPA_vect) {
  Scheduler::timerISR();
}
