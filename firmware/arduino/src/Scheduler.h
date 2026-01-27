/**
 * @file Scheduler.h
 * @brief Cooperative multitasking scheduler using Timer1
 *
 * This module provides a priority-based cooperative scheduler for periodic tasks.
 * Timer1 generates a 1kHz interrupt (1ms period) that updates task ready flags.
 * The main loop calls tick() to execute the highest-priority ready task.
 *
 * Features:
 * - Up to 8 registered tasks
 * - Priority levels 0-7 (0 = highest priority)
 * - Non-preemptive (tasks run to completion)
 * - Deterministic timing via hardware timer
 *
 * Usage:
 *   Scheduler::init();                                  // Configure Timer1
 *   Scheduler::registerTask(callback, 5, 0);            // 5ms period, priority 0
 *   Scheduler::registerTask(callback2, 10, 1);          // 10ms period, priority 1
 *
 *   void loop() {
 *     Scheduler::tick();  // Execute highest-priority ready task
 *   }
 */

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <Arduino.h>

// Maximum number of tasks that can be registered
#define MAX_TASKS               8

// Task callback function type
typedef void (*TaskCallback)(void);

/**
 * @brief Task structure (internal use only)
 */
struct Task {
  TaskCallback callback;        // Function to call when task is ready
  uint16_t period;              // Task period in milliseconds
  uint16_t countdown;           // Countdown until next execution (ms)
  uint8_t priority;             // Priority level (0 = highest)
  bool enabled;                 // Is this task slot active?
  volatile bool ready;          // Is task ready to run? (set by ISR)
};

/**
 * @brief Cooperative multitasking scheduler
 */
class Scheduler {
public:
  /**
   * @brief Initialize Timer1 for 1kHz interrupts
   *
   * Configures Timer1 in CTC mode with OCR1A compare match at 1ms period.
   * Must be called once in setup() before registering tasks.
   */
  static void init();

  /**
   * @brief Register a periodic task
   *
   * @param callback Function to call when task period expires
   * @param periodMs Task period in milliseconds (must be >= 1)
   * @param priority Priority level (0 = highest, 7 = lowest)
   * @return Task ID (0-7) on success, -1 if no slots available
   *
   * @note Tasks are NOT preemptive. Each task should complete quickly
   *       to avoid delaying lower-priority tasks.
   */
  static int8_t registerTask(TaskCallback callback, uint16_t periodMs, uint8_t priority);

  /**
   * @brief Execute the highest-priority ready task
   *
   * Call this function continuously from loop(). It will execute at most
   * one task per call, selecting the highest-priority ready task.
   *
   * @note If no tasks are ready, this function returns immediately.
   */
  static void tick();

  /**
   * @brief Enable or disable a registered task
   *
   * @param taskId Task ID returned by registerTask()
   * @param enabled True to enable, false to disable
   * @return True on success, false if invalid task ID
   */
  static bool setTaskEnabled(int8_t taskId, bool enabled);

  /**
   * @brief Timer1 ISR handler (called internally)
   *
   * This function is called from the Timer1 COMPA interrupt at 1kHz.
   * It updates task countdown timers and sets ready flags.
   *
   * @note Do NOT call this function directly. It is invoked by the ISR.
   */
  static void timerISR();

private:
  static Task tasks[MAX_TASKS];   // Task registry
  static uint8_t taskCount;       // Number of registered tasks
};

#endif // SCHEDULER_H
