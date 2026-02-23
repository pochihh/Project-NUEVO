/**
 * @file Scheduler.h
 * @brief Soft (millis-based) scheduler for non-time-critical tasks
 *
 * This module provides a millis()-based soft scheduler for tasks that do NOT
 * need hard real-time guarantees. It is safe to call from loop() even if loop()
 * contains delay() or other blocking code — blocked time simply increases the
 * jitter of soft tasks, but NEVER affects the hard real-time ISR tasks.
 *
 * Hard real-time tasks (DC motor PID, UART, sensors) are handled by ISR timers
 * configured in ISRScheduler and SensorManager. Do NOT register those tasks here.
 *
 * Suitable for this scheduler (jitter-tolerant):
 *   - LED animations (blink, breathe)
 *   - Button / limit-switch polling
 *   - NeoPixel status updates
 *   - Battery warning LED
 *   - User debug output
 *
 * Usage:
 *   Scheduler::init();
 *   Scheduler::registerTask(myCallback, 50, 0);   // 50 ms period, priority 0
 *
 *   void loop() {
 *     Scheduler::tick();     // Run at most one task per call
 *     // delay() or other blocking code here — ISRs are unaffected
 *   }
 *
 * tick() selects the highest-priority (lowest priority number) task whose
 * period has elapsed and runs it. At most one task executes per tick() call.
 * If multiple tasks are overdue, the next call handles the next one.
 */

#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <Arduino.h>

// Maximum number of registered tasks
#define MAX_TASKS   8

// Task callback type
typedef void (*TaskCallback)(void);

/**
 * @brief Soft scheduler task descriptor (internal use)
 */
struct Task {
    TaskCallback callback;   // Function to call
    uint16_t     period;     // Period in milliseconds
    uint32_t     lastRunMs;  // millis() at last execution
    uint8_t      priority;   // 0 = highest priority
    bool         enabled;    // Slot active?
};

/**
 * @brief Millis-based soft scheduler
 */
class Scheduler {
public:
    /**
     * @brief Initialise the task registry (no timer hardware configured)
     *
     * Call once in setup(). Does NOT configure any hardware timer —
     * Timer1 is now owned by ISRScheduler.
     */
    static void init();

    /**
     * @brief Register a periodic task
     *
     * @param callback  Function to call when the period elapses
     * @param periodMs  Task period in milliseconds (>= 1)
     * @param priority  Priority level (0 = highest, 7 = lowest)
     * @return Task ID (0–7) on success, -1 if no slots are available
     */
    static int8_t registerTask(TaskCallback callback, uint16_t periodMs, uint8_t priority);

    /**
     * @brief Run the highest-priority overdue task (call from loop())
     *
     * Executes at most one task per call.  Returns immediately if no task
     * is due.  Timing is best-effort — jitter equals the time spent in
     * blocking code between tick() calls.
     */
    static void tick();

    /**
     * @brief Enable or disable a registered task
     *
     * Enabling a task resets its last-run timestamp to now, so it fires
     * one period after re-enabling (not immediately).
     *
     * @param taskId  ID returned by registerTask()
     * @param enabled True to enable, false to disable
     * @return True on success, false for invalid ID
     */
    static bool setTaskEnabled(int8_t taskId, bool enabled);

private:
    static Task    tasks[MAX_TASKS];
    static uint8_t taskCount;
};

#endif // SCHEDULER_H
