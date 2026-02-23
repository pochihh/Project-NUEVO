/**
 * @file Scheduler.cpp
 * @brief Millis-based soft scheduler implementation
 *
 * See Scheduler.h for design notes and usage.
 */

#include "Scheduler.h"
#include "config.h"

// Static member initialisation
Task    Scheduler::tasks[MAX_TASKS];
uint8_t Scheduler::taskCount = 0;

// ============================================================================
// INIT
// ============================================================================

/**
 * @brief Initialise the task registry
 *
 * No hardware timer is configured here. Timer1 is now owned by ISRScheduler
 * (configured via ISRScheduler::init() at the end of setup()).
 */
void Scheduler::init() {
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        tasks[i].enabled   = false;
        tasks[i].lastRunMs = 0;
    }
    taskCount = 0;

#ifdef DEBUG_SCHEDULER
    DEBUG_SERIAL.println(F("[Scheduler] Soft scheduler initialised (millis-based)"));
#endif
}

// ============================================================================
// REGISTER TASK
// ============================================================================

int8_t Scheduler::registerTask(TaskCallback callback, uint16_t periodMs, uint8_t priority) {
    if (callback == nullptr) {
#ifdef DEBUG_SCHEDULER
        DEBUG_SERIAL.println(F("[Scheduler] ERROR: null callback"));
#endif
        return -1;
    }
    if (periodMs < 1) {
#ifdef DEBUG_SCHEDULER
        DEBUG_SERIAL.println(F("[Scheduler] ERROR: period must be >= 1 ms"));
#endif
        return -1;
    }
    if (priority > 7) {
#ifdef DEBUG_SCHEDULER
        DEBUG_SERIAL.println(F("[Scheduler] ERROR: priority must be 0-7"));
#endif
        return -1;
    }
    if (taskCount >= MAX_TASKS) {
#ifdef DEBUG_SCHEDULER
        DEBUG_SERIAL.println(F("[Scheduler] ERROR: no task slots available"));
#endif
        return -1;
    }

    // Find first free slot
    int8_t id = -1;
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        if (!tasks[i].enabled) { id = (int8_t)i; break; }
    }
    if (id < 0) return -1;

    tasks[id].callback  = callback;
    tasks[id].period    = periodMs;
    tasks[id].lastRunMs = millis();   // First fire after one full period
    tasks[id].priority  = priority;
    tasks[id].enabled   = true;

    taskCount++;

#ifdef DEBUG_SCHEDULER
    DEBUG_SERIAL.print(F("[Scheduler] Task #"));
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(F(" registered @ "));
    DEBUG_SERIAL.print(periodMs);
    DEBUG_SERIAL.print(F(" ms, priority "));
    DEBUG_SERIAL.println(priority);
#endif

    return id;
}

// ============================================================================
// TICK (call from loop())
// ============================================================================

/**
 * @brief Execute the highest-priority overdue task.
 *
 * Scans all enabled tasks and runs the one whose deadline has elapsed AND
 * has the highest priority (lowest number).  At most one task per call.
 */
void Scheduler::tick() {
    uint32_t now = millis();

    int8_t  bestTask     = -1;
    uint8_t bestPriority = 255;

    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        if (!tasks[i].enabled) continue;
        if ((now - tasks[i].lastRunMs) >= tasks[i].period) {
            if (tasks[i].priority < bestPriority) {
                bestPriority = tasks[i].priority;
                bestTask     = (int8_t)i;
            }
        }
    }

    if (bestTask >= 0) {
        tasks[bestTask].lastRunMs = now;
        tasks[bestTask].callback();
    }
}

// ============================================================================
// SET TASK ENABLED
// ============================================================================

bool Scheduler::setTaskEnabled(int8_t taskId, bool enabled) {
    if (taskId < 0 || taskId >= MAX_TASKS) return false;

    if (tasks[taskId].enabled == enabled) return true;

    tasks[taskId].enabled = enabled;

    if (enabled) {
        // Reset so the task fires after one full period (not immediately)
        tasks[taskId].lastRunMs = millis();
        taskCount++;
    } else {
        if (taskCount > 0) taskCount--;
    }

#ifdef DEBUG_SCHEDULER
    DEBUG_SERIAL.print(F("[Scheduler] Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.println(enabled ? F(" enabled") : F(" disabled"));
#endif

    return true;
}
