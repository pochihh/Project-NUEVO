/**
 * @file SafetyManager.h
 * @brief Hard real-time safety watchdog
 *
 * SafetyManager::check() is the single authority for hard safety fault
 * detection and actuator shutdown. It runs inside TIMER1_OVF_vect at 100 Hz
 * (every other 200 Hz PID tick).
 *
 * Fault table:
 *
 *   Condition                       Active in state(s)   Response
 *   ──────────────────────────────  ───────────────────  ───────────────────────────────
 *   Heartbeat lost (RPi timeout)    RUNNING only          disableAllActuators + ERROR
 *   Battery voltage < VBAT_CUTOFF   IDLE or RUNNING       disableAllActuators + ERROR
 *   Battery voltage > VBAT_OVERV    IDLE or RUNNING       disableAllActuators + ERROR
 *
 * Recovery:
 *   Only SYS_CMD_RESET (from RPi) can transition ERROR → IDLE.
 *   The underlying fault must also be resolved (battery OK, RPi reconnected).
 *
 * Note: check() is a no-op when state is INIT, ERROR, or ESTOP — already safe.
 */

#ifndef SAFETYMANAGER_H
#define SAFETYMANAGER_H

#include <Arduino.h>
#include <stdint.h>

class SafetyManager {
public:
    /**
     * @brief Run all hard real-time safety checks.
     *
     * Call from TIMER1_OVF_vect at the 100 Hz tick (every other PID cycle).
     * Executes in O(1) time on the common (no-fault) path.
     *
     * On any fault: immediately disables all actuators and forces
     * SYS_STATE_ERROR via SystemManager::forceState().
     */
    static void check();
};

#endif // SAFETYMANAGER_H
