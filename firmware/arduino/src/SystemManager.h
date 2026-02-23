/**
 * @file SystemManager.h
 * @brief Global firmware state machine — single source of truth for SystemState
 *
 * All modules that need to read or change system state include this header.
 * SafetyManager (hard RT) uses forceState(). MessageCenter and other soft
 * modules use requestTransition() which enforces FSM guards.
 *
 * Valid state transitions:
 *
 *   INIT    → IDLE         requestTransition  (setup() complete)
 *   IDLE    → RUNNING      requestTransition  (SYS_CMD_START)
 *   RUNNING → IDLE         requestTransition  (SYS_CMD_STOP)
 *   ERROR   → IDLE         requestTransition  (SYS_CMD_RESET)
 *   ESTOP   → IDLE         requestTransition  (SYS_CMD_RESET)
 *   any     → ESTOP        requestTransition  (SYS_CMD_ESTOP)
 *   any     → ERROR        forceState only    (SafetyManager hard RT fault)
 *
 * Thread safety:
 *   state_ is declared volatile. getState() performs an atomic single-byte
 *   read (safe from any context). requestTransition() uses cli/sei to guard
 *   the read-modify-write. forceState() assumes interrupts are already
 *   disabled (ISR context) — do not call it from soft-scheduler code.
 */

#ifndef SYSTEMMANAGER_H
#define SYSTEMMANAGER_H

#include <Arduino.h>
#include <stdint.h>
#include "messages/TLV_Payloads.h"   // SystemState enum

class SystemManager {
public:
    /**
     * @brief Set initial state to SYS_STATE_INIT.
     * Call once at the very top of setup() before any other module init.
     */
    static void init();

    /**
     * @brief Read current firmware state.
     * SystemState is uint8_t — single-byte read is atomic on AVR.
     * Safe to call from any context (ISR or soft-scheduler).
     */
    static SystemState getState();

    /**
     * @brief Request a state transition with FSM guard.
     *
     * Only valid transitions (per the state diagram above) are accepted.
     * The caller is responsible for any actuator cleanup that must accompany
     * the transition (e.g., disabling motors on STOP or RESET).
     *
     * Uses cli/sei for an atomic read-modify-write. Do NOT call from an ISR
     * where interrupts must remain disabled — use forceState() instead.
     *
     * @param  newState  Target state
     * @return true if transition was accepted and applied
     */
    static bool requestTransition(SystemState newState);

    /**
     * @brief Force a state change — bypasses FSM guards.
     *
     * Intended ONLY for the SafetyManager hard RT fault path.
     * Must be called from within an ISR (interrupts already disabled by hw).
     *
     * @param s  New state to impose
     */
    static void forceState(SystemState s);

private:
    static volatile SystemState state_;
};

#endif // SYSTEMMANAGER_H
