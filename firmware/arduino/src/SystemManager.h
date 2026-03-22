/**
 * @file SystemManager.h
 * @brief Global firmware state machine — single source of truth for SystemState
 *
 * All modules that need to read system state include this header.
 * Transition policy lives here behind named trigger functions so the rules for
 * START / STOP / RESET / ESTOP / safety faults stay in one place.
 *
 * Valid state transitions:
 *
 *   INIT    → IDLE         triggerBootCompleted()
 *   IDLE    → RUNNING      triggerStartCommand()
 *   RUNNING → IDLE         triggerStopCommand()
 *   ERROR   → IDLE         triggerResetCommand()
 *   ESTOP   → IDLE         triggerResetCommand()
 *   any     → ESTOP        triggerEstopCommand()
 *   any     → ERROR        triggerSafetyFaultFromIsr()
 *
 * Actuator power policy:
 *   - The firmware may enter RUNNING with no battery present.
 *   - Actuator-enable commands are rejected while battery power is absent.
 *   - Once battery power has been seen during a RUNNING session, losing it
 *     or falling below safety thresholds triggers ERROR.
 *
 * Thread safety:
 *   state_ is declared volatile. getState() performs an atomic single-byte
 *   read (safe from any context). Internal transition helpers use cli/sei to
 *   guard read-modify-write. The ISR fault path assumes interrupts are already
 *   disabled by hardware.
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
     * @brief Move the FSM from INIT to IDLE after setup() completes.
     *
     * @return true if transition was accepted and applied
     */
    static bool triggerBootCompleted();

    /**
     * @brief Handle SYS_CMD_START state policy.
     *
     * @return true if transition was accepted and applied
     */
    static bool triggerStartCommand();

    /**
     * @brief Handle SYS_CMD_STOP state policy.
     *
     * Disables actuators and cancels deferred command work on success.
     *
     * @return true if transition was accepted and applied
     */
    static bool triggerStopCommand();

    /**
     * @brief Handle SYS_CMD_RESET state policy.
     *
     * Disables actuators, clears deferred work, clears fault latches, and
     * clears loop-monitor fault state on success.
     *
     * @return true if transition was accepted and applied
     */
    static bool triggerResetCommand();

    /**
     * @brief Handle SYS_CMD_ESTOP state policy.
     *
     * This always disables actuators and cancels deferred work before forcing
     * the ESTOP transition.
     *
     * @return true if transition was accepted and applied
     */
    static bool triggerEstopCommand();

    /**
     * @brief Handle a hard safety fault from ISR context.
     *
     * This is the ISR-safe path for liveness / battery faults. It disables all
     * actuators, latches the triggering error flags, and forces ERROR.
     *
     * @param triggerFlags  SystemErrorFlags bitmask describing the fault cause
     */
    static void triggerSafetyFaultFromIsr(uint8_t triggerFlags);

    /**
     * @brief True when DC/stepper enable commands are allowed to take effect.
     *
     * Policy:
     *   - battery power must be present
     *   - state must be IDLE or RUNNING
     */
    static bool canEnableDriveActuator();

    /**
     * @brief True when DC motion commands that auto-enable motors may take effect.
     *
     * Policy:
     *   - battery power must be present
     *   - state must be RUNNING
     */
    static bool canRunDriveActuator();

    /**
     * @brief True when servo enable commands are allowed to take effect.
     *
     * Policy:
     *   - battery power must be present
     *   - state must be RUNNING
     */
    static bool canEnableServoActuator();

    /**
     * @brief Heartbeat timeout only matters while RUNNING.
     */
    static bool shouldTripHeartbeatFault();

    /**
     * @brief Determine whether battery power should trigger a hard ERROR.
     *
     * RUNNING may begin without a battery. Once battery power has been seen in
     * the current RUNNING session, losing it or crossing safety thresholds
     * becomes a hard fault.
     */
    static bool shouldTripBatteryFault();

    /**
     * @brief Report whether the current no-battery condition should remain a warning.
     *
     * Missing battery is only a warning while no RUNNING-session battery fault
     * has been armed yet. Once battery power has been seen during RUNNING,
     * later loss of battery should be treated as an error condition only.
     */
    static bool shouldWarnNoBattery();

    /**
     * @brief Report the currently active battery-related error flags.
     *
     * Returns ERR_UNDERVOLTAGE and/or ERR_OVERVOLTAGE according to the current
     * measured power state. Intended for safety-fault latching and status use.
     */
    static uint8_t getBatteryFaultFlags();

private:
    static bool requestTransition(SystemState newState);
    static void forceState(SystemState s);
    static void resetRunningPowerMonitor();
    static volatile SystemState state_;
    static bool batteryFaultArmed_;
};

#endif // SYSTEMMANAGER_H
