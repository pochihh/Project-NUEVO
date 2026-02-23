/**
 * @file SafetyManager.cpp
 * @brief Hard real-time safety watchdog implementation
 */

#include "SafetyManager.h"
#include "MessageCenter.h"
#include "SensorManager.h"
#include "../SystemManager.h"

void SafetyManager::check() {
    SystemState state = SystemManager::getState();

    // Already in a safe or terminal state — nothing to do (common path on boot)
    if (state == SYS_STATE_ERROR ||
        state == SYS_STATE_ESTOP ||
        state == SYS_STATE_INIT) {
        return;
    }

    // ── Fault detection ───────────────────────────────────────────────────────

    // Heartbeat loss is only a fault while RUNNING:
    // In IDLE, no RPi contact is expected (robot may be tethered or stopped).
    bool heartbeatFault = (state == SYS_STATE_RUNNING) &&
                          !MessageCenter::isHeartbeatValid();

    // Battery faults apply in IDLE and RUNNING — prevent motor enable on bad power
    bool batteryFault = SensorManager::isBatteryCritical() ||
                        SensorManager::isBatteryOvervoltage();

    // Common case: all OK — return immediately
    if (!heartbeatFault && !batteryFault) return;

    // ── Fault response ────────────────────────────────────────────────────────

    // 1. Disable all actuators atomically (DC motors, steppers, servos)
    MessageCenter::disableAllActuators();

    // 2. Force ERROR state (bypasses FSM guard — we are inside an ISR,
    //    interrupts are already disabled by hardware)
    SystemManager::forceState(SYS_STATE_ERROR);
}
