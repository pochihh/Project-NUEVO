/**
 * @file SystemManager.cpp
 * @brief Firmware state machine implementation
 */

#include "SystemManager.h"
#include "modules/LoopMonitor.h"
#include "modules/MessageCenter.h"
#include "modules/SensorManager.h"
#include "modules/UserIO.h"
#include <avr/interrupt.h>

namespace {

void queueUserIoState(SystemState state)
{
    switch (state) {
        case SYS_STATE_INIT:
            UserIO::setPixelStateInit();
            break;
        case SYS_STATE_IDLE:
            UserIO::setPixelStateIdle();
            break;
        case SYS_STATE_RUNNING:
            UserIO::setPixelStateRunning();
            break;
        case SYS_STATE_ERROR:
            UserIO::setPixelStateError();
            break;
        case SYS_STATE_ESTOP:
            UserIO::setPixelStateEstop();
            break;
        default:
            break;
    }
}

} // namespace

// ── Static member initialization ─────────────────────────────────────────────

volatile SystemState SystemManager::state_ = SYS_STATE_INIT;
bool SystemManager::batteryFaultArmed_ = false;

// ── Public API ───────────────────────────────────────────────────────────────

void SystemManager::init() {
    state_ = SYS_STATE_INIT;
    batteryFaultArmed_ = false;
}

SystemState SystemManager::getState() {
    return state_;   // uint8_t read — atomic on AVR, no guard needed
}

bool SystemManager::triggerBootCompleted() {
    resetRunningPowerMonitor();
    if (!requestTransition(SYS_STATE_IDLE)) {
        return false;
    }
    queueUserIoState(SYS_STATE_IDLE);
    return true;
}

bool SystemManager::triggerStartCommand() {
    if (!requestTransition(SYS_STATE_RUNNING)) {
        return false;
    }

    // RUNNING may start without a battery. Once a battery is seen during this
    // session, loss of power becomes a hard safety fault.
    batteryFaultArmed_ = SensorManager::isBatteryPresent();
    queueUserIoState(SYS_STATE_RUNNING);
    return true;
}

bool SystemManager::triggerStopCommand() {
    if (!requestTransition(SYS_STATE_IDLE)) {
        return false;
    }

    resetRunningPowerMonitor();
    MessageCenter::disableAllActuators();
    MessageCenter::cancelDeferredActions();
    queueUserIoState(SYS_STATE_IDLE);
    return true;
}

bool SystemManager::triggerResetCommand() {
    if (!requestTransition(SYS_STATE_IDLE)) {
        return false;
    }

    resetRunningPowerMonitor();
    MessageCenter::disableAllActuators();
    MessageCenter::cancelDeferredActions();
    MessageCenter::clearFaultLatch();
    LoopMonitor::clearFaults();
    queueUserIoState(SYS_STATE_IDLE);
    return true;
}

bool SystemManager::triggerEstopCommand() {
    resetRunningPowerMonitor();
    MessageCenter::disableAllActuators();
    MessageCenter::cancelDeferredActions();
    if (!requestTransition(SYS_STATE_ESTOP)) {
        return false;
    }
    queueUserIoState(SYS_STATE_ESTOP);
    return true;
}

void SystemManager::triggerSafetyFaultFromIsr(uint8_t triggerFlags) {
    resetRunningPowerMonitor();
    MessageCenter::disableAllActuators();
    MessageCenter::latchFaultFlags(triggerFlags);
    queueUserIoState(SYS_STATE_ERROR);
    forceState(SYS_STATE_ERROR);
}

bool SystemManager::canEnableDriveActuator() {
    SystemState st = getState();
    return SensorManager::isBatteryPresent() &&
           (st == SYS_STATE_IDLE || st == SYS_STATE_RUNNING);
}

bool SystemManager::canRunDriveActuator() {
    return SensorManager::isBatteryPresent() &&
           (getState() == SYS_STATE_RUNNING);
}

bool SystemManager::canEnableServoActuator() {
    SystemState st = getState();
    return (st == SYS_STATE_IDLE || st == SYS_STATE_RUNNING);
}

bool SystemManager::shouldTripHeartbeatFault() {
    return (getState() == SYS_STATE_RUNNING);
}

bool SystemManager::shouldTripBatteryFault() {
    if (getState() != SYS_STATE_RUNNING) {
        return false;
    }

    const bool batteryPresent = SensorManager::isBatteryPresent();
    const bool batteryCritical = SensorManager::isBatteryCritical();
    const bool batteryOvervoltage = SensorManager::isBatteryOvervoltage();

    if (batteryPresent) {
        batteryFaultArmed_ = true;
    }

    if (!batteryFaultArmed_) {
        return false;
    }

    return (!batteryPresent) || batteryCritical || batteryOvervoltage;
}

bool SystemManager::shouldWarnNoBattery() {
    if (SensorManager::isBatteryPresent()) {
        return false;
    }
    return !(getState() == SYS_STATE_RUNNING && batteryFaultArmed_);
}

uint8_t SystemManager::getBatteryFaultFlags() {
    uint8_t flags = 0;
    if (!SensorManager::isBatteryPresent() || SensorManager::isBatteryCritical()) {
        flags |= ERR_UNDERVOLTAGE;
    }
    if (SensorManager::isBatteryOvervoltage()) {
        flags |= ERR_OVERVOLTAGE;
    }
    return flags;
}

bool SystemManager::requestTransition(SystemState newState) {
    // Atomic read-modify-write via cli/sei pair
    uint8_t savedSREG = SREG;
    cli();

    SystemState cur = state_;
    bool ok = false;

    switch (newState) {
        case SYS_STATE_IDLE:
            // From: INIT (setup done), RUNNING (stop), ERROR/ESTOP (reset)
            ok = (cur == SYS_STATE_INIT   ||
                  cur == SYS_STATE_RUNNING ||
                  cur == SYS_STATE_ERROR   ||
                  cur == SYS_STATE_ESTOP);
            break;

        case SYS_STATE_RUNNING:
            // From: IDLE only
            ok = (cur == SYS_STATE_IDLE);
            break;

        case SYS_STATE_ESTOP:
            // From: any state
            ok = true;
            break;

        case SYS_STATE_ERROR:
            // From: any except INIT and ESTOP
            // (Prefer forceState() from ISR; requestTransition() works for soft path)
            ok = (cur != SYS_STATE_INIT && cur != SYS_STATE_ESTOP);
            break;

        default:
            break;
    }

    if (ok) state_ = newState;
    SREG = savedSREG;
    return ok;
}

void SystemManager::forceState(SystemState s) {
    // Called from an ISR — interrupts are already disabled by hardware.
    // Simple volatile write; no cli/sei guard needed.
    state_ = s;
}

void SystemManager::resetRunningPowerMonitor() {
    batteryFaultArmed_ = false;
}
