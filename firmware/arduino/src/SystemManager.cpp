/**
 * @file SystemManager.cpp
 * @brief Firmware state machine implementation
 */

#include "SystemManager.h"
#include <avr/interrupt.h>

// ── Static member initialization ─────────────────────────────────────────────

volatile SystemState SystemManager::state_ = SYS_STATE_INIT;

// ── Public API ───────────────────────────────────────────────────────────────

void SystemManager::init() {
    state_ = SYS_STATE_INIT;
}

SystemState SystemManager::getState() {
    return state_;   // uint8_t read — atomic on AVR, no guard needed
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
