/**
 * @file StepperManager.cpp
 * @brief Implementation of Timer3-based stepper motor coordination
 */

#include "StepperManager.h"
#include "../pins.h"

// ============================================================================
// STATIC MEMBER INITIALIZATION
// ============================================================================

StepperMotor StepperManager::steppers_[NUM_STEPPERS];
bool StepperManager::initialized_ = false;

// ============================================================================
// TIMER3 ISR
// ============================================================================

/**
 * @brief Timer3 Overflow Interrupt
 *
 * Fires at 10 kHz (100 µs period) for stepper pulse generation.
 *
 * Timer3 is configured in Fast PWM mode 14 (ICR3 as TOP) instead of CTC
 * so that OC3A can independently drive hardware PWM on pin 5 while the OVF
 * ISR still fires at 10 kHz.  See ISRScheduler.h for the full explanation.
 */
ISR(TIMER3_OVF_vect) {
    StepperManager::timerISR();
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void StepperManager::init() {
    if (initialized_) return;

    // Initialize stepper instances
    for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
        steppers_[i].init(i);
    }

    // Configure stepper pins
    steppers_[0].setPins(PIN_ST1_STEP, PIN_ST1_DIR, PIN_ST1_EN);
#if defined(PIN_ST1_LIMIT)
    steppers_[0].setLimitPin(PIN_ST1_LIMIT, LIMIT_ACTIVE_LOW ? LOW : HIGH);
#endif

    steppers_[1].setPins(PIN_ST2_STEP, PIN_ST2_DIR, PIN_ST2_EN);
#if defined(PIN_ST2_LIMIT)
    steppers_[1].setLimitPin(PIN_ST2_LIMIT, LIMIT_ACTIVE_LOW ? LOW : HIGH);
#endif

    steppers_[2].setPins(PIN_ST3_STEP, PIN_ST3_DIR, PIN_ST3_EN);
#if defined(PIN_ST3_LIMIT)
    steppers_[2].setLimitPin(PIN_ST3_LIMIT, LIMIT_ACTIVE_LOW ? LOW : HIGH);
#endif

    steppers_[3].setPins(PIN_ST4_STEP, PIN_ST4_DIR, PIN_ST4_EN);
#if defined(PIN_ST4_LIMIT)
    steppers_[3].setLimitPin(PIN_ST4_LIMIT, LIMIT_ACTIVE_LOW ? LOW : HIGH);
#endif

    // ========================================================================
    // Configure Timer3: Fast PWM mode 14 (ICR3 as TOP) — 10 kHz OVF
    // ========================================================================
    //
    //   f_cpu = 16 MHz, prescaler = 8  →  f_timer = 2 MHz
    //   ICR3  = (2 000 000 / STEPPER_TIMER_FREQ_HZ) - 1 = 199  →  10 kHz OVF
    //
    //   Fast PWM mode 14: WGM33:30 = 1110
    //     TCCR3A: WGM31=1
    //     TCCR3B: WGM33=1, WGM32=1, CS31=1 (prescaler 8)
    //
    //   ICR3 replaces OCR3A as the TOP register (same numeric value: 199).
    //   This frees OCR3A for independent hardware PWM on OC3A (pin 5):
    //     Rev A — OC3A drives M1_EN (motor PWM)
    //     Rev B — OC3A drives LED_RED
    //
    //   TIMSK3: TOIE3 (OVF interrupt) instead of OCIE3A (compare-A interrupt).
    // ========================================================================

    noInterrupts();

    TCCR3A = (1 << WGM31);
    TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS31);
    ICR3   = (uint16_t)((F_CPU / (8UL * STEPPER_TIMER_FREQ_HZ)) - 1);   // 199
    TCNT3  = 0;

    // Connect OC3A output pin based on revision (set in pins.h)
#if defined(PIN_M1_EN_IS_OC3A)
    // Rev A: OC3A (pin 5) → M1_EN, non-inverting fast PWM
    TCCR3A |= (1 << COM3A1);
    OCR3A   = 0;   // Motor off at startup
#elif defined(PIN_LED_RED_IS_OC3A)
    // Rev B: OC3A (pin 5) → LED_RED, non-inverting fast PWM
    TCCR3A |= (1 << COM3A1);
    OCR3A   = 0;   // LED off at startup
#endif

    TIMSK3 = (1 << TOIE3);   // OVF interrupt (was OCIE3A in CTC mode)

    interrupts();

    initialized_ = true;

#ifdef DEBUG_STEPPER
    DEBUG_SERIAL.println(F("[StepperManager] Timer3 @ 10kHz (Fast PWM mode 14)"));
    DEBUG_SERIAL.print(F("  - ICR3 = "));
    DEBUG_SERIAL.println(ICR3);
#endif
}

// ============================================================================
// STEPPER ACCESS
// ============================================================================

StepperMotor* StepperManager::getStepper(uint8_t stepperId) {
    if (stepperId >= NUM_STEPPERS) {
        return nullptr;
    }
    return &steppers_[stepperId];
}

// ============================================================================
// TIMER ISR HANDLER
// ============================================================================

void StepperManager::timerISR() {
#if DEBUG_PINS_ENABLED
    digitalWrite(DEBUG_PIN_STEPPER_ISR, HIGH);
#endif

    // Call timerCallback on all stepper channels
    for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
        steppers_[i].timerCallback();
    }

#if DEBUG_PINS_ENABLED
    digitalWrite(DEBUG_PIN_STEPPER_ISR, LOW);
#endif
}

// ============================================================================
// CONTROL FUNCTIONS
// ============================================================================

void StepperManager::emergencyStopAll() {
    for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
        steppers_[i].stop();
    }

#ifdef DEBUG_STEPPER
    DEBUG_SERIAL.println(F("[StepperManager] Emergency stop ALL"));
#endif
}

void StepperManager::disableAll() {
    for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
        steppers_[i].disable();
    }

#ifdef DEBUG_STEPPER
    DEBUG_SERIAL.println(F("[StepperManager] Disabled ALL"));
#endif
}

bool StepperManager::anyMoving() {
    for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
        if (steppers_[i].isMoving()) {
            return true;
        }
    }
    return false;
}
