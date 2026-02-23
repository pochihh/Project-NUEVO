/**
 * @file ISRScheduler.cpp
 * @brief Implementation of hard real-time ISR timer configuration
 *
 * See ISRScheduler.h for full design notes and architecture overview.
 */

#include "ISRScheduler.h"
#include "pins.h"
#include "config.h"

void ISRScheduler::init() {
    noInterrupts();

    // ========================================================================
    // Timer1: Fast PWM mode 14 — 200 Hz OVF → PID + UART ISR
    // ========================================================================
    //
    //   f_cpu     = 16 MHz
    //   prescaler = 8   →   f_timer = 2 MHz
    //   ICR1      = (2 000 000 / DC_PID_FREQ_HZ) - 1 = 9999  →  200 Hz OVF
    //
    //   Fast PWM mode 14: WGM13:10 = 1110
    //     TCCR1A: WGM11=1, WGM10=0
    //     TCCR1B: WGM13=1, WGM12=1, CS11=1 (prescaler 8)
    //
    //   OC1A output connected (Rev A, pin 11) for LED_RED hardware PWM.
    //   Writing OCR1A controls LED brightness; the OVF ISR fires independently.
    // ========================================================================

    TCCR1A = (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    ICR1   = (uint16_t)((F_CPU / (8UL * DC_PID_FREQ_HZ)) - 1);   // 9999
    TCNT1  = 0;

#if defined(PIN_LED_RED_IS_OC1A)
    // Rev A: OC1A (pin 11) → LED_RED, non-inverting fast PWM
    TCCR1A |= (1 << COM1A1);
    OCR1A   = 0;   // LED off at startup
#endif

    TIMSK1 = (1 << TOIE1);   // OVF interrupt only (no compare-match interrupts)

    // ========================================================================
    // Timer4: Fast PWM mode 14 — 10 kHz carrier → sensor dispatch ISR
    // ========================================================================
    //
    //   ICR4 = (2 000 000 / STEPPER_TIMER_FREQ_HZ) - 1 = 199  →  10 kHz OVF
    //
    //   SensorManager's TIMER4_OVF_vect ISR counts 100 OVF ticks before
    //   calling isrTick() — yielding 100 Hz sensor dispatch.
    //
    //   OC4A (pin 6) always connected: M2_EN (Rev A) or M1_EN (Rev B).
    //   OC4B (pin 7) connected only when PIN_M2_EN_IS_OC4B is defined (Rev B).
    //
    //   Fast PWM mode 14: WGM43:40 = 1110
    //     TCCR4A: WGM41=1
    //     TCCR4B: WGM43=1, WGM42=1, CS41=1 (prescaler 8)
    // ========================================================================

    TCCR4A = (1 << WGM41);
    TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS41);
    ICR4   = (uint16_t)((F_CPU / (8UL * STEPPER_TIMER_FREQ_HZ)) - 1);   // 199
    TCNT4  = 0;

    // OC4A: M2_EN (Rev A, pin 6) or M1_EN (Rev B, pin 6) — always connected
    TCCR4A |= (1 << COM4A1);
    OCR4A   = 0;   // Motor off at startup

#if defined(PIN_M2_EN_IS_OC4B)
    // Rev B only: OC4B (pin 7) → M2_EN, non-inverting fast PWM
    TCCR4A |= (1 << COM4B1);
    OCR4B   = 0;   // Motor off at startup
#endif

    TIMSK4 = (1 << TOIE4);   // OVF interrupt only

    interrupts();
}
