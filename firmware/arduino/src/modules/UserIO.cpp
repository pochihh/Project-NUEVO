/**
 * @file UserIO.cpp
 * @brief Implementation of user I/O management
 *
 * LED_RED PWM (v0.8.0+)
 * ---------------------
 * LED_RED uses direct OCR writes (via the LED_RED_OCR / LED_RED_ICR macros
 * defined in pins.h) instead of analogWrite(). This resolves the previously
 * documented Timer3 CTC conflict in Rev. B and enables full PWM + breathing
 * effects on LED_RED in both hardware revisions.
 *
 *   Rev A: LED_RED on pin 11 — Timer1 OC1A — LED_RED_OCR = OCR1A
 *   Rev B: LED_RED on pin 5  — Timer3 OC3A — LED_RED_OCR = OCR3A
 *
 * See: firmware/notes/TIMER3_CONFLICT_ANALYSIS.md (marked RESOLVED v0.8.0)
 */

#include "UserIO.h"
#include "../pins.h"
#include "../SystemManager.h"

// ============================================================================
// STATIC MEMBER INITIALIZATION
// ============================================================================

NeoPixelDriver UserIO::neopixel_;

UserIO::LEDState UserIO::leds_[LED_COUNT];

volatile uint16_t UserIO::buttonStates_     = 0;
volatile uint16_t UserIO::prevButtonStates_ = 0;
uint8_t  UserIO::limitStates_    = 0;
uint8_t  UserIO::animPhase_      = 0;
SystemState UserIO::lastAnimState_ = SYS_STATE_INIT;

bool UserIO::initialized_ = false;

// ============================================================================
// INITIALIZATION
// ============================================================================

void UserIO::init() {
    if (initialized_) return;

    // Initialize buttons (INPUT_PULLUP for active-low buttons)
    for (uint8_t i = 0; i < 10; i++) {
        pinMode(BUTTON_PINS[i], INPUT_PULLUP);
    }

    // Initialize LEDs
    leds_[LED_RED].pin = PIN_LED_RED;
    leds_[LED_RED].mode = LED_OFF;
    pinMode(PIN_LED_RED, OUTPUT);
    digitalWrite(PIN_LED_RED, LOW);

    leds_[LED_GREEN].pin = PIN_LED_GREEN;
    leds_[LED_GREEN].mode = LED_OFF;
    pinMode(PIN_LED_GREEN, OUTPUT);
    digitalWrite(PIN_LED_GREEN, LOW);

    leds_[LED_BLUE].pin = PIN_LED_BLUE;
    leds_[LED_BLUE].mode = LED_OFF;
    pinMode(PIN_LED_BLUE, OUTPUT);
    digitalWrite(PIN_LED_BLUE, LOW);

    leds_[LED_ORANGE].pin = PIN_LED_ORANGE;
    leds_[LED_ORANGE].mode = LED_OFF;
    pinMode(PIN_LED_ORANGE, OUTPUT);
    digitalWrite(PIN_LED_ORANGE, LOW);

    leds_[LED_PURPLE].pin = PIN_LED_PURPLE;
    leds_[LED_PURPLE].mode = LED_OFF;
    pinMode(PIN_LED_PURPLE, OUTPUT);
    digitalWrite(PIN_LED_PURPLE, LOW);

    // Initialize all LED states
    for (uint8_t i = 0; i < LED_COUNT; i++) {
        leds_[i].brightness = 255;
        leds_[i].periodMs = 1000;
        leds_[i].lastToggle = 0;
        leds_[i].state = false;
        leds_[i].breathePhase = 0;
    }

    // Initialize NeoPixel
    neopixel_.init(PIN_NEOPIXEL, 1, 128);  // 1 LED, 50% brightness
    neopixel_.setPixel(0, STATUS_IDLE);    // Start with blue (idle)
    neopixel_.show();

    // Read initial button states (readButtons() is the ISR-callable version)
    readButtons();
    updateLimitSwitches();

    initialized_ = true;

#ifdef DEBUG_USERIO
    DEBUG_SERIAL.println(F("[UserIO] Initialized"));
    DEBUG_SERIAL.println(F("  - 10 buttons configured"));
    DEBUG_SERIAL.println(F("  - 5 LEDs configured"));
    DEBUG_SERIAL.println(F("  - NeoPixel initialized"));
#endif
}

// ============================================================================
// UPDATE
// ============================================================================

void UserIO::update() {
    if (!initialized_) return;

    // Note: button reads moved to readButtons() called from TIMER1 ISR at 100 Hz
    updateLimitSwitches();
    updateLEDs();
    updateNeoPixelAnimation();
}

// ============================================================================
// BUTTON INTERFACE
// ============================================================================

bool UserIO::isButtonPressed(uint8_t buttonId) {
    if (buttonId >= 10) return false;

    return (buttonStates_ & (uint16_t)(1u << buttonId)) != 0;
}

uint16_t UserIO::getButtonStates() {
    return buttonStates_;
}

bool UserIO::wasButtonPressed(uint8_t buttonId) {
    if (buttonId >= 10) return false;

    // Snapshot volatile values once to avoid mid-read change
    uint16_t cur  = buttonStates_;
    uint16_t prev = prevButtonStates_;

    return ((cur & (uint16_t)(1u << buttonId)) != 0) &&
           ((prev & (uint16_t)(1u << buttonId)) == 0);
}

// ============================================================================
// LIMIT SWITCH INTERFACE
// ============================================================================

bool UserIO::isLimitTriggered(uint8_t limitId) {
    if (limitId >= 8) return false;

    return (limitStates_ & (1 << limitId)) != 0;
}

uint8_t UserIO::getLimitStates() {
    return limitStates_;
}

// ============================================================================
// LED CONTROL
// ============================================================================

void UserIO::setLED(LEDId ledId, LEDMode mode, uint8_t brightness, uint16_t periodMs) {
    if (ledId >= LED_COUNT) return;

    LEDState& led = leds_[ledId];
    led.mode = mode;
    led.brightness = brightness;
    led.periodMs = periodMs;
    led.lastToggle = millis();
    led.breathePhase = 0;

    // Immediately apply state for OFF, ON, and PWM modes
    if (mode == LED_OFF) {
        digitalWrite(led.pin, LOW);
        led.state = false;
    } else if (mode == LED_ON) {
        digitalWrite(led.pin, HIGH);
        led.state = true;
    } else if (mode == LED_PWM) {
        // LED_RED uses a direct OCR write — analogWrite() would corrupt the timer.
        if (led.pin == PIN_LED_RED) {
            LED_RED_OCR = ((uint16_t)brightness * LED_RED_ICR) / 255;
        } else {
            analogWrite(led.pin, brightness);
        }
        led.state = true;
    }
}

void UserIO::setAllLEDsOff() {
    for (uint8_t i = 0; i < LED_COUNT; i++) {
        setLED((LEDId)i, LED_OFF);
    }
}

// ============================================================================
// NEOPIXEL SYSTEM STATUS
// ============================================================================

void UserIO::setSystemStatus(uint32_t status) {
    neopixel_.setPixel(0, status);
    neopixel_.show();
}

void UserIO::setNeoPixelColor(uint32_t color) {
    neopixel_.setPixel(0, color);
    neopixel_.show();
}

void UserIO::setNeoPixelColor(uint8_t r, uint8_t g, uint8_t b) {
    neopixel_.setPixel(0, r, g, b);
    neopixel_.show();
}

void UserIO::setNeoPixelBrightness(uint8_t brightness) {
    neopixel_.setBrightness(brightness);
    neopixel_.show();
}

// ============================================================================
// INTERNAL HELPERS
// ============================================================================

void UserIO::readButtons() {
    // Pure GPIO reads — no millis(), no blocking. ISR-safe.
    // Snapshot previous before overwriting (used by wasButtonPressed()).
    prevButtonStates_ = buttonStates_;
    uint16_t states = 0;

    for (uint8_t i = 0; i < 10; i++) {
        // Buttons are active LOW (pressed = LOW)
        if (digitalRead(BUTTON_PINS[i]) == LOW) {
            states |= (uint16_t)(1u << i);
        }
    }
    buttonStates_ = states;
}

void UserIO::updateLimitSwitches() {
    limitStates_ = 0;

    // Read all limit switch states
    for (uint8_t i = 0; i < 8; i++) {
        // Limit switches share pins with buttons 3-10
        uint8_t activeState = LIMIT_ACTIVE_LOW ? LOW : HIGH;
        if (digitalRead(LIMIT_PINS[i]) == activeState) {
            limitStates_ |= (1 << i);
        }
    }
}

void UserIO::updateLEDs() {
    for (uint8_t i = 0; i < LED_COUNT; i++) {
        updateLED(leds_[i]);
    }
}

void UserIO::updateLED(LEDState& led) {
    uint32_t now = millis();

    switch (led.mode) {
        case LED_OFF:
            // Already handled in setLED()
            break;

        case LED_ON:
        case LED_PWM:
            // State was applied once in setLED() — nothing more to do here.
            // LED_RED uses OCR1A/OCR3A (hardware PWM), which persists without
            // re-writing on every update tick.
            break;

        case LED_BLINK:
            // Toggle LED at specified period
            if (now - led.lastToggle >= led.periodMs / 2) {
                led.lastToggle = now;
                led.state = !led.state;
                digitalWrite(led.pin, led.state ? HIGH : LOW);
            }
            break;

        case LED_BREATHE: {
            // Smooth breathing via triangle-wave brightness ramp
            uint32_t elapsed = now - led.lastToggle;
            uint32_t phase   = (elapsed * 255) / led.periodMs;

            if (phase >= 255) {
                led.lastToggle = now;
                phase = 0;
            }

            uint8_t brightness;
            if (phase < 128) {
                brightness = (uint8_t)(phase * 2);          // Fade in
            } else {
                brightness = (uint8_t)((255 - phase) * 2);  // Fade out
            }

            // Apply per-LED brightness ceiling
            brightness = (uint8_t)((brightness * led.brightness) / 255);

            // Apply to hardware:
            //   LED_RED    → direct OCR write (Timer1 OC1A / Timer3 OC3A)
            //   LED_PURPLE → digital ON/OFF (non-PWM pin)
            //   Others     → analogWrite() safe on Timer2/5
            if (led.pin == PIN_LED_RED) {
                LED_RED_OCR = ((uint16_t)brightness * LED_RED_ICR) / 255;
            } else if (led.pin == PIN_LED_PURPLE) {
                digitalWrite(led.pin, (brightness > 128) ? HIGH : LOW);
            } else {
                analogWrite(led.pin, brightness);
            }
            break;
        }

        default:
            break;
    }
}

// ============================================================================
// NEOPIXEL ANIMATION
// ============================================================================

void UserIO::hsvToRgb(uint8_t h, uint8_t s, uint8_t v,
                      uint8_t& r, uint8_t& g, uint8_t& b) {
    if (s == 0) { r = g = b = v; return; }

    uint8_t region    = h / 43;                   // 0–5 (six hue sextants)
    uint8_t remainder = (uint8_t)((h - (uint8_t)(region * 43)) * 6);

    // Fast /256 approximation avoids costly division on AVR
    uint8_t p = (uint8_t)(((uint16_t)v * (255u - s)) >> 8);
    uint8_t q = (uint8_t)(((uint16_t)v * (255u - (((uint16_t)s * remainder) >> 8))) >> 8);
    uint8_t t = (uint8_t)(((uint16_t)v * (255u - (((uint16_t)s * (255u - remainder)) >> 8))) >> 8);

    switch (region) {
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        default: r = v; g = p; b = q; break;
    }
}

void UserIO::updateNeoPixelAnimation() {
    SystemState state = SystemManager::getState();

    // Reset animation phase on state change for a clean visual transition
    if (state != lastAnimState_) {
        animPhase_     = 0;
        lastAnimState_ = state;
    }

    uint8_t r = 0, g = 0, b = 0;

    switch (state) {

        // ── INIT: static yellow ────────────────────────────────────────────
        case SYS_STATE_INIT:
            r = 255; g = 180; b = 0;
            break;

        // ── IDLE: slow breathing emerald (~3 s period @ 20 Hz = 60 ticks) ──
        case SYS_STATE_IDLE: {
            uint8_t phase = animPhase_ % 60;   // 0–59 repeating

            uint8_t bri;
            if (phase < 30) {
                // Fade in: 0 → 225
                bri = (uint8_t)(15u + ((uint16_t)phase * 210u) / 29u);
            } else {
                // Fade out: 225 → 15 (keep a faint glow at the bottom)
                bri = (uint8_t)(15u + ((uint16_t)(59u - phase) * 210u) / 29u);
            }
            hsvToRgb(100, 240, bri, r, g, b);   // Emerald hue ≈ H=141°
            animPhase_++;
            break;
        }

        // ── RUNNING: rainbow hue sweep (~3.2 s full cycle @ 20 Hz) ─────────
        case SYS_STATE_RUNNING:
            hsvToRgb((uint8_t)((uint16_t)animPhase_ * 4u), 255, 200, r, g, b);
            animPhase_++;   // Overflows at 256 → wraps hue naturally
            break;

        // ── ERROR: fast flashing red + slight hue shift (0.5 s, 10 ticks) ──
        case SYS_STATE_ERROR: {
            uint8_t phase = animPhase_ % 10;   // 0–9 repeating (0.5 s period)
            if (phase < 5) {
                r = 255; g = 0; b = 0;              // Bright pure red (on)
            } else {
                hsvToRgb(10, 255, 60, r, g, b);     // Dim orange-red (off pulse)
            }
            animPhase_++;
            break;
        }

        // ── ESTOP: solid bright red (non-blinking — distinct from ERROR) ───
        case SYS_STATE_ESTOP:
            r = 255; g = 0; b = 0;
            break;

        default:
            r = 0; g = 0; b = 30;   // Dim blue for any undefined/unknown state
            break;
    }

    neopixel_.setPixel(0, r, g, b);
    neopixel_.show();
}
