# Timer 3 Conflict Analysis: LED_RED PWM vs Stepper Pulse Generation

> **✅ RESOLVED — v0.8.0**
>
> The conflict described in this document has been fully resolved. Timer3 now runs in
> **Fast PWM mode 14** (ICR3 as TOP) instead of CTC mode.  This frees OCR3A to drive
> hardware PWM on OC3A (pin 5) independently while the OVF ISR fires at the same 10 kHz
> rate.  Full LED_RED breathing is active in both Rev. A and Rev. B.
>
> **Implementation:** `StepperManager::init()` now configures `TCCR3A/B` for Fast PWM
> mode 14 and conditionally connects OC3A via `#if defined(PIN_LED_RED_IS_OC3A)` /
> `#if defined(PIN_M1_EN_IS_OC3A)`.  All PWM writes use direct `OCR3A` writes instead of
> `analogWrite()` (which would reconfigure the timer).  See `pins.h` and `UserIO.cpp`.
>
> The original analysis below is preserved for educational reference.

---

## Executive Summary (Historical — Pre v0.8.0)

**⚠️ CONFIRMED CONFLICT: LED_RED on pin 5 (Rev. B) CANNOT use PWM while Timer3 generates stepper pulses.**

The Rev. B design moves LED_RED from pin 11 (Timer 1) to pin 5 (Timer 3), creating a hardware conflict. Timer 3 is already configured in CTC mode for 10kHz stepper pulse generation and cannot simultaneously generate PWM for LED brightness control.

**Recommendation:** Disable PWM/breathing modes for LED_RED. Use simple ON/OFF control instead (no brightness control).

---

## Technical Details

### Arduino Mega 2560 Timer-to-Pin Mapping

| Pin | Timer | Output Compare | PWM Capable | Current Use (Rev. A) | Rev. B Change |
|-----|-------|----------------|-------------|----------------------|---------------|
| 5 | **Timer 3** | OC3A | Yes | M1_EN (Motor PWM) | **LED_RED** ⚠️ |
| 11 | Timer 1 | OC1A | Yes | **LED_RED** | M4_ENC_A (encoder) |

**The Problem:**
- Pin 5 is controlled by Timer 3's Output Compare A (OC3A)
- Timer 3 is already allocated for stepper motor pulse generation
- A timer can only operate in ONE mode at a time (either CTC or PWM, not both)

---

## Current Timer 3 Configuration (Stepper Pulses)

**File:** `firmware/arduino/src/modules/StepperManager.cpp` (lines 77-96)

```cpp
// Configure Timer3 for 10kHz interrupt (100µs period)
TCCR3A = 0;
TCCR3B = 0;
TCNT3 = 0;

OCR3A = (F_CPU / (8UL * STEPPER_TIMER_FREQ_HZ)) - 1;  // 199 for 10kHz

// Configure for CTC mode (Clear Timer on Compare Match)
TCCR3B |= (1 << WGM32);   // CTC mode with OCR3A as TOP
TCCR3B |= (1 << CS31);    // Prescaler = 8
TIMSK3 |= (1 << OCIE3A);  // Enable Compare Match A interrupt
```

**Timer 3 Mode:** CTC (Clear Timer on Compare)
- Timer counts: 0 → 1 → 2 → ... → OCR3A (199) → **RESET to 0**
- When TCNT3 == OCR3A, ISR(TIMER3_COMPA_vect) fires at 10kHz
- OC3A pin (pin 5) is **NOT connected** to timer output in this mode
- Used for precise timing interrupt, not PWM generation

**ISR Callback:**
```cpp
ISR(TIMER3_COMPA_vect) {
    StepperManager::timerISR();  // Generate step pulses for 4 stepper motors
}
```

---

## What Happens if LED_RED Uses PWM on Pin 5?

**File:** `firmware/arduino/src/modules/UserIO.cpp` (lines 159-162, 253-280)

### LED PWM Mode (line 159-162):
```cpp
else if (mode == LED_PWM) {
    analogWrite(led.pin, brightness);  // ← Calls Arduino analogWrite()
    led.state = true;
}
```

### LED Breathing Mode (line 253-280):
```cpp
case LED_BREATHE: {
    // ... calculate brightness using triangle wave ...
    analogWrite(led.pin, brightness);  // ← Calls Arduino analogWrite() every update
    break;
}
```

### Arduino Core `analogWrite()` Behavior for Pin 5:

When you call `analogWrite(5, value)`, the Arduino core:

1. **Detects pin 5 is controlled by Timer 3 (OC3A)**
2. **Reconfigures Timer 3 registers for Fast PWM mode:**
   ```cpp
   // Arduino core sets:
   TCCR3A |= (1 << COM3A1);  // Enable OC3A PWM output
   TCCR3A |= (1 << WGM31);   // Fast PWM mode (8-bit)
   TCCR3B |= (1 << WGM32);   // Fast PWM mode
   OCR3A = value;            // PWM duty cycle (0-255)
   ```
3. **This OVERWRITES the CTC configuration set by StepperManager!**

### Consequences:

| Scenario | What Happens | Impact |
|----------|--------------|--------|
| **StepperManager inits BEFORE UserIO** | `analogWrite()` reconfigures Timer3 to PWM mode | ⚠️ **Stepper pulses STOP** - ISR may still fire but timing is wrong |
| **UserIO inits BEFORE StepperManager** | StepperManager reconfigures Timer3 to CTC mode | ⚠️ **LED PWM STOPS** - Pin 5 stuck at fixed voltage |
| **Both try to use Timer3** | Whichever initializes last wins | ⚠️ **One feature breaks the other** |

**In all cases, you CANNOT have both stepper pulses AND LED PWM on pin 5 simultaneously.**

---

## Code Evidence of Current Usage

### Rev. A (No Conflict):

**pins.h (line 142):**
```cpp
#define PIN_LED_RED  11  // Pin 11 uses Timer 1 (OC1A)
```

**Pin 11 Timer Mapping:**
- Timer 1, Output Compare A (OC1A)
- Timer 1 is used by Arduino `millis()` / `micros()` but in a compatible way
- Pin 11 PWM works correctly in Rev. A ✅

### Rev. B (Conflict):

**pin_table_rev_B.md (line 11):**
```
| 5 (PWM) | LED_RED | Status LED Red | No | Error/low battery, relocated from pin 11 (Rev. A) |
```

**Pin 5 Timer Mapping:**
- Timer 3, Output Compare A (OC3A)
- Timer 3 is **already in use** for stepper pulse generation
- Pin 5 PWM will conflict with stepper timing ❌

---

## Breathing Effect Implementation

The LED breathing effect relies on `analogWrite()` being called repeatedly with changing brightness values:

**UserIO.cpp (lines 253-280):**
```cpp
case LED_BREATHE: {
    uint32_t elapsed = now - led.lastToggle;
    uint32_t phase = (elapsed * 255) / led.periodMs;

    // Calculate brightness using triangle wave
    uint8_t brightness;
    if (phase < 128) {
        brightness = phase * 2;         // Fade in
    } else {
        brightness = (255 - phase) * 2; // Fade out
    }

    brightness = (brightness * led.brightness) / 255;

    // PIN_LED_PURPLE doesn't support PWM, so use ON/OFF only
    if (led.pin == PIN_LED_PURPLE) {
        digitalWrite(led.pin, (brightness > 128) ? HIGH : LOW);
    } else {
        analogWrite(led.pin, brightness);  // ← REQUIRES PWM HARDWARE
    }
    break;
}
```

**This breathing effect CANNOT work on pin 5 because:**
- It requires hardware PWM support
- Pin 5's PWM timer (Timer 3) is already allocated for steppers
- Software PWM would be too slow and block interrupts

---

## Solution Options

### ✅ RECOMMENDED: Disable LED_RED PWM/Breathing (Accept Hardware Limitation)

**Change required:** Modify `UserIO.cpp` to treat LED_RED as digital-only (ON/OFF)

**Implementation:**
```cpp
void UserIO::updateLED(LEDState& led) {
    // ...

    case LED_BREATHE: {
        // Special handling for LED_RED (pin 5) - no PWM available
        if (led.pin == PIN_LED_RED) {
            // Blink instead of breathe (fallback to digital ON/OFF)
            if (now - led.lastToggle >= led.periodMs / 2) {
                led.lastToggle = now;
                led.state = !led.state;
                digitalWrite(led.pin, led.state ? HIGH : LOW);
            }
        } else {
            // Normal breathing for PWM-capable LEDs
            // ... existing triangle wave code ...
            analogWrite(led.pin, brightness);
        }
        break;
    }

    case LED_PWM: {
        // Special handling for LED_RED (pin 5) - no PWM available
        if (led.pin == PIN_LED_RED) {
            // Treat as ON with threshold
            digitalWrite(led.pin, led.brightness > 128 ? HIGH : LOW);
        } else {
            analogWrite(led.pin, led.brightness);
        }
        break;
    }
}
```

**Pros:**
- ✅ Simple software fix, no hardware changes needed
- ✅ Preserves stepper functionality (higher priority)
- ✅ LED_RED still works for error indication (ON/OFF is sufficient)
- ✅ Other LEDs (pins 44-46) retain PWM capability

**Cons:**
- ❌ No smooth breathing effect on LED_RED
- ❌ No brightness control (full ON or OFF only)

**Educational Value:**
- Students learn about hardware resource conflicts
- Demonstrates priority-based design decisions (steppers > LED effects)

---

### ❌ NOT RECOMMENDED: Move LED_RED to Different Pin

**Option:** Use pin 44 (LED_GREEN) for red LED, reassign green LED elsewhere

**Pin 44 Timer Mapping:**
- Timer 5, Output Compare B (OC5B)
- Timer 5 is available and not used by stepper or encoder systems
- Supports full PWM/breathing effects

**Why NOT recommended:**
- ⚠️ **PCB Rev. B is already fabricated** - would require hardware rework
- ⚠️ Requires trace cuts and wire jumpers (not ideal for educational platform)
- ⚠️ Confusing for students (physical red LED controlled by "green" code)

---

### ❌ NOT RECOMMENDED: Use Software PWM for LED_RED

**Concept:** Manually toggle pin 5 in a timer interrupt to simulate PWM

**Why NOT recommended:**
- ⚠️ Requires additional timer interrupt (consumes CPU)
- ⚠️ Blocks other interrupts during bit-banging
- ⚠️ Cannot achieve smooth brightness at reasonable frequencies
- ⚠️ Adds unnecessary complexity for a status LED

---

## Required Code Changes for Rev. B

### 1. Update `pins.h` for Rev. B
```cpp
// Change line 142 from:
#define PIN_LED_RED  11  // Rev. A
// To:
#define PIN_LED_RED  5   // Rev. B (WARNING: No PWM - Timer3 conflict)
```

### 2. Modify `UserIO.cpp` to Disable PWM for LED_RED

Add pin check in `updateLED()` method:

```cpp
void UserIO::updateLED(LEDState& led) {
    uint32_t now = millis();

    switch (led.mode) {
        // ... existing OFF, ON, BLINK cases ...

        case LED_PWM:
            // Pin 5 (LED_RED) cannot use PWM - Timer3 conflict with steppers
            if (led.pin == PIN_LED_RED) {
                digitalWrite(led.pin, led.brightness > 128 ? HIGH : LOW);
            } else {
                analogWrite(led.pin, led.brightness);
            }
            break;

        case LED_BREATHE:
            // Pin 5 (LED_RED) cannot breathe - use blink fallback
            if (led.pin == PIN_LED_RED) {
                // Fallback: Blink at breathing period
                if (now - led.lastToggle >= led.periodMs / 2) {
                    led.lastToggle = now;
                    led.state = !led.state;
                    digitalWrite(led.pin, led.state ? HIGH : LOW);
                }
            } else {
                // Normal breathing effect for other LEDs
                // ... existing triangle wave code ...
                analogWrite(led.pin, brightness);
            }
            break;
    }
}
```

### 3. Add Comment in `config.h` (Documentation)

```cpp
// ============================================================================
// STATUS AND USER LEDs
// ============================================================================

// NOTE: In Rev. B, LED_RED is on pin 5 which shares Timer3 with stepper motors.
// PWM and breathing effects are DISABLED for LED_RED to avoid timer conflicts.
// LED_RED operates in ON/OFF mode only (sufficient for error/battery warning).

#define PIN_LED_RED             5       // Status LED Red (Rev. B - no PWM)
```

---

## Testing Checklist

After implementing the fix:

- [ ] Compile firmware with Rev. B pin definitions
- [ ] Verify LED_RED can turn ON and OFF (digital mode)
- [ ] Verify LED_RED blinks when set to LED_BLINK mode
- [ ] Verify LED_RED does NOT dim/fade when set to LED_BREATHE (should blink instead)
- [ ] Verify all 4 stepper motors still generate pulses correctly
- [ ] Verify other LEDs (pins 44-46) still support PWM/breathing
- [ ] Test under load: Run steppers while toggling LED_RED (no glitches)

---

## Conclusion

**Final Answer:** YES, there IS a conflict between LED PWM on pin 5 and Timer3 stepper pulse generation.

**Solution:** Disable PWM/breathing for LED_RED in Rev. B firmware. Use simple ON/OFF control instead.

**Impact:** Low - LED_RED is a status indicator (error/low battery) which doesn't need brightness control. ON/OFF is sufficient for its purpose.

**Priority:** Stepper motors are more important than LED breathing effects. The trade-off is acceptable.

---

## Summary for Documentation

Add this note to `pin_table_rev_B.md` and `REV_A_TO_REV_B_CHANGES.md`:

```markdown
### LED_RED Timer Conflict (Pin 5)

**Issue:** Pin 5 is controlled by Timer 3 (OC3A), which is already used for stepper pulse generation at 10kHz. A timer cannot operate in both CTC mode (steppers) and PWM mode (LED brightness) simultaneously.

**Resolution:** LED_RED operates in digital ON/OFF mode only (no PWM or breathing effects). This is acceptable since LED_RED is used for error/low-battery indication, which does not require brightness control.

**Code Changes:**
- `UserIO.cpp`: Added pin check to disable `analogWrite()` for PIN_LED_RED
- `pins.h`: Updated PIN_LED_RED definition with warning comment
- Other LEDs (pins 44-46) retain full PWM capability
```
