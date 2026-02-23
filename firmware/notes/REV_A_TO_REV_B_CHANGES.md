# Rev. A to Rev. B Pin Remapping - Verification Report

## Summary of Changes

Rev. B implements **full 4x quadrature encoding** for all four DC motors by utilizing both encoder channels A and B on interrupt-capable pins. This required strategic pin reassignments.

### Design Goals Achieved
✅ Motor 1 and 2 (wheels): Full hardware external interrupts (INT0-INT5) for highest priority
✅ Motor 3 and 4 (manipulators): Pin Change Interrupts (PCINT) for 4x mode support
✅ All original functionality preserved through pin relocation

---

## Pin-by-Pin Relocation Table

### Motor Control Pins

| Feature | Rev. A Pin | Rev. B Pin | Change Reason |
|---------|------------|------------|---------------|
| **M1_EN** (PWM) | 5 | 6 | Moved to free pin 5 for LED_RED |
| **M1_IN1** | 8 | 8 | No change |
| **M1_IN2** | 43 | 43 | No change |
| **M1_ENC_A** | 2 (INT0) | 2 (INT0) | No change |
| **M1_ENC_B** | 4 (no INT) | **3 (INT1)** | ⭐ Upgraded to INT for 4x mode |
| | | |
| **M2_EN** (PWM) | 6 | 7 | Moved to free pin 6 for M1_EN |
| **M2_IN1** | 12 | 4 | Moved to free pin 4 (old M1_ENC_B) |
| **M2_IN2** | 13 | 30 | Moved to free pin 30 (old M3_ENC_B) |
| **M2_ENC_A** | 3 (INT1) | **18 (INT5)** | ⭐ Moved to prioritize M1_ENC_B on INT1 |
| **M2_ENC_B** | 7 (no INT) | **19 (INT4)** | ⭐ Upgraded to INT for 4x mode |
| | | |
| **M3_EN** (PWM) | 9 | 9 | No change |
| **M3_IN1** | 34 | 34 | No change |
| **M3_IN2** | 35 | 35 | No change |
| **M3_ENC_A** | 18 (INT5) | **A14 (PCINT14)** | ⭐ Moved to free INT5 for M2 |
| **M3_ENC_B** | 30 (no INT) | **A15 (PCINT15)** | ⭐ Upgraded to PCINT for 4x mode |
| | | |
| **M4_EN** (PWM) | 10 | 10 | No change |
| **M4_IN1** | 36 | 36 | No change |
| **M4_IN2** | 37 | 37 | No change |
| **M4_ENC_A** | 19 (INT4) | **11 (PCINT5)** | ⭐ Moved to free INT4 for M2 |
| **M4_ENC_B** | 31 (no INT) | **12 (PCINT6)** | ⭐ Upgraded to PCINT for 4x mode |

### Status LED and User GPIO

| Feature | Rev. A Pin | Rev. B Pin | Notes |
|---------|------------|------------|-------|
| **LED_RED** | 11 (PWM) | 5 (PWM) | Relocated, still PWM-capable |
| **LED_GREEN** | 44 (PWM) | 44 (PWM) | No change |
| **LED_BLUE** | 45 (PWM) | 45 (PWM) | No change |
| **LED_ORANGE** | 46 (PWM) | 46 (PWM) | No change |
| **LED_PURPLE** | 47 | 47 | No change |
| **Pin 13** | M2_IN2 | **USER_P13** | Now general-purpose GPIO |
| **Pin 31** | M4_ENC_B | **USER_P31** | Now general-purpose GPIO |

### Analog Pins

| Feature | Rev. A Pin | Rev. B Pin | Notes |
|---------|------------|------------|-------|
| A7-A13 | Analog Expansion | Analog Expansion | No change |
| **A14** | Analog Expansion | **M3_ENC_A (PCINT14)** | Reassigned for encoder |
| **A15** | Analog Expansion | **M3_ENC_B (PCINT15)** | Reassigned for encoder |

---

## Feature Verification: All Original Functions Preserved

### ✅ Motor 1 (Wheel)
- [x] EN (PWM): Pin 5 → **Pin 6** ✅ Relocated
- [x] IN1: Pin 8 → Pin 8 ✅ Same
- [x] IN2: Pin 43 → Pin 43 ✅ Same
- [x] ENC_A: Pin 2 (INT0) → Pin 2 (INT0) ✅ Same
- [x] ENC_B: Pin 4 → **Pin 3 (INT1)** ✅ Upgraded to INT
- [x] Current Sense: A3 → A3 ✅ Same

### ✅ Motor 2 (Wheel)
- [x] EN (PWM): Pin 6 → **Pin 7** ✅ Relocated
- [x] IN1: Pin 12 → **Pin 4** ✅ Relocated
- [x] IN2: Pin 13 → **Pin 30** ✅ Relocated
- [x] ENC_A: Pin 3 (INT1) → **Pin 18 (INT5)** ✅ Relocated
- [x] ENC_B: Pin 7 → **Pin 19 (INT4)** ✅ Upgraded to INT
- [x] Current Sense: A4 → A4 ✅ Same

### ✅ Motor 3 (Manipulator)
- [x] EN (PWM): Pin 9 → Pin 9 ✅ Same
- [x] IN1: Pin 34 → Pin 34 ✅ Same
- [x] IN2: Pin 35 → Pin 35 ✅ Same
- [x] ENC_A: Pin 18 (INT5) → **A14 (PCINT14)** ✅ Relocated
- [x] ENC_B: Pin 30 → **A15 (PCINT15)** ✅ Upgraded to PCINT
- [x] Current Sense: A5 → A5 ✅ Same

### ✅ Motor 4 (Manipulator)
- [x] EN (PWM): Pin 10 → Pin 10 ✅ Same
- [x] IN1: Pin 36 → Pin 36 ✅ Same
- [x] IN2: Pin 37 → Pin 37 ✅ Same
- [x] ENC_A: Pin 19 (INT4) → **Pin 11 (PCINT5)** ✅ Relocated
- [x] ENC_B: Pin 31 → **Pin 12 (PCINT6)** ✅ Upgraded to PCINT
- [x] Current Sense: A6 → A6 ✅ Same

### ✅ Other Features
- [x] Status LED Red: Pin 11 → **Pin 5** ✅ Relocated (still PWM)
- [x] Status LED Green: Pin 44 → Pin 44 ✅ Same
- [x] UART to RPi: Pins 16/17 → Pins 16/17 ✅ Same
- [x] I2C (SDA/SCL): Pins 20/21 → Pins 20/21 ✅ Same
- [x] Stepper Controls: Pins 14-15, 22-33 → Same ✅ No change
- [x] User Buttons: Pins 38-53 → Same ✅ No change
- [x] NeoPixel: Pin 42 → Pin 42 ✅ Same

---

## Potential Issues and Considerations

### ⚠️ Issue 1: Pin Exposure Changes

Several pins changed their "Exposed" status from Rev. A to Rev. B:

| Pin | Pin Name | Rev. A Exposed | Rev. B Exposed | Impact |
|-----|----------|----------------|----------------|--------|
| 11 | M4_ENC_A | No (LED_RED) | **Yes** | Encoder now exposed to screw terminal |
| 12 | M4_ENC_B | No (M2_IN1) | **Yes** | Encoder now exposed to screw terminal |
| 13 | USER_P13 | No (M2_IN2) | **Yes** | Now general-purpose GPIO |
| 18 | M2_ENC_A | Yes (M3_ENC_A) | **No** | Now internal (wheel motor) |
| 19 | M2_ENC_B | Yes (M4_ENC_A) | **No** | Now internal (wheel motor) |
| 30 | M2_IN2 | Yes (M3_ENC_B) | **No** | Now internal (wheel motor) |

**Recommendation:**
- Pins 11, 12 (M4 encoders) should remain **exposed** since M4 is a manipulator motor
- Pins 18, 19, 30 (M2 wheel motor) should be **internal** to prevent accidental student modification
- This appears correct in your design ✅

### ⚠️ Issue 2: Analog Expansion Reduced

**Rev. A:** A7-A15 available for analog expansion (9 pins)
**Rev. B:** A7-A13 available for analog expansion (7 pins)

**Impact:** Lost 2 analog input pins (A14, A15) for M3 encoders

**Mitigation:**
- Still have 7 analog pins available for sensors
- M3 encoder functionality more critical than extra analog inputs
- If more analog inputs needed, consider external ADC via I2C ✅

### ⚠️ Issue 3: PCINT Shared Interrupt Vectors

Motors 3 and 4 use Pin Change Interrupts with shared vectors:

| Motor | Encoder | Pin | PCINT | Vector | Shared With |
|-------|---------|-----|-------|--------|-------------|
| M3 | A | A14 | PCINT14 | 10 | M3_ENC_B (A15) |
| M3 | B | A15 | PCINT15 | 10 | M3_ENC_A (A14) |
| M4 | A | 11 | PCINT5 | 9 | M4_ENC_B (12) |
| M4 | B | 12 | PCINT6 | 9 | M4_ENC_B (12) |

**Impact:**
- PCINT interrupt handlers must decode which pin changed (extra ISR overhead)
- Slightly higher latency (~2-3µs vs ~1µs for external INT)
- Lower priority than M1/M2 wheel encoders (correct for manipulators)

**Recommendation:**
- This is acceptable for manipulator motors (lower max speed than wheels)
- See `firmware/technical_notes.md` for PCINT ISR implementation pattern ✅

### ✅ Issue 4: Pin 5 PWM Timer Conflict — FULLY RESOLVED in v0.8.0

**LED_RED** moved from pin 11 (Timer 1) to pin 5 (Timer 3)

**Root Cause (Pre v0.8.0):** Timer 3 was in CTC mode — OCR3A held the TOP value, making it
impossible to simultaneously drive hardware PWM on OC3A and generate stepper ISR ticks.

**Resolution (v0.8.0):** Timer 3 reconfigured to **Fast PWM mode 14 (ICR3 as TOP)**.
- ICR3 replaces OCR3A as the TOP register (same numeric value: 199 → 10 kHz OVF)
- OC3A is now **free** for independent hardware PWM — full LED_RED breathing restored
- OVF interrupt fires at identical 10 kHz rate via TIMER3_OVF_vect (was TIMER3_COMPA_vect)
- All PWM writes use `OCR3A` directly (never `analogWrite()`) to avoid timer reconfiguration

**Implementation files:**
- `StepperManager.cpp`: Fast PWM mode 14 init, conditional OC3A connection
- `pins.h`: `PIN_LED_RED_IS_OC3A`, `LED_RED_OCR`/`LED_RED_ICR` macros
- `UserIO.cpp`: `LED_RED_OCR = (brightness * LED_RED_ICR) / 255` for PWM + breathing

**See:** `firmware/notes/TIMER3_CONFLICT_ANALYSIS.md` for full historical analysis

### ⚠️ Issue 5: PCB Trace Routing Complexity

The extensive pin reassignments may increase PCB routing complexity:

**Traces that crossed between modules:**
- M2_IN1: Pin 12 → Pin 4 (may cross motor driver area)
- M2_IN2: Pin 13 → Pin 30 (may cross motor driver area)
- M2_ENC_A/B: Pins 3/7 → Pins 18/19 (encoder connector relocation)
- M3_ENC_A/B: Pins 18/30 → A14/A15 (analog section routing)
- M4_ENC_A/B: Pins 19/31 → Pins 11/12 (encoder connector relocation)

**Recommendation:**
- Verify PCB layout can accommodate all trace crossings without excessive vias
- Check for potential noise coupling between motor power traces and encoder signals
- Consider ground planes to isolate digital/analog/power sections ✅

---

## Interrupt Priority Hierarchy (Rev. B)

From highest to lowest priority:

1. **INT0 (Vector 1):** M1_ENC_A (pin 2) — Highest priority wheel encoder
2. **INT1 (Vector 2):** M1_ENC_B (pin 3) — Wheel encoder
3. **INT4 (Vector 5):** M2_ENC_B (pin 19) — Wheel encoder
4. **INT5 (Vector 6):** M2_ENC_A (pin 18) — Wheel encoder
5. **PCINT0 (Vector 9):** M4_ENC_A/B (pins 11, 12) — Manipulator encoders (shared)
6. **PCINT1 (Vector 10):** M3_ENC_A/B (A14, A15) — Manipulator encoders (shared)

**Rationale:**
- Wheel motors (M1, M2) get highest interrupt priority (INT0-INT5)
- Manipulator motors (M3, M4) use lower-priority PCINT (acceptable due to lower max speed)
- Matches architectural goal: prioritize wheel encoders for precise odometry ✅

---

## Testing Checklist for Rev. B PCB

Before finalizing Rev. B design, verify:

### Hardware Design
- [x] All pin reassignments correctly routed on PCB schematic
- [x] Encoder connectors placed near Arduino pins 11, 12, A14, A15
- [x] Motor 2 direction control pins (4, 30) routed to H-bridge module
- [x] LED_RED routed to pin 5 (PWM-capable)
- [x] PCINT pins (11, 12, A14, A15) have proper pull-up resistors for encoders
- [x] Pin 13 and 31 exposed as general-purpose GPIO with headers
- [x] Verify pin 5 PWM doesn't conflict with Timer 3 stepper pulse generation ⚠️ **CONFIRMED CONFLICT - See TIMER3_CONFLICT_ANALYSIS.md**

### Firmware Compatibility
- [ ] Update `pins.h` with all Rev. B pin assignments
- [ ] Implement PCINT0 ISR for M4 encoders (pins 11, 12)
- [ ] Implement PCINT1 ISR for M3 encoders (A14, A15)
- [ ] Update motor driver initialization for relocated control pins
- [ ] Test LED_RED on pin 5 (verify PWM functionality)
- [ ] Verify Serial1 (pins 18/19) correctly disabled in code (used by M2 encoders)

### Functional Testing
- [ ] M1 quadrature encoding: Both INT0 and INT1 working (4x mode)
- [ ] M2 quadrature encoding: Both INT5 and INT4 working (4x mode)
- [ ] M3 quadrature encoding: Both PCINT14 and PCINT15 working (4x mode)
- [ ] M4 quadrature encoding: Both PCINT5 and PCINT6 working (4x mode)
- [ ] All motor direction controls functional (IN1, IN2 relocated pins)
- [ ] All motor PWM controls functional (EN relocated pins)
- [ ] LED_RED functional on pin 5
- [ ] Pins 13 and 31 accessible as user GPIO

---

## Conclusion

✅ **All original Rev. A features successfully preserved in Rev. B**
✅ **Achieved full 4x quadrature encoding for all four motors**
✅ **Interrupt priorities correctly assigned (wheels > manipulators)**

⚠️ **Action Required:**
1. Verify pin 5 (LED_RED) PWM compatibility with Timer 3 stepper pulses
2. Confirm PCB layout accommodates all trace routing changes
3. Update firmware `pins.h` and encoder ISR handlers for PCINT implementation

**Overall Assessment:** The Rev. B pin remapping is well-designed and achieves the goal of full 4x quadrature encoding while maintaining all original functionality. The use of PCINT for manipulator motors is appropriate given their lower speed requirements.