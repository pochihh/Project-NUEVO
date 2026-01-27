# Arduino Firmware Technical Notes

This document contains detailed technical information about the ATmega2560 hardware, interrupt behavior, timing analysis, and design rationale. Refer to this document when you need to understand **why** design decisions were made or **how** the hardware works.

For implementation guidance (what to build and in what order), see [implementation.md](implementation.md).

---

## Table of Contents

1. [ATmega2560 Interrupt System](#atmega2560-interrupt-system)
2. [Two-Level Priority System](#two-level-priority-system)
3. [Encoder Timing Analysis](#encoder-timing-analysis)
4. [ISR Design Principles](#isr-design-principles)
5. [Timer Configuration Details](#timer-configuration-details)
6. [Design Decisions and Rationale](#design-decisions-and-rationale)

---

## ATmega2560 Interrupt System

### How Hardware Priority Works

The ATmega2560 microcontroller uses a **fixed hardware priority system** based on the interrupt vector table. This is fundamentally different from software-based priority systems (like RTOS task priorities):

**Vector Table Mechanism:**
- Each interrupt source has a fixed vector address in flash memory (0x0000 to 0x0072)
- When multiple interrupts occur simultaneously, the CPU services the one with the **lowest vector address first**
- Vector 1 (INT0) is the highest priority, vector 57 is the lowest
- **You cannot change these priorities** - they are hardwired into the microcontroller

**Our System's Interrupt Vector Map:**

```
Interrupt Vector Table (relevant entries for our firmware):
┌────────────────────────────────────────────────────────────┐
│ Vector 1  (0x0002): INT0 - Motor 1 Encoder A (Pin 2)       │ ← HIGHEST
│ Vector 2  (0x0004): INT1 - Motor 2 Encoder A (Pin 3)       │
│ Vector 3  (0x0006): INT2 - Available (Pin 21)              │
│ Vector 4  (0x0008): INT3 - Available (Pin 20)              │
│ Vector 5  (0x000A): INT4 - Motor 4 Encoder A (Pin 19)      │
│ Vector 6  (0x000C): INT5 - Motor 3 Encoder A (Pin 18)      │
│ ...                                                        │
│ Vector 17 (0x0022): TIMER0 COMPA - DO NOT USE (millis())   │
│ Vector 19 (0x0026): TIMER1 COMPA - Scheduler Tick (1kHz)   │
│ ...                                                        │
│ Vector 35 (0x0046): TIMER3 COMPA - Stepper Pulses (10kHz)  │
└────────────────────────────────────────────────────────────┘
```

### Interrupt Nesting Behavior

**Default Behavior (What We Use):**
- By default, when ANY ISR is executing, **all other interrupts are disabled** (global interrupt flag is cleared)
- This means ISRs execute to completion without preemption
- If multiple interrupts occur while an ISR is running, they execute in priority order AFTER the current ISR completes

**Exception (Not Used in Our Firmware):**
- If an ISR explicitly calls `sei()` to re-enable interrupts, higher-priority interrupts CAN nest
- **We do NOT use interrupt nesting** - adds complexity and can cause stack overflow

### Visual: Interrupt Priority Flow

```
Hardware Interrupt Priority Levels (Interrupt Nesting DISABLED - Arduino Default):
┌────────────────────────────────────────────────────────────┐
│ HIGHEST: External INT0-INT5 (Encoder ISRs)                 │ ← Vector 1-6
│          - Event-driven, ~1200 Hz max per motor            │
│          - When triggered, run immediately if no ISR active│
├────────────────────────────────────────────────────────────┤
│ HIGH: Timer0 (Arduino millis/micros - DO NOT TOUCH)        │ ← Vector 17-18
│       - 1 kHz (used by Arduino core)                       │
│       - When triggered, run immediately if no ISR active   │
├────────────────────────────────────────────────────────────┤
│ MEDIUM-HIGH: Timer1 (Scheduler tick @ 1kHz)                │ ← Vector 19-22
│              - When triggered, run immediately if no ISR active│
├────────────────────────────────────────────────────────────┤
│ MEDIUM: Timer3 (Stepper pulses @ 10kHz)                    │ ← Vector 34-38
│         - When triggered, run immediately if no ISR active │
├────────────────────────────────────────────────────────────┤
│ LOWEST: Main loop (Cooperative scheduler)                  │ ← No vector
│         - Runs only when NO ISRs are active                │
│         - Any ISR immediately suspends main loop           │
└────────────────────────────────────────────────────────────┘

KEY BEHAVIOR (Nesting Disabled): ALL ISRs run to completion without interruption.
When an ISR is running, the global interrupt flag is CLEARED, blocking all other interrupts.
If multiple interrupts occur while an ISR is running, they execute in priority order AFTER
the current ISR completes.

Example: When encoder edge triggers during Timer3 ISR:
  1. Timer3 ISR running → generates step pulse (global interrupts DISABLED)
  2. Encoder edge detected → interrupt flag set in hardware, but CANNOT preempt
  3. Timer3 ISR completes → returns, global interrupts RE-ENABLED
  4. INT0 ISR executes IMMEDIATELY (highest pending interrupt)
  5. INT0 ISR completes → returns to main loop

Example: When Timer3 interrupt occurs during encoder ISR:
  1. INT0 ISR running → counts encoder edge (global interrupts DISABLED)
  2. Timer3 compare match → interrupt flag set in hardware, but CANNOT preempt
  3. INT0 ISR completes → returns, global interrupts RE-ENABLED
  4. Timer3 ISR executes immediately (only pending interrupt)
  5. Timer3 ISR completes → returns to main loop

WHAT IF NESTING WAS ENABLED? (We don't do this, but for understanding):
  - If an ISR called sei() to re-enable interrupts, higher-priority interrupts COULD preempt
  - Encoder ISR could interrupt Timer3 ISR mid-execution
  - Requires careful stack management and introduces complexity
  - Our firmware keeps nesting DISABLED for predictability
```

### Practical Implications for Our Firmware

1. **Encoder ISRs Always Win**: If an encoder edge arrives while Timer1 or Timer3 ISR is running, the encoder ISR will execute immediately after the current ISR completes.

2. **Timer Priority Doesn't Match Frequency**: Timer1 (1kHz, vector 19) has higher priority than Timer3 (10kHz, vector 35), even though Timer3 runs 10× faster. This is fine because both ISRs are very short (<10µs).

3. **Main Loop Has No Priority**: Scheduler tasks in `loop()` can be interrupted by ANY hardware ISR at any time.

### Why We Chose INT0-INT5 for Encoders

**Design Rationale:**
- External interrupts (INT0-INT5) have the **highest hardware priority** on ATmega2560
- Ensures encoder edges are NEVER missed, even under heavy timer or UART load
- Motor 1 (INT0, vector 1) gets slightly higher priority than Motor 3/4 (INT4/INT5, vectors 5-6)
- At 100 RPM with 1440 PPR encoders, priority difference between motors is negligible (<1µs)

**Pin Assignment Summary:**
```
Motor 1: Pin 2  → INT0 (Vector 1) - Highest priority
Motor 2: Pin 3  → INT1 (Vector 2) - Second highest
Motor 4: Pin 19 → INT4 (Vector 5) - Third
Motor 3: Pin 18 → INT5 (Vector 6) - Fourth
```

All motors are within vectors 1-6, giving them priority over all timer interrupts.

### ⚠️ CRITICAL: Common Misconceptions

> **You Cannot Change Hardware Interrupt Priorities**
>
> Unlike some RTOSes or ARM Cortex-M processors (which have NVIC priority registers), the ATmega2560 has **FIXED** interrupt priorities determined solely by the vector table. You cannot:
> - Rearrange interrupt priorities with software configuration
> - Make Timer3 higher priority than external interrupts
> - Dynamically adjust priorities at runtime
> - Create priority groups or sub-priorities
>
> Your ONLY control is **which physical pins** you attach interrupts to. Lower vector number = higher priority.

> **Interrupt Nesting is Dangerous**
>
> While you CAN enable interrupt nesting by calling `sei()` inside an ISR, this is **NOT RECOMMENDED** for our educational platform:
> - Increases stack usage (nested ISRs consume stack frames)
> - Risk of stack overflow if many interrupts nest
> - Harder to debug and reason about timing
> - Can lead to race conditions if ISRs share data
>
> Our firmware keeps ALL ISRs minimal (<10µs) and non-nested for predictability.

---

## Two-Level Priority System

Our firmware uses TWO distinct priority systems that work together:

### 1. Hardware ISR Priority (Fixed by ATmega2560)

**External Interrupts (Highest):**
- INT0 (Vector 1): Motor 1 encoder
- INT1 (Vector 2): Motor 2 encoder
- INT4 (Vector 5): Motor 4 encoder
- INT5 (Vector 6): Motor 3 encoder

**Timer Interrupts (Medium):**
- Timer0 (Vectors 17-18): Arduino `millis()` / `micros()` - DO NOT MODIFY
- Timer1 (Vectors 19-22): Our scheduler tick (1kHz)
- Timer3 (Vectors 34-38): Our stepper pulse generator (10kHz)

**Main Loop (Lowest):**
- No interrupt vector, runs whenever no ISR is active
- Executes cooperative scheduler tasks

**Key Point**: ANY hardware ISR (even lowest-priority Timer3) will ALWAYS preempt ANY scheduler task in the main loop.

### 2. Software Scheduler Task Priority (Our Cooperative System)

Within the main loop, our scheduler chooses which task to run based on software-defined priorities:

| Priority | Task | Period | Typical Execution Time |
|----------|------|--------|------------------------|
| 0 (highest) | DC Motor PID | 5ms (200Hz) | ~200µs |
| 1 | UART Communication | 10ms (100Hz) | ~500µs |
| 2 | Sensor Reading | 20ms (50Hz) | ~1ms |
| 3 (lowest) | User I/O (LEDs) | 50ms (20Hz) | ~100µs |

**How Scheduler Priority Works:**
- Timer1 ISR (1kHz) checks each task's countdown timer
- When countdown reaches zero, task's `ready` flag is set
- `Scheduler::tick()` in main loop finds highest-priority task with `ready=true`
- That task executes to completion (cooperative, not preemptive)
- Scheduler returns to main loop, repeats

**Critical Distinction:**
- **Hardware priority**: Which ISR runs first when multiple interrupts occur
- **Scheduler priority**: Which task runs first when multiple tasks are ready
- Hardware interrupts ALWAYS preempt scheduler tasks, regardless of task priority

**Example Scenario:**
```
Timeline:
 0ms: DC Motor PID task ready (priority 0)
 5ms: UART task ready (priority 1)

If both are ready at the same time:
  → DC Motor PID executes first (higher scheduler priority)
  → UART task executes after PID completes

But if an encoder interrupt occurs during PID execution:
  → PID task is suspended immediately
  → Encoder ISR runs (highest hardware priority)
  → PID task resumes after ISR completes
```

---

## Encoder Timing Analysis

### Encoder Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Pulses per revolution | 1440 PPR | Already 4x counted (360 base CPR) |
| Maximum motor RPM | 100 RPM | Geared motor output shaft |
| Max edge rate (4x mode) | ~2400 edges/sec | 1440 × (100/60) = 2400 Hz |
| Max edge rate (2x mode) | ~1200 edges/sec | Half of 4x rate |
| ISR period at max speed | ~417 µs (4x) / ~833 µs (2x) | Plenty of margin |

### Timing Budget Analysis

**With 1440 PPR encoders at 100 RPM max:**
- Worst case: 4 motors × 2400 Hz = 9600 ISR/sec = ~104 µs between interrupts
- ISR execution: ~10-20 cycles = ~0.6-1.2 µs at 16 MHz
- Overhead: <2% CPU utilization for encoder counting

**Conclusion:** At 100 RPM max, the encoder ISR rate is very manageable. Even with 4 motors in 4x mode running at max speed, total ISR rate would be ~9600/sec (~104 µs between interrupts on average), leaving ample time for other processing.

### Why Encoder Edges Are Never Missed

Even though encoder ISRs cannot preempt Timer3 ISR (interrupt nesting disabled), encoder edges are guaranteed not to be missed:

1. **Hardware Latching**: When an encoder edge triggers, the interrupt flag is SET in hardware and latched
2. **Minimal ISR Duration**: Timer3 ISR completes in <10µs
3. **Immediate Execution**: After Timer3 ISR returns, encoder ISR executes before returning to main loop
4. **Total Delay**: Maximum delay is ~10µs (one Timer3 ISR execution)
5. **Safe Margin**: At max speed (1200 Hz), encoder edges are 833µs apart - plenty of margin

The key insight: **interrupt flags are latched in hardware**, so even if an ISR is running, the edge is captured and will be serviced immediately after.

---

## ISR Design Principles

### General ISR Guidelines

1. **Keep ISRs Minimal**: Aim for <10µs execution time
2. **No Function Calls**: Inline all operations (except unavoidable Arduino functions like `micros()`)
3. **No Floating Point**: Use integer math only
4. **No Serial Operations**: Never call `Serial.print()` or similar in ISR
5. **Volatile Variables**: Use `volatile` for any variables shared between ISR and main code
6. **Atomic Operations**: Disable interrupts when reading multi-byte volatile variables in main code

### Encoder ISR Design

**ISR Body (minimal operations only):**
```cpp
void encoderISR_M1() {
  // Read direction pin
  bool direction = digitalRead(PIN_M1_ENC_B);

  // Update counter
  if (direction) {
    encoderCount[0]++;
  } else {
    encoderCount[0]--;
  }

  // Capture timestamp for velocity estimation (computed in PID loop)
  lastEdgeTime[0] = micros();

  // Optional: Debug pin toggle
  #ifdef DEBUG_PINS_ENABLED
    TOGGLE_PIN(DEBUG_PIN_ENCODER_ISR);
  #endif
}
```

**What NOT to do in ISR:**
- ❌ Compute velocity (floating point math)
- ❌ Apply filtering
- ❌ Call Serial.print() for debugging
- ❌ Access I2C/SPI peripherals
- ❌ Call delay() or delayMicroseconds()

**Where to do complex operations:**
- ✅ Compute velocity in PID task (200Hz, main loop)
- ✅ Apply filters in main loop
- ✅ Debug output in main loop (check flags set by ISR)

### Timer ISR Design

**Timer1 (Scheduler Tick) - Keep it Simple:**
```cpp
ISR(TIMER1_COMPA_vect) {
  // Only update flags, no task execution in ISR
  for (uint8_t i = 0; i < numTasks; i++) {
    if (--taskCounters[i] == 0) {
      taskReady[i] = true;
      taskCounters[i] = taskPeriods[i];  // Reload
    }
  }
}
```

**Timer3 (Stepper Pulses) - Minimize Per-Stepper Work:**
```cpp
ISR(TIMER3_COMPA_vect) {
  for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
    if (stepperEnabled[i]) {
      // Decrement step interval counter, toggle pin, update position
      // Target: <10µs per stepper = <40µs total for 4 steppers
    }
  }
}
```

---

## Timer Configuration Details

### Timer Allocation

| Timer | Resolution | Usage | Notes |
|-------|------------|-------|-------|
| Timer0 | 8-bit | Arduino millis()/micros() | DO NOT MODIFY |
| Timer1 | 16-bit | Scheduler base tick (1kHz) | OCR1A compare match |
| Timer2 | 8-bit | Available | Could use for tone/PWM |
| Timer3 | 16-bit | Stepper pulse generation (10kHz) | OCR3A compare match |
| Timer4 | 16-bit | Available | Reserved for future use |
| Timer5 | 16-bit | Available | Reserved for future use |

### Timer1 Configuration (1kHz Scheduler Tick)

**CTC Mode with OCR1A:**
```cpp
// CTC mode, no prescaler needed for 1kHz
// CPU clock: 16 MHz
// Desired frequency: 1 kHz (1ms period)
// OCR1A = (F_CPU / (prescaler × desired_freq)) - 1

// With prescaler = 8:
// OCR1A = (16,000,000 / (8 × 1000)) - 1 = 1999

TCCR1A = 0;                    // CTC mode
TCCR1B = (1 << WGM12) | (1 << CS11);  // CTC mode, prescaler 8
OCR1A = 1999;                  // Compare value for 1kHz
TIMSK1 = (1 << OCIE1A);        // Enable compare match interrupt
```

### Timer3 Configuration (10kHz Stepper Pulse)

**CTC Mode with OCR3A:**
```cpp
// Desired frequency: 10 kHz (100µs period)
// OCR3A = (16,000,000 / (1 × 10000)) - 1 = 1599

TCCR3A = 0;                    // CTC mode
TCCR3B = (1 << WGM32) | (1 << CS30);  // CTC mode, no prescaler
OCR3A = 1599;                  // Compare value for 10kHz
TIMSK3 = (1 << OCIE3A);        // Enable compare match interrupt
```

---

## Design Decisions and Rationale

### Why Cooperative Scheduler Instead of RTOS?

**Decision:** Use Timer1-based cooperative scheduler with ISRs for time-critical operations.

**Rationale:**
- **Deterministic Timing**: ISRs provide precise timing for encoders and steppers
- **No RTOS Overhead**: Context switching, kernel overhead, stack per task
- **Educational Clarity**: Students can understand the full system
- **Adequate for Application**: 200Hz PID loop is sufficient for DC motor control
- **Predictability**: Task execution order is explicit and debuggable

**Trade-offs:**
- ❌ Tasks must be cooperative (can't preempt each other)
- ❌ Long-running tasks will delay lower-priority tasks
- ✅ But: All our tasks are short (<1ms), this is acceptable

### Why Velocity Computed in PID Loop, Not ISR?

**Decision:** ISR only captures timestamp, velocity computed at 200Hz in PID task.

**Rationale:**
- **Minimal ISR Time**: ISR stays under 10 cycles (~0.6µs)
- **Floating Point in Main Loop**: No FP math in ISR (slow and disables interrupts longer)
- **Adequate Update Rate**: 200Hz velocity update is sufficient for PID control
- **Allows Filtering**: Can apply moving average filter in main loop

**Alternative Considered:** Compute velocity in ISR
- ❌ ISR time would increase to ~50 cycles
- ❌ Floating point in ISR is slow
- ❌ Blocks other interrupts for longer

### Why 2x Mode Default Instead of 4x?

**Decision:** Default to 2x encoder resolution mode.

**Rationale:**
- **Hardware Limitation**: Current PCB only routes phase A to INT0-INT5
- **Sufficient Resolution**: 1440 PPR × 2 = 2880 counts/rev (0.125° resolution)
- **Lower CPU Load**: Half the interrupt rate vs 4x mode
- **Future Flexibility**: Can enable 4x mode per-motor if phase B routed to INT pins

**When to Use 4x Mode:**
- Need maximum resolution (0.0625° per count)
- Low-speed precise positioning
- Hardware supports both phases on INT-capable pins

### Why Timer3 at 10kHz for Steppers?

**Decision:** Run Timer3 at 10kHz for stepper pulse generation.

**Rationale:**
- **Maximum Step Rate**: 10kHz gives up to ~5000 steps/sec per motor (with margin)
- **Smooth Acceleration**: 100µs resolution allows smooth trapezoidal profiles
- **CPU Budget**: <50µs total ISR time (4 steppers × ~10µs each)
- **Acceptable Priority**: Vector 35 is lower than encoders (correct priority)

**Alternative Considered:** Higher frequency (20kHz)
- ❌ Higher CPU overhead
- ❌ Stepper motors can't respond faster than ~5kHz anyway
- ✅ 10kHz is the sweet spot

---

## References

- ATmega2560 Datasheet: Section 6 (Interrupts), Section 10 (External Interrupts), Section 17-20 (Timers)
- Arduino Mega 2560 Pin Mapping
- Interrupt Vector Table: 0x0000-0x0072 (vectors 0-57)

---

**For implementation guidance, see [implementation.md](implementation.md).**
