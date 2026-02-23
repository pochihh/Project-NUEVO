# Firmware Test Sketches

Each subdirectory is a standalone Arduino sketch that exercises one subsystem. Upload a test sketch the same way you upload the main firmware — select **Arduino Mega 2560**, choose the port, and click Upload. Open the **Serial Monitor at 115200 baud** unless noted otherwise.

## The `src` Symlink

The Arduino IDE requires all source files to be inside the sketch directory. Every test folder contains a `src` symlink pointing to the shared firmware source at `firmware/arduino/src/`. Create it after cloning:

```bash
# macOS / Linux
cd firmware/tests
for dir in test_*/; do
    ln -sf ../../arduino/src "$dir/src"
done
```

```powershell
# Windows — run PowerShell as Administrator
cd firmware\tests
Get-ChildItem -Directory -Filter "test_*" | ForEach-Object {
    $target = Resolve-Path "..\..\arduino\src"
    New-Item -ItemType Junction -Path "$($_.FullName)\src" -Target $target -Force
}
```

Verify: `ls -la firmware/tests/test_scheduler/src` should show `src -> ../../arduino/src`.

---

## Test Index

| Test | Phase | What it tests |
|------|-------|---------------|
| [test_scheduler](#test_scheduler) | 1 | Timer1 1 kHz cooperative scheduler |
| [test_uart_tlv](#test_uart_tlv) | 2 | TLV v2.0 protocol, MessageCenter state machine |
| [test_encoder](#test_encoder) | 3 | Quadrature encoder counting (2x/4x), direction |
| [test_dc_motor_pwm](#test_dc_motor_pwm) | 3 | H-bridge direct PWM + encoder readback |
| [test_dc_motor_pid](#test_dc_motor_pid) | 3 | Closed-loop PID velocity and position control |
| [test_current_sensing](#test_current_sensing) | 3 | ADC current sensor, per-motor current feedback |
| [test_servo](#test_servo) | 4 | PCA9685 I2C PWM controller, 16-channel servo |
| [test_stepper](#test_stepper) | 4 | Timer3 stepper pulse generation, trapezoidal profile |
| [test_voltage](#test_voltage) | 5 | ADC battery/rail voltage monitoring |
| [test_user_io](#test_user_io) | 5 | Buttons, PWM LEDs, NeoPixel patterns |
| [test_eeprom](#test_eeprom) | 5 | EEPROM read/write, power-cycle persistence |
| [test_i2c_scanner](#test_i2c_scanner) | — | I2C bus scan utility |

---

## test_scheduler

**Subsystem:** Scheduler (Timer1 @ 1 kHz)
**Hardware:** No external hardware needed (uses on-board LEDs)

Registers four LED tasks at different rates and verifies that each fires at the right interval. Optional oscilloscope pins (A9, A10) let you measure the scheduler tick with a scope.

**Expected output:** LEDs blink at their programmed rates. Serial Monitor shows initialization; no further output unless `DEBUG_SCHEDULER` is uncommented.

| LED | Rate |
|-----|------|
| LED_GREEN | 2 Hz |
| LED_BLUE | 1 Hz |
| LED_ORANGE | 0.5 Hz |
| LED_RED | heartbeat pulse every 5 s |

---

## test_uart_tlv

**Subsystem:** MessageCenter, TLV v2.0 protocol
**Hardware:** None required for compile verification; Serial2 loopback (pin 16 → 17) or RPi/PC for protocol testing

Initializes MessageCenter and runs `processIncoming()` + `sendTelemetry()` at 100 Hz via the scheduler. Prints system state and liveness status to USB Serial at 1 Hz.

**State machine summary:**

| Command | type | cmd | Transition |
|---------|------|-----|------------|
| SYS_CMD_START | 3 | 1 | IDLE → RUNNING |
| SYS_CMD_STOP | 3 | 2 | RUNNING → IDLE |
| SYS_CMD_RESET | 3 | 3 | ERROR/ESTOP → IDLE |
| SYS_CMD_ESTOP | 3 | 4 | any → ESTOP |

- **IDLE:** only SYS_STATUS sent (1 Hz)
- **RUNNING:** full telemetry on Serial2 up to 100 Hz
- **Liveness timeout:** 500 ms without any TLV — actuators disabled

**Loopback test:** connect pin 16 (TX2) → pin 17 (RX2). Outgoing packets loop back and reset the liveness timer, so State stays IDLE and Liveness shows "OK".

---

## test_encoder

**Subsystem:** EncoderCounter (2x/4x quadrature)
**Hardware:** DC motors with encoders connected (encoder power only — no motor power needed)

Initializes all four encoder channels and prints counts + last-edge timestamps at 5 Hz. Manually rotate the motor shafts to verify counts increment/decrement in the correct direction.

**Verify:**
- Forward rotation → count increases
- Backward rotation → count decreases
- Count resolution = `ENCODER_PPR × mode_multiplier` per revolution

---

## test_dc_motor_pwm

**Subsystem:** DCMotor pin interface, EncoderCounter
**Hardware:** Motors with H-bridge connected, motor power supply, encoders

Cycles motors 1 and 2 through: Forward (25% PWM) → Brake → Reverse (25% PWM) → Brake every 2 seconds. Prints encoder counts at 5 Hz to confirm the encoders track motion and direction.

**Start conservatively:** the default PWM is 25% (64/255). Adjust `TEST_PWM_FORWARD` / `TEST_PWM_REVERSE` if needed.

---

## test_dc_motor_pid

**Subsystem:** DCMotor closed-loop control, PID, VelocityEstimator
**Hardware:** Motors 1 & 2 with encoders, motor power supply; shafts free to spin

Automatically sequences through five test states (5 seconds each):

| State | What happens |
|-------|-------------|
| Velocity +500 t/s | Motor tracks 500 ticks/sec forward |
| Velocity −500 t/s | Motor tracks 500 ticks/sec reverse |
| Velocity 0 | Motor holds zero (braking) |
| Position +720 ticks | Motor moves to +720 ticks (≈1 rev in 2x mode) |
| Position −720 ticks | Motor moves to −720 ticks |

Prints position, velocity, PWM output, and current at 5 Hz. Tune `DEFAULT_VEL_KP/KI/KD` and `DEFAULT_POS_KP/KI/KD` in `config.h` if tracking is poor.

---

## test_current_sensing

**Subsystem:** ADC current sensing (CT pins)
**Hardware:** Motors 1–4 connected with current sensors (CT pins A3–A6), motor power supply

Interactive test. Select a motor with `1`–`4`, then command forward/reverse/stop or an automatic PWM sweep. Prints raw ADC, voltage, and current (mA) at 10 Hz.

| Key | Action |
|-----|--------|
| `1`–`4` | Select motor |
| `f` | Forward at current PWM |
| `r` | Reverse at current PWM |
| `s` | Stop |
| `a` | Auto sweep: 0 → 50 → 100 → 150 → 200 → 255 PWM |
| `5` | Set PWM = 128 (50%) |
| `6`–`9` | Set PWM proportional to key |
| `u` | Toggle bipolar offset (2.5 V at 0 A) |
| `n` | Toggle noise filtering |
| `h` | Help |

---

## test_servo

**Subsystem:** ServoController (PCA9685 I2C)
**Hardware:** PCA9685 on I2C bus (SDA=20, SCL=21), servo(s) connected, servo power supply

Interactive test for all 16 PCA9685 channels.

| Command | Action |
|---------|--------|
| `e` | Enable servo outputs (global OE) |
| `d` | Disable servo outputs |
| `s<ch>,<us>` | Set channel to pulse width in µs (e.g., `s0,1500`) |
| `a<ch>,<deg>` | Set channel to angle in degrees (e.g., `a0,90`) |
| `c<ch>` | Center channel (1500 µs) |
| `w<ch>` | Sweep single channel back and forth |
| `W` | Sweep all 16 channels |
| `0`–`9`, `A`–`F` | Quick-center channel 0–15 |
| `?` | Print status |
| `help` | Show command list |

---

## test_stepper

**Subsystem:** StepperManager, StepperMotor (Timer3 @ 10 kHz)
**Hardware:** Stepper driver(s) (A4988/DRV8825), stepper motor(s); optional limit switch(es)

Interactive test for all four stepper channels. Timer3 runs at 10 kHz for pulse generation. Status auto-prints every 2 seconds.

| Command | Action |
|---------|--------|
| `e<id>` | Enable stepper (0–3) |
| `d<id>` | Disable stepper |
| `m<id>,<steps>` | Move relative steps (e.g., `m0,200`) |
| `p<id>,<pos>` | Move to absolute position (e.g., `p0,0`) |
| `v<id>,<vel>` | Set max velocity in steps/sec |
| `a<id>,<accel>` | Set acceleration in steps/sec² |
| `h<id>,<dir>` | Home stepper (dir: `1` or `-1`) |
| `s<id>` | Emergency stop one stepper |
| `S` | Emergency stop all steppers |
| `?` or `help` | Status / command list |

Default parameters: 1000 steps/sec velocity, 500 steps/sec² acceleration.

---

## test_voltage

**Subsystem:** SensorManager voltage monitoring (ADC)
**Hardware:** Battery and power rails connected; voltage dividers on A0 (battery), A1 (5 V), A2 (servo)

Prints battery, 5 V rail, and servo rail voltages in a table at 1 Hz. Compare against a multimeter to verify calibration.

| Key | Action |
|-----|--------|
| `r` | Print raw ADC values |
| `s` | Print battery noise statistics (over last 10 samples) |
| `c` | Calibration instructions |
| `h` | Help |

Divider ratios are in `config.h` (`VBAT_DIVIDER_RATIO`, `V5_DIVIDER_RATIO`, `VSERVO_DIVIDER_RATIO`). Adjust if readings differ from multimeter by more than ±5%.

---

## test_user_io

**Subsystem:** UserIO (buttons, LEDs, NeoPixel)
**Hardware:** 2 on-board buttons, 5 LEDs (pins 44–47 + LED_RED), NeoPixel WS2812B (pin 42)

Runs a startup LED sweep (each LED on for 500 ms), then enters interactive mode. Buttons also work: BTN1–5 toggle LEDs, BTN6–9 cycle NeoPixel colors, BTN10 turns all LEDs off.

**LED commands** (letters select LED: `r`=RED, `g`=GREEN, `b`=BLUE, `o`=ORANGE, `p`=PURPLE):

| Command | Action |
|---------|--------|
| `r1` / `r0` | RED LED on / off |
| `rb` | RED LED blink |
| `rt` | RED LED breathe |
| `n<0–7>` | NeoPixel color: 0=off 1=red 2=green 3=blue 4=yellow 5=cyan 6=magenta 7=white |
| `s<0–3>` | System status: 0=OK 1=BUSY 2=WARNING 3=ERROR |
| `?` / `h` | Status / help |

---

## test_eeprom

**Subsystem:** PersistentStorage (EEPROM)
**Hardware:** No external hardware needed

Demonstrates EEPROM non-volatility across power cycles.

**Run 1** (storage uninitialised): writes test values (wheel diameter, wheel base, mag offsets), verifies immediate read-back, then prompts you to power off the Arduino.

**Run 2** (after power cycle): reads the values back and confirms they survived.

| Key | Action |
|-----|--------|
| `d` | Dump raw EEPROM bytes with field annotations |
| `r` | Erase all stored data (next boot = Run 1) |
| `w` | Write test values again |
| `h` | Help |

---

## test_i2c_scanner

**Subsystem:** I2C bus (Wire library)
**Hardware:** I2C devices on SDA (pin 20) / SCL (pin 21)

Scans addresses 0x01–0x7F and prints any that respond. Runs once on boot; reset the Arduino to scan again.

**Expected addresses:**

| Address | Device |
|---------|--------|
| 0x40 | PCA9685 PWM controller |
| 0x68 | ICM-20948 IMU |
| 0x70 | PCA9685 ALL_CALL (broadcast, normal) |
| 0x62 | Garmin LIDAR-Lite v4 (default) |
| 0x00–0x7F | Any other Qwiic device on the bus |
