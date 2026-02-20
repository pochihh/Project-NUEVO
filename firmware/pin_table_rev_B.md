## GPIO Pin Assignment Table [Rev. B]
**Exposed pins are available on screw terminals/headers for reuse; core comms and default wheel drive pins remain internal.**

| Pin(s) | Pin Name | Function | Exposed? | Notes |
|--------|----------|----------|----------|-------|
| 0 (RX0) | RX0 | USB Serial | No | Programming/debug only |
| 1 (TX0) | TX0 | USB Serial | No | Programming/debug only |
| 2 (INT0) | M1_ENC_A | Motor 1 Encoder A | No | Default left/right wheel encoder A |
| 3 (INT1) | M1_ENC_B | Motor 1 Encoder B | No | 4x quadrature mode for M1 (wheel) |
| 4 | M2_IN1 | Motor 2 Direction IN1 | No | Relocated from pin 12 (Rev. A) |
| 5 (PWM) | LED_RED | Status LED Red | No | Error/low battery, relocated from pin 11 (Rev. A) |
| 6 (PWM Timer 4) | M1_EN | Motor 1 PWM Enable | No | Relocated from pin 5 (Rev. A) |
| 7 (PWM Timer 4) | M2_EN | Motor 2 PWM Enable | No | Relocated from pin 6 (Rev. A) |
| 8 (PWM) | M1_IN1 | Motor 1 Direction IN1 | No | Default wheel direction |
| 9 (PWM, timer 2) | M3_EN | Motor 3 PWM Enable | Yes | Manipulator motor |
| 10 (PWM, timer 2) | M4_EN | Motor 4 PWM Enable | Yes | Manipulator motor |
| 11 (PWM) | M4_ENC_A | Motor 4 Encoder A (PCINT5) | Yes | 4x mode via PCINT, relocated from pin 19 (Rev. A) |
| 12 | M4_ENC_B | Motor 4 Encoder B (PCINT6) | Yes | 4x mode via PCINT, relocated from pin 31 (Rev. A) |
| 13 | USER_P13 | User GPIO / General Purpose | Yes | Available for custom use |
| 14 | ST1_STEP | Stepper 1 STEP | Yes | Stepper control |
| 15 | ST2_STEP | Stepper 2 STEP | Yes | Stepper control |
| 16 (TX2) | TX_RPI | **UART to RPi5** | No | Via level shifter (5V → 3.3V) |
| 17 (RX2) | RX_RPI | **UART from RPi5** | No | Via level shifter (3.3V → 5V) |
| 18 (INT5) | M2_ENC_A | Motor 2 Encoder A | No | 4x quadrature mode for M2 (wheel), relocated from pin 3 (Rev. A) |
| 19 (INT4) | M2_ENC_B | Motor 2 Encoder B | No | 4x quadrature mode for M2 (wheel), relocated from pin 7 (Rev. A) |
| 20 (SDA) | SDA | I2C Data | Yes | Qwiic + PCA9685 module |
| 21 (SCL) | SCL | I2C Clock | Yes | Qwiic + PCA9685 module |
| 22 | ST1_DIR | Stepper 1 DIR | Yes | Stepper direction |
| 23 | ST2_DIR | Stepper 2 DIR | Yes | Stepper direction |
| 24 | ST3_DIR | Stepper 3 DIR | Yes | Stepper direction |
| 25 | ST4_DIR | Stepper 4 DIR | Yes | Stepper direction |
| 26 | ST1_EN | Stepper 1 ENABLE | Yes | Individual enable |
| 27 | ST2_EN | Stepper 2 ENABLE | Yes | Individual enable |
| 28 | ST3_EN | Stepper 3 ENABLE | Yes | Individual enable |
| 29 | ST4_EN | Stepper 4 ENABLE | Yes | Individual enable |
| 30 | M2_IN2 | Motor 2 Direction IN2 | No | Relocated from pin 13 (Rev. A) |
| 31 | USER_P31 | User GPIO / General Purpose | Yes | Available for custom use |
| 32 | ST3_STEP | Stepper 3 STEP | Yes | Stepper control |
| 33 | ST4_STEP | Stepper 4 STEP | Yes | Stepper control |
| 34 | M3_IN1 | Motor 3 IN1 | Yes | Direction |
| 35 | M3_IN2 | Motor 3 IN2 | Yes | Direction |
| 36 | M4_IN1 | Motor 4 IN1 | Yes | Direction |
| 37 | M4_IN2 | Motor 4 IN2 | Yes | Direction |
| 38 | BTN1 | User Button 1 | No | On-board only, INPUT_PULLUP |
| 39 | BTN2 | User Button 2 | No | On-board only, INPUT_PULLUP |
| 40 | LIM1 / BTN3 | Limit Switch 1 / Button 3 | Yes | JST XH 3-pin (V, S, G) |
| 41 | LIM2 / BTN4 | Limit Switch 2 / Button 4 | Yes | JST XH 3-pin (V, S, G) |
| 42 | NEOPIXEL_DIN | WS2812B RGB LED Data | No | NeoPixel control |
| 43 | M1_IN2 | Motor 1 IN2 | No | Default wheel direction |
| 44 (PWM) | LED_GREEN | Status LED Green | No | System OK (default) |
| 45 (PWM) | LED_BLUE | User LED Blue | Yes | Exposed for user |
| 46 (PWM) | LED_ORANGE | User LED Orange | Yes | Exposed for user |
| 47 | LED_PURPLE | User LED Purple | Yes | Exposed for user (non-PWM) |
| 48 | LIM3 / BTN5 | Limit Switch 3 / Button 5 | Yes | JST XH 3-pin (V, S, G) |
| 49 | LIM4 / BTN6 | Limit Switch 4 / Button 6 | Yes | JST XH 3-pin (V, S, G) |
| 50 | LIM5 / BTN7 | Limit Switch 5 / Button 7 | Yes | JST XH 3-pin (V, S, G) |
| 51 | LIM6 / BTN8 | Limit Switch 6 / Button 8 | Yes | JST XH 3-pin (V, S, G) |
| 52 | LIM7 / BTN9 | Limit Switch 7 / Button 9 | Yes | JST XH 3-pin (V, S, G) |
| 53 | LIM8 / BTN10 | Limit Switch 8 / Button 10 | Yes | JST XH 3-pin (V, S, G) |
| A0 | VBAT_SENSE | Battery Voltage Monitor | No | Divider 1:6 on BAT_IN |
| A1 | V5_SENSE | 5V Rail Monitor | No | Divider 1:2 after 5V buck |
| A2 | VSERVO_SENSE | Servo Rail Monitor | No | Divider 1:3 servo rail |
| A3 | M1_CT | Motor 1 Current Sense (CT) | No | H-bridge feedback |
| A4 | M2_CT | Motor 2 Current Sense (CT) | No | H-bridge feedback |
| A5 | M3_CT | Motor 3 Current Sense (CT) | Yes | H-bridge feedback |
| A6 | M4_CT | Motor 4 Current Sense (CT) | Yes | H-bridge feedback |
| A7-A13 | ANALOG_EXP | Analog Expansion | Yes | Available for sensors |
| A14 | M3_ENC_A | Motor 3 Encoder A (PCINT14) | Yes | 4x mode via PCINT, relocated from pin 18 (Rev. A) |
| A15 | M3_ENC_B | Motor 3 Encoder B (PCINT15) | Yes | 4x mode via PCINT, relocated from pin 30 (Rev. A) |


**DC Motor Control Summary (per motor):**
| Motor | EN (PWM) | IN1 | IN2 | Encoder A | Encoder B | Current (CT) |
|-------|----------|-----|-----|-----------|-----------|--------------|
| 1 | M1_EN (pin 6) | M1_IN1 (pin 8) | M1_IN2 (pin 43) | M1_ENC_A (INT0 / pin 2) | M1_ENC_B (INT1 / pin 3) | M1_CT (A3) |
| 2 | M2_EN (pin 7) | M2_IN1 (pin 4) | M2_IN2 (pin 30) | M2_ENC_A (INT5 / pin 18) | M2_ENC_B (INT4 / pin 19) | M2_CT (A4) |
| 3 | M3_EN (pin 9) | M3_IN1 (pin 34) | M3_IN2 (pin 35) | M3_ENC_A (PCINT14 / A14) | M3_ENC_B (PCINT15 / A15) | M3_CT (A5) |
| 4 | M4_EN (pin 10) | M4_IN1 (pin 36) | M4_IN2 (pin 37) | M4_ENC_A (PCINT5 / pin 11) | M4_ENC_B (PCINT6 / pin 12) | M4_CT (A6) |

**Hardware Interrupts Used:**
- INT0 (pin 2): M1_ENC_A — Motor 1 Encoder A (Vector 1, highest priority)
- INT1 (pin 3): M1_ENC_B — Motor 1 Encoder B (Vector 2)
- INT4 (pin 19): M2_ENC_B — Motor 2 Encoder B (Vector 5)
- INT5 (pin 18): M2_ENC_A — Motor 2 Encoder A (Vector 6)
- PCINT0 (pin 11): M4_ENC_A — Motor 4 Encoder A (Vector 9, shared bank)
- PCINT0 (pin 12): M4_ENC_B — Motor 4 Encoder B (Vector 9, shared bank)
- PCINT1 (A14): M3_ENC_A — Motor 3 Encoder A (Vector 10, shared bank)
- PCINT1 (A15): M3_ENC_B — Motor 3 Encoder B (Vector 10, shared bank)

**Key Changes from Rev. A:**
- M1 and M2 (wheel motors) now use both encoder channels on dedicated INT pins for full 4x quadrature resolution
- M3 and M4 (manipulator motors) use Pin Change Interrupts (PCINT) for 4x mode, relocated to analog pins (A14/A15) and PWM pins (11/12)

**Serial Ports:**
- Serial0 (pins 0/1): USB programming/debug
- Serial2 (pins 16/17 — TX_RPI / RX_RPI): Raspberry Pi communication via level shifter (5V ↔ 3.3V)
- Serial1 (pins 18/19): NOT AVAILABLE (used by M2_ENC_A / M2_ENC_B encoder interrupts)
- Serial3 (pins 14/15): NOT AVAILABLE (used by stepper STEP signals ST1_STEP / ST2_STEP)
