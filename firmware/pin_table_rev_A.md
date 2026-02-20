## GPIO Pin Assignment Table [Rev. A]
**Exposed pins are available on screw terminals/headers for reuse; core comms and default wheel drive pins remain internal.**

| Pin(s) | Pin Name | Function | Exposed? | Notes |
|--------|----------|----------|----------|-------|
| 0 (RX0) | RX0 | USB Serial | No | Programming/debug only |
| 1 (TX0) | TX0 | USB Serial | No | Programming/debug only |
| 2 (INT0) | M1_ENC_A | Motor 1 Encoder A | No | Default left/right wheel encoder A |
| 3 (INT1) | M2_ENC_A | Motor 2 Encoder A | No | Default left/right wheel encoder A |
| 4 | M1_ENC_B | Motor 1 Encoder B | No | Default wheel encoder B |
| 5 (PWM) | M1_EN | Motor 1 EN | No | Default wheel PWM |
| 6 (PWM) | M2_EN | Motor 2 EN | No | Default wheel PWM |
| 7 | M2_ENC_B | Motor 2 Encoder B | No | Default wheel encoder B |
| 8 | M1_IN1 | Motor 1 IN1 | No | Default wheel direction |
| 9 (PWM) | M3_EN | Motor 3 EN | Yes | PWM-capable |
| 10 (PWM) | M4_EN | Motor 4 EN | Yes | PWM-capable |
| 11 (PWM) | LED_RED | Status LED Red | No | Error/low battery indicator |
| 12 | M2_IN1 | Motor 2 IN1 | No | Default wheel direction |
| 13 | M2_IN2 | Motor 2 IN2 | No | Default wheel direction |
| 14 | ST1_STEP | Stepper 1 STEP | Yes | Stepper control |
| 15 | ST2_STEP | Stepper 2 STEP | Yes | Stepper control |
| 16 (TX2) | TX_RPI | **UART to RPi5** | No | Via level shifter (5V → 3.3V) |
| 17 (RX2) | RX_RPI | **UART from RPi5** | No | Via level shifter (3.3V → 5V) |
| 18 (INT5) | M3_ENC_A | Motor 3 Encoder A | Yes | Interrupt-capable |
| 19 (INT4) | M4_ENC_A | Motor 4 Encoder A | Yes | Interrupt-capable |
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
| 30 | M3_ENC_B | Motor 3 Encoder B | Yes | Quadrature input |
| 31 | M4_ENC_B | Motor 4 Encoder B | Yes | Quadrature input |
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
| A7-A15 | ANALOG_EXP | Analog Expansion | Yes | Available for sensors |

**DC Motor Control Summary (per motor):**
| Motor | EN (PWM) | IN1 | IN2 | Encoder A | Encoder B | Current (CT) |
|-------|----------|-----|-----|-----------|-----------|--------------|
| 1 | Pin 5 | Pin 8 | Pin 43 | Pin 2 (INT0) | Pin 4 | A3 |
| 2 | Pin 6 | Pin 12 | Pin 13 | Pin 3 (INT1) | Pin 7 | A4 |
| 3 | Pin 9 | Pin 34 | Pin 35 | Pin 18 (INT5) | Pin 30 | A5 |
| 4 | Pin 10 | Pin 36 | Pin 37 | Pin 19 (INT4) | Pin 31 | A6 |

**Hardware Interrupts Used:**
- INT0 (pin 2): Motor 1 Encoder A
- INT1 (pin 3): Motor 2 Encoder A
- INT4 (pin 19): Motor 4 Encoder A
- INT5 (pin 18): Motor 3 Encoder A

**Serial Ports:**
- Serial0 (pins 0/1): USB programming/debug
- Serial2 (pins 16/17): Raspberry Pi communication via level shifter
- Serial1 (pins 18/19): NOT AVAILABLE (used for encoder interrupts)
- Serial3 (pins 14/15): NOT AVAILABLE (used for stepper STEP signals)
