/**
 * @file test_current_sensing.ino
 * @brief Test sketch for DC motor current sensing (Phase 3 - Current Feedback)
 *
 * This test verifies current sensor readings by:
 * 1. Applying various PWM levels (0 to 255)
 * 2. Reading raw ADC values from CT pins
 * 3. Converting to voltage and current (mA)
 * 4. Testing both forward and reverse directions
 * 5. Testing all 4 motors individually
 * 6. Displaying all values for debugging
 *
 * Expected Behavior:
 * - Raw ADC should change with motor load
 * - Voltage should correlate with motor current
 * - Current (mA) should increase with PWM level
 * - Bipolar sensors show ~2.5V at zero current
 * - Unipolar sensors show ~0V at zero current
 *
 * How to Test:
 * 1. Upload this sketch
 * 2. Open Serial Monitor @ 115200 baud
 * 3. Select motor to test (1-4)
 * 4. Test will step through PWM levels
 * 5. Observe ADC, voltage, and current readings
 * 6. Manually stall motor to see current spike
 * 7. Type '1'-'4' to switch motors, 'f' for forward, 'r' for reverse
 *
 * Hardware Setup:
 * - Connect motors 1-4 with current sensors
 * - Motor 1: CT on A3, Motor 2: CT on A4, Motor 3: CT on A5, Motor 4: CT on A6
 * - Connect motor power supply
 * - Ensure CT sensor output is connected to ADC pin
 * - Check if your sensor is bipolar (2.5V offset) or unipolar (0V offset)
 *
 * Safety:
 * - Motor will run at various speeds
 * - Press 's' to stop motor immediately
 * - Watch for overcurrent conditions
 *
 * Verification:
 * 1. At PWM=0: Current should be near zero
 * 2. At PWM=255: Current should be maximum
 * 3. Raw ADC should increase with load
 * 4. Values should be consistent and stable
 */

#include "src/config.h"
#include "src/pins.h"

// ============================================================================
// CONFIGURATION
// ============================================================================

// Motor pin arrays (indexed 0-3 for motors 1-4)
const uint8_t MOTOR_PIN_EN[4]  = {PIN_M1_EN,  PIN_M2_EN,  PIN_M3_EN,  PIN_M4_EN};
const uint8_t MOTOR_PIN_IN1[4] = {PIN_M1_IN1, PIN_M2_IN1, PIN_M3_IN1, PIN_M4_IN1};
const uint8_t MOTOR_PIN_IN2[4] = {PIN_M1_IN2, PIN_M2_IN2, PIN_M3_IN2, PIN_M4_IN2};
const uint8_t MOTOR_PIN_CT[4]  = {PIN_M1_CT,  PIN_M2_CT,  PIN_M3_CT,  PIN_M4_CT};

// Current motor being tested (0-3 for motors 1-4)
uint8_t currentMotorIndex = 0;

// Current pin assignments (updated when motor is selected)
uint8_t testPinEN;
uint8_t testPinIN1;
uint8_t testPinIN2;
uint8_t testPinCT;

// Current sensor configuration
const float CT_MA_PER_VOLT = CURRENT_SENSE_MA_PER_VOLT;  // From config.h: 6451.6

// Test mode
enum TestMode {
    MODE_STOPPED,
    MODE_FORWARD,
    MODE_REVERSE,
    MODE_AUTO_SWEEP
};

TestMode currentMode = MODE_STOPPED;
uint8_t currentPWM = 0;

// Auto sweep parameters
const uint32_t SWEEP_INTERVAL_MS = 2000;  // 2 seconds per PWM level
uint32_t lastSweepTime = 0;
uint8_t sweepPWMlevels[] = {0, 50, 100, 150, 200, 255};
uint8_t sweepIndex = 0;

bool useBipolarOffset = false;  // Set to true if using bipolar sensor (2.5V at 0A)
bool useFiltering = true;      // Enable noise filtering

// Filtering state
int16_t filteredCurrentMa = 0;

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

void setMotorPWM(int16_t pwm);
void printCurrentReadings(bool applyOffset);

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

/**
 * @brief Select motor and update pin assignments
 */
void selectMotor(uint8_t motorIndex) {
    // Stop current motor
    setMotorPWM(0);
    currentMode = MODE_STOPPED;

    // Update motor index
    currentMotorIndex = motorIndex;

    // Update pin assignments
    testPinEN  = MOTOR_PIN_EN[motorIndex];
    testPinIN1 = MOTOR_PIN_IN1[motorIndex];
    testPinIN2 = MOTOR_PIN_IN2[motorIndex];
    testPinCT  = MOTOR_PIN_CT[motorIndex];

    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(F("========================================"));
    DEBUG_SERIAL.print(F("  Selected Motor "));
    DEBUG_SERIAL.println(motorIndex + 1);
    DEBUG_SERIAL.println(F("========================================"));
    DEBUG_SERIAL.print(F("  EN  Pin: "));
    DEBUG_SERIAL.println(testPinEN);
    DEBUG_SERIAL.print(F("  IN1 Pin: "));
    DEBUG_SERIAL.println(testPinIN1);
    DEBUG_SERIAL.print(F("  IN2 Pin: "));
    DEBUG_SERIAL.println(testPinIN2);
    DEBUG_SERIAL.print(F("  CT  Pin: A"));
    DEBUG_SERIAL.println(testPinCT - A0);
    DEBUG_SERIAL.println(F("========================================"));
    DEBUG_SERIAL.println();

    // Read initial values
    DEBUG_SERIAL.println(F("Initial readings (motor stopped):"));
    for (int i = 0; i < 5; i++) {
        printCurrentReadings(useBipolarOffset);
        delay(100);
    }
    DEBUG_SERIAL.println();
}

/**
 * @brief Set motor PWM and direction
 */
void setMotorPWM(int16_t pwm) {
    if (pwm > 0) {
        // Forward
        digitalWrite(testPinIN1, HIGH);
        digitalWrite(testPinIN2, LOW);
        analogWrite(testPinEN, pwm);
    } else if (pwm < 0) {
        // Reverse
        digitalWrite(testPinIN1, LOW);
        digitalWrite(testPinIN2, HIGH);
        analogWrite(testPinEN, -pwm);
    } else {
        // Stop
        digitalWrite(testPinIN1, LOW);
        digitalWrite(testPinIN2, LOW);
        analogWrite(testPinEN, 0);
    }
}

/**
 * @brief Read current sensor and return all diagnostic values
 */
void readCurrentSensor(int *rawADC, float *voltage, int16_t *currentMa, bool applyOffset = false) {
    // Multiple ADC reads and average to reduce noise (if filtering enabled)
    uint32_t adcSum = 0;
    const uint8_t NUM_SAMPLES = useFiltering ? 8 : 1;

    for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
        adcSum += analogRead(testPinCT);
    }

    *rawADC = adcSum / NUM_SAMPLES;

    // Convert to voltage (5V reference)
    *voltage = (*rawADC / 1023.0f) * 5.0f;

    // Apply offset if using bipolar sensor (2.5V at zero current)
    float voltageOffset = *voltage;
    if (applyOffset) {
        voltageOffset = *voltage - 2.5f;
    }

    // Convert to milliamps
    int16_t newCurrentMa = (int16_t)(voltageOffset * CT_MA_PER_VOLT);

    // Apply deadband when motor is stopped (if filtering enabled)
    if (useFiltering && currentPWM == 0 && abs(newCurrentMa) < 50) {
        newCurrentMa = 0;
    }

    // Low-pass filter (exponential moving average) if enabled
    if (useFiltering) {
        const float ALPHA = 0.3f;  // 30% new, 70% old
        filteredCurrentMa = (int16_t)((ALPHA * newCurrentMa) + ((1.0f - ALPHA) * filteredCurrentMa));
        *currentMa = filteredCurrentMa;
    } else {
        *currentMa = newCurrentMa;
    }
}

/**
 * @brief Print current sensor readings
 */
void printCurrentReadings(bool applyOffset = false) {
    int rawADC;
    float voltage;
    int16_t currentMa;

    readCurrentSensor(&rawADC, &voltage, &currentMa, applyOffset);

    DEBUG_SERIAL.print(F("M"));
    DEBUG_SERIAL.print(currentMotorIndex + 1);
    DEBUG_SERIAL.print(F(" | PWM="));
    DEBUG_SERIAL.print(currentPWM);
    DEBUG_SERIAL.print(F("  |  ADC="));
    DEBUG_SERIAL.print(rawADC);
    DEBUG_SERIAL.print(F("  |  V="));
    DEBUG_SERIAL.print(voltage, 3);
    DEBUG_SERIAL.print(F("V  |  I="));
    DEBUG_SERIAL.print(currentMa);
    DEBUG_SERIAL.print(F("mA"));

    if (currentMode == MODE_FORWARD) {
        DEBUG_SERIAL.print(F("  [FWD]"));
    } else if (currentMode == MODE_REVERSE) {
        DEBUG_SERIAL.print(F("  [REV]"));
    } else {
        DEBUG_SERIAL.print(F("  [STOP]"));
    }

    if (useFiltering) {
        DEBUG_SERIAL.print(F(" [FILT]"));
    }

    DEBUG_SERIAL.println();
}

/**
 * @brief Print help menu
 */
void printMenu() {
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(F("========================================"));
    DEBUG_SERIAL.println(F("  Motor Current Sensing Test"));
    DEBUG_SERIAL.println(F("========================================"));
    DEBUG_SERIAL.println(F("Motor Selection:"));
    DEBUG_SERIAL.println(F("  1-4 - Select motor to test"));
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(F("Motor Control:"));
    DEBUG_SERIAL.println(F("  f - Forward direction"));
    DEBUG_SERIAL.println(F("  r - Reverse direction"));
    DEBUG_SERIAL.println(F("  s - Stop motor"));
    DEBUG_SERIAL.println(F("  a - Auto sweep PWM (0->255)"));
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(F("PWM Level:"));
    DEBUG_SERIAL.println(F("  0 - PWM=0   (0%)"));
    DEBUG_SERIAL.println(F("  5 - PWM=128 (50%)"));
    DEBUG_SERIAL.println(F("  9 - PWM=255 (100%)"));
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(F("Configuration:"));
    DEBUG_SERIAL.println(F("  u - Toggle bipolar offset (2.5V)"));
    DEBUG_SERIAL.println(F("  n - Toggle noise filtering"));
    DEBUG_SERIAL.println(F("  h - Show this menu"));
    DEBUG_SERIAL.println(F("========================================"));
    DEBUG_SERIAL.println();
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Initialize Debug Serial
    DEBUG_SERIAL.begin(DEBUG_BAUD_RATE);
    while (!DEBUG_SERIAL && millis() < 2000);

    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(F("========================================"));
    DEBUG_SERIAL.println(F("  Current Sensing Test - All Motors"));
    DEBUG_SERIAL.println(F("========================================"));
    DEBUG_SERIAL.print(F("[Setup] Current sensor scaling: "));
    DEBUG_SERIAL.print(CT_MA_PER_VOLT);
    DEBUG_SERIAL.println(F(" mA/V"));
    DEBUG_SERIAL.println(F("[Setup] CT formula: V = I(A) × 0.155"));
    DEBUG_SERIAL.println(F("[Setup] Noise filtering: ENABLED"));
    DEBUG_SERIAL.println(F("  - 8x ADC averaging"));
    DEBUG_SERIAL.println(F("  - Low-pass filter (Alpha=0.3)"));
    DEBUG_SERIAL.println(F("  - 50mA deadband when stopped"));
    DEBUG_SERIAL.println();

    // Initialize all motor pins
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(MOTOR_PIN_EN[i], OUTPUT);
        pinMode(MOTOR_PIN_IN1[i], OUTPUT);
        pinMode(MOTOR_PIN_IN2[i], OUTPUT);

        // Initialize to stopped state
        digitalWrite(MOTOR_PIN_IN1[i], LOW);
        digitalWrite(MOTOR_PIN_IN2[i], LOW);
        analogWrite(MOTOR_PIN_EN[i], 0);
    }

    DEBUG_SERIAL.println(F("[Setup] All motor pins initialized"));

    // Select motor 1 by default
    selectMotor(0);

    printMenu();
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
    static uint32_t lastPrintTime = 0;
    uint32_t currentTime = millis();

    // Print readings at 10Hz
    if (currentTime - lastPrintTime >= 100) {
        lastPrintTime = currentTime;
        printCurrentReadings(useBipolarOffset);
    }

    // Auto sweep mode
    if (currentMode == MODE_AUTO_SWEEP) {
        if (currentTime - lastSweepTime >= SWEEP_INTERVAL_MS) {
            lastSweepTime = currentTime;
            sweepIndex++;

            if (sweepIndex >= sizeof(sweepPWMlevels)) {
                // End of sweep
                sweepIndex = 0;
                currentMode = MODE_STOPPED;
                currentPWM = 0;
                setMotorPWM(0);
                DEBUG_SERIAL.println(F("\n*** Auto sweep complete ***\n"));
            } else {
                currentPWM = sweepPWMlevels[sweepIndex];
                setMotorPWM(currentPWM);
                DEBUG_SERIAL.print(F("\n*** Sweep step "));
                DEBUG_SERIAL.print(sweepIndex);
                DEBUG_SERIAL.print(F("/"));
                DEBUG_SERIAL.print(sizeof(sweepPWMlevels) - 1);
                DEBUG_SERIAL.print(F(": PWM="));
                DEBUG_SERIAL.print(currentPWM);
                DEBUG_SERIAL.println(F(" ***\n"));
            }
        }
    }

    // Process serial commands
    if (DEBUG_SERIAL.available()) {
        char cmd = DEBUG_SERIAL.read();

        switch (cmd) {
            // Motor selection
            case '1':
                selectMotor(0);
                break;
            case '2':
                selectMotor(1);
                break;
            case '3':
                selectMotor(2);
                break;
            case '4':
                selectMotor(3);
                break;

            // Motor control
            case 'f':
            case 'F':
                currentMode = MODE_FORWARD;
                if (currentPWM == 0) currentPWM = 128;  // Default to 50%
                setMotorPWM(currentPWM);
                DEBUG_SERIAL.print(F("\n*** M"));
                DEBUG_SERIAL.print(currentMotorIndex + 1);
                DEBUG_SERIAL.print(F(" FORWARD at PWM="));
                DEBUG_SERIAL.print(currentPWM);
                DEBUG_SERIAL.println(F(" ***\n"));
                break;

            case 'r':
            case 'R':
                currentMode = MODE_REVERSE;
                if (currentPWM == 0) currentPWM = 128;
                setMotorPWM(-currentPWM);
                DEBUG_SERIAL.print(F("\n*** M"));
                DEBUG_SERIAL.print(currentMotorIndex + 1);
                DEBUG_SERIAL.print(F(" REVERSE at PWM="));
                DEBUG_SERIAL.print(currentPWM);
                DEBUG_SERIAL.println(F(" ***\n"));
                break;

            case 's':
            case 'S':
                currentMode = MODE_STOPPED;
                currentPWM = 0;
                setMotorPWM(0);
                DEBUG_SERIAL.print(F("\n*** M"));
                DEBUG_SERIAL.print(currentMotorIndex + 1);
                DEBUG_SERIAL.println(F(" STOPPED ***\n"));
                break;

            case 'a':
            case 'A':
                currentMode = MODE_AUTO_SWEEP;
                sweepIndex = 0;
                currentPWM = sweepPWMlevels[0];
                lastSweepTime = millis();
                setMotorPWM(currentPWM);
                DEBUG_SERIAL.print(F("\n*** M"));
                DEBUG_SERIAL.print(currentMotorIndex + 1);
                DEBUG_SERIAL.println(F(" AUTO SWEEP STARTED ***\n"));
                break;

            case 'u':
            case 'U':
                useBipolarOffset = !useBipolarOffset;
                DEBUG_SERIAL.print(F("\n*** Bipolar offset: "));
                DEBUG_SERIAL.print(useBipolarOffset ? F("ENABLED") : F("DISABLED"));
                DEBUG_SERIAL.println(F(" (2.5V) ***\n"));
                break;

            case 'n':
            case 'N':
                useFiltering = !useFiltering;
                filteredCurrentMa = 0;  // Reset filter state
                DEBUG_SERIAL.print(F("\n*** Noise filtering: "));
                DEBUG_SERIAL.print(useFiltering ? F("ENABLED") : F("DISABLED"));
                DEBUG_SERIAL.println(F(" ***"));
                if (useFiltering) {
                    DEBUG_SERIAL.println(F("  - 8x ADC averaging"));
                    DEBUG_SERIAL.println(F("  - Exponential filter (Alpha=0.3)"));
                    DEBUG_SERIAL.println(F("  - 50mA deadband when stopped"));
                }
                DEBUG_SERIAL.println();
                break;

            case 'h':
            case 'H':
                printMenu();
                break;

            // PWM level shortcuts
            case '0':
                {
                    currentPWM = 0;
                    if (currentMode != MODE_STOPPED) {
                        if (currentMode == MODE_FORWARD) {
                            setMotorPWM(currentPWM);
                        } else if (currentMode == MODE_REVERSE) {
                            setMotorPWM(-currentPWM);
                        }
                    }
                    DEBUG_SERIAL.println(F("\n*** PWM=0 ***\n"));
                }
                break;

            case '5':
                {
                    currentPWM = 128;
                    if (currentMode == MODE_STOPPED) {
                        currentMode = MODE_FORWARD;
                    }
                    if (currentMode == MODE_FORWARD) {
                        setMotorPWM(currentPWM);
                    } else if (currentMode == MODE_REVERSE) {
                        setMotorPWM(-currentPWM);
                    }
                    DEBUG_SERIAL.println(F("\n*** PWM=128 (50%) ***\n"));
                }
                break;

            case '6': case '7': case '8': case '9':
                {
                    uint8_t level = cmd - '0';
                    currentPWM = map(level, 0, 9, 0, 255);

                    if (currentMode == MODE_STOPPED) {
                        currentMode = MODE_FORWARD;
                    }

                    if (currentMode == MODE_FORWARD) {
                        setMotorPWM(currentPWM);
                    } else if (currentMode == MODE_REVERSE) {
                        setMotorPWM(-currentPWM);
                    }

                    DEBUG_SERIAL.print(F("\n*** PWM level "));
                    DEBUG_SERIAL.print(level);
                    DEBUG_SERIAL.print(F(" ("));
                    DEBUG_SERIAL.print(currentPWM);
                    DEBUG_SERIAL.println(F(") ***\n"));
                }
                break;

            default:
                // Ignore other characters
                break;
        }
    }
}
