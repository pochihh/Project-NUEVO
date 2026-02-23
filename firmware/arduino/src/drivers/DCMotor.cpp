/**
 * @file DCMotor.cpp
 * @brief Implementation of DC motor driver with cascade PID control
 */

#include "DCMotor.h"
#include "../config.h"
#include "../pins.h"

// ============================================================================
// PID CONTROLLER IMPLEMENTATION
// ============================================================================

PIDController::PIDController()
    : kp_(0.0f)
    , ki_(0.0f)
    , kd_(0.0f)
    , integral_(0.0f)
    , prevMeasurement_(0.0f)
    , output_(0.0f)
    , minOutput_(-255.0f)
    , maxOutput_(255.0f)
{
}

void PIDController::setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::setLimits(float min, float max) {
    minOutput_ = min;
    maxOutput_ = max;
}

float PIDController::compute(float setpoint, float measurement, float dt) {
    // Proportional term
    float error = setpoint - measurement;
    float pTerm = kp_ * error;

    // Integral term with anti-windup
    integral_ += error * dt;

    // Clamp integral to prevent windup
    float maxIntegral = maxOutput_ / (ki_ + 0.001f);  // Avoid division by zero
    float minIntegral = minOutput_ / (ki_ + 0.001f);
    if (integral_ > maxIntegral) integral_ = maxIntegral;
    if (integral_ < minIntegral) integral_ = minIntegral;

    float iTerm = ki_ * integral_;

    // Derivative term (derivative-on-measurement to avoid setpoint kick)
    float derivative = -(measurement - prevMeasurement_) / dt;
    float dTerm = kd_ * derivative;

    // Compute total output
    output_ = pTerm + iTerm + dTerm;

    // Clamp output to limits
    if (output_ > maxOutput_) output_ = maxOutput_;
    if (output_ < minOutput_) output_ = minOutput_;

    // Store measurement for next iteration
    prevMeasurement_ = measurement;

    return output_;
}

void PIDController::reset() {
    integral_ = 0.0f;
    prevMeasurement_ = 0.0f;
    output_ = 0.0f;
}

// ============================================================================
// DC MOTOR IMPLEMENTATION
// ============================================================================

DCMotor::DCMotor()
    : motorId_(0)
    , pinEN_(0)
    , pinIN1_(0)
    , pinIN2_(0)
    , pinCT_(0)
    , invertDir_(false)
    , hasCurrentSense_(false)
    , mode_(DC_MODE_DISABLED)
    , targetPosition_(0)
    , targetVelocity_(0.0f)
    , directPwm_(0)
    , pwmOutput_(0)
    , currentMa_(-1)
    , maPerVolt_(1000.0f)
    , encoder_(nullptr)
    , velocityEst_(nullptr)
    , lastUpdateUs_(0)
{
}

void DCMotor::init(uint8_t motorId, IEncoderCounter *encoder, IVelocityEstimator *velocityEst, bool invertDir) {
    motorId_ = motorId;
    encoder_ = encoder;
    velocityEst_ = velocityEst;
    invertDir_ = invertDir;
    lastUpdateUs_ = micros();
}

void DCMotor::setPins(uint8_t pinEN, uint8_t pinIN1, uint8_t pinIN2) {
    pinEN_ = pinEN;
    pinIN1_ = pinIN1;
    pinIN2_ = pinIN2;

    // Configure pins
    pinMode(pinEN_, OUTPUT);
    pinMode(pinIN1_, OUTPUT);
    pinMode(pinIN2_, OUTPUT);

    // Initialize to stopped state.
    // Do NOT call analogWrite(pinEN_, 0) here — that would reconfigure the
    // timer (Timer3 for M1_EN, Timer4 for M2_EN) and corrupt ISRScheduler's
    // Fast PWM setup.  pinMode() already drives the pin LOW by default.
    digitalWrite(pinIN1_, LOW);
    digitalWrite(pinIN2_, LOW);
}

void DCMotor::setCurrentPin(uint8_t pinCT, float maPerVolt) {
    pinCT_ = pinCT;
    maPerVolt_ = maPerVolt;
    hasCurrentSense_ = true;

    // Configure as analog input (no pinMode needed for analog pins)
    currentMa_ = -1;  // Will be updated in update()
}

void DCMotor::enable(DCMotorMode mode) {
    mode_ = mode;

    // Reset PID controllers when enabling
    positionPID_.reset();
    velocityPID_.reset();

    // Reset velocity estimator
    if (velocityEst_) {
        velocityEst_->reset();
    }

    lastUpdateUs_ = micros();

#ifdef DEBUG_MOTOR_CONTROL
    DEBUG_SERIAL.print(F("[Motor "));
    DEBUG_SERIAL.print(motorId_);
    DEBUG_SERIAL.print(F("] Enabled in mode "));
    DEBUG_SERIAL.println(mode_);
#endif
}

void DCMotor::disable() {
    mode_ = DC_MODE_DISABLED;
    pwmOutput_ = 0;
    setPWM(0);

#ifdef DEBUG_MOTOR_CONTROL
    DEBUG_SERIAL.print(F("[Motor "));
    DEBUG_SERIAL.print(motorId_);
    DEBUG_SERIAL.println(F("] Disabled"));
#endif
}

bool DCMotor::isEnabled() const {
    return mode_ != DC_MODE_DISABLED;
}

void DCMotor::setTargetPosition(int32_t position) {
    targetPosition_ = position;
}

void DCMotor::setTargetVelocity(float velocity) {
    targetVelocity_ = velocity;
}

void DCMotor::setPositionPID(float kp, float ki, float kd) {
    positionPID_.setGains(kp, ki, kd);

    // Position PID outputs velocity setpoint in ticks/sec
    // Limit to reasonable velocity range
    positionPID_.setLimits(-10000.0f, 10000.0f);
}

void DCMotor::setVelocityPID(float kp, float ki, float kd) {
    velocityPID_.setGains(kp, ki, kd);

    // Velocity PID outputs PWM value
    velocityPID_.setLimits(-255.0f, 255.0f);
}

void DCMotor::setDirectPWM(int16_t pwm) {
    directPwm_ = pwm;
}

void DCMotor::update() {
    if (mode_ == DC_MODE_DISABLED) {
        return;  // Motor is disabled, do nothing
    }

    // PWM mode: apply stored direct PWM immediately, skip PID
    if (mode_ == DC_MODE_PWM) {
        pwmOutput_ = directPwm_;
        setPWM(pwmOutput_);
        return;
    }

    if (!encoder_ || !velocityEst_) {
        // Missing encoder or velocity estimator - cannot control
        return;
    }

    // Compute time step
    uint32_t currentUs = micros();
    float dt = (currentUs - lastUpdateUs_) / 1000000.0f;  // Convert to seconds
    lastUpdateUs_ = currentUs;

    // Update velocity estimator
    int32_t currentPosition = encoder_->getCount();
    uint32_t lastEdgeUs = encoder_->getLastEdgeUs();
    velocityEst_->update(lastEdgeUs, currentPosition);

    float currentVelocity = velocityEst_->getVelocity();

    // Read motor current if current sensing is configured
    if (hasCurrentSense_) {
        // Multiple ADC reads and average to reduce noise
        const uint8_t NUM_SAMPLES = 4;
        uint32_t adcSum = 0;

        for (uint8_t i = 0; i < NUM_SAMPLES; i++) {
            adcSum += analogRead(pinCT_);
        }

        int rawADC = adcSum / NUM_SAMPLES;

        // Convert to voltage (assuming 5V reference)
        float voltage = (rawADC / 1023.0f) * 5.0f;

        // Convert to milliamps using scaling factor
        int16_t newCurrentMa = (int16_t)(voltage * maPerVolt_);

        // Apply deadband: ignore small currents when motor is stopped
        if (pwmOutput_ == 0 && abs(newCurrentMa) < 50) {
            newCurrentMa = 0;
        }

        // Simple low-pass filter (exponential moving average)
        // Alpha = 0.3 means 30% new value, 70% old value
        const float ALPHA = 0.3f;
        currentMa_ = (int16_t)((ALPHA * newCurrentMa) + ((1.0f - ALPHA) * currentMa_));
    }

    // Compute control output based on mode
    float velocitySetpoint = targetVelocity_;

    if (mode_ == DC_MODE_POSITION) {
        // Position mode: Outer loop (position PID) computes velocity setpoint
        velocitySetpoint = positionPID_.compute(
            (float)targetPosition_,
            (float)currentPosition,
            dt
        );

#ifdef DEBUG_MOTOR_CONTROL
        if (motorId_ == 0) {  // Only debug motor 0 to reduce spam
            DEBUG_SERIAL.print(F("[M0] Pos: "));
            DEBUG_SERIAL.print(currentPosition);
            DEBUG_SERIAL.print(F(" / "));
            DEBUG_SERIAL.print(targetPosition_);
            DEBUG_SERIAL.print(F(" -> VelSP: "));
            DEBUG_SERIAL.println(velocitySetpoint);
        }
#endif
    }

    // Inner loop: Velocity PID computes PWM output
    float pwmOutput = velocityPID_.compute(
        velocitySetpoint,
        currentVelocity,
        dt
    );

    pwmOutput_ = (int16_t)pwmOutput;

    // Apply PWM to motor
    setPWM(pwmOutput_);

#ifdef DEBUG_MOTOR_CONTROL
    if (mode_ == DC_MODE_VELOCITY && motorId_ == 0) {  // Debug velocity mode
        DEBUG_SERIAL.print(F("[M0] Vel: "));
        DEBUG_SERIAL.print(currentVelocity);
        DEBUG_SERIAL.print(F(" / "));
        DEBUG_SERIAL.print(targetVelocity_);
        DEBUG_SERIAL.print(F(" -> PWM: "));
        DEBUG_SERIAL.println(pwmOutput_);
    }
#endif
}

int32_t DCMotor::getPosition() const {
    if (encoder_) {
        return encoder_->getCount();
    }
    return 0;
}

float DCMotor::getVelocity() const {
    if (velocityEst_) {
        return velocityEst_->getVelocity();
    }
    return 0.0f;
}

void DCMotor::setPWM(int16_t pwm) {
    // Clamp PWM to valid range
    if (pwm > 255) pwm = 255;
    if (pwm < -255) pwm = -255;

    // Apply motor direction inversion if configured
    if (invertDir_) {
        pwm = -pwm;
    }

    // Determine direction and extract unsigned speed
    uint8_t speed;
    if (pwm > 0) {
        digitalWrite(pinIN1_, HIGH);
        digitalWrite(pinIN2_, LOW);
        speed = (uint8_t)pwm;
    } else if (pwm < 0) {
        digitalWrite(pinIN1_, LOW);
        digitalWrite(pinIN2_, HIGH);
        speed = (uint8_t)(-pwm);
    } else {
        // Brake: both direction pins LOW
        digitalWrite(pinIN1_, LOW);
        digitalWrite(pinIN2_, LOW);
        speed = 0;
    }

    // Write speed to hardware PWM.
    //
    // Motors 0 and 1 (M1_EN / M2_EN) share their timer with an ISR, so
    // analogWrite() is forbidden (it would reconfigure the timer and break
    // the ISR rate).  Write OCRnx directly using the macros from pins.h.
    // The OCR value is scaled from 0–255 to 0–ICRn to match the Fast PWM
    // resolution.
    //
    // Motors 2 and 3 (M3_EN / M4_EN) are on Timer2 which is not used by
    // any ISR, so analogWrite() is safe for those.

#if defined(PIN_M1_EN_OCR)
    if (motorId_ == 0) {
        PIN_M1_EN_OCR = ((uint16_t)speed * PIN_M1_EN_ICR) / 255;
        return;
    }
#endif

#if defined(PIN_M2_EN_OCR)
    if (motorId_ == 1) {
        PIN_M2_EN_OCR = ((uint16_t)speed * PIN_M2_EN_ICR) / 255;
        return;
    }
#endif

    // M3 / M4: Timer2 is free — analogWrite() is safe
    analogWrite(pinEN_, speed);
}
