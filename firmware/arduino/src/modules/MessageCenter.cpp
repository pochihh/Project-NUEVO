/**
 * @file MessageCenter.cpp
 * @brief Implementation of central message routing
 */

#include "MessageCenter.h"
#include "SensorManager.h"
#include "UserIO.h"
#include "../drivers/DCMotor.h"
#include "../drivers/StepperMotor.h"
#include "../drivers/ServoController.h"
#include <string.h>

// External references to motor arrays (defined in arduino.ino)
extern DCMotor dcMotors[NUM_DC_MOTORS];
extern StepperMotor steppers[NUM_STEPPERS];

// ============================================================================
// STATIC MEMBER INITIALIZATION
// ============================================================================

UARTDriver MessageCenter::uart_;
uint32_t MessageCenter::lastHeartbeatMs_ = 0;
bool MessageCenter::heartbeatValid_ = false;
uint32_t MessageCenter::lastEncoderSendMs_ = 0;
uint32_t MessageCenter::lastVoltageSendMs_ = 0;
uint32_t MessageCenter::lastDCStatusSendMs_ = 0;
uint32_t MessageCenter::lastStepStatusSendMs_ = 0;
uint32_t MessageCenter::lastButtonSendMs_ = 0;
uint32_t MessageCenter::lastStatusSendMs_ = 0;
bool MessageCenter::initialized_ = false;

// ============================================================================
// INITIALIZATION
// ============================================================================

void MessageCenter::init() {
    if (initialized_) return;

    // Initialize UART driver
    uart_.init();

    // Reset heartbeat timer
    lastHeartbeatMs_ = millis();
    heartbeatValid_ = false;

    // Reset telemetry timers
    lastEncoderSendMs_ = 0;
    lastVoltageSendMs_ = 0;
    lastDCStatusSendMs_ = 0;
    lastStepStatusSendMs_ = 0;
    lastButtonSendMs_ = 0;
    lastStatusSendMs_ = 0;

    initialized_ = true;

#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.println(F("[MessageCenter] Initialized"));
#endif
}

// ============================================================================
// MESSAGE PROCESSING
// ============================================================================

void MessageCenter::processIncoming() {
    uint32_t msgType;
    uint8_t payload[MAX_TLV_PAYLOAD_SIZE];
    uint32_t length;

    // Process all available messages
    while (uart_.receive(&msgType, payload, &length)) {
        // Valid message received - route it
        routeMessage(msgType, payload, length);
    }

    // Check for heartbeat timeout
    checkHeartbeatTimeout();
}

void MessageCenter::sendTelemetry() {
    uint32_t currentMs = millis();

    // Send encoder data at 100Hz
    if (currentMs - lastEncoderSendMs_ >= (1000 / UART_COMMS_FREQ_HZ)) {
        lastEncoderSendMs_ = currentMs;
        sendEncoderData();
    }

    // Send voltage data at 50Hz
    if (currentMs - lastVoltageSendMs_ >= (1000 / SENSOR_UPDATE_FREQ_HZ)) {
        lastVoltageSendMs_ = currentMs;
        sendVoltageData();
    }

    // Send DC motor status at 50Hz
    if (currentMs - lastDCStatusSendMs_ >= (1000 / SENSOR_UPDATE_FREQ_HZ)) {
        lastDCStatusSendMs_ = currentMs;
        sendDCStatus();
    }

    // Send stepper status at 20Hz
    if (currentMs - lastStepStatusSendMs_ >= (1000 / USER_IO_FREQ_HZ)) {
        lastStepStatusSendMs_ = currentMs;
        sendStepperStatus();
    }

    // Send button states when changed
    static uint16_t prevButtonMask = 0;
    static uint8_t prevLimitMask = 0;
    uint16_t buttonMask = UserIO::getButtonStates();
    uint8_t limitMask = (uint8_t)UserIO::getLimitStates();

    if (buttonMask != prevButtonMask || limitMask != prevLimitMask) {
        prevButtonMask = buttonMask;
        prevLimitMask = limitMask;
        sendButtonStates();
    }

    // Send system status at 1Hz
    if (currentMs - lastStatusSendMs_ >= 1000) {
        lastStatusSendMs_ = currentMs;
        sendSystemStatus();
    }
}

// ============================================================================
// HEARTBEAT MONITORING
// ============================================================================

bool MessageCenter::isHeartbeatValid() {
    return heartbeatValid_;
}

uint32_t MessageCenter::getTimeSinceHeartbeat() {
    return millis() - lastHeartbeatMs_;
}

void MessageCenter::updateHeartbeat() {
    lastHeartbeatMs_ = millis();
    heartbeatValid_ = true;
}

void MessageCenter::checkHeartbeatTimeout() {
    if (getTimeSinceHeartbeat() > HEARTBEAT_TIMEOUT_MS) {
        if (heartbeatValid_) {
            // Heartbeat just timed out
            heartbeatValid_ = false;

#ifdef DEBUG_TLV_PACKETS
            DEBUG_SERIAL.println(F("[MessageCenter] HEARTBEAT TIMEOUT!"));
#endif

            // Safety: Disable all DC motors
            for (uint8_t i = 0; i < NUM_DC_MOTORS; i++) {
                if (dcMotors[i].isEnabled()) {
                    dcMotors[i].disable();
                }
            }

            // Safety: Stop all steppers
            for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
                steppers[i].stop();
            }
        }
    }
}

// ============================================================================
// MESSAGE ROUTING
// ============================================================================

void MessageCenter::routeMessage(uint32_t type, const uint8_t* payload, uint32_t length) {
    switch (type) {
        case SYS_HEARTBEAT:
            if (length == sizeof(PayloadHeartbeat)) {
                handleHeartbeat((const PayloadHeartbeat*)payload);
            }
            break;

        case SYS_SET_PID:
            if (length == sizeof(PayloadSetPID)) {
                handleSetPID((const PayloadSetPID*)payload);
            }
            break;

        case DC_ENABLE:
            if (length == sizeof(PayloadDCEnable)) {
                handleDCEnable((const PayloadDCEnable*)payload);
            }
            break;

        case DC_SET_POSITION:
            if (length == sizeof(PayloadDCSetPosition)) {
                handleDCSetPosition((const PayloadDCSetPosition*)payload);
            }
            break;

        case DC_SET_VELOCITY:
            if (length == sizeof(PayloadDCSetVelocity)) {
                handleDCSetVelocity((const PayloadDCSetVelocity*)payload);
            }
            break;

        case STEP_ENABLE:
            if (length == sizeof(PayloadStepEnable)) {
                handleStepEnable((const PayloadStepEnable*)payload);
            }
            break;

        case STEP_SET_ACCEL:
            if (length == sizeof(PayloadStepSetAccel)) {
                handleStepSetAccel((const PayloadStepSetAccel*)payload);
            }
            break;

        case STEP_SET_VEL:
            if (length == sizeof(PayloadStepSetVel)) {
                handleStepSetVel((const PayloadStepSetVel*)payload);
            }
            break;

        case STEP_MOVE:
            if (length == sizeof(PayloadStepMove)) {
                handleStepMove((const PayloadStepMove*)payload);
            }
            break;

        case STEP_HOME:
            if (length == sizeof(PayloadStepHome)) {
                handleStepHome((const PayloadStepHome*)payload);
            }
            break;

        case SERVO_ENABLE:
            if (length == sizeof(PayloadServoEnable)) {
                handleServoEnable((const PayloadServoEnable*)payload);
            }
            break;

        case SERVO_SET:
            if (length == sizeof(PayloadServoSetSingle)) {
                handleServoSet((const PayloadServoSetSingle*)payload);
            }
            break;

        case IO_SET_LED:
            if (length == sizeof(PayloadSetLED)) {
                handleSetLED((const PayloadSetLED*)payload);
            }
            break;

        case IO_SET_NEOPIXEL:
            if (length == sizeof(PayloadSetNeoPixel)) {
                handleSetNeoPixel((const PayloadSetNeoPixel*)payload);
            }
            break;

        default:
#ifdef DEBUG_TLV_PACKETS
            DEBUG_SERIAL.print(F("[RX] Unknown type: "));
            DEBUG_SERIAL.println(type);
#endif
            break;
    }
}

// ============================================================================
// SYSTEM MESSAGE HANDLERS
// ============================================================================

void MessageCenter::handleHeartbeat(const PayloadHeartbeat* payload) {
    updateHeartbeat();

#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.print(F("[RX] Heartbeat: "));
    DEBUG_SERIAL.println(payload->timestamp);
#endif
}

void MessageCenter::handleSetPID(const PayloadSetPID* payload) {
    if (payload->motorType == 0 && payload->motorId < NUM_DC_MOTORS) {
        // DC motor PID tuning
        DCMotor& motor = dcMotors[payload->motorId];

        if (payload->loopType == 0) {
            // Position PID
            motor.setPositionPID(payload->kp, payload->ki, payload->kd);
        } else if (payload->loopType == 1) {
            // Velocity PID
            motor.setVelocityPID(payload->kp, payload->ki, payload->kd);
        }
    }
}

// ============================================================================
// DC MOTOR MESSAGE HANDLERS
// ============================================================================

void MessageCenter::handleDCEnable(const PayloadDCEnable* payload) {
    if (payload->motorId >= NUM_DC_MOTORS) return;

    DCMotor& motor = dcMotors[payload->motorId];

    if (payload->enable) {
        motor.enable((DCMotorMode)payload->mode);
    } else {
        motor.disable();
    }
}

void MessageCenter::handleDCSetPosition(const PayloadDCSetPosition* payload) {
    if (payload->motorId >= NUM_DC_MOTORS) return;

    DCMotor& motor = dcMotors[payload->motorId];
    motor.setTargetPosition(payload->targetPos);
}

void MessageCenter::handleDCSetVelocity(const PayloadDCSetVelocity* payload) {
    if (payload->motorId >= NUM_DC_MOTORS) return;

    DCMotor& motor = dcMotors[payload->motorId];
    motor.setTargetVelocity(payload->targetVel);
}

// ============================================================================
// STEPPER MOTOR MESSAGE HANDLERS
// ============================================================================

void MessageCenter::handleStepEnable(const PayloadStepEnable* payload) {
    if (payload->stepperId >= NUM_STEPPERS) return;

    StepperMotor& stepper = steppers[payload->stepperId];

    if (payload->enable) {
        stepper.enable();
    } else {
        stepper.disable();
    }
}

void MessageCenter::handleStepSetAccel(const PayloadStepSetAccel* payload) {
    if (payload->stepperId >= NUM_STEPPERS) return;

    StepperMotor& stepper = steppers[payload->stepperId];
    stepper.setAcceleration(payload->accel);
}

void MessageCenter::handleStepSetVel(const PayloadStepSetVel* payload) {
    if (payload->stepperId >= NUM_STEPPERS) return;

    StepperMotor& stepper = steppers[payload->stepperId];
    stepper.setMaxVelocity(payload->maxVelocity);
}

void MessageCenter::handleStepMove(const PayloadStepMove* payload) {
    if (payload->stepperId >= NUM_STEPPERS) return;

    StepperMotor& stepper = steppers[payload->stepperId];

    if (payload->moveType == STEP_MOVE_ABSOLUTE) {
        stepper.moveToPosition(payload->target);
    } else if (payload->moveType == STEP_MOVE_RELATIVE) {
        stepper.moveSteps(payload->target);
    }
}

void MessageCenter::handleStepHome(const PayloadStepHome* payload) {
    if (payload->stepperId >= NUM_STEPPERS) return;

    StepperMotor& stepper = steppers[payload->stepperId];
    stepper.home(payload->direction);
}

// ============================================================================
// SERVO MESSAGE HANDLERS
// ============================================================================

void MessageCenter::handleServoEnable(const PayloadServoEnable* payload) {
    if (payload->enable) {
        ServoController::enable();
    } else {
        ServoController::disable();
    }
}

void MessageCenter::handleServoSet(const PayloadServoSetSingle* payload) {
    if (payload->channel >= NUM_SERVO_CHANNELS) return;

    ServoController::setPositionUs(payload->channel, payload->pulseUs);
}

// ============================================================================
// USER I/O MESSAGE HANDLERS
// ============================================================================

void MessageCenter::handleSetLED(const PayloadSetLED* payload) {
    if (payload->ledId >= LED_COUNT) return;

    UserIO::setLED((LEDId)payload->ledId,
                   (LEDMode)payload->mode,
                   payload->brightness,
                   payload->periodMs);
}

void MessageCenter::handleSetNeoPixel(const PayloadSetNeoPixel* payload) {
    if (payload->index == 0xFF) {
        // Set all pixels
        UserIO::setNeoPixelColor(payload->red, payload->green, payload->blue);
    } else if (payload->index < NEOPIXEL_COUNT) {
        // Set specific pixel (for future multi-pixel support)
        UserIO::setNeoPixelColor(payload->red, payload->green, payload->blue);
    }
}

// ============================================================================
// TELEMETRY SENDERS
// ============================================================================

void MessageCenter::sendEncoderData() {
    PayloadSensorEncoder payload;

    for (uint8_t i = 0; i < NUM_DC_MOTORS; i++) {
        payload.position[i] = dcMotors[i].getPosition();
        payload.velocity[i] = dcMotors[i].getVelocity();
    }
    payload.timestamp = micros();

    uart_.send(SENSOR_ENCODER, &payload, sizeof(payload));
}

void MessageCenter::sendVoltageData() {
    PayloadSensorVoltage payload;

    payload.batteryMv = (uint16_t)(SensorManager::getBatteryVoltage() * 1000);
    payload.rail5vMv = (uint16_t)(SensorManager::get5VRailVoltage() * 1000);
    payload.servoRailMv = (uint16_t)(SensorManager::getServoVoltage() * 1000);
    payload.reserved = 0;

    uart_.send(SENSOR_VOLTAGE, &payload, sizeof(payload));
}

void MessageCenter::sendDCStatus() {
    for (uint8_t i = 0; i < NUM_DC_MOTORS; i++) {
        PayloadDCStatus payload;

        payload.motorId = i;
        payload.enabled = dcMotors[i].isEnabled() ? 1 : 0;
        payload.mode = (uint8_t)dcMotors[i].getMode();
        payload.faultFlags = 0;  // No fault detection yet
        payload.position = dcMotors[i].getPosition();
        payload.velocity = dcMotors[i].getVelocity();
        payload.targetPos = dcMotors[i].getTargetPosition();
        payload.targetVel = dcMotors[i].getTargetVelocity();
        payload.pwmOutput = dcMotors[i].getPWMOutput();
        payload.currentMa = -1;  // Current sensing not implemented

        uart_.send(DC_STATUS, &payload, sizeof(payload));
    }
}

void MessageCenter::sendStepperStatus() {
    for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
        PayloadStepStatus payload;

        payload.stepperId = i;
        payload.enabled = steppers[i].isEnabled() ? 1 : 0;
        payload.state = (uint8_t)steppers[i].getState();
        payload.limitHit = 0;  // TODO: Read from limit switches
        payload.position = steppers[i].getPosition();
        payload.targetPos = steppers[i].getTargetPosition();
        payload.currentVel = 0;  // Velocity not tracked by StepperMotor
        payload.stepsRemaining = steppers[i].getStepsRemaining();

        uart_.send(STEP_STATUS, &payload, sizeof(payload));
    }
}

void MessageCenter::sendButtonStates() {
    PayloadButtonState buttonPayload;
    buttonPayload.buttonMask = UserIO::getButtonStates();
    buttonPayload.changedMask = 0;  // TODO: Track changes
    buttonPayload.timestamp = millis();
    uart_.send(IO_BUTTON_STATE, &buttonPayload, sizeof(buttonPayload));

    PayloadLimitState limitPayload;
    limitPayload.limitMask = (uint8_t)UserIO::getLimitStates();
    limitPayload.changedMask = 0;  // TODO: Track changes
    limitPayload.reserved = 0;
    limitPayload.timestamp = millis();
    uart_.send(IO_LIMIT_STATE, &limitPayload, sizeof(limitPayload));
}

void MessageCenter::sendSystemStatus() {
    PayloadSystemStatus payload;

    payload.uptimeMs = millis();
    payload.lastHeartbeatMs = getTimeSinceHeartbeat();
    payload.batteryMv = (uint16_t)(SensorManager::getBatteryVoltage() * 1000);
    payload.rail5vMv = (uint16_t)(SensorManager::get5VRailVoltage() * 1000);
    payload.servoRailMv = (uint16_t)(SensorManager::getServoVoltage() * 1000);

    // Set error flags
    payload.errorFlags = ERR_NONE;
    if (SensorManager::isBatteryLow()) {
        payload.errorFlags |= ERR_LOW_BATTERY;
    }
    if (!heartbeatValid_) {
        payload.errorFlags |= ERR_HEARTBEAT_LOST;
    }

    // Set motor enable masks
    payload.motorEnableMask = 0;
    for (uint8_t i = 0; i < NUM_DC_MOTORS; i++) {
        if (dcMotors[i].isEnabled()) {
            payload.motorEnableMask |= (1 << i);
        }
    }

    payload.stepperEnableMask = 0;
    for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
        if (steppers[i].isEnabled()) {
            payload.stepperEnableMask |= (1 << i);
        }
    }

    payload.reserved = 0;

    uart_.send(SYS_STATUS, &payload, sizeof(payload));
}
