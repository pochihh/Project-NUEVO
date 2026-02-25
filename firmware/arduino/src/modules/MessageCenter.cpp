/**
 * @file MessageCenter.cpp
 * @brief Implementation of central message routing (TLV protocol v2.0)
 *
 * Telemetry batching:
 *   sendTelemetry() calls beginFrame() once, appends each due TLV with
 *   addTlvPacket(), then calls sendFrame() — transmitting a single
 *   multi-TLV frame per 100 Hz tick instead of N separate frames.
 *
 * Incoming:
 *   processIncoming() feeds raw bytes into the TLV decoder; decodeCallback()
 *   is invoked synchronously for each complete frame and routes every TLV in
 *   that frame to routeMessage().
 */

#include "MessageCenter.h"
#include "SensorManager.h"
#include "UserIO.h"
#include "../SystemManager.h"
#include "../drivers/DCMotor.h"
#include "../drivers/StepperMotor.h"
#include "../drivers/ServoController.h"
#include <string.h>
#include <math.h>

// External references to motor arrays (defined in arduino.ino)
extern DCMotor      dcMotors[NUM_DC_MOTORS];
extern StepperMotor steppers[NUM_STEPPERS];

// ============================================================================
// STATIC MEMBER INITIALIZATION
// ============================================================================

struct TlvEncodeDescriptor  MessageCenter::encodeDesc_;
struct TlvDecodeDescriptor  MessageCenter::decodeDesc_;
uint8_t                     MessageCenter::txBuffer_[1024];
uint8_t                     MessageCenter::rxBuffer_[512];

uint32_t    MessageCenter::lastHeartbeatMs_   = 0;
uint32_t    MessageCenter::lastCmdMs_         = 0;
bool        MessageCenter::heartbeatValid_    = false;
uint16_t    MessageCenter::heartbeatTimeoutMs_ = HEARTBEAT_TIMEOUT_MS;

float       MessageCenter::wheelDiameterMm_  = DEFAULT_WHEEL_DIAMETER_MM;
float       MessageCenter::wheelBaseMm_      = DEFAULT_WHEEL_BASE_MM;
uint8_t     MessageCenter::motorDirMask_     = 0;
uint8_t     MessageCenter::neoPixelCount_    = NEOPIXEL_COUNT;

float       MessageCenter::odomX_           = 0.0f;
float       MessageCenter::odomY_           = 0.0f;
float       MessageCenter::odomTheta_       = 0.0f;
int32_t     MessageCenter::prevLeftTicks_   = 0;
int32_t     MessageCenter::prevRightTicks_  = 0;

uint16_t    MessageCenter::servoEnabledMask_ = 0;
uint16_t    MessageCenter::loopTimeAvgUs_    = 0;
uint16_t    MessageCenter::loopTimeMaxUs_    = 0;
uint16_t    MessageCenter::uartRxErrors_     = 0;

uint32_t    MessageCenter::lastDCStatusSendMs_    = 0;
uint32_t    MessageCenter::lastStepStatusSendMs_  = 0;
uint32_t    MessageCenter::lastServoStatusSendMs_ = 0;
uint32_t    MessageCenter::lastIMUSendMs_          = 0;
uint32_t    MessageCenter::lastKinematicsSendMs_  = 0;
uint32_t    MessageCenter::lastVoltageSendMs_     = 0;
uint32_t    MessageCenter::lastIOStatusSendMs_    = 0;
uint32_t    MessageCenter::lastStatusSendMs_      = 0;
uint32_t    MessageCenter::lastMagCalSendMs_      = 0;

bool        MessageCenter::pendingMagCal_ = false;
bool        MessageCenter::initialized_  = false;

// ============================================================================
// INITIALIZATION
// ============================================================================

void MessageCenter::init() {
    if (initialized_) return;

    // Open Serial2 for RPi communication
    RPI_SERIAL.begin(RPI_BAUD_RATE);

    // Initialise TX encoder
    initEncodeDescriptor(&encodeDesc_, sizeof(txBuffer_), DEVICE_ID, ENABLE_CRC_CHECK);
    encodeDesc_.buffer = txBuffer_;

    // Initialise RX decoder with callback
    initDecodeDescriptor(&decodeDesc_, sizeof(rxBuffer_), ENABLE_CRC_CHECK, decodeCallback);
    decodeDesc_.buffer = rxBuffer_;

    lastHeartbeatMs_ = millis();
    lastCmdMs_       = millis();
    heartbeatValid_  = false;
    pendingMagCal_   = false;
    SystemManager::requestTransition(SYS_STATE_IDLE);

    initialized_ = true;

#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.println(F("[MessageCenter] Initialized (v2.0, batched TX)"));
#endif
}

// ============================================================================
// FRAME HELPERS
// ============================================================================

void MessageCenter::beginFrame() {
    resetDescriptor(&encodeDesc_);
}

void MessageCenter::sendFrame() {
    // Only send if at least one TLV was appended
    if (encodeDesc_.frameHeader.numTlvs == 0) return;

    int n = wrapupBuffer(&encodeDesc_);
    if (n > 0) {
        RPI_SERIAL.write(txBuffer_, (size_t)n);
    }

#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.print(F("[TX] Frame: "));
    DEBUG_SERIAL.print(encodeDesc_.frameHeader.numTlvs);
    DEBUG_SERIAL.print(F(" TLVs, "));
    DEBUG_SERIAL.print(n);
    DEBUG_SERIAL.println(F(" bytes"));
#endif
}

// ============================================================================
// MESSAGE PROCESSING
// ============================================================================

void MessageCenter::processIncoming() {
    // Feed all available bytes into the decoder.
    // decodeCallback() fires synchronously for each complete frame and
    // routes every TLV in that frame to routeMessage().
    while (RPI_SERIAL.available()) {
        uint8_t byte = (uint8_t)RPI_SERIAL.read();
        decode(&decodeDesc_, &byte, 1);
    }

    checkHeartbeatTimeout();
}

void MessageCenter::sendTelemetry() {
    uint32_t currentMs = millis();

    SystemState state    = SystemManager::getState();
    bool running         = (state == SYS_STATE_RUNNING);
    bool runningOrError  = (state == SYS_STATE_RUNNING ||
                            state == SYS_STATE_ERROR);

    // Open a new frame; all send*() calls below append TLVs to it.
    beginFrame();

    // ---- 100 Hz telemetry (RUNNING only) ----
    if (running) {
        if (currentMs - lastDCStatusSendMs_ >= 10) {
            lastDCStatusSendMs_ = currentMs;
            sendDCStatusAll();
        }

        if (currentMs - lastStepStatusSendMs_ >= 10) {
            lastStepStatusSendMs_ = currentMs;
            sendStepStatusAll();
        }

        if (currentMs - lastKinematicsSendMs_ >= 10) {
            lastKinematicsSendMs_ = currentMs;
            sendSensorKinematics();
        }

        if (currentMs - lastIOStatusSendMs_ >= 10) {
            lastIOStatusSendMs_ = currentMs;
            sendIOStatus();
        }

        if (SensorManager::isIMUAvailable() &&
            currentMs - lastIMUSendMs_ >= 10) {
            lastIMUSendMs_ = currentMs;
            sendSensorIMU();
        }

        // ---- 50 Hz telemetry ----
        if (currentMs - lastServoStatusSendMs_ >= 20) {
            lastServoStatusSendMs_ = currentMs;
            sendServoStatusAll();
        }
    }

    // ---- 10 Hz voltage (RUNNING or ERROR) ----
    if (runningOrError && currentMs - lastVoltageSendMs_ >= 100) {
        lastVoltageSendMs_ = currentMs;
        sendVoltageData();
    }

    // ---- System status: 10 Hz (RUNNING/ERROR), 1 Hz otherwise ----
    uint32_t statusInterval = runningOrError ? 100UL : 1000UL;
    if (currentMs - lastStatusSendMs_ >= statusInterval) {
        lastStatusSendMs_ = currentMs;
        sendSystemStatus();
    }

    // ---- Mag cal status at 10 Hz while sampling ----
    if (SensorManager::getMagCalData().state == MAG_CAL_SAMPLING) {
        if (currentMs - lastMagCalSendMs_ >= 100) {
            lastMagCalSendMs_ = currentMs;
            sendMagCalStatus();
        }
    }

    // ---- Queued immediate mag cal response (STOP / SAVE / APPLY / CLEAR) ----
    if (pendingMagCal_) {
        pendingMagCal_ = false;
        sendMagCalStatus();
    }

    // Transmit the completed frame (no-ops if nothing was appended)
    sendFrame();
}

// ============================================================================
// LIVENESS MONITORING
// ============================================================================

bool MessageCenter::isHeartbeatValid() {
    return heartbeatValid_;
}

uint32_t MessageCenter::getTimeSinceHeartbeat() {
    return millis() - lastHeartbeatMs_;
}

void MessageCenter::checkHeartbeatTimeout() {
    // Only mark heartbeat invalid — SafetyManager::check() responds on next 100 Hz tick
    if (getTimeSinceHeartbeat() > (uint32_t)heartbeatTimeoutMs_) {
        heartbeatValid_ = false;
    }
}

// ============================================================================
// DECODE CALLBACK
// ============================================================================

void MessageCenter::decodeCallback(enum DecodeErrorCode* error,
                                   const struct FrameHeader* frameHeader,
                                   struct TlvHeader* tlvHeaders,
                                   uint8_t** tlvData)
{
    if (*error != NoError) {
        uartRxErrors_++;
#ifdef DEBUG_TLV_PACKETS
        DEBUG_SERIAL.print(F("[RX] Decode error: "));
        DEBUG_SERIAL.println(*error);
#endif
        return;
    }

    // Route every TLV in the frame — supports multi-TLV incoming frames
    for (uint32_t i = 0; i < frameHeader->numTlvs; i++) {
        uint32_t length = tlvHeaders[i].tlvLen;
        if (length > MAX_TLV_PAYLOAD_SIZE) {
            uartRxErrors_++;
#ifdef DEBUG_TLV_PACKETS
            DEBUG_SERIAL.print(F("[RX] Oversized payload, type="));
            DEBUG_SERIAL.println(tlvHeaders[i].tlvType);
#endif
            continue;
        }

        routeMessage(tlvHeaders[i].tlvType, tlvData[i], length);

#ifdef DEBUG_TLV_PACKETS
        DEBUG_SERIAL.print(F("[RX] Type: "));
        DEBUG_SERIAL.print(tlvHeaders[i].tlvType);
        DEBUG_SERIAL.print(F(", Len: "));
        DEBUG_SERIAL.println(length);
#endif
    }
}

// ============================================================================
// MESSAGE ROUTING
// ============================================================================

void MessageCenter::routeMessage(uint32_t type, const uint8_t* payload, uint32_t length) {
    // Any received TLV resets the liveness timer
    lastHeartbeatMs_ = millis();
    heartbeatValid_  = true;

    // Non-heartbeat commands update the command timer
    if (type != SYS_HEARTBEAT) {
        lastCmdMs_ = millis();
    }

    // ---- State-based command gating ----
    SystemState state   = SystemManager::getState();
    // Motor commands only accepted in RUNNING state
    bool allowMotorCmds = (state == SYS_STATE_RUNNING);
    // Config only accepted in IDLE state
    bool allowConfig    = (state == SYS_STATE_IDLE);
    // SYS_CMD and SYS_HEARTBEAT accepted in any state
    // SYS_SET_PID accepted in IDLE or RUNNING
    bool allowPID       = (state == SYS_STATE_IDLE || state == SYS_STATE_RUNNING);

    switch (type) {
        // ---- System messages (always accepted) ----
        case SYS_HEARTBEAT:
            if (length == sizeof(PayloadHeartbeat))
                handleHeartbeat((const PayloadHeartbeat*)payload);
            break;

        case SYS_CMD:
            if (length == sizeof(PayloadSysCmd))
                handleSysCmd((const PayloadSysCmd*)payload);
            break;

        case SYS_CONFIG:
            if (allowConfig && length == sizeof(PayloadSysConfig))
                handleSysConfig((const PayloadSysConfig*)payload);
            break;

        case SYS_SET_PID:
            if (allowPID && length == sizeof(PayloadSetPID))
                handleSetPID((const PayloadSetPID*)payload);
            break;

        // ---- DC motor commands (RUNNING only) ----
        case DC_ENABLE:
            if (length == sizeof(PayloadDCEnable))
                handleDCEnable((const PayloadDCEnable*)payload);
            break;

        case DC_SET_POSITION:
            if (allowMotorCmds && length == sizeof(PayloadDCSetPosition))
                handleDCSetPosition((const PayloadDCSetPosition*)payload);
            break;

        case DC_SET_VELOCITY:
            if (allowMotorCmds && length == sizeof(PayloadDCSetVelocity))
                handleDCSetVelocity((const PayloadDCSetVelocity*)payload);
            break;

        case DC_SET_PWM:
            if (allowMotorCmds && length == sizeof(PayloadDCSetPWM))
                handleDCSetPWM((const PayloadDCSetPWM*)payload);
            break;

        // ---- Stepper commands (RUNNING only) ----
        case STEP_ENABLE:
            if (length == sizeof(PayloadStepEnable))
                handleStepEnable((const PayloadStepEnable*)payload);
            break;

        case STEP_SET_PARAMS:
            if (allowMotorCmds && length == sizeof(PayloadStepSetParams))
                handleStepSetParams((const PayloadStepSetParams*)payload);
            break;

        case STEP_MOVE:
            if (allowMotorCmds && length == sizeof(PayloadStepMove))
                handleStepMove((const PayloadStepMove*)payload);
            break;

        case STEP_HOME:
            if (allowMotorCmds && length == sizeof(PayloadStepHome))
                handleStepHome((const PayloadStepHome*)payload);
            break;

        // ---- Servo commands ----
        case SERVO_ENABLE:
            if (length == sizeof(PayloadServoEnable))
                handleServoEnable((const PayloadServoEnable*)payload);
            break;

        case SERVO_SET:
            if (length == sizeof(PayloadServoSetSingle)) {
                handleServoSet((const PayloadServoSetSingle*)payload);
            } else if (length == sizeof(PayloadServoSetBulk)) {
                // Bulk servo update
                const PayloadServoSetBulk* b = (const PayloadServoSetBulk*)payload;
                if ((uint16_t)b->startChannel + b->count <= NUM_SERVO_CHANNELS) {
                    ServoController::setMultiplePositionsUs(
                        b->startChannel, b->count, b->pulseUs);
                }
            }
            break;

        // ---- User I/O commands ----
        case IO_SET_LED:
            if (length == sizeof(PayloadSetLED))
                handleSetLED((const PayloadSetLED*)payload);
            break;

        case IO_SET_NEOPIXEL:
            if (length == sizeof(PayloadSetNeoPixel))
                handleSetNeoPixel((const PayloadSetNeoPixel*)payload);
            break;

        // ---- Magnetometer calibration (IDLE only) ----
        case SENSOR_MAG_CAL_CMD:
            if (allowConfig && length == sizeof(PayloadMagCalCmd))
                handleMagCalCmd((const PayloadMagCalCmd*)payload);
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
    // Liveness timer already updated in routeMessage()
    (void)payload;
#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.print(F("[RX] Heartbeat ts="));
    DEBUG_SERIAL.println(payload->timestamp);
#endif
}

void MessageCenter::handleSysCmd(const PayloadSysCmd* payload) {
    switch ((SysCmdType)payload->command) {
        case SYS_CMD_START:
            if (SystemManager::requestTransition(SYS_STATE_RUNNING)) {
#ifdef DEBUG_TLV_PACKETS
                DEBUG_SERIAL.println(F("[SYS] → RUNNING"));
#endif
            }
            break;

        case SYS_CMD_STOP:
            if (SystemManager::requestTransition(SYS_STATE_IDLE)) {
                disableAllActuators();
#ifdef DEBUG_TLV_PACKETS
                DEBUG_SERIAL.println(F("[SYS] → IDLE (stopped)"));
#endif
            }
            break;

        case SYS_CMD_RESET:
            if (SystemManager::requestTransition(SYS_STATE_IDLE)) {
                disableAllActuators();
#ifdef DEBUG_TLV_PACKETS
                DEBUG_SERIAL.println(F("[SYS] → IDLE (reset)"));
#endif
            }
            break;

        case SYS_CMD_ESTOP:
            disableAllActuators();
            SystemManager::requestTransition(SYS_STATE_ESTOP);
#ifdef DEBUG_TLV_PACKETS
            DEBUG_SERIAL.println(F("[SYS] → ESTOP"));
#endif
            break;

        default:
            break;
    }
}

void MessageCenter::handleSysConfig(const PayloadSysConfig* payload) {
    // IDLE state only — enforced by routeMessage()
    if (payload->wheelDiameterMm != 0.0f)
        wheelDiameterMm_ = payload->wheelDiameterMm;

    if (payload->wheelBaseMm != 0.0f)
        wheelBaseMm_ = payload->wheelBaseMm;

    if (payload->neoPixelCount != 0)
        neoPixelCount_ = payload->neoPixelCount;

    if (payload->heartbeatTimeoutMs != 0)
        heartbeatTimeoutMs_ = payload->heartbeatTimeoutMs;

    // Apply direction mask changes (store; motors read this in sendSystemStatus)
    if (payload->motorDirChangeMask != 0) {
        motorDirMask_ = (motorDirMask_ & ~payload->motorDirChangeMask) |
                        (payload->motorDirMask & payload->motorDirChangeMask);
    }

    if (payload->resetOdometry) {
        resetOdometry();
    }
}

void MessageCenter::handleSetPID(const PayloadSetPID* payload) {
    if (payload->motorId >= NUM_DC_MOTORS) return;

    DCMotor& motor = dcMotors[payload->motorId];

    if (payload->loopType == 0) {
        motor.setPositionPID(payload->kp, payload->ki, payload->kd);
    } else if (payload->loopType == 1) {
        motor.setVelocityPID(payload->kp, payload->ki, payload->kd);
    }
}

// ============================================================================
// DC MOTOR MESSAGE HANDLERS
// ============================================================================

void MessageCenter::handleDCEnable(const PayloadDCEnable* payload) {
    if (payload->motorId >= NUM_DC_MOTORS) return;

    DCMotor& motor = dcMotors[payload->motorId];

    if (payload->mode == DC_MODE_DISABLED) {
        motor.disable();
    } else {
        motor.enable((DCMotorMode)payload->mode);
    }
}

void MessageCenter::handleDCSetPosition(const PayloadDCSetPosition* payload) {
    if (payload->motorId >= NUM_DC_MOTORS) return;

    dcMotors[payload->motorId].setTargetPosition(payload->targetTicks);
    // maxVelTicks (velocity cap) is not exposed via current API; ignored.
}

void MessageCenter::handleDCSetVelocity(const PayloadDCSetVelocity* payload) {
    if (payload->motorId >= NUM_DC_MOTORS) return;

    dcMotors[payload->motorId].setTargetVelocity((float)payload->targetTicks);
}

void MessageCenter::handleDCSetPWM(const PayloadDCSetPWM* payload) {
    if (payload->motorId >= NUM_DC_MOTORS) return;

    dcMotors[payload->motorId].setDirectPWM(payload->pwm);
}

// ============================================================================
// STEPPER MOTOR MESSAGE HANDLERS
// ============================================================================

void MessageCenter::handleStepEnable(const PayloadStepEnable* payload) {
    if (payload->stepperId >= NUM_STEPPERS) return;

    if (payload->enable) {
        steppers[payload->stepperId].enable();
    } else {
        steppers[payload->stepperId].disable();
    }
}

void MessageCenter::handleStepSetParams(const PayloadStepSetParams* payload) {
    if (payload->stepperId >= NUM_STEPPERS) return;

    StepperMotor& s = steppers[payload->stepperId];
    s.setMaxVelocity((uint16_t)payload->maxVelocity);
    s.setAcceleration((uint16_t)payload->acceleration);
}

void MessageCenter::handleStepMove(const PayloadStepMove* payload) {
    if (payload->stepperId >= NUM_STEPPERS) return;

    StepperMotor& s = steppers[payload->stepperId];

    if (payload->moveType == STEP_MOVE_ABSOLUTE) {
        s.moveToPosition(payload->target);
    } else if (payload->moveType == STEP_MOVE_RELATIVE) {
        s.moveSteps(payload->target);
    }
}

void MessageCenter::handleStepHome(const PayloadStepHome* payload) {
    if (payload->stepperId >= NUM_STEPPERS) return;

    StepperMotor& s = steppers[payload->stepperId];

    // Apply home velocity if specified
    if (payload->homeVelocity > 0) {
        s.setMaxVelocity((uint16_t)payload->homeVelocity);
    }

    s.home(payload->direction);
}

// ============================================================================
// SERVO MESSAGE HANDLERS
// ============================================================================

void MessageCenter::handleServoEnable(const PayloadServoEnable* payload) {
    if (payload->channel == 0xFF) {
        // All channels
        if (payload->enable) {
            servoEnabledMask_ = 0xFFFF;
            ServoController::enable();
        } else {
            servoEnabledMask_ = 0;
            ServoController::disable();
        }
    } else if (payload->channel < NUM_SERVO_CHANNELS) {
        if (payload->enable) {
            servoEnabledMask_ |= (uint16_t)(1u << payload->channel);
            // Ensure global outputs are enabled
            if (!ServoController::isEnabled()) {
                ServoController::enable();
            }
        } else {
            servoEnabledMask_ &= (uint16_t)~(1u << payload->channel);
            ServoController::setChannelOff(payload->channel);
        }
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
    } else if (payload->index < neoPixelCount_) {
        UserIO::setNeoPixelColor(payload->red, payload->green, payload->blue);
    }
}

// ============================================================================
// MAGNETOMETER CALIBRATION HANDLER
// ============================================================================

void MessageCenter::handleMagCalCmd(const PayloadMagCalCmd* payload) {
    // IDLE state only — enforced by routeMessage()
    switch ((MagCalCmdType)payload->command) {
        case MAG_CAL_START:
            SensorManager::startMagCalibration();
            lastMagCalSendMs_ = 0;  // Force immediate first status send
            break;

        case MAG_CAL_STOP:
            SensorManager::cancelMagCalibration();
            pendingMagCal_ = true;  // Queue final status for next telemetry frame
            break;

        case MAG_CAL_SAVE:
            SensorManager::saveMagCalibration();
            pendingMagCal_ = true;
            break;

        case MAG_CAL_APPLY:
            SensorManager::applyMagCalibration(
                payload->offsetX, payload->offsetY, payload->offsetZ);
            pendingMagCal_ = true;
            break;

        case MAG_CAL_CLEAR:
            SensorManager::clearMagCalibration();
            pendingMagCal_ = true;
            break;

        default:
            break;
    }
}

// ============================================================================
// TELEMETRY APPENDERS
// ============================================================================
// Each function appends one TLV to the current frame via addTlvPacket().
// Must be called after beginFrame() and before sendFrame().

void MessageCenter::sendDCStatusAll() {
    PayloadDCStatusAll payload;

    for (uint8_t i = 0; i < 4; i++) {
        DCMotorStatus& s = payload.motors[i];
        s.mode       = (uint8_t)dcMotors[i].getMode();
        s.faultFlags = 0;
        s.position   = dcMotors[i].getPosition();
        s.velocity   = (int32_t)dcMotors[i].getVelocity();
        s.targetPos  = dcMotors[i].getTargetPosition();
        s.targetVel  = (int32_t)dcMotors[i].getTargetVelocity();
        s.pwmOutput  = dcMotors[i].getPWMOutput();
        s.currentMa  = dcMotors[i].getCurrent();
        s.posKp      = dcMotors[i].getPosKp();
        s.posKi      = dcMotors[i].getPosKi();
        s.posKd      = dcMotors[i].getPosKd();
        s.velKp      = dcMotors[i].getVelKp();
        s.velKi      = dcMotors[i].getVelKi();
        s.velKd      = dcMotors[i].getVelKd();
    }

    addTlvPacket(&encodeDesc_, DC_STATUS_ALL, sizeof(payload), &payload);
}

void MessageCenter::sendStepStatusAll() {
    PayloadStepStatusAll payload;

    for (uint8_t i = 0; i < 4; i++) {
        StepperStatus& s = payload.steppers[i];
        s.enabled       = steppers[i].isEnabled() ? 1 : 0;
        s.motionState   = (uint8_t)steppers[i].getState();
        s.limitHit      = 0;
        s.reserved      = 0;
        s.commandedCount = steppers[i].getPosition();
        s.targetCount   = steppers[i].getTargetPosition();
        s.currentSpeed  = steppers[i].getCurrentSpeed();
        s.maxSpeed      = steppers[i].getMaxVelocity();
        s.acceleration  = steppers[i].getAcceleration();
    }

    addTlvPacket(&encodeDesc_, STEP_STATUS_ALL, sizeof(payload), &payload);
}

void MessageCenter::sendServoStatusAll() {
    PayloadServoStatusAll payload;

    payload.pca9685Connected = 1;  // Assumed connected when SERVO_CONTROLLER_ENABLED
    payload.pca9685Error     = 0;
    payload.enabledMask      = servoEnabledMask_;

    for (uint8_t i = 0; i < 16; i++) {
        if (servoEnabledMask_ & (1u << i)) {
            payload.pulseUs[i] = ServoController::getPositionUs(i);
        } else {
            payload.pulseUs[i] = 0;
        }
    }

    addTlvPacket(&encodeDesc_, SERVO_STATUS_ALL, sizeof(payload), &payload);
}

void MessageCenter::sendSensorIMU() {
    if (!SensorManager::isIMUAvailable()) return;

    PayloadSensorIMU payload;

    SensorManager::getQuaternion(
        payload.quatW, payload.quatX, payload.quatY, payload.quatZ);
    SensorManager::getEarthAcceleration(
        payload.earthAccX, payload.earthAccY, payload.earthAccZ);

    payload.rawAccX  = SensorManager::getRawAccX();
    payload.rawAccY  = SensorManager::getRawAccY();
    payload.rawAccZ  = SensorManager::getRawAccZ();
    payload.rawGyroX = SensorManager::getRawGyrX();
    payload.rawGyroY = SensorManager::getRawGyrY();
    payload.rawGyroZ = SensorManager::getRawGyrZ();
    payload.magX     = SensorManager::getRawMagX();
    payload.magY     = SensorManager::getRawMagY();
    payload.magZ     = SensorManager::getRawMagZ();

    payload.magCalibrated = SensorManager::isMagCalibrated() ? 1 : 0;
    payload.reserved      = 0;
    payload.timestamp     = micros();

    addTlvPacket(&encodeDesc_, SENSOR_IMU, sizeof(payload), &payload);
}

void MessageCenter::sendSensorKinematics() {
    updateOdometry();

    PayloadSensorKinematics payload;
    payload.x     = odomX_;
    payload.y     = odomY_;
    payload.theta = odomTheta_;

    // Instantaneous velocities in mm/s (from motor velocity estimates)
    if (wheelDiameterMm_ > 0.0f) {
        static const uint8_t kEncModes[4] = {
            ENCODER_1_MODE, ENCODER_2_MODE, ENCODER_3_MODE, ENCODER_4_MODE
        };
        const float mmPerTickL =
            (PI * wheelDiameterMm_) / (float)(ENCODER_PPR * kEncModes[ODOM_LEFT_MOTOR]);
        const float mmPerTickR =
            (PI * wheelDiameterMm_) / (float)(ENCODER_PPR * kEncModes[ODOM_RIGHT_MOTOR]);

        float vLeft  = dcMotors[ODOM_LEFT_MOTOR].getVelocity() * mmPerTickL;
        float vRight = dcMotors[ODOM_RIGHT_MOTOR].getVelocity() * mmPerTickR;

        payload.vx     = (vLeft + vRight) * 0.5f;
        payload.vy     = 0.0f;  // Always 0 for differential drive
        payload.vTheta = (wheelBaseMm_ > 0.0f)
                         ? (vRight - vLeft) / wheelBaseMm_
                         : 0.0f;
    } else {
        payload.vx = payload.vy = payload.vTheta = 0.0f;
    }

    payload.timestamp = micros();

    addTlvPacket(&encodeDesc_, SENSOR_KINEMATICS, sizeof(payload), &payload);
}

void MessageCenter::sendVoltageData() {
    PayloadSensorVoltage payload;

    payload.batteryMv  = (uint16_t)(SensorManager::getBatteryVoltage() * 1000.0f);
    payload.rail5vMv   = (uint16_t)(SensorManager::get5VRailVoltage()  * 1000.0f);
    payload.servoRailMv = (uint16_t)(SensorManager::getServoVoltage()  * 1000.0f);
    payload.reserved   = 0;

    addTlvPacket(&encodeDesc_, SENSOR_VOLTAGE, sizeof(payload), &payload);
}

void MessageCenter::sendIOStatus() {
    // Variable-length payload: 10 bytes fixed + 3 bytes per NeoPixel
    uint8_t buf[10 + 3 * NEOPIXEL_COUNT];
    memset(buf, 0, sizeof(buf));

    PayloadIOStatus* p = (PayloadIOStatus*)buf;
    p->buttonMask        = UserIO::getButtonStates();
    p->ledBrightness[0]  = 0;  // LED brightness getters not yet exposed in UserIO
    p->ledBrightness[1]  = 0;
    p->ledBrightness[2]  = 0;
    p->reserved          = 0;
    p->timestamp         = millis();
    // NeoPixel RGB bytes appended as zero (no getter available yet)

    uint8_t sendLen = 10 + (uint8_t)(neoPixelCount_ * 3);
    if (sendLen > sizeof(buf)) sendLen = sizeof(buf);

    addTlvPacket(&encodeDesc_, IO_STATUS, sendLen, buf);
}

void MessageCenter::sendSystemStatus() {
    PayloadSystemStatus payload;
    memset(&payload, 0, sizeof(payload));

    payload.firmwareMajor = (FIRMWARE_VERSION >> 24) & 0xFF;
    payload.firmwareMinor = (FIRMWARE_VERSION >> 16) & 0xFF;
    payload.firmwarePatch = (FIRMWARE_VERSION >> 8)  & 0xFF;
    payload.state         = (uint8_t)SystemManager::getState();
    payload.uptimeMs      = millis();
    payload.lastRxMs      = getTimeSinceHeartbeat();
    payload.lastCmdMs     = millis() - lastCmdMs_;
    payload.batteryMv     = (uint16_t)(SensorManager::getBatteryVoltage() * 1000.0f);
    payload.rail5vMv      = (uint16_t)(SensorManager::get5VRailVoltage()  * 1000.0f);

    // Error flags
    payload.errorFlags = ERR_NONE;
    if (SensorManager::isBatteryLow())        payload.errorFlags |= ERR_UNDERVOLTAGE;
    if (SensorManager::isBatteryOvervoltage()) payload.errorFlags |= ERR_OVERVOLTAGE;
    if (!heartbeatValid_)                      payload.errorFlags |= ERR_LIVENESS_LOST;

    // Attached sensors bitmask: bit0=IMU, bit1=Lidar, bit2=Ultrasonic
    if (SensorManager::isIMUAvailable())           payload.attachedSensors |= 0x01;
    if (SensorManager::getLidarCount() > 0)        payload.attachedSensors |= 0x02;
    if (SensorManager::getUltrasonicCount() > 0)   payload.attachedSensors |= 0x04;

    payload.freeSram         = getFreeRAM();
    payload.loopTimeAvgUs    = loopTimeAvgUs_;
    payload.loopTimeMaxUs    = loopTimeMaxUs_;
    payload.uartRxErrors     = uartRxErrors_;
    payload.wheelDiameterMm  = wheelDiameterMm_;
    payload.wheelBaseMm      = wheelBaseMm_;
    payload.motorDirMask     = motorDirMask_;
    payload.neoPixelCount    = neoPixelCount_;
    payload.heartbeatTimeoutMs = heartbeatTimeoutMs_;
    payload.limitSwitchMask  = 0;

    // 0xFF = no home limit GPIO configured for this stepper
    memset(payload.stepperHomeLimitGpio, 0xFF, sizeof(payload.stepperHomeLimitGpio));

    addTlvPacket(&encodeDesc_, SYS_STATUS, sizeof(payload), &payload);
}

void MessageCenter::sendMagCalStatus() {
    const MagCalData& cal = SensorManager::getMagCalData();

    PayloadMagCalStatus payload;
    payload.state        = (uint8_t)cal.state;
    payload.sampleCount  = cal.sampleCount;
    payload.reserved     = 0;
    payload.minX         = cal.minX;
    payload.maxX         = cal.maxX;
    payload.minY         = cal.minY;
    payload.maxY         = cal.maxY;
    payload.minZ         = cal.minZ;
    payload.maxZ         = cal.maxZ;
    payload.offsetX      = cal.offsetX;
    payload.offsetY      = cal.offsetY;
    payload.offsetZ      = cal.offsetZ;
    payload.savedToEeprom = cal.savedToEeprom ? 1 : 0;
    memset(payload.reserved2, 0, sizeof(payload.reserved2));

    addTlvPacket(&encodeDesc_, SENSOR_MAG_CAL_STATUS, sizeof(payload), &payload);
}

// ============================================================================
// HELPERS
// ============================================================================

void MessageCenter::disableAllActuators() {
    for (uint8_t i = 0; i < NUM_DC_MOTORS; i++) {
        if (dcMotors[i].isEnabled()) dcMotors[i].disable();
    }
    for (uint8_t i = 0; i < NUM_STEPPERS; i++) {
        steppers[i].stop();
    }
    ServoController::disable();
    servoEnabledMask_ = 0;
}

void MessageCenter::updateOdometry() {
    // Requires wheel geometry to be configured
    if (wheelDiameterMm_ <= 0.0f || wheelBaseMm_ <= 0.0f) return;

    // Encoder modes indexed by motor ID — resolves automatically with ODOM_*_MOTOR config
    static const uint8_t kEncModes[4] = {
        ENCODER_1_MODE, ENCODER_2_MODE, ENCODER_3_MODE, ENCODER_4_MODE
    };

    int32_t leftTicks  = dcMotors[ODOM_LEFT_MOTOR].getPosition();
    int32_t rightTicks = dcMotors[ODOM_RIGHT_MOTOR].getPosition();

    int32_t dL = leftTicks  - prevLeftTicks_;
    int32_t dR = rightTicks - prevRightTicks_;
    prevLeftTicks_  = leftTicks;
    prevRightTicks_ = rightTicks;

    if (dL == 0 && dR == 0) return;  // No movement, skip trig

    const float mmPerTickL =
        (PI * wheelDiameterMm_) / (float)(ENCODER_PPR * kEncModes[ODOM_LEFT_MOTOR]);
    const float mmPerTickR =
        (PI * wheelDiameterMm_) / (float)(ENCODER_PPR * kEncModes[ODOM_RIGHT_MOTOR]);

    float dLeft   = (float)dL * mmPerTickL;
    float dRight  = (float)dR * mmPerTickR;
    float dCenter = (dLeft + dRight) * 0.5f;
    float dTheta  = (dRight - dLeft) / wheelBaseMm_;

    // Midpoint heading for integration (reduces discretization error)
    float headingMid = odomTheta_ + dTheta * 0.5f;
    odomX_     += dCenter * cosf(headingMid);
    odomY_     += dCenter * sinf(headingMid);
    odomTheta_ += dTheta;
}

void MessageCenter::resetOdometry() {
    odomX_     = 0.0f;
    odomY_     = 0.0f;
    odomTheta_ = 0.0f;
    prevLeftTicks_  = dcMotors[ODOM_LEFT_MOTOR].getPosition();
    prevRightTicks_ = dcMotors[ODOM_RIGHT_MOTOR].getPosition();
}

uint16_t MessageCenter::getFreeRAM() {
    extern int __heap_start, *__brkval;
    uint8_t v;
    const uint8_t* stackPtr = &v;
    const uint8_t* heapEnd  = (__brkval == 0)
                               ? (const uint8_t*)&__heap_start
                               : (const uint8_t*)__brkval;
    if (stackPtr > heapEnd) {
        return (uint16_t)(stackPtr - heapEnd);
    }
    return 0;
}
