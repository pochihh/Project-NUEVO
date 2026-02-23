/**
 * @file MessageCenter.h
 * @brief Central message routing and TLV communication handler (v2.0)
 *
 * This module manages all UART communication with the Raspberry Pi:
 * - Receives and parses incoming TLV messages (v2.0 protocol)
 * - Routes commands to appropriate subsystems (motors, servos, LEDs, IMU)
 * - Generates and sends outgoing telemetry at appropriate rates
 * - Monitors heartbeat and enforces liveness timeout
 *
 * Message Flow:
 *   RPi → UART → MessageCenter → Parse TLV → Route to module
 *   Module → Generate TLV → MessageCenter → UART → RPi
 *
 * Safety:
 * - Liveness timeout: heartbeatValid_ goes false; SafetyManager responds on next 100 Hz check
 * - State machine now owned by SystemManager; MessageCenter routes transitions via requestTransition()
 *
 * Usage:
 *   MessageCenter::init();
 *
 *   // In scheduler task @ 100Hz:
 *   MessageCenter::processIncoming();
 *   MessageCenter::sendTelemetry();
 */

#ifndef MESSAGECENTER_H
#define MESSAGECENTER_H

#include <Arduino.h>
#include <stdint.h>
#include "../drivers/UARTDriver.h"
#include "../messages/TLV_TypeDefs.h"
#include "../messages/TLV_Payloads.h"
#include "../config.h"

// ============================================================================
// MESSAGE CENTER CLASS (Static)
// ============================================================================

/**
 * @brief Central TLV message routing and communication handler
 *
 * Static class providing:
 * - TLV v2.0 message parsing and routing
 * - Firmware state machine management
 * - Telemetry generation and transmission
 * - Liveness monitoring and safety timeout
 */
class MessageCenter {
public:
    /**
     * @brief Initialize message center
     *
     * Initializes UART driver and message buffers.
     * Sets firmware state to SYS_STATE_IDLE.
     * Must be called once in setup().
     */
    static void init();

    // ========================================================================
    // MESSAGE PROCESSING
    // ========================================================================

    /**
     * @brief Process incoming messages from UART
     *
     * Reads available bytes from UART, parses TLV messages,
     * and routes commands to appropriate modules.
     * Should be called from scheduler at 100Hz.
     */
    static void processIncoming();

    /**
     * @brief Send telemetry data to RPi
     *
     * Sends periodic telemetry based on current firmware state:
     * - DC_STATUS_ALL      (100 Hz, RUNNING)
     * - STEP_STATUS_ALL    (100 Hz, RUNNING)
     * - SERVO_STATUS_ALL   ( 50 Hz, RUNNING)
     * - SENSOR_IMU         (100 Hz, RUNNING, if IMU attached)
     * - SENSOR_KINEMATICS  (100 Hz, RUNNING)
     * - IO_STATUS          (100 Hz, RUNNING)
     * - SENSOR_VOLTAGE     ( 10 Hz, RUNNING or ERROR)
     * - SYS_STATUS         (  1 Hz IDLE/ESTOP, 10 Hz RUNNING/ERROR)
     * - SENSOR_MAG_CAL_STATUS (10 Hz, while calibrating)
     *
     * Should be called from scheduler at configured rate.
     */
    static void sendTelemetry();

    // ========================================================================
    // LIVENESS
    // ========================================================================

    /**
     * @brief Check if liveness is valid
     *
     * Returns false if no TLV received within timeout period.
     *
     * @return True if liveness is valid
     */
    static bool isHeartbeatValid();

    /**
     * @brief Get time since last received TLV
     *
     * @return Milliseconds since last TLV received from RPi
     */
    static uint32_t getTimeSinceHeartbeat();

    /**
     * @brief Immediately disable all DC motors, steppers, and servos
     *
     * Called by SafetyManager on hard fault, and by handleSysCmd() on STOP/RESET/ESTOP.
     * Safe to call from ISR context.
     */
    static void disableAllActuators();

private:
    // UART driver instance
    static UARTDriver uart_;

    // ---- Liveness tracking ----
    static uint32_t lastHeartbeatMs_;   // millis() of last received TLV
    static uint32_t lastCmdMs_;         // millis() of last non-heartbeat command
    static bool     heartbeatValid_;    // False if liveness timeout
    static uint16_t heartbeatTimeoutMs_; // Configurable timeout (ms)

    // ---- Configuration (from SYS_CONFIG) ----
    static float    wheelDiameterMm_;   // Wheel diameter (mm), for odometry
    static float    wheelBaseMm_;       // Wheel base center-to-center (mm)
    static uint8_t  motorDirMask_;      // Direction inversion bitmask
    static uint8_t  neoPixelCount_;     // Configured NeoPixel count

    // ---- Odometry state ----
    static float    odomX_;            // X position from start (mm)
    static float    odomY_;            // Y position from start (mm)
    static float    odomTheta_;        // Heading from start (radians, CCW+)
    static int32_t  prevLeftTicks_;    // Previous left encoder count
    static int32_t  prevRightTicks_;   // Previous right encoder count

    // ---- Servo enable tracking (bit N = channel N enabled) ----
    static uint16_t servoEnabledMask_;

    // ---- Loop timing statistics (updated externally) ----
    static uint16_t loopTimeAvgUs_;
    static uint16_t loopTimeMaxUs_;

    // ---- UART error counter ----
    static uint16_t uartRxErrors_;

    // ---- Telemetry timing ----
    static uint32_t lastDCStatusSendMs_;
    static uint32_t lastStepStatusSendMs_;
    static uint32_t lastServoStatusSendMs_;
    static uint32_t lastIMUSendMs_;
    static uint32_t lastKinematicsSendMs_;
    static uint32_t lastVoltageSendMs_;
    static uint32_t lastIOStatusSendMs_;
    static uint32_t lastStatusSendMs_;
    static uint32_t lastMagCalSendMs_;

    // ---- Initialization flag ----
    static bool initialized_;

    // ========================================================================
    // MESSAGE ROUTING
    // ========================================================================

    /**
     * @brief Route received message to appropriate handler
     *
     * Updates liveness timer for any received TLV.
     * Enforces state-based command gating.
     *
     * @param type Message type (from TLV_TypeDefs.h)
     * @param payload Pointer to message payload
     * @param length Payload length in bytes
     */
    static void routeMessage(uint32_t type, const uint8_t* payload, uint32_t length);

    /**
     * @brief Check liveness timeout and disable actuators if expired
     */
    static void checkHeartbeatTimeout();

    // ---- System message handlers ----
    static void handleHeartbeat(const PayloadHeartbeat* payload);
    static void handleSysCmd(const PayloadSysCmd* payload);
    static void handleSysConfig(const PayloadSysConfig* payload);
    static void handleSetPID(const PayloadSetPID* payload);

    // ---- DC motor message handlers ----
    static void handleDCEnable(const PayloadDCEnable* payload);
    static void handleDCSetPosition(const PayloadDCSetPosition* payload);
    static void handleDCSetVelocity(const PayloadDCSetVelocity* payload);
    static void handleDCSetPWM(const PayloadDCSetPWM* payload);

    // ---- Stepper motor message handlers ----
    static void handleStepEnable(const PayloadStepEnable* payload);
    static void handleStepSetParams(const PayloadStepSetParams* payload);
    static void handleStepMove(const PayloadStepMove* payload);
    static void handleStepHome(const PayloadStepHome* payload);

    // ---- Servo message handlers ----
    static void handleServoEnable(const PayloadServoEnable* payload);
    static void handleServoSet(const PayloadServoSetSingle* payload);

    // ---- User I/O message handlers ----
    static void handleSetLED(const PayloadSetLED* payload);
    static void handleSetNeoPixel(const PayloadSetNeoPixel* payload);

    // ---- Magnetometer calibration handler ----
    static void handleMagCalCmd(const PayloadMagCalCmd* payload);

    // ========================================================================
    // TELEMETRY SENDERS
    // ========================================================================

    /** @brief Send all DC motor status (184 bytes) */
    static void sendDCStatusAll();

    /** @brief Send all stepper motor status (96 bytes) */
    static void sendStepStatusAll();

    /** @brief Send servo status for all 16 channels (36 bytes) */
    static void sendServoStatusAll();

    /** @brief Send IMU quaternion and raw sensor data (52 bytes) */
    static void sendSensorIMU();

    /** @brief Send wheel odometry kinematics (28 bytes) */
    static void sendSensorKinematics();

    /** @brief Send battery and rail voltages (8 bytes) */
    static void sendVoltageData();

    /** @brief Send button/limit states and NeoPixel RGB (variable) */
    static void sendIOStatus();

    /** @brief Send system state, version, and diagnostics (48 bytes) */
    static void sendSystemStatus();

    /** @brief Send magnetometer calibration progress (44 bytes) */
    static void sendMagCalStatus();

    // ========================================================================
    // HELPERS
    // ========================================================================

    /** @brief Integrate encoder deltas into odomX_, odomY_, odomTheta_ */
    static void updateOdometry();

    /** @brief Reset odometry to (0, 0, 0) */
    static void resetOdometry();

    /** @brief Return available SRAM in bytes */
    static uint16_t getFreeRAM();
};

#endif // MESSAGECENTER_H
