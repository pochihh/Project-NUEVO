/**
 * @file MessageCenter.h
 * @brief Central message routing and TLV communication handler
 *
 * This module manages all UART communication with the Raspberry Pi:
 * - Receives and parses incoming TLV messages
 * - Routes commands to appropriate subsystems (motors, servos, LEDs)
 * - Generates and sends outgoing telemetry (encoder, sensor, status data)
 * - Monitors heartbeat and enforces safety timeout
 *
 * Message Flow:
 *   RPi → UART → MessageCenter → Parse TLV → Route to module
 *   Module → Generate TLV → MessageCenter → UART → RPi
 *
 * Safety:
 * - Heartbeat timeout: Disables all motors if no message received for 500ms
 * - Invalid message handling: Sends error response
 * - Overflow protection: Limits message queue size
 *
 * Usage:
 *   MessageCenter::init();
 *
 *   // In scheduler task @ 100Hz:
 *   MessageCenter::processIncoming();
 *   MessageCenter::sendTelemetry();
 *
 *   // Check heartbeat:
 *   if (!MessageCenter::isHeartbeatValid()) {
 *     // Safety: Disable motors
 *   }
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
 * - TLV message parsing and routing
 * - Telemetry generation and transmission
 * - Heartbeat monitoring and safety timeout
 */
class MessageCenter {
public:
    /**
     * @brief Initialize message center
     *
     * Initializes UART driver and message buffers.
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
     * Generates and sends periodic telemetry:
     * - Encoder positions and velocities (100Hz)
     * - Voltage sensors (50Hz)
     * - DC motor status (50Hz)
     * - Stepper status (20Hz)
     * - System status (1Hz)
     *
     * Should be called from scheduler at configured rate.
     */
    static void sendTelemetry();

    // ========================================================================
    // HEARTBEAT MONITORING
    // ========================================================================

    /**
     * @brief Check if heartbeat is valid
     *
     * Returns false if no heartbeat received within timeout period.
     * Used by motor controllers to enforce safety shutdown.
     *
     * @return True if heartbeat is valid
     */
    static bool isHeartbeatValid();

    /**
     * @brief Get time since last heartbeat
     *
     * @return Milliseconds since last heartbeat received
     */
    static uint32_t getTimeSinceHeartbeat();

    /**
     * @brief Reset heartbeat timer
     *
     * Called when heartbeat message is received.
     */
    static void updateHeartbeat();

private:
    // UART driver instance
    static UARTDriver uart_;

    // Heartbeat tracking
    static uint32_t lastHeartbeatMs_;
    static bool heartbeatValid_;

    // Telemetry timing
    static uint32_t lastEncoderSendMs_;
    static uint32_t lastVoltageSendMs_;
    static uint32_t lastDCStatusSendMs_;
    static uint32_t lastStepStatusSendMs_;
    static uint32_t lastButtonSendMs_;
    static uint32_t lastStatusSendMs_;

    // Initialization flag
    static bool initialized_;

    // ========================================================================
    // MESSAGE HANDLERS
    // ========================================================================

    /**
     * @brief Route received message to appropriate handler
     *
     * @param type Message type (from TLV_TypeDefs.h)
     * @param payload Pointer to message payload
     * @param length Payload length
     */
    static void routeMessage(uint32_t type, const uint8_t* payload, uint32_t length);

    /**
     * @brief Check heartbeat timeout
     *
     * Disables motors if heartbeat timeout exceeded.
     */
    static void checkHeartbeatTimeout();

    // System message handlers
    static void handleHeartbeat(const PayloadHeartbeat* payload);
    static void handleSetPID(const PayloadSetPID* payload);

    // DC motor message handlers
    static void handleDCEnable(const PayloadDCEnable* payload);
    static void handleDCSetPosition(const PayloadDCSetPosition* payload);
    static void handleDCSetVelocity(const PayloadDCSetVelocity* payload);

    // Stepper motor message handlers
    static void handleStepEnable(const PayloadStepEnable* payload);
    static void handleStepSetAccel(const PayloadStepSetAccel* payload);
    static void handleStepSetVel(const PayloadStepSetVel* payload);
    static void handleStepMove(const PayloadStepMove* payload);
    static void handleStepHome(const PayloadStepHome* payload);

    // Servo message handlers
    static void handleServoEnable(const PayloadServoEnable* payload);
    static void handleServoSet(const PayloadServoSetSingle* payload);

    // User I/O message handlers
    static void handleSetLED(const PayloadSetLED* payload);
    static void handleSetNeoPixel(const PayloadSetNeoPixel* payload);

    // ========================================================================
    // TELEMETRY SENDERS
    // ========================================================================

    /**
     * @brief Send encoder data for all motors
     */
    static void sendEncoderData();

    /**
     * @brief Send voltage sensor readings
     */
    static void sendVoltageData();

    /**
     * @brief Send DC motor status for all motors
     */
    static void sendDCStatus();

    /**
     * @brief Send stepper motor status for all steppers
     */
    static void sendStepperStatus();

    /**
     * @brief Send button/limit switch states
     */
    static void sendButtonStates();

    /**
     * @brief Send system status message
     */
    static void sendSystemStatus();
};

#endif // MESSAGECENTER_H
