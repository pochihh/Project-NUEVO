/**
 * @file UARTDriver.h
 * @brief UART communication driver with TLV protocol encoding
 *
 * This driver provides a simple C++ interface for TLV message communication
 * over UART Serial2 (connected to Raspberry Pi).
 *
 * Hardware:
 * - Arduino Mega 2560 Serial2 (pins 16 TX, 17 RX)
 * - 5V ↔ 3.3V level shifter (bi-directional)
 * - Raspberry Pi 5 UART
 *
 * Features:
 * - TLV message framing and parsing
 * - CRC error detection
 * - Transmit and receive buffering
 *
 * Usage:
 *   UARTDriver uart;
 *   uart.init();
 *
 *   // Send message:
 *   PayloadHeartbeat hb = {millis(), 0};
 *   uart.send(SYS_HEARTBEAT, &hb, sizeof(hb));
 *
 *   // Receive message:
 *   uint32_t msgType;
 *   uint8_t payload[128];
 *   uint32_t length;
 *   if (uart.receive(&msgType, payload, &length)) {
 *     // Process msgType, payload, length
 *   }
 */

#ifndef UARTDRIVER_H
#define UARTDRIVER_H

#include <Arduino.h>
#include <stdint.h>

extern "C" {
    #include "../lib/tlvcodec.h"
}

#include "../messages/TLV_TypeDefs.h"
#include "../messages/TLV_Payloads.h"
#include "../config.h"

// Maximum TLV message payload size
#define MAX_TLV_PAYLOAD_SIZE 256

// ============================================================================
// UART DRIVER CLASS
// ============================================================================

/**
 * @brief UART communication driver with TLV protocol
 *
 * C++ wrapper around the C-based TLV codec library.
 * Provides simple message send/receive interface.
 */
class UARTDriver {
public:
    UARTDriver();
    ~UARTDriver();

    /**
     * @brief Initialize UART and TLV codec
     *
     * Configures Serial2 at specified baud rate and
     * initializes TLV encode/decode descriptors.
     */
    void init();

    /**
     * @brief Check if complete message is available
     *
     * @return True if at least one complete TLV frame has been received
     */
    bool available();

    /**
     * @brief Send TLV message
     *
     * Encodes and transmits a TLV message over UART.
     *
     * @param type Message type constant (from TLV_TypeDefs.h)
     * @param payload Pointer to payload data (can be nullptr if length=0)
     * @param length Payload length in bytes
     * @return True if message was sent successfully
     */
    bool send(uint32_t type, const void* payload, uint32_t length);

    /**
     * @brief Receive TLV message
     *
     * Parses received bytes and extracts complete TLV message.
     *
     * @param type Pointer to store message type
     * @param payload Buffer to store payload data
     * @param length Pointer to store payload length
     * @return True if a valid message was received
     */
    bool receive(uint32_t* type, uint8_t* payload, uint32_t* length);

    /**
     * @brief Get number of bytes available in Serial buffer
     *
     * @return Number of bytes waiting to be read
     */
    int bytesAvailable();

private:
    HardwareSerial& serial_;

    // C-style TLV descriptors
    struct TlvEncodeDescriptor encodeDesc_;
    struct TlvDecodeDescriptor decodeDesc_;

    // Buffers for encoding/decoding
    uint8_t txBuffer_[512];
    uint8_t rxBuffer_[512];

    bool initialized_;
    bool messageReady_;

    // Callback for decoder (static member function)
    static void decodeCallback(enum DecodeErrorCode* error,
                             const struct FrameHeader* frameHeader,
                             struct TlvHeader* tlvHeaders,
                             uint8_t** tlvData);

    // Pointer to current instance for callback
    static UARTDriver* instance_;

    // Storage for decoded message
    uint32_t lastMsgType_;
    uint8_t lastMsgPayload_[MAX_TLV_PAYLOAD_SIZE];
    uint32_t lastMsgLength_;
};

#endif // UARTDRIVER_H
