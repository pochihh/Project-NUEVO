/**
 * @file UARTDriver.cpp
 * @brief Implementation of UART communication driver
 */

#include "UARTDriver.h"
#include <string.h>

// Static instance pointer for callback
UARTDriver* UARTDriver::instance_ = nullptr;

// ============================================================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================================================

UARTDriver::UARTDriver()
    : serial_(RPI_SERIAL)
    , initialized_(false)
    , messageReady_(false)
    , lastMsgType_(0)
    , lastMsgLength_(0)
{
    // Zero out buffers
    memset(txBuffer_, 0, sizeof(txBuffer_));
    memset(rxBuffer_, 0, sizeof(rxBuffer_));
    memset(lastMsgPayload_, 0, sizeof(lastMsgPayload_));
}

UARTDriver::~UARTDriver() {
    if (initialized_) {
        releaseEncodeDescriptor(&encodeDesc_);
        releaseDecodeDescriptor(&decodeDesc_);
    }
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void UARTDriver::init() {
    if (initialized_) return;

    // Initialize Serial2 for RPi communication
    serial_.begin(RPI_BAUD_RATE);

    // Initialize TLV encoder
    initEncodeDescriptor(&encodeDesc_, sizeof(txBuffer_), DEVICE_ID, ENABLE_CRC_CHECK);
    encodeDesc_.buffer = txBuffer_;

    // Initialize TLV decoder with callback
    initDecodeDescriptor(&decodeDesc_, sizeof(rxBuffer_), ENABLE_CRC_CHECK, decodeCallback);
    decodeDesc_.buffer = rxBuffer_;

    // Set instance pointer for callback
    instance_ = this;

    initialized_ = true;

#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.print(F("[UARTDriver] Initialized @ "));
    DEBUG_SERIAL.print(RPI_BAUD_RATE);
    DEBUG_SERIAL.println(F(" baud"));
#endif
}

// ============================================================================
// MESSAGE AVAILABILITY
// ============================================================================

bool UARTDriver::available() {
    if (!initialized_) return false;

    // Feed incoming bytes to decoder
    while (serial_.available()) {
        uint8_t byte = serial_.read();
        decode(&decodeDesc_, &byte, 1);
    }

    // Check if we have a decoded message ready
    return messageReady_;
}

int UARTDriver::bytesAvailable() {
    return serial_.available();
}

// ============================================================================
// MESSAGE TRANSMISSION
// ============================================================================

bool UARTDriver::send(uint32_t type, const void* payload, uint32_t length) {
    if (!initialized_) return false;

    // Reset encoder for new frame
    resetDescriptor(&encodeDesc_);

    // Add TLV packet to frame
    addTlvPacket(&encodeDesc_, type, length, payload);

    // Finalize frame (adds header, CRC, etc.)
    int totalBytes = wrapupBuffer(&encodeDesc_);
    if (totalBytes <= 0) {
        return false;  // Encoding failed
    }

    // Transmit encoded frame
    size_t written = serial_.write(txBuffer_, totalBytes);

#ifdef DEBUG_TLV_PACKETS
    DEBUG_SERIAL.print(F("[TX] Type: "));
    DEBUG_SERIAL.print(type);
    DEBUG_SERIAL.print(F(", Len: "));
    DEBUG_SERIAL.print(length);
    DEBUG_SERIAL.print(F(", Bytes: "));
    DEBUG_SERIAL.println(totalBytes);
#endif

    return (written == (size_t)totalBytes);
}

// ============================================================================
// MESSAGE RECEPTION
// ============================================================================

bool UARTDriver::receive(uint32_t* type, uint8_t* payload, uint32_t* length) {
    if (!initialized_ || !type || !payload || !length) return false;

    // Feed incoming bytes to decoder
    while (serial_.available()) {
        uint8_t byte = serial_.read();
        decode(&decodeDesc_, &byte, 1);
    }

    // Check if message is ready
    if (!messageReady_) {
        return false;
    }

    // Copy decoded message to output parameters
    *type = lastMsgType_;
    *length = lastMsgLength_;
    if (lastMsgLength_ > 0) {
        memcpy(payload, lastMsgPayload_, lastMsgLength_);
    }

    // Clear the message ready flag
    messageReady_ = false;

    return true;
}

// ============================================================================
// DECODER CALLBACK
// ============================================================================

void UARTDriver::decodeCallback(enum DecodeErrorCode* error,
                                const struct FrameHeader* frameHeader,
                                struct TlvHeader* tlvHeaders,
                                uint8_t** tlvData)
{
    if (!instance_) return;

    if (*error != NoError) {
#ifdef DEBUG_TLV_PACKETS
        DEBUG_SERIAL.print(F("[RX] Decode error: "));
        DEBUG_SERIAL.println(*error);
#endif
        return;
    }

    // Process the first TLV in the frame
    // (For now, we assume one TLV per frame for simplicity)
    if (frameHeader->numTlvs > 0 && tlvHeaders && tlvData) {
        instance_->lastMsgType_ = tlvHeaders[0].tlvType;
        instance_->lastMsgLength_ = tlvHeaders[0].tlvLen;

        // Copy payload data
        if (instance_->lastMsgLength_ > 0 && instance_->lastMsgLength_ <= MAX_TLV_PAYLOAD_SIZE) {
            memcpy(instance_->lastMsgPayload_, tlvData[0], instance_->lastMsgLength_);
        } else if (instance_->lastMsgLength_ > MAX_TLV_PAYLOAD_SIZE) {
            // Payload too large - truncate
            memcpy(instance_->lastMsgPayload_, tlvData[0], MAX_TLV_PAYLOAD_SIZE);
            instance_->lastMsgLength_ = MAX_TLV_PAYLOAD_SIZE;

#ifdef DEBUG_TLV_PACKETS
            DEBUG_SERIAL.println(F("[RX] Payload truncated!"));
#endif
        }

        instance_->messageReady_ = true;

#ifdef DEBUG_TLV_PACKETS
        DEBUG_SERIAL.print(F("[RX] Type: "));
        DEBUG_SERIAL.print(instance_->lastMsgType_);
        DEBUG_SERIAL.print(F(", Len: "));
        DEBUG_SERIAL.println(instance_->lastMsgLength_);
#endif
    }
}
