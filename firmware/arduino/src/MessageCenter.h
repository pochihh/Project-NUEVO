#ifndef MESSAGECENTER_H
#define MESSAGECENTER_H

#define DEVICE_ID 0x02
#define MAX_SERIAL_BUFFER_LEN 1024 

// define the TLV types
#define NO_OBJECT_DETECTED 100
#define YOLO_OBJECT_DETECTED 101
#define FACE_DETECTED 102
#define TRAFFIC_LIGHT_DETECTED 103
#define GPS_XY_COORDINATE 110

#include "Arduino.h"

// detection object types for YOLO_OBJECT_DETECTED (101)
struct Detection
{
    int32_t object;
    struct BBox
    {
        int32_t x;
        int32_t y;
        int32_t w;
        int32_t h;
    } bbox;
    float confidence;
};

// gps coordinate types for GPS_XY_COORDINATE (110)
struct gps_xy_coordinate
{
    float x;
    float y;
};

// traffic light status types for TRAFFIC_LIGHT_DETECTED (111)
struct traffic_light_status 
{
    int32_t status;
    float confidence;
};

struct face_detected
{
    int32_t res;
};


// bool traffic_light_status = false; // 0: red, 1: green
// int face_detected = 0;

// MessageCenter class
class MessageCenter
{
public:
    MessageCenter();
    ~MessageCenter();

    void init();

    // to be called at every tick
    void processingTick();
    
    // function to be called when a message is received
    // static void decodeCallback(enum DecodeErrorCode *error, const struct FrameHeader *frameHeader, struct TlvHeader *tlvHeaders, uint8_t **tlvData);

    // can be called anytime and the message will be queued in the encoder buffer
    void addMessage(uint32_t tlvType, uint32_t tlvLen, const void *dataAddr);

// private:
    TlvDecodeDescriptor decoder;
    TlvEncodeDescriptor encoder;

    byte serialBuffer[MAX_SERIAL_BUFFER_LEN];
    int bytesRead = 0;
    int messageCount = 0;
    
};

#endif // MESSAGECENTER_H