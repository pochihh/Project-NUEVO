// c lib for TLV codec
extern "C"
{
#include "../lib/tlvcodec.h"
}

#include "MessageCenter.h"

extern MessageCenter RoverGlobalMsg;
extern int32_t StopSignDetected;             // Indicate that the stop sign is detected
extern float StopSignDetectedConfidence; // Indicate the confidence of the stop sign detection
extern float RoverGlobalCoordX;          // GPS x coordinate
extern float RoverGlobalCoordY;          // GPS y coordinate
extern int32_t TrafficLightStatus;          // 0: red, 1: green
extern int32_t FaceDetected;                 // Indicate that the face is detected

void decodeCallback(DecodeErrorCode *error, const FrameHeader *frameHeader, TlvHeader *tlvHeaders, uint8_t **tlvData)
{
    if (*error == NoError)
    {
        // Successfully decoded a message
        // take actions based on the TLV type
        for (size_t i = 0; i < frameHeader->numTlvs; ++i)
        {
            switch (tlvHeaders[i].tlvType)
            {
            case YOLO_OBJECT_DETECTED:
                // copy the data to a local variable
                Detection det;
                memcpy(&det, tlvData[i], sizeof(det));

                if (det.object == 2)
                {
                    // stop sign detected
                    // add to global variable
                    StopSignDetected++;
                    if (det.confidence > StopSignDetectedConfidence)
                    {
                        StopSignDetectedConfidence = det.confidence;
                    }
                }
                else if (det.object == 5)
                {
                    // no object detected

                }
                else
                {
                    // other objects detected, do nothing
                }
                break;

            case GPS_XY_COORDINATE:
                // copy the data to a local variable
                gps_xy_coordinate gps_coord;
                memcpy(&gps_coord, tlvData[i], sizeof(gps_coord));

                // Process the GPS coordinates
                RoverGlobalCoordX = gps_coord.x;
                RoverGlobalCoordY = gps_coord.y;
                break;

            case TRAFFIC_LIGHT_DETECTED:
                // copy the data to a local variable
                traffic_light_status light_status;
                memcpy(&light_status, tlvData[i], sizeof(light_status));

                // Process the traffic light
                TrafficLightStatus = light_status.status;
                // light_status.confidence is not used for now
                break;

            case FACE_DETECTED:
                // copy the data to a local variable
                face_detected face_det;
                memcpy(&face_det, tlvData[i], sizeof(face_det));

                // Process the face detection results
                FaceDetected = face_det.res;
                break;

            default:
                // do nothing
                break;
            }
        }
        // Output messegagejust for debugging
#ifdef __DEBUG__
        for (size_t i = 0; i < frameHeader->numTlvs; ++i)
        {
            switch (tlvHeaders[i].tlvType)
            {
            case YOLO_OBJECT_DETECTED:
                // Print the detection result
                Serial.print("Received detection result. Object: ");
                Serial.print(StopSignDetected);
                Serial.print(", Confidence: ");
                Serial.println(StopSignDetectedConfidence);
                break;

            case GPS_XY_COORDINATE:
                // Print the GPS coordinates
                Serial.print("Received GPS data. x: ");
                Serial.print(RoverGlobalCoordX);
                Serial.print(", y: ");
                Serial.println(RoverGlobalCoordY);
                break;

            case TRAFFIC_LIGHT_DETECTED:
                Serial.print("Received traffic light status: ");
                Serial.println(TrafficLightStatus);
                break;

            case FACE_DETECTED:
                Serial.print("Received face detection result: ");
                Serial.println(FaceDetected);
                break;

            default:
                Serial.print("Received unknown TLV type: ");
                Serial.println(tlvHeaders[i].tlvType);
                Serial.print("TLV length: ");
                Serial.println(tlvHeaders[i].tlvLen);
                RoverGlobalMsg.addMessage(tlvHeaders[i].tlvType, tlvHeaders[i].tlvLen, tlvData[i]);
                RoverGlobalMsg.addMessage(tlvHeaders[i].tlvType + 1, tlvHeaders[i].tlvLen, tlvData[i]);
                break;
            }
        }
#endif
    }
    else
    {
        // TODO: log error result to a global variable
#ifdef __DEBUG__
        Serial.print("Error decoding message: ");
        Serial.println(*error);
        Serial.print("total bytes: ");
        Serial.println(frameHeader->numTotalBytes.value);
#endif
    }
}

MessageCenter::MessageCenter()
{
    // Initialize the encoder
    initEncodeDescriptor(&encoder, 256, DEVICE_ID, true);

    // Initialize the decoder; use the callback function to decode the message
    initDecodeDescriptor(&decoder, 512, true, &decodeCallback);
}

MessageCenter::~MessageCenter()
{
    // Release the encoder and decoder descriptors
    releaseEncodeDescriptor(&encoder);
    releaseDecodeDescriptor(&decoder);
}

void MessageCenter::init()
{
    // set up serial communication
    Serial1.begin(9600);
}

void MessageCenter::processingTick()
{
    // Use readBytes to read serial data and put any new data into the decoder
    // The callback function will be called when a message packet is ready
    if (Serial1.available() > 0)
    {
        bytesRead = Serial1.readBytes(serialBuffer, min(Serial1.available(), MAX_SERIAL_BUFFER_LEN));
        decode(&decoder, serialBuffer, bytesRead);
#ifdef __DEBUG__
        // for (int i = 0; i < bytesRead; i++)
        // {
        //     Serial.print(serialBuffer[i]);
        //     Serial.print(" ");
        // }
        // Serial.println();
#endif
    }

    if (messageCount > 0) // There are messaged added to the encoder. Send them out
    {
        int bufferSize = wrapupBuffer(&encoder);
        Serial1.write(encoder.buffer, bufferSize);
        resetDescriptor(&encoder); // reset the encoder buffer
        messageCount = 0;
    }
}

void MessageCenter::addMessage(uint32_t tlvType, uint32_t tlvLen, const void *dataAddr)
{
    addTlvPacket(&encoder, tlvType, tlvLen, dataAddr);
    messageCount++;
}
