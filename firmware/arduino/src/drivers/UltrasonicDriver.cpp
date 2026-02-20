/**
 * @file UltrasonicDriver.cpp
 * @brief Implementation of Qwiic Ultrasonic driver wrapper
 */

#include "UltrasonicDriver.h"

UltrasonicDriver::UltrasonicDriver()
    : connected_(false)
    , i2cAddr_(0x2F)
{
}

bool UltrasonicDriver::init(uint8_t i2cAddr) {
#if ULTRASONIC_COUNT > 0
    i2cAddr_ = i2cAddr;

    if (!ultrasonic_.begin(i2cAddr_)) {
#ifdef DEBUG_SENSOR
        DEBUG_SERIAL.print(F("[Ultrasonic] Not found at 0x"));
        DEBUG_SERIAL.println(i2cAddr_, HEX);
#endif
        connected_ = false;
        return false;
    }

    connected_ = true;

#ifdef DEBUG_SENSOR
    DEBUG_SERIAL.print(F("[Ultrasonic] Connected at 0x"));
    DEBUG_SERIAL.println(i2cAddr_, HEX);
#endif

    return true;
#else
    (void)i2cAddr;
    return false;
#endif
}

uint16_t UltrasonicDriver::getDistanceMm() {
#if ULTRASONIC_COUNT > 0
    if (!connected_) return ULTRASONIC_ERROR_DISTANCE;

    uint16_t distance = 0;

    // triggerAndRead() returns sfTkError_t; 0 (ksfTkErrOk) means success
    if (ultrasonic_.triggerAndRead(distance) != 0) {
        return ULTRASONIC_ERROR_DISTANCE;
    }

    // The library already returns mm; 0 indicates a sensor error
    return distance;
#else
    return ULTRASONIC_ERROR_DISTANCE;
#endif
}
