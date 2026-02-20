/**
 * @file LidarDriver.cpp
 * @brief Implementation of LIDAR-Lite v4 driver wrapper
 */

#include "LidarDriver.h"

LidarDriver::LidarDriver()
    : connected_(false)
    , i2cAddr_(0x62)
{
}

bool LidarDriver::init(uint8_t i2cAddr) {
#if LIDAR_COUNT > 0
    i2cAddr_ = i2cAddr;

    // begin() checks for I2C acknowledgement; returns false if not found
    if (!lidar_.begin(i2cAddr_, Wire)) {
#ifdef DEBUG_SENSOR
        DEBUG_SERIAL.print(F("[Lidar] Not found at 0x"));
        DEBUG_SERIAL.println(i2cAddr_, HEX);
#endif
        connected_ = false;
        return false;
    }

    connected_ = true;

#ifdef DEBUG_SENSOR
    DEBUG_SERIAL.print(F("[Lidar] Connected at 0x"));
    DEBUG_SERIAL.println(i2cAddr_, HEX);
#endif

    return true;
#else
    (void)i2cAddr;
    return false;
#endif
}

uint16_t LidarDriver::getDistanceMm() {
#if LIDAR_COUNT > 0
    if (!connected_) return LIDAR_ERROR_DISTANCE;

    // getDistance() returns distance in cm as uint16_t; 0 = sensor error or out of range
    uint16_t cm = lidar_.getDistance();

    if (cm == 0) {
        return LIDAR_ERROR_DISTANCE;
    }

    return cm * 10;  // cm → mm
#else
    return LIDAR_ERROR_DISTANCE;
#endif
}
