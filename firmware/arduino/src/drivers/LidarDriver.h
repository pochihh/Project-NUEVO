/**
 * @file LidarDriver.h
 * @brief Wrapper for Garmin LIDAR-Lite v4 distance sensor (SparkFun library)
 *
 * The LIDAR-Lite v4 is a time-of-flight laser rangefinder that communicates
 * over I2C. Default address is 0x62; the address can be changed via the
 * SparkFun library if multiple sensors are needed on the same bus.
 *
 * The sensor has a range of approximately 5 cm to 10 m with ~1 cm resolution.
 * `getDistance()` returns the measurement in centimeters (float); this driver
 * converts it to millimeters (uint16_t) for TLV transmission.
 *
 * Usage:
 *   LidarDriver lidar;
 *   lidar.init(0x62);                // default I2C address
 *
 *   uint16_t mm = lidar.getDistanceMm();  // returns 0 on error
 */

#ifndef LIDARDRIVER_H
#define LIDARDRIVER_H

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include "../config.h"

#if LIDAR_COUNT > 0
  #include "../lib/SparkFun_Garmin_LIDAR-Lite_v4_Arduino_Library/src/LIDARLite_v4LED.h"
#endif

// Maximum status distance (sensor returns 0 on error)
#define LIDAR_ERROR_DISTANCE    0

class LidarDriver {
public:
    LidarDriver();

    /**
     * @brief Initialize the lidar sensor at the given I2C address
     *
     * @param i2cAddr I2C address (default 0x62 for LIDAR-Lite v4)
     * @return True if sensor acknowledged
     */
    bool init(uint8_t i2cAddr = 0x62);

    /**
     * @brief Check if sensor responded during init
     */
    bool isConnected() const { return connected_; }

    /**
     * @brief Trigger a measurement and read the distance
     *
     * Blocking — takes a few milliseconds. Call at 50 Hz or slower.
     *
     * @return Distance in mm, or 0 if sensor error / out of range
     */
    uint16_t getDistanceMm();

private:
#if LIDAR_COUNT > 0
    LIDARLite_v4LED lidar_;
#endif
    bool connected_;
    uint8_t i2cAddr_;
};

#endif // LIDARDRIVER_H
