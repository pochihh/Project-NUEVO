/**
 * @file UltrasonicDriver.h
 * @brief Wrapper for SparkFun Qwiic Ultrasonic distance sensor (HC-SR04 via I2C)
 *
 * The Qwiic Ultrasonic converts the HC-SR04 ultrasonic module to I2C using
 * a small microcontroller on the breakout board. The default I2C address is
 * 0x2F; it can be changed using Example 2 from the SparkFun library.
 *
 * Range: approximately 2 cm to 400 cm with ~3 mm accuracy.
 * `triggerAndRead()` returns the measurement in millimeters (uint16_t).
 *
 * Usage:
 *   UltrasonicDriver ultrasonic;
 *   ultrasonic.init(0x2F);               // default I2C address
 *
 *   uint16_t mm = ultrasonic.getDistanceMm();
 */

#ifndef ULTRASONICDRIVER_H
#define ULTRASONICDRIVER_H

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include "../config.h"

#if ULTRASONIC_COUNT > 0
  #include "../lib/SparkFun_Qwiic_Ultrasonic_Arduino_Library/src/SparkFun_Qwiic_Ultrasonic_Arduino_Library.h"
#endif


#define ULTRASONIC_ERROR_DISTANCE   0

class UltrasonicDriver {
public:
    UltrasonicDriver();

    /**
     * @brief Initialize the ultrasonic sensor at the given I2C address
     *
     * @param i2cAddr I2C address (default 0x2F = kQwiicUltrasonicDefaultAddress)
     * @return True if sensor acknowledged
     */
    bool init(uint8_t i2cAddr = 0x2F);

    /**
     * @brief Check if sensor responded during init
     */
    bool isConnected() const { return connected_; }

    /**
     * @brief Trigger a measurement and read the distance
     *
     * Ultrasonic sensors have a maximum update rate of ~15 Hz due to sound travel
     * time. Calling faster than this will return stale or incorrect readings.
     *
     * @return Distance in mm, or 0 if sensor error / out of range
     */
    uint16_t getDistanceMm();

private:
#if ULTRASONIC_COUNT > 0
    QwiicUltrasonic ultrasonic_;
#endif
    bool connected_;
    uint8_t i2cAddr_;
};

#endif // ULTRASONICDRIVER_H
