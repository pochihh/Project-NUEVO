/**
 * @file sfDevUltrasonicDist.h
 * @brief Header file for the Ultrasonic Distance sensor class
 *
 * This file contains the class definition for the ultrasonic distance sensor, including
 * methods for initialization, measuring distance values, and setting the I2C address.
 *
 * @author SparkFun Electronics
 * @date 2024
 * @copyright Copyright (c) 2024-2025, SparkFun Electronics Inc. This project is released under the MIT License.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>

// include the sparkfun toolkit headers
// (relative path to local SparkFun_Toolkit embedded in src/lib/)
#include "../../../SparkFun_Toolkit/src/sfTk/sfToolkit.h"

// Bus interfaces
#include "../../../SparkFun_Toolkit/src/sfTk/sfTkII2C.h"

// Available I2C addresses of the Qwiic Ultrasonic
const uint8_t kQwiicUltrasonicDefaultAddress = 0x2F;

// Firmware versions. The later hardware version is v10 and so the "latest" here
// refers to that. The previous version is randomly given the value v01.
const uint8_t kQwiicUltrasonicFWLatest = 0x10;
const uint8_t kQwiicUltrasonicFWOld = 0x01;

// These addresses are the min and max (respectively) of valid I2C addresses that can
// be used for the newest revision of the Qwiic Ultrasonic sensor.
const uint8_t kQwiicUltrasonicI2CAddressMin = 0x08;
const uint8_t kQwiicUltrasonicI2CAddressMax = 0x77;

// Available I2C addresses of the older SparkFun Qwiic Ultrasonic Sensor.
const uint8_t kQwiicUltrasonicMinAddress = 0x20;
const uint8_t kQwiicUltrasonicMaxAddress = 0x2F;
;
// I2C commands
const uint8_t kUltrasonicDistanceReadCommand = 0x01;
const uint8_t kUltrasonicAddressChangeCommand = 0x04;

class sfDevUltrasonicDist
{
  public:
    /// @brief Default constructor
    ///
    sfDevUltrasonicDist() : _theBus(nullptr), _fwVersion(kQwiicUltrasonicFWLatest)
    {
    }

    /// @brief Alternate constructor
    /// @param fwVersion Firmware version of the Qwiic Ultrasonic Sensor. If using an older version
    ///         of the sensor, the list of available I2C addresses differs.
    sfDevUltrasonicDist(const uint8_t fwVersion) : _theBus(nullptr), _fwVersion(fwVersion)
    {
    }

    /// @brief Begins the Qwiic Ultrasonic sensor
    /// @param theBus I2C bus to use for communication
    /// @return 0 for success, negative for errors, positive for warnings
    sfTkError_t begin(sfTkII2C *theBus = nullptr);

    /// @brief Checks if the Qwiic Ultrasonic sensor is connected
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t isConnected();

    /// @brief Triggers a new measurement and reads the previous one
    /// @param distance Distance in mm
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t triggerAndRead(uint16_t &distance);

    /// @brief Changes the I2C address of older Qwiic Ultrasonic sensors.
    /// @param address New address, must be in the range 0x20 to 0x2F
    /// @return 0 for succuss, negative for errors, positive for warnings
    sfTkError_t changeAddress(uint8_t &address);

    /// @brief Gets the current I2C address being used by the library for the Qwiic Ultrasonic sensor
    /// @return The current I2C address, 7-bit unshifted
    uint8_t getAddress();

  protected:
    sfTkII2C *_theBus;

  private:
    uint8_t _fwVersion = 0x00;
};
