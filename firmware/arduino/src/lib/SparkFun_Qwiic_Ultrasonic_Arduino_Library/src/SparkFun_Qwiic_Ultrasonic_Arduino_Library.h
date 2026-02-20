/*!
 * @file SparkFun_Qwiic_Ultrasonic_Arduino_Library.h
 * @brief Arduino header file for the SparkFun Qwiic Ultrasonic Arduino library
 *
 * This file contains the class definitions for the SparkFun Qwiic Ultrasonic Arduino library,
 * including methods for initialization, reading ultrasonic values, and changing the I2C address.
 *
 * @author SparkFun Electronics
 * @date 2024
 * @copyright Copyright (c) 2024-2025, SparkFun Electronics Inc. This project is released under the MIT License.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

// clang-format off
#include <SparkFun_Toolkit.h>
#include "sfTk/sfDevUltrasonicDist.h"
#include "Arduino.h"
#include <Wire.h>
// clang-format on

class QwiicUltrasonic : public sfDevUltrasonicDist
{
  public:
    /**
     * @brief Begins the Qwiic Ultrasonic sensor
     *
     * This method initializes the Qwiic Ultrasonic sensor by setting up the I2C bus
     * and starting communication with the sensor.
     *
     * @param address I2C device address to use for the sensor
     * @param wirePort Wire port to use for I2C communication
     * @return True if successful, false otherwise
     */
    bool begin(uint8_t address = kQwiicUltrasonicDefaultAddress, TwoWire &wirePort = Wire)
    {
        // Setup Arduino I2C bus
        _theI2CBus.init(wirePort, address);

        // Begin the sensor
        return sfDevUltrasonicDist::begin(&_theI2CBus) == ksfTkErrOk;
    }

    /**
     * @brief Checks if the Qwiic Ultrasonic sensor is connected
     *
     * This method verifies if the Qwiic Ultrasonic sensor is properly connected
     * by checking the communication status.
     *
     * @return True if the sensor is connected, false otherwise
     */
    bool isConnected()
    {
        return sfDevUltrasonicDist::isConnected() == ksfTkErrOk;
    }

  private:
    /**
     * @brief I2C bus instance for communication with the sensor
     *
     * This variable holds the I2C bus instance used for communication with the Qwiic Ultrasonic sensor.
     */
    sfTkArdI2C _theI2CBus;
};
