/**
 * @file sfDevUltrasonicDist.cpp
 * @brief Implementation file for the Ultrasonic Distance sensor class
 *
 * This file contains the implementation of the ultrasonic distance sensor class, including
 * methods for initialization, measuring distance values, and setting the I2C address.
 *
 * @author SparkFun Electronics
 * @date 2024
 * @copyright Copyright (c) 2024-2025, SparkFun Electronics Inc. This project is released under the MIT License.
 *
 * SPDX-License-Identifier: MIT
 */

#include "sfDevUltrasonicDist.h"

//------------------------------------------------------------------------
sfTkError_t sfDevUltrasonicDist::begin(sfTkII2C *theBus)
{
    // Nullptr check
    if (theBus == nullptr)
        return ksfTkErrFail;

    // Check the device address
    if (_fwVersion == kQwiicUltrasonicFWOld)
    {
        if (theBus->address() < kQwiicUltrasonicMinAddress || theBus->address() > kQwiicUltrasonicMaxAddress)
        {
            // An older version of the firmware used 0x00 as the default address.
            // It's not really a valid address, but we need to allow it. Any other
            // address can't be used
            if (theBus->address() != 0x00)
                return ksfTkErrFail;
        }
    }
    // Set bus pointer
    _theBus = theBus;

    // Just check if the device is connected, no other setup is needed
    return isConnected();
}

//------------------------------------------------------------------------
sfTkError_t sfDevUltrasonicDist::isConnected()
{
    // Just ping the device address, there's no product ID register to check
    return _theBus->ping();
}

//------------------------------------------------------------------------
sfTkError_t sfDevUltrasonicDist::triggerAndRead(uint16_t &distance)
{
    size_t bytesRead;
    size_t numBytes = 2;
    uint8_t rawData[2] = {};

    // Get the distance
    sfTkError_t err = _theBus->readRegister(kUltrasonicDistanceReadCommand, rawData, numBytes, bytesRead);

    // Check whether the read was successful
    if (err != ksfTkErrOk)
        return err;

    // Store raw data
    distance = (rawData[0] << 8) | rawData[1];

    // Done!
    return ksfTkErrOk;
}

//------------------------------------------------------------------------
sfTkError_t sfDevUltrasonicDist::changeAddress(uint8_t &address)
{
    // Check whether the address is valid

    sfTkError_t err;

    if (_fwVersion == kQwiicUltrasonicFWOld)
    {
        // Old firmware only supports a limited range of addresses.
        if (address < kQwiicUltrasonicMinAddress || address > kQwiicUltrasonicMaxAddress)
            return ksfTkErrFail;

        // Write the new address to the device. The first bit must be set to 1
        err = _theBus->writeUInt8(address | 0x80);
    }
    else if (_fwVersion == kQwiicUltrasonicFWLatest)
    {
        size_t numBytes = 2;
        // Latest firmware versions supports all of the available I2C addresses.
        if (address < kQwiicUltrasonicI2CAddressMin || address > kQwiicUltrasonicI2CAddressMax)
            return ksfTkErrFail;

        // We want to shift the address left before we send it.
        uint8_t tempAddress = address << 1;
        const uint8_t toWrite[2] = {kUltrasonicAddressChangeCommand, tempAddress};

        // Write the new address to the device
        err = _theBus->writeData(toWrite, numBytes);
    }
    else
    {
        // There was some error setting the version in the constructor
        // return an error.
        return ksfTkErrFail;
    }

    _theBus->setAddress(address);

    // return err
    return err;
}

//------------------------------------------------------------------------
uint8_t sfDevUltrasonicDist::getAddress()
{
    return _theBus->address();
}
