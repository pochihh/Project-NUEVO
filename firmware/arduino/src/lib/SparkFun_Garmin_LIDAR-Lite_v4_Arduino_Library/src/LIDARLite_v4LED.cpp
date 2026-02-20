/*------------------------------------------------------------------------------

  LIDARLite_v4LED Arduino Library
  LIDARLite_v4LED.cpp

  This library is modified by SparkFun but originally comes from Garmin. 
  You can find the original library code here:
  https://github.com/garmin/LIDARLite_Arduino_Library

  This library provides quick access to the basic functions of LIDAR-Lite
  via the Arduino interface. Additionally, it can provide a user of any
  platform with a template for their own application code.

  Copyright (c) 2019 Garmin Ltd. or its subsidiaries.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

------------------------------------------------------------------------------*/

#include <Arduino.h>
#include <Wire.h>
#include <stdarg.h>
#include <stdint.h>
#include "LIDARLite_v4LED.h"

//Initialize the I2C port
bool LIDARLite_v4LED::begin(uint8_t address, TwoWire &wirePort)
{
    _deviceAddress = address; //grab the address that the sensor is on
    _i2cPort = &wirePort;     //grab which port the user wants to use

    //return true if the device is connected
    return (isConnected());
}

bool LIDARLite_v4LED::isConnected()
{
    _i2cPort->beginTransmission(_deviceAddress);
    if (_i2cPort->endTransmission() == 0)
        return true;
    return false;
}

/*------------------------------------------------------------------------------
  Configure

  Selects one of several preset configurations.

  Parameters
  ------------------------------------------------------------------------------
  configuration:  Default 0.
    0: Maximum range. Uses maximum acquisition count.
    1: Balanced performance.
    2: Short range, high speed. Reduces maximum acquisition count.
    3: Mid range, higher speed. Turns on quick termination
         detection for faster measurements at short range (with decreased
         accuracy)
    4: Maximum range, higher speed on short range targets. Turns on quick
         termination detection for faster measurements at short range (with
         decreased accuracy)
    5: Very short range, higher speed, high error. Reduces maximum
         acquisition count to a minimum for faster rep rates on very
         close targets with high error.
------------------------------------------------------------------------------*/
void LIDARLite_v4LED::configure(uint8_t configuration)
{
    uint8_t sigCountMax;
    uint8_t acqConfigReg;

    switch (configuration)
    {
    case 0: // Default mode - Maximum range
        sigCountMax = 0xff;
        acqConfigReg = 0x08;
        break;

    case 1: // Balanced performance
        sigCountMax = 0x80;
        acqConfigReg = 0x08;
        break;

    case 2: // Short range, high speed
        sigCountMax = 0x18;
        acqConfigReg = 0x00;
        break;

    case 3: // Mid range, higher speed on short range targets
        sigCountMax = 0x80;
        acqConfigReg = 0x00;
        break;

    case 4: // Maximum range, higher speed on short range targets
        sigCountMax = 0xff;
        acqConfigReg = 0x00;
        break;

    case 5: // Very short range, higher speed, high error
        sigCountMax = 0x04;
        acqConfigReg = 0x00;
        break;
    }

    write(ACQUISITION_COUNT, &sigCountMax, 1);
    write(QUICK_TERMINATION, &acqConfigReg, 1);
} /* LIDARLite_v4LED::configure */

/*------------------------------------------------------------------------------
  Set I2C Address

  Set Alternate I2C Device Address. See Operation Manual for additional info.

  Parameters
  ------------------------------------------------------------------------------
  newAddress: desired secondary I2C device address
  disableDefault: a non-zero value here means the default 0x62 I2C device
    address will be disabled.
------------------------------------------------------------------------------*/
bool LIDARLite_v4LED::setI2Caddr(uint8_t newAddress, bool disableDefaultI2CAddress)
{
    //Check if address is within range
    if (newAddress < 0x08 || newAddress > 0x77)
    {
        return false;
    }

    if (newAddress == 0x62)
    {
        return useDefaultAddress();
    }

    //Clear out array
    uint8_t dataBytes[5];
    for (int i = 0; i < 5; i++)
        dataBytes[i] = 0;

    // Enable flash storage
    enableFlash(true);
    delay(100);

    // Read 4-byte device serial number
    read(UNIT_ID_0, dataBytes, 4);

    // Append the desired I2C address to the end of the serial number byte array
    dataBytes[4] = newAddress;

    // Write the serial number and new address in one 5-byte transaction
    write(UNIT_ID_0, dataBytes, 5);

    // Wait for the I2C peripheral to be restarted with new device address
    delay(100);

    //Change _deviceAddress to reflect the changed address
    _deviceAddress = newAddress;

    // If desired, disable default I2C device address (using the new I2C device address)
    if (disableDefaultI2CAddress)
    {
        useNewAddressOnly();

        // Wait for the I2C peripheral to be restarted with new device address
        delay(100);
    }

    // Disable flash storage
    enableFlash(false);
    delay(100);

    return true;
} /* LIDARLite_v4LED::setI2Caddr */

bool LIDARLite_v4LED::useDefaultAddress()
{
    uint8_t temp = 0x00; // set bit to disable default address
    bool success = write(I2C_CONFIG, &temp, 1);
    if (success == false)
    {
        return false;
    }

    _deviceAddress = LIDARLITE_ADDR_DEFAULT;

    //Wait for LIDAR to acknowledge on the new address
    byte counter = 0;
    while (1)
    {
        delay(10);

        if (isConnected() == true)
            break;
        if (counter++ > 100)
        {
            return false;
        }
    }

    //disable flash storage after changing address
    enableFlash(false);
    return true;
}

bool LIDARLite_v4LED::useNewAddressOnly()
{
    uint8_t temp = 0x01; // set bit to disable default address
    bool success = write(I2C_CONFIG, &temp, 1);
    if (success == false)
    {
        return false;
    }
    return true;
}

bool LIDARLite_v4LED::useBothAddresses()
{
    uint8_t temp = 0x02;
    bool success = write(I2C_CONFIG, &temp, 1);
    if (success == false)
    {
        return false;
    }
    return true;
}
void LIDARLite_v4LED::enableFlash(bool enable)
{
    uint8_t temp;
    if (enable)
    {
        temp = 0x11;
    }
    else
    {
        temp = 0x00;
    }
    write(ENABLE_FLASH_STORAGE, &temp, 1);
}

/*------------------------------------------------------------------------------
  Take Range

  Initiate a distance measurement by writing to register 0x00.
------------------------------------------------------------------------------*/
void LIDARLite_v4LED::takeRange()
{
    uint8_t dataByte = 0x04;

    write(ACQ_COMMANDS, &dataByte, 1);
} /* LIDARLite_v4LED::takeRange */

/*------------------------------------------------------------------------------
  Wait for Busy Flag

  Blocking function to wait until the Lidar Lite's internal busy flag goes low
------------------------------------------------------------------------------*/
void LIDARLite_v4LED::waitForBusy()
{
    uint8_t busyFlag;

    do
    {
        busyFlag = getBusyFlag();
    } while (busyFlag);

} /* LIDARLite_v4LED::waitForBusy */

/*------------------------------------------------------------------------------
  Get Busy Flag

  Read BUSY flag from device registers. Function will return 0x00 if not busy.
------------------------------------------------------------------------------*/
uint8_t LIDARLite_v4LED::getBusyFlag()
{
    uint8_t statusByte = 0;
    uint8_t busyFlag; // busyFlag monitors when the device is done with a measurement

    // Read status register to check busy flag
    read(STATUS, &statusByte, 1);

    // STATUS bit 0 is busyFlag
    busyFlag = statusByte & 0x01;

    return busyFlag;
} /* LIDARLite_v4LED::getBusyFlag */

/*------------------------------------------------------------------------------
  Read Distance

  Read and return the result of the most recent distance measurement.
------------------------------------------------------------------------------*/
uint16_t LIDARLite_v4LED::readDistance()
{
    uint16_t distance;
    uint8_t *dataBytes = (uint8_t *)&distance;

    // Read two bytes from registers 0x10 and 0x11
    read(FULL_DELAY_LOW, dataBytes, 2);

    return (distance); //This is the distance in centimeters
} /* LIDARLite_v4LED::readDistance */

uint16_t LIDARLite_v4LED::getDistance()
{
    // 1. Trigger a range measurement.
    takeRange();

    // 2. Wait for busyFlag to indicate the device is idle.
    waitForBusy();

    // 3. Read new distance data from device registers
    return readDistance();

    // return *distance; //This is the distance in centimeters
}

/*------------------------------------------------------------------------------
  Take Range using Trigger / Monitor Pins

  Initiate a distance measurement by toggling the trigger pin

  Parameters
  ------------------------------------------------------------------------------
  triggerPin: digital output pin connected to trigger input of LIDAR-Lite
  monitorPin: digital input pin connected to monitor output of LIDAR-Lite
------------------------------------------------------------------------------*/
void LIDARLite_v4LED::takeRangeGpio(uint8_t triggerPin, uint8_t monitorPin)
{
    uint8_t busyFlag;

    if (digitalRead(triggerPin))
        digitalWrite(triggerPin, LOW);
    else
        digitalWrite(triggerPin, HIGH);

    // When LLv4 receives trigger command it will drive monitor pin low.
    // Wait for LLv4 to acknowledge receipt of command before moving on.
    do
    {
        busyFlag = getBusyFlagGpio(monitorPin);
    } while (!busyFlag);
} /* LIDARLite_v4LED::takeRangeGpio */

/*------------------------------------------------------------------------------
  Wait for Busy Flag using Trigger / Monitor Pins

  Blocking function to wait until the Lidar Lite's internal busy flag goes low

  Parameters
  ------------------------------------------------------------------------------
  monitorPin: digital input pin connected to monitor output of LIDAR-Lite
------------------------------------------------------------------------------*/
void LIDARLite_v4LED::waitForBusyGpio(uint8_t monitorPin)
{
    uint8_t busyFlag;

    do
    {
        busyFlag = getBusyFlagGpio(monitorPin);
    } while (busyFlag);

} /* LIDARLite_v4LED::waitForBusyGpio */

/*------------------------------------------------------------------------------
  Get Busy Flag using Trigger / Monitor Pins

  Check BUSY status via Monitor pin. Function will return 0x00 if not busy.

  Parameters
  ------------------------------------------------------------------------------
  monitorPin: digital input pin connected to monitor output of LIDAR-Lite
------------------------------------------------------------------------------*/
uint8_t LIDARLite_v4LED::getBusyFlagGpio(uint8_t monitorPin)
{
    uint8_t busyFlag; // busyFlag monitors when the device is done with a measurement

    // Check busy flag via monitor pin
    if (digitalRead(monitorPin))
        busyFlag = 1;
    else
        busyFlag = 0;

    return busyFlag;
} /* LIDARLite_v4LED::getBusyFlagGpio */

/*------------------------------------------------------------------------------
  Get the temperature of the board
  
  Read the BOARD_TEMPERATURE register. This function returns the temperature in 
  two's complement in Celcius.
------------------------------------------------------------------------------*/
uint8_t LIDARLite_v4LED::getBoardTemp()
{
    uint8_t temp = 0;
    read(BOARD_TEMPERATURE, &temp, 1);
    return temp;
}

/*------------------------------------------------------------------------------
  Get the temperature of the SOC (nRF52840)

  Read the SOC_TEMPERATURE register. This function returns the temperature in 
  two's complement in Celcius.
------------------------------------------------------------------------------*/
uint8_t LIDARLite_v4LED::getSOCTemp()
{
    uint8_t temp = 0;
    read(SOC_TEMPERATURE, &temp, 1);
    return temp;
}

/*------------------------------------------------------------------------------
  Set the power mode of the nRF52840 to be always on.

  The coprocessor is not turned off, allowing for the fastest measurement possible.
  This is the default setting.
------------------------------------------------------------------------------*/
bool LIDARLite_v4LED::setPowerModeAlwaysOn()
{
    uint8_t on = 0xFF;
    return write(POWER_MODE, &on, 1);
}

/*------------------------------------------------------------------------------
  Set the nRF52840 to operate in asynchronous mode.

  The coprocessor is always off unless a distance measurement is needed or a 
  register access is required. Disable high accuracy mode first.
------------------------------------------------------------------------------*/
bool LIDARLite_v4LED::setPowerModeAsync()
{
    uint8_t async = 0x00;
    return write(POWER_MODE, &async, 1);
}

/*------------------------------------------------------------------------------
  Enable and disable high accuracy mode

  By default, the LIDAR operates in high accuracy mode, so use this function to 
  disable and/or re-enable high accuracy mode
  mode before setting the power mode to asynchronous.

  Parameters
  ------------------------------------------------------------------------------
  enable: boolean when set to true will ENABLE high accuracy mode and when false
  will DISABLE high accuracy mode.
------------------------------------------------------------------------------*/
bool LIDARLite_v4LED::enableHighAccuracyMode(bool enable)
{
    uint8_t writeByte;

    if (enable){
        writeByte = 0x14;
    } else {
        writeByte = 0x00;
    }

    return write(HIGH_ACCURACY_MODE, &writeByte, 1);
}

/*------------------------------------------------------------------------------
  Call this function to put the nRF52840 SOC back to factory settings.

  Resets the NVM/Flash storage information back to default settings and executes 
  a SoftDevice reset.
------------------------------------------------------------------------------*/
bool LIDARLite_v4LED::factoryReset()
{
    uint8_t resetByte = 0x01;
    return write(FACTORY_RESET, &resetByte, 1);
}

/*------------------------------------------------------------------------------
  Write

  Perform I2C write to device. The I2C peripheral in the LidarLite v3 HP
  will receive multiple bytes in one I2C transmission. The first byte is
  always the register address. The the bytes that follow will be written
  into the specified register address first and then the internal address
  in the Lidar Lite will be auto-incremented for all following bytes.

  Parameters
  ------------------------------------------------------------------------------
  regAddr:   register address to write to
  dataBytes: pointer to array of bytes to write
  numBytes:  number of bytes in 'dataBytes' array to write
------------------------------------------------------------------------------*/
bool LIDARLite_v4LED::write(uint8_t regAddr, uint8_t *dataBytes,
                            uint8_t numBytes)
{
    int nackCatcher;

    _i2cPort->beginTransmission(_deviceAddress);

    // First byte of every write sets the LidarLite's internal register address pointer
    _i2cPort->write((int)regAddr);

    // Subsequent bytes are data writes
    _i2cPort->write(dataBytes, (int)numBytes);

    // A nack means the device is not responding. Report the error over serial.
    nackCatcher = _i2cPort->endTransmission();
    if (nackCatcher != 0)
    {
        // handle nack issues in here
        return false;
    }
    else
        return true;
} /* LIDARLite_v4LED::write */

/*------------------------------------------------------------------------------
  Read

  Perform I2C read from device.  The I2C peripheral in the LidarLite v3 HP
  will send multiple bytes in one I2C transmission. The register address must
  be set up by a previous I2C write. The bytes that follow will be read
  from the specified register address first and then the internal address
  pointer in the Lidar Lite will be auto-incremented for following bytes.

  Will detect an unresponsive device and report the error over serial.

  Parameters
  ------------------------------------------------------------------------------
  regAddr:   register address to read
  dataBytes: pointer to array of bytes to write
  numBytes:  number of bytes in 'dataBytes' array to read
------------------------------------------------------------------------------*/
void LIDARLite_v4LED::read(uint8_t regAddr, uint8_t *dataBytes,
                           uint8_t numBytes)
{
    uint16_t i = 0;
    int nackCatcher = 0;

    // Set the internal register address pointer in the Lidar Lite
    _i2cPort->beginTransmission(_deviceAddress);
    _i2cPort->write((int)regAddr); // Set the register to be read

    // A nack means the device is not responding, report the error over serial
    nackCatcher = _i2cPort->endTransmission(false); // false means perform repeated start
    if (nackCatcher != 0)
    {
        // handle nack issues in here
    }

    // Perform read, save in dataBytes array
    _i2cPort->requestFrom(_deviceAddress, numBytes);
    if ((int)numBytes <= _i2cPort->available())
    {
        while (i < numBytes)
        {
            dataBytes[i] = (uint8_t)_i2cPort->read();
            i++;
        }
    }

} /* LIDARLite_v4LED::read */

/*------------------------------------------------------------------------------
  Correlation Record Read

  The correlation record used to calculate distance can be read from the device.
  It has a bipolar wave shape, transitioning from a positive going portion to a
  roughly symmetrical negative going pulse. The point where the signal crosses
  zero represents the effective delay for the reference and return signals.

  Process
  ------------------------------------------------------------------------------
  1.  Take a distance reading (there is no correlation record without at least
      one distance reading being taken)
  2.  For as many points as you want to read from the record (max is 192) read
      the two byte signed correlation data point from 0x52

  Parameters
  ------------------------------------------------------------------------------
  correlationArray: pointer to memory location to store the correlation record
                    ** Two bytes for every correlation value must be
                       allocated by calling function
  numberOfReadings: Default = 192. Maximum = 192
------------------------------------------------------------------------------*/
void LIDARLite_v4LED::correlationRecordRead(
    int16_t *correlationArray, uint8_t numberOfReadings)
{
    uint8_t i;
    int16_t correlationValue;
    uint8_t *dataBytes = (uint8_t *)&correlationValue;

    for (i = 0; i < numberOfReadings; i++)
    {
        read(CORR_DATA, dataBytes, 2);
        correlationArray[i] = correlationValue;
    }
} /* LIDARLite_v4LED::correlationRecordRead */
