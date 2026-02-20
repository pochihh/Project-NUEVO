/*------------------------------------------------------------------------------

  LIDARLite_v4LED Arduino Library
  LIDARLite_v4LED.h

  This library is modified by SparkFun but originally comes from Garmin. 
  You can find the original library code here:
  https://github.com/garmin/LIDARLite_Arduino_Library
  
  This library provides quick access to all the basic functions of LIDAR-Lite
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
#ifndef LIDARLite_v4LED_h
#define LIDARLite_v4LED_h

#define LIDARLITE_ADDR_DEFAULT 0x62

#include <Wire.h>
#include <Arduino.h>
#include <stdint.h>

class LIDARLite_v4LED
{
private:
  TwoWire *_i2cPort;      //generic connection to user's chosen I2C I2C port
  uint8_t _deviceAddress; //I2C address of the button/switch

  //Register map
  enum
  {
    ACQ_COMMANDS = 0x00,
    STATUS = 0x01,
    ACQUISITION_COUNT = 0x05,
    FULL_DELAY_LOW = 0x10,
    FULL_DELAY_HIGH = 0x11,
    UNIT_ID_0 = 0x16,
    UNIT_ID_1 = 0x17,
    UNIT_ID_2 = 0x18,
    UNIT_ID_3 = 0x19,
    I2C_SEC_ADDR = 0x1A,
    I2C_CONFIG = 0x1B,
    DETECTION_SENSITIVITY = 0x1C,
    LIB_VERSION = 0x30,
    CORR_DATA = 0x52,
    CP_VER_LO = 0x72,
    CP_VER_HI = 0x73, 
    BOARD_TEMPERATURE = 0xE0,
    HARDWARE_VERSION = 0xE1,
    POWER_MODE = 0xE2,
    MEASUREMENT_INTERVAL = 0xE3,
    FACTORY_RESET = 0xE4,
    QUICK_TERMINATION = 0xE5,
    START_BOOTLOADER = 0xE6,
    ENABLE_FLASH_STORAGE = 0xEA,
    HIGH_ACCURACY_MODE = 0xEB,
    SOC_TEMPERATURE = 0xEC,
    ENABLE_ANT_RADIO = 0xF0,
  };

public:
  //Device status
  bool begin(uint8_t address = LIDARLITE_ADDR_DEFAULT, TwoWire &wirePort = Wire); //Sets device I2C address to a user-specified address, over whatever port the user specifies.
  bool isConnected();                                                             //Returns true if the button/switch will acknowledge over I2C, and false otherwise

  //LIDAR configure
  void configure(uint8_t configuration = 0);                                 //Configure LIDAR to one of several measurement configurations
  bool setI2Caddr(uint8_t newAddress, bool disableDefaultI2CAddress = true); //Configures the connected device to attach to the I2C bus using the specified address
  bool useDefaultAddress();
  bool useNewAddressOnly();
  bool useBothAddresses();
  void enableFlash(bool enable); //Toggle between RAM and FLASH/NVM storage

  //Get distance measurement helper functions
  void takeRange();        //Initiate a distance measurement by writing to register 0x00
  void waitForBusy();      //Blocking function to wait until the LIDAR Lite's internal busy flag goes low
  uint8_t getBusyFlag();   //Read BUSY flag from device registers. Function will return 0x00 if not busy
  uint16_t readDistance(); //Read and return the result of the most recent distance measurement in centimeters

  //Get distance measurement function
  uint16_t getDistance(); //Asks for, waits, and returns new measurement reading in centimeters

  //Gpio functions
  void takeRangeGpio(uint8_t triggerPin, uint8_t monitorPin); //Initiate a distance measurement by toggling the trigger pin
  void waitForBusyGpio(uint8_t monitorPin);                   //Blocking function to wait until the LIDAR Lite's internal busy flag goes low
  uint8_t getBusyFlagGpio(uint8_t monitorPin);                //Check BUSY status via Monitor pin. Function will return 0x00 if not busy

  // Extra functions if the user needs them
  // Only use these functions if you have read the datasheet an know what you're doing
  uint8_t getBoardTemp(); // Reads the BOARD_TEMPERATURE register and returns the two's complement value in celcius
  uint8_t getSOCTemp();   // Reads the SOC_TEMPERATURE register and returns the two's complement value in celcius
  bool setPowerModeAlwaysOn();  // Writes 0xFF to POWER_MODE register. The coprocessor is never turned off, allowing for the fastest measurement possible. This is the default
  bool setPowerModeAsync(); // Writes 0x00 to POWER_MODE register. The coprocessor is always off unless a distance measurement is requested or register access is required
  bool enableHighAccuracyMode(bool enable); // By default, LIDAR operates in high accuracy mode. Disable high accuracy mode if you wish to operate in asynchronous mode.  
  bool factoryReset();    // Resets the NVM/Flash storage information back to default settings and executes a SoftDevice reset

  //Internal I2C abstraction
  bool write(uint8_t regAddr, uint8_t *dataBytes, uint8_t numBytes); //Perform I2C write to the device. Can specify the number of bytes to be written
  void read(uint8_t regAddr, uint8_t *dataBytes, uint8_t numBytes);  //Perform I2C read from device. Can specify the number of bytes to be read

  void correlationRecordRead(int16_t *correlationArray, uint8_t numberOfReadings = 192);
};

#endif
