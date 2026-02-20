/* SparkFun Ulrasonic Distance Sensor - Example 4 Changing the Ultrasonic's Address
 * To reset the original I2C address, ground the "RST" pad on the backside of the board.
 * by touching a wire to the pad and then to ground. The address will be reset to 0x2F.
 *
 * Product:
 *  *  SparkFun Qwiic Ultrasonic Distance Sensor - HC-SR04 (SEN-1XXXX)
 *  *  https://www.sparkfun.com/1XXXX
 *
 * Written By: Elias Santistevan
 * Date: 06/2024
 *
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2024 SparkFun Electronics
 */

#include "SparkFun_Qwiic_Ultrasonic_Arduino_Library.h"

// Create an ultrasonic sensor object
QwiicUltrasonic myUltrasonic;
// If you're using the older version of the Ultrasonic Distance
// sensor, then uncomment the line below and comment out the line above.
// You can check the version in copper by the "R" on the front of the board.
// Newer versions are >v10 and older versions simply do not have a version number.
// QwiicUltrasonic myUltrasonic(kQwiicUltrasonicFWOld);

// Here we set the device address. Note that an older version of the Qwiic
// Ultrasonic firmware used a default address of 0x00. If yours uses 0x00,
// you'll need to change the address below. It is also recommended to run
// Example 2 to change the address to the new default.
uint8_t deviceAddress = kQwiicUltrasonicDefaultAddress; // 0x2F
// uint8_t deviceAddress = 0x00;

// New addres is 7-bit unshifted.
uint8_t newAddr = 0x20;
// If using an older version of the Qwiic Ultrasonic, your address range is: 0x20 - 0x2F
// uint8_t newAddr = 0x2F;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("Ultrasonic Distance Sensor - Example 4 - Change Address");

    Wire.begin();

    // This sketch wont' run until you open the serial monitor
    while (!Serial)
        ;

    // Attempt to begin the sensor
    while (myUltrasonic.begin(deviceAddress) == false)
    {
        Serial.println("Ultrasonic sensor not connected, check your wiring and I2C address!");
        delay(2000);
    }

    Serial.println("Ready to change address.");
    delay(1000);

    Serial.print("Changing Address To: ");
    Serial.println(newAddr, HEX);

    // Call change address.....
    sfTkError_t err = myUltrasonic.changeAddress(newAddr);

    if (err != ksfTkErrOk)
    {
        while (1)
        {
            Serial.print("Error changing address: ");
            Serial.println(err);
            delay(1000);
        }
    }

    Serial.println("Address changed successfully!");
    Serial.println("Reading Distance at new address......");
    delay(3000);
}

void loop()
{
    uint16_t distance = 0;
    myUltrasonic.triggerAndRead(distance);

    // Print measurement
    Serial.print("Distance (mm): ");
    Serial.println(distance);

    // Serial.println("Distance (cm): ");
    // Serial.print((distance / 10.0), 2);

    // Serial.println("Distace (in): ");
    // Serial.print((distance / 25.4), 2);

    // Wait a bit
    delay(100);
}
