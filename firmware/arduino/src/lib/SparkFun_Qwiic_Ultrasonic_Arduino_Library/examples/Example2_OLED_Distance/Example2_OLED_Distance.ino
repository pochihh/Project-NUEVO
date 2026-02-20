/* SparkFun Ulrasonic Distance Sensor - Example 2 Basic Distance Sensing on an OLED Display
 * 
 * Products: 
 *  *  SparkFun Qwiic Ultrasonic Distance Sensor - HC-SR04 (SEN-1XXXX)
 *  *  https://www.sparkfun.com/1XXXX
 *  *  SparkFun Qwiic Narrow OLED Display (LCD-1XXXX)
 *  *  https://www.sparkfun.com/1XXXX
 *
 * Link to OLED library: https://github.com/sparkfun/SparkFun_Qwiic_OLED_Arduino_Library
 * 
 * Written By: Elias Santistevan
 * Date: 06/2024
 *
 * SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2024 SparkFun Electronics
 */

#include "SparkFun_Qwiic_OLED.h"
// For the narrow LED, I prefer the slightly larger font included in the OLED library.
// This is completely optional and can be deleted or commented out. By default the font
// is slightly smaller. 
#include "res/qw_fnt_8x16.h"
#include "SparkFun_Qwiic_Ultrasonic_Arduino_Library.h"

// Create an ultrasonic sensor object
QwiicUltrasonic myUltrasonic;
// Creat an OLED object
QwiicNarrowOLED myOLED;

char distanceBuff[4] = {}; 
String distanceStr = "";
int centerX; 
int centerY; 

// Here we set the device address. Note that an older version of the Qwiic
// Ultrasonic firmware used a default address of 0x00. If yours uses 0x00,
// you'll need to change the address below. It is also recommended to run
// Example 2 to change the address to the new default.
uint8_t deviceAddress = kQwiicUltrasonicDefaultAddress; // 0x2F
// uint8_t deviceAddress = 0x00;

void setup()
{

  Serial.begin(115200); // Default config settings
  Serial.println("Ultrasonic Distance Sensor - Example 4 - Distance on an OLED Display");
  Wire.begin();

  while (myOLED.begin() == false) {
    Serial.println("OLED sensor not connected, check your wiring and I2C address!");
    delay(1000);
  }
  while(myUltrasonic.begin(deviceAddress) == false)
  {
    Serial.println("Ultrasonic sensor not connected, check your wiring and I2C address!");
    delay(1000);
  }

  String hello = "Hello, Ultrasonic!";

  // This is good for the narrow OLED screen. You can also just remove this 
  // and it will default to a slightly smaller font. 
  myOLED.setFont(QW_FONT_8X16);

  // This will center the text onto the screen. 
  int x0 = (myOLED.getWidth() - myOLED.getStringWidth(hello)) / 2;
  int y0 = (myOLED.getHeight() - myOLED.getStringHeight(hello)) / 2;

  myOLED.text(x0, y0, hello);

  // There's nothing on the screen yet - Now send the graphics to the device
  myOLED.display();
  delay(2000);
}

void loop() 
{
  uint16_t distance = 0;
  myUltrasonic.triggerAndRead(distance);

  // Convert distance, which is an integer, to char so that we can print it.
  snprintf(distanceBuff, 6, "%d", distance);

  // Put the distance in a string so that we can also print "mm".
  distanceStr = distanceBuff; 
  distanceStr += "mm";

  myOLED.erase();
  centerX = (myOLED.getWidth() - myOLED.getStringWidth(distanceStr)) / 2;
  centerY = (myOLED.getHeight() - myOLED.getStringHeight(distanceStr)) / 2;
  myOLED.text(centerX, centerY, distanceStr);
  myOLED.display();

  delay(250);
}


