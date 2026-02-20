/* SparkFun Ulrasonic Distance Sensor - Example 3 - Distance using Trigger and Echo Pins
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

// Here we set the device address. Note that an older version of the Qwiic
// Ultrasonic firmware used a default address of 0x00. If yours uses 0x00,
// you'll need to change the address below. It is also recommended to run
// Example 2 to change the address to the new default.
uint8_t deviceAddress = kQwiicUltrasonicDefaultAddress; // 0x2F
// uint8_t deviceAddress = 0x00;

// Adjust these to your setup.
const int triggerPin = 7; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 8; // Echo Pin of Ultrasonic Sensor

// Used for distance calculation
float distance = 0.0;
float duration = 0.0;
const float speedOfSound = 340.00; // Speed of sound in m/s
const float millisPerSecond= 1000.00; // Number of milliseconds in a second
     
void setup() {

  Wire.begin(); 

  Serial.begin(115200);  
  Serial.println("Ultrasonic Distance Sensor - Example 2 Distance using Trigger and Echo Pins.");

  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Attempt to begin the sensor
  while (myUltrasonic.begin(deviceAddress) == false)
  {
    Serial.println("Ultrasonic sensor not connected, check your wiring and I2C address!");
    delay(2000);
  }

  Serial.println("Ultrasonic sensor connected!");
}

void loop() {

  digitalWrite(triggerPin, HIGH);
  delay(5);
  digitalWrite(triggerPin, LOW);
  // We don't want continually trigger while data is being retrieved from the sensor.

  duration = pulseIn(echoPin, HIGH);
  // Time until sound detected * speed of sound * conversion to mm 
  // Divide by two because we only want the time the wave traveled to the object, 
  // not to the object and back.
  distance = (duration * speedOfSound * millisPerSecond) / 2; 

  // Print measurement
  Serial.print("Distance (mm): ");
  Serial.println(distance);

  //Serial.println("Distance (cm): "); 
  //Serial.print((distance / 10.0), 2);         

  //Serial.println("Distace (in): "); 
  //Serial.print((distance / 25.4), 2);         

  delay(500);
}
