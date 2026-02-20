/******************************************************************************
  Reads the distance measurements of two LIDARs daisy-chained together.

  Priyanka Makin @ SparkX Labs
  Original Creation Date: Sept 30, 2019

  This code is Lemonadeware; if you see me (or any other SparkFun employee) at the
  local, and you've found our code helpful, please buy us a round!

  Hardware Connections:
  Plug Qwiic LIDAR into Qwiic RedBoard using Qwiic cable. 
  Add another Qwiic LIDAR using Qwiic cable.
  For this example to work, the two Qwiic LIDAR's need to have different addresses. 
  One is at the default 0x62 and the other at 0x5B. Revisit example 2 to change the
  address of your Qwiic LIDAR sensor.
  Set serial monitor to 115200 baud.

  Distributed as-is; no warranty is given.
******************************************************************************/

//For this example to work, you must daisy chain together two Qwiic LIDAR sensors with different I2C addresses
//To change the address of a Qwiic LIDAR sensor, please visit example 2

#include <LIDARLite_v4LED.h> //Click here to get the library: http://librarymanager/All#SparkFun_LIDARLitev4 by SparkFun

//instantiate both LIDARs
LIDARLite_v4LED myLIDAR1;
LIDARLite_v4LED myLIDAR2;

void setup() {
  Serial.begin(115200);
  Serial.println("Qwiic LIDARLite_v4 examples");
  Wire.begin(); //Join I2C bus

  //check if LIDARs will acknowledge over I2C
  //Connect to Qwiic LIDAR at address 0x5B
  if (myLIDAR1.begin(0x5B) == false) {
    Serial.println("LIDAR 1 did not acknowledge! Freezing.");
    while(1);
  }
  //Connect to Qwiic button at default address, 0x6F
  if (myLIDAR2.begin() == false) {
    Serial.println("LIDAR 2 did not acknowledge! Freezing.");
    while(1);
  }
  Serial.println("Both LIDARs acknowledged.");
}

void loop() {
  float newDistance1;
  float newDistance2;

  newDistance1 = myLIDAR1.getDistance();
  newDistance2 = myLIDAR2.getDistance();

  //Print to Serial port
  Serial.print("LIDAR 1 distance: ");
  Serial.print(newDistance1/100);
  Serial.println(" m");
  Serial.print("LIDAR 2 distance: ");
  Serial.print(newDistance2/100);
  Serial.println(" m");

  delay(20); //Don't hammer too hard on the I2C bus
}
