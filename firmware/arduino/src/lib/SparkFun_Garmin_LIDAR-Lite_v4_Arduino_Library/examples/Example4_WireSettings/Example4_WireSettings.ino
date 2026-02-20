/******************************************************************************
  Initializes the LIDAR using Wire1 and a fast clock speed. Distance measurements
  printed to serial port.

  Priyanka Makin @ SparkX Labs
  Original Creation Date: Sept 30, 2019

  This code is Lemonadeware; if you see me (or any other SparkFun employee) at the
  local, and you've found our code helpful, please buy us a round!

  Hardware Connections:
  Plug Qwiic LIDAR into Qwiic RedBoard using Qwiic connector.
  Compilation will fail if your platform doesn't have multiple I2C ports.
  Set serial monitor to 115200 baud.

  Distributed as-is; no warranty is given.
******************************************************************************/
#include <LIDARLite_v4LED.h>
#include <Wire.h>

LIDARLite_v4LED myLIDAR; //Click here to get the library: http://librarymanager/All#SparkFun_LIDARLitev4 by SparkFun

void setup() {
  Serial.begin(115200);
  Serial.println("Qwiic LIDAR examples");
  Wire1.begin();  //Compilation will fail here if your platform doesn't have multiple I2C ports
  Wire1.setClock(400000); //set I2C communication to 400kHz

  //check if LIDAR will acknowledge over I2C
  if (myLIDAR.begin(0x6F, Wire1) == false) {
    Serial.println("Device did not acknowledge! Freezing.");
    while(1);
  }
  Serial.println("LIDAR acknowledged!");
}

void loop() {
  float newDistance;

  newDistance = myLIDAR.getDistance();

  //Print to serial port
  Serial.print("New distance: ");
  Serial.print(newDistance/100);
  Serial.println(" m");

  delay(20);  //Don't hammer too hard on the I2C bus
}
