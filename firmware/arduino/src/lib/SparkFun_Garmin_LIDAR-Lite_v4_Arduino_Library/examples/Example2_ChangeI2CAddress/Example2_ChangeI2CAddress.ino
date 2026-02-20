/******************************************************************************
  Changes the I2C address of the LIDAR from the default to what the user specifies.

  Priyanka Makin @ SparkX Labs
  Original Creation Date: Sept 30, 2019

  This code is Lemonadeware; if you see me (or any other SparkFun employee) at the
  local, and you've found our code helpful, please buy us a round!

  Hardware Connections:
  Plug Qwiic LIDAR into Qwiic RedBoard using Qwiic cable.
  Set serial monitor to 115200 baud.

  Distributed as-is; no warranty is given.
******************************************************************************/
#include <LIDARLite_v4LED.h>

LIDARLite_v4LED myLIDAR; //Click here to get the library: http://librarymanager/All#SparkFun_LIDARLitev4 by SparkFun

void setup() {
  Serial.begin(115200);
  Serial.println("Qwiic LIDARLite_v4 examples");
  Wire.begin(); //Join I2C bus

  //check if LIDAR will acknowledge over I2C
  if (myLIDAR.begin() == false) {
    Serial.println("Device did not acknowledge! Running scanner.");
  }

  else {
    Serial.println("Device acknowledged!");

    Serial.println();
    Serial.println("Enter a new I2C address for the Qwiic LIDAR to use!");
    Serial.println("Don't use the 0x prefix. For instance, if you wanted to");
    Serial.println("change the address to 0x5B, you would enter 5B and press enter.");
    Serial.println();
    Serial.println("One more thing! Make sure your line ending is set to 'Both NL & CR' ");
    Serial.println("in the Serial Monitor.");
    Serial.println();

    //Wait for serial data to be available
    while (Serial.available() == 0);

    if (Serial.available()) {
      uint8_t newAddress = 0;
      String stringBuffer = Serial.readString();
      char charBuffer[10];
      stringBuffer.toCharArray(charBuffer, 10);
      uint8_t success = sscanf(charBuffer, "%x", &newAddress);

      if (success) {
        if (newAddress > 0x08 && newAddress < 0x77) {
          Serial.println("Character recieved, and device address is valid!");
          Serial.print("Attempting to set device address to 0x");
          Serial.println(newAddress, HEX);

          if (myLIDAR.setI2Caddr(newAddress, true) == true) {
            Serial.println("Device address set succeeded!");
          }
          else {
            Serial.println("Device address set failed!");
          }

          delay(100); //give the hardware time to do whatever configuration it needs to do

          if (myLIDAR.isConnected()) {
            Serial.println("Device will acknowledge on new I2C address!");
          }
          else {
            Serial.println("Device will not acknowledge on new I2C address.");
          }
        }

        else {
          Serial.println("Address out of range! Try an address between 0x08 and 0x77");
        }
      }

      else {
        Serial.println("Invalid text, try again.");
      }
    }
    delay(100);
  }
}

void loop() {
  //if no I2C device found or Qwiic button correctly sest to new address,
  //scan for available I2C devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    //The i2c_scanner uses the return value of
    //the Write.endTransmission to see if
    //a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);  //wait 5 seconds for next scan

}
