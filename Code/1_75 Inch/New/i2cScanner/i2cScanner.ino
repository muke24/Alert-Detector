#include <Wire.h>
#include "HWCDC.h" // For USB Serial on your specific board

void setup() {
  // Use the main I2C bus pins defined in your project
  // For your board, these are GPIO 15 (SDA) and GPIO 14 (SCL)
  Wire.begin(15, 14); 
  USBSerial.begin(115200);
  while (!USBSerial) {
    ; // wait for serial port to connect.
  }
  USBSerial.println("\nI2C Scanner");
}

void loop() {
  byte error, address;
  int nDevices;

  USBSerial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      USBSerial.print("I2C device found at address 0x");
      if (address < 16) {
        USBSerial.print("0");
      }
      USBSerial.println(address, HEX);
      nDevices++;
    } else if (error == 4) {
      USBSerial.print("Unknown error at address 0x");
      if (address < 16) {
        USBSerial.print("0");
      }
      USBSerial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    USBSerial.println("No I2C devices found\n");
  } else {
    USBSerial.println("done\n");
  }

  delay(2000); // wait 5 seconds for next scan
}