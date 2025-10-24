#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
BNO08x myIMU;

// Pin definitions for BNO08x interrupt and reset on LILYGO T-SIM7000G V1.1
#define BNO08X_INT  19  // GPIO19 (VSPI_MISO, available)
#define BNO08X_RST  0   // GPIO0 (available)
#define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B

void setup() {
  Serial.begin(9600); // For debugging, uses GPIO1 (TX) and GPIO3 (RX)
  
  // Initialize I2C on GPIO21 (SDA) and GPIO22 (SCL)
  Wire.begin(21, 22);

  // Initialize BNO08x with INT and RST pins
  if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
    Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }
  Serial.println("BNO08x found!");

  // Optional: Increase I2C speed to 400kHz (uncomment if needed)
  // Wire.setClock(400000);

  setReports();

  Serial.println("Reading events");
  delay(100);
}

// Define sensor outputs to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (myIMU.enableRotationVector() == true) {
     Serial.println(F("Rotation vector enabled"));
     Serial.println(F("Output in form real, i, j, k, accuracy"));
  } else {
     Serial.println("Could not enable rotation vector");
  }
}

void loop() {
  delay(10);

  if (myIMU.wasReset()) {
    // Serial.print("sensor was reset ");
    setReports();
  }

  // Check for new sensor data
  if (myIMU.getSensorEvent() == true) {
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
      float quatI = myIMU.getQuatI();
      float quatJ = myIMU.getQuatJ();
      float quatK = myIMU.getQuatK();
      float quatReal = myIMU.getQuatReal();
      float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();

      Serial.print(quatReal, 2);
      Serial.print(F(","));
      Serial.print(quatI, 2);
      Serial.print(F(","));
      Serial.print(quatJ, 2);
      Serial.print(F(","));
      Serial.print(quatK, 2);
      Serial.println();
    }
  }
}