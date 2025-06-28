// SUCCESSFUL TEST SCRIPT FOR BNO086 USED WITH LILYGO T-SIM7000G V1.1
// Tested to be reliable and works successfully everytime without any issues.
// BNO086_Test.ino

#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"

BNO08x myIMU;

// Pin definitions for LILYGO T-SIM7000G V1.1
#define BNO08X_INT  19  // GPIO19 for interrupt
#define BNO08X_RST  23  // GPIO23 for reset
#define BNO08X_ADDR 0x4B  // Default I2C address (0x4A if ADR jumper closed)

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);  // Wait for Serial on native USB boards
  Serial.println("BNO086 True North Test");

  Wire.begin();  // Default I2C pins: SDA=GPIO21, SCL=GPIO22

  if (!myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST)) {
    Serial.println("BNO08x not detected. Check connections and address.");
    while (1);
  }
  Serial.println("BNO08x detected");

  setReports();
}

void setReports() {
  if (myIMU.enableGeomagneticRotationVector(100)) {  // Report every 100ms
    Serial.println("Geomagnetic rotation vector enabled");
  } else {
    Serial.println("Failed to enable geomagnetic rotation vector");
  }
}

void loop() {
  delay(10);

  if (myIMU.wasReset()) {
    Serial.println("Sensor reset detected");
    setReports();
  }

  if (myIMU.getSensorEvent()) {
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR) {
      float w = myIMU.getQuatReal();
      float x = myIMU.getQuatI();
      float y = myIMU.getQuatJ();
      float z = myIMU.getQuatK();

      // Calculate yaw (heading) in radians
      float yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
      float headingMagnetic = yaw * (180.0 / PI);
      if (headingMagnetic < 0) headingMagnetic += 360;

      // Adjust for true north with magnetic declination
      // Replace with your location's declination (e.g., 10.0 for 10°E, -14.0 for 14°W)
      float declination = 0.0;  // Placeholder: set your local value
      float headingTrue = headingMagnetic + declination;
      if (headingTrue >= 360) headingTrue -= 360;
      if (headingTrue < 0) headingTrue += 360;

      Serial.print("Magnetic Heading: ");
      Serial.print(headingMagnetic);
      Serial.print("° | True Heading: ");
      Serial.print(headingTrue);
      Serial.println("°");
    }
  }
}