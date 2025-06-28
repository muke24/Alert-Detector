// sensor_handler.cpp
#include "sensor_handler.h"
#include "api_handler.h"      // Needed for calculateDeclination()
#include "initialization.h"   // Needed for initBNO086() and setReports() for re-initialization

/**
 * @brief Updates the current location by polling the SIM7000G's GPS.
 * It checks the direct NMEA feed as a fallback.
 */
void updateGPS() {
  float lat, lon;
  // First, try the modem's direct GPS command
  if (modem.getGPS(&lat, &lon)) {
    if (lat != 0 && lon != 0) {
      currentLocation.latitude = lat;
      currentLocation.longitude = lon;
      if (!isLocationInitialized) {
        lastCheckedLocation = currentLocation;
        lastWmmLocation = currentLocation;
        isLocationInitialized = true;
      }
      return; // Got location, we are done
    }
  }

  // If that fails, parse the raw NMEA data stream
  while (SerialAT.available() > 0) {
    gps.encode(SerialAT.read());
  }

  if (gps.location.isValid()) {
    currentLocation.latitude = gps.location.lat();
    currentLocation.longitude = gps.location.lng();
    if (!isLocationInitialized) {
      lastCheckedLocation = currentLocation;
      lastWmmLocation = currentLocation;
      isLocationInitialized = true;
    }
  }
}

/**
 * @brief Prints the current GPS coordinates to the Serial Monitor periodically.
 */
void printGPS(unsigned long currentTime) {
  if (currentTime - lastGpsPrint >= printInterval) {
    if (isLocationInitialized) {
      Serial.print("GPS Lat: "); Serial.print(currentLocation.latitude, 6);
      Serial.print(", Lon: "); Serial.println(currentLocation.longitude, 6);
    } else {
      Serial.println("GPS: No fix");
    }
    lastGpsPrint = currentTime;
  }
}

/**
 * @brief Updates the IMU data, calculates true heading, and handles sensor resets.
 */
void updateIMU(unsigned long currentTime) {
  if (!bnoInitialized) {
    if (currentTime - lastBnoRetry >= retryInterval) {
      Serial.println("Attempting to re-initialize BNO086...");
      initBNO086(); // This function is in initialization.cpp
      lastBnoRetry = currentTime;
    }
    return;
  }

  if (myIMU.wasReset()) {
    Serial.println("BNO086 sensor reset detected. Re-enabling reports.");
    setReports(); // This function is in initialization.cpp
  }

  // Check for new sensor data
  if (myIMU.getSensorEvent()) {
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR) {
      float w = myIMU.getQuatReal();
      float x = myIMU.getQuatI();
      float y = myIMU.getQuatJ();
      float z = myIMU.getQuatK();
      imuFailureCount = 0; // Reset failure count on successful read

      // Calculate yaw (heading) in radians from the quaternion
      float yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
      float headingMagnetic = yaw * (180.0 / PI);
      if (headingMagnetic < 0) headingMagnetic += 360;

      // Adjust for true north using magnetic declination
      float declination = calculateDeclination(currentLocation.latitude, currentLocation.longitude);
      float headingTrue = headingMagnetic + declination;
      if (headingTrue >= 360) headingTrue -= 360;
      if (headingTrue < 0) headingTrue += 360;
      
      currentDirection = headingTrue;

      if (currentTime - lastImuPrint >= printInterval) {
        Serial.print("IMU True Heading: "); Serial.print(headingTrue, 1);
        Serial.print("° (Mag: "); Serial.print(headingMagnetic, 1);
        Serial.print("°, Decl: "); Serial.print(declination, 1);
        Serial.println("°)");
        lastImuPrint = currentTime;
      }
    }
  } else {
    // Optional: track consecutive read failures
    // imuFailureCount++;
    // if (imuFailureCount > maxImuFailures) {
    //   Serial.println("Max IMU read failures reached. Flagging for re-initialization.");
    //   bnoInitialized = false;
    //   imuFailureCount = 0;
    // }
  }
}