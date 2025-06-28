// bno086_imu.cpp
#include "bno086_imu.h"
#include "config.h"
#include "global_types.h"
#include "network_manager.h" // Will be created next, needed for calculateDeclination()
#include <Wire.h>

// Helper function prototype (local to this file)
static void setReports();

// Timestamps for internal logic
static unsigned long lastImuPrint = 0;
static unsigned long lastBnoRetry = 0;

void initBNO086() {
    Wire.begin(BNO08X_SDA, BNO08X_SCL);
    if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST)) {
        Serial.println("BNO086 initialized successfully!");
        bnoInitialized = true;
        setReports();
    } else {
        Serial.println("Failed to find BNO086. Check wiring!");
        bnoInitialized = false;
    }
}

void updateIMU() {
    unsigned long currentTime = millis();

    // If the BNO is not initialized, periodically try to initialize it.
    if (!bnoInitialized) {
        if (currentTime - lastBnoRetry >= BNO_RETRY_INTERVAL_MS) {
            Serial.println("Retrying BNO086 initialization...");
            initBNO086();
            lastBnoRetry = currentTime;
        }
        return;
    }

    // Check if the sensor has reset itself
    if (myIMU.wasReset()) {
        Serial.println("BNO086 sensor reset detected. Re-enabling reports.");
        setReports();
    }

    // Check for a new sensor event
    if (myIMU.getSensorEvent()) {
        // We are only interested in the Geomagnetic Rotation Vector report
        if (myIMU.getSensorEventID() == SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR) {
            float w = myIMU.getQuatReal();
            float x = myIMU.getQuatI();
            float y = myIMU.getQuatJ();
            float z = myIMU.getQuatK();

            // Calculate yaw (heading) from the quaternion
            float yaw_rad = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
            float headingMagnetic = yaw_rad * (180.0 / PI);
            if (headingMagnetic < 0) {
                headingMagnetic += 360;
            }

            // Get magnetic declination to adjust for true north
            // This function will be defined in the network_manager module
            float declination = calculateDeclination(currentLocation.latitude, currentLocation.longitude);

            // Calculate true heading
            float headingTrue = headingMagnetic + declination;
            if (headingTrue >= 360) headingTrue -= 360;
            if (headingTrue < 0) headingTrue += 360;

            // Update the global direction variable
            currentDirection = headingTrue;

            // Print the heading data periodically for debugging
            if (currentTime - lastImuPrint >= GPS_IMU_PRINT_INTERVAL_MS) {
                Serial.print("BNO086 Magnetic Heading: ");
                Serial.print(headingMagnetic, 1);
                Serial.print("° | Declination: ");
                Serial.print(declination, 2);
                Serial.print("° | True Heading: ");
                Serial.print(headingTrue, 1);
                Serial.println("°");
                lastImuPrint = currentTime;
            }
        }
    }
}

// Private helper function to configure the reports from the BNO086
static void setReports() {
    Serial.println("Setting BNO086 reports...");
    // Enable the Geomagnetic Rotation Vector - this gives absolute heading
    // with respect to magnetic north, corrected for tilt.
    if (myIMU.enableGeomagneticRotationVector(100)) { // Report every 100ms
        Serial.println("Geomagnetic rotation vector enabled.");
    } else {
        Serial.println("Failed to enable geomagnetic rotation vector.");
    }
}