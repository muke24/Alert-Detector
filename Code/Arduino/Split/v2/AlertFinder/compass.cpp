// compass.cpp (Corrected)
#include "compass.h"
#include "config.h"
#include "global_types.h"
#include "network_manager.h"
#include <Wire.h>
#include <math.h>

// Timestamps for internal logic
static unsigned long lastImuPrint = 0;
static unsigned long lastImuRetry = 0;

// =========================================================================
// PRIVATE HELPER FUNCTION
// =========================================================================
// Calculates heading with tilt compensation. Must be defined before it is called.
static float getTiltCompensatedHeading(sensors_event_t* accel_event, sensors_event_t* mag_event) {
    // Get accelerometer readings
    float ax = accel_event->acceleration.x;
    float ay = accel_event->acceleration.y;
    float az = accel_event->acceleration.z;

    // Calculate roll and pitch
    float roll = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay * ay + az * az));

    // Get magnetometer readings
    float mx = mag_event->magnetic.x;
    float my = mag_event->magnetic.y;
    float mz = mag_event->magnetic.z;
    
    // Apply tilt compensation
    float xh = mx * cos(pitch) + my * sin(roll) * sin(pitch) - mz * cos(roll) * sin(pitch);
    float yh = my * cos(roll) + mz * sin(roll);

    // Calculate heading in radians
    float heading_rad = atan2(yh, xh);

    // Convert heading to degrees
    float heading_deg = heading_rad * 180.0 / PI;

    // Normalize to 0-360
    if (heading_deg < 0) {
        heading_deg += 360;
    }
    
    return heading_deg;
}


// =========================================================================
// PUBLIC FUNCTIONS
// =========================================================================

void initCompass() {
    Wire.begin(IMU_SDA, IMU_SCL);

    if (dsox.begin_I2C() && lis3mdl.begin_I2C()) {
        Serial.println("LSM6DSOX and LIS3MDL initialized successfully!");
        imuInitialized = true;

        dsox.setAccelDataRate(LSM6DS_RATE_104_HZ);
        dsox.setGyroDataRate(LSM6DS_RATE_104_HZ);
        lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
        lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
        // <<< FIX 1: The correct enum is LIS3MDL_HIGHMODE, not LIS3MDL_HIGH_PERFORMANCE_MODE
        lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE);
        lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

    } else {
        Serial.println("Failed to find LSM6DSOX or LIS3MDL. Check wiring!");
        imuInitialized = false;
    }
}

void updateIMU() {
    unsigned long currentTime = millis();

    if (!imuInitialized) {
        if (currentTime - lastImuRetry >= IMU_RETRY_INTERVAL_MS) {
            Serial.println("Retrying IMU initialization...");
            initCompass();
            lastImuRetry = currentTime;
        }
        return;
    }

    // <<< FIX 2: Added 'temp' event for the required third argument
    sensors_event_t accel, gyro, mag, temp;
    
    // <<< FIX 2: dsox.getEvent() now has the correct number of arguments
    if (dsox.getEvent(&accel, &gyro, &temp) && lis3mdl.getEvent(&mag)) {
        
        float headingMagnetic = getTiltCompensatedHeading(&accel, &mag);
        float declination = calculateDeclination(currentLocation.latitude, currentLocation.longitude);
        float headingTrue = headingMagnetic + declination;
        
        if (headingTrue >= 360) headingTrue -= 360;
        if (headingTrue < 0) headingTrue += 360;

        currentDirection = headingTrue;

        if (currentTime - lastImuPrint >= GPS_IMU_PRINT_INTERVAL_MS) {
            Serial.print("IMU Magnetic Heading: ");
            Serial.print(headingMagnetic, 1);
            Serial.print("° | Declination: ");
            Serial.print(declination, 2);
            Serial.print("° | True Heading: ");
            Serial.print(headingTrue, 1);
            Serial.println("°");
            lastImuPrint = currentTime;
        }
    } else {
        Serial.println("Failed to read from IMU sensors.");
    }
}