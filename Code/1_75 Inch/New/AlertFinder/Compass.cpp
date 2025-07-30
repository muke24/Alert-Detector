/**
 * @file      Compass.cpp
 * @author    Peter Thompson
 * @brief     Implementation of the Compass module.
 * @version   1.90 (Final Saturation Fix)
 * @date      2025-07-31
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "Compass.h"
#include <Wire.h>
#include "SensorQMI8658.hpp"
// Using the correct Adafruit library for the HMC5883L sensor at address 0x1E
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_Sensor.h>
#include "MadgwickAHRS.h"
#include "HWCDC.h" // For USBSerial printing

// --- Sensor Objects ---
static SensorQMI8658 qmi;
static Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
static Madgwick filter;

// --- Module State ---
static bool qmi_ready = false;
static bool mag_ready = false;
static float heading = 0.0f;
static int orientation = 0; // 0 = normal, 1 = flipped

// Magnetic declination for Springfield, NSW, Australia (approximate)
const float MAGNETIC_DECLINATION = 12.5;
const byte HMC5883L_ADDRESS = 0x1E; // Confirmed I2C address

/**
 * @brief Initializes the sensors on the main I2C bus.
 */
bool compass_setup() {
    // The QMI8658 is on the main I2C bus (Wire)
    if (qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS)) {
        qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_1000Hz, SensorQMI8658::LPF_MODE_3);
        qmi.configGyroscope(SensorQMI8658::GYR_RANGE_512DPS, (SensorQMI8658::GyroODR)SensorQMI8658::ACC_ODR_1000Hz, SensorQMI8658::LPF_MODE_3);
        qmi_ready = true;
        USBSerial.println("Compass module setup: QMI8658 IMU FOUND.");
    } else {
        USBSerial.println("Compass module setup: QMI8658 IMU NOT FOUND.");
    }

    // Manually check if a device acknowledges the HMC5883L's I2C address.
    Wire.beginTransmission(HMC5883L_ADDRESS);
    if (Wire.endTransmission() == 0) {
        if (mag.begin()) {
            // ** SATURATION FIX **
            // The default gain is too high and causes the Z-axis to saturate.
            // We set it to the lowest sensitivity (highest range) to prevent this.
            mag.setMagGain(HMC5883_MAGGAIN_8_1);
            mag_ready = true;
            USBSerial.println("Compass module setup: HMC5883L FOUND.");
        } else {
            mag_ready = false;
            USBSerial.println("Compass module setup: HMC5883L detected but library init failed.");
        }
    } else {
        mag_ready = false;
        USBSerial.println("Compass module setup: HMC5883L NOT FOUND (no I2C device at address).");
    }

    // Match the filter's sample rate to the sensor's ODR (1000Hz)
    filter.begin(1000); 

    return qmi_ready && mag_ready;
}

/**
 * @brief Reads sensors and updates the filter. Continuously tries to connect to missing sensors.
 */
void compass_loop() {
    // Periodically re-attempt connection to sensors if they failed initially.
    static unsigned long last_reconnect_attempt = 0;
    if (millis() - last_reconnect_attempt > 2000) { // Try every 2 seconds
        if (!qmi_ready) {
            if (qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS)) {
                qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_1000Hz, SensorQMI8658::LPF_MODE_3);
                qmi.configGyroscope(SensorQMI8658::GYR_RANGE_512DPS, (SensorQMI8658::GyroODR)SensorQMI8658::ACC_ODR_1000Hz, SensorQMI8658::LPF_MODE_3);
                qmi_ready = true;
                USBSerial.println("Compass module re-check: QMI8658 IMU FOUND.");
            }
        }
        if (!mag_ready) {
            Wire.beginTransmission(HMC5883L_ADDRESS);
            if (Wire.endTransmission() == 0) {
                if(mag.begin()) {
                    mag.setMagGain(HMC5883_MAGGAIN_8_1);
                    mag_ready = true;
                    USBSerial.println("Compass module re-check: HMC5883L FOUND.");
                }
            }
        }
        last_reconnect_attempt = millis();
    }


    IMUdata acc, gyr;
    sensors_event_t mag_event;

    bool got_accel = false;
    bool got_gyro = false;
    bool got_mag = false;

    if (qmi_ready && qmi.getDataReady()) {
        if (qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
            got_accel = true;
        }
        if (qmi.getGyroscope(gyr.x, gyr.y, gyr.z)) {
            got_gyro = true;
        }
    }

    if (mag_ready) {
        mag.getEvent(&mag_event);
        got_mag = true;
        
        // --- MAGNETOMETER DEBUGGING ---
        static unsigned long last_mag_print = 0;
        if(millis() - last_mag_print > 500) { // Print every 500ms
            USBSerial.printf("Mag X: %.2f, Y: %.2f, Z: %.2f uT\n", mag_event.magnetic.x, mag_event.magnetic.y, mag_event.magnetic.z);
            last_mag_print = millis();
        }
    }

    if (got_accel && got_gyro && got_mag) {
        // The Madgwick filter requires gyroscope data in radians/sec.
        float gyr_x_rad = gyr.x * DEG_TO_RAD;
        float gyr_y_rad = gyr.y * DEG_TO_RAD;
        float gyr_z_rad = gyr.z * DEG_TO_RAD;

        // Remap magnetometer axes to align with the IMU's coordinate frame.
        float mag_x = mag_event.magnetic.y;
        float mag_y = mag_event.magnetic.x;
        float mag_z = -mag_event.magnetic.z;

        // Update the filter with the new, corrected sensor data.
        filter.update(gyr_x_rad, gyr_y_rad, gyr_z_rad,
                      acc.x, acc.y, acc.z,
                      mag_x, mag_y, mag_z);

        // Get orientation data from the filter
        float roll = filter.getRoll();
        float pitch = filter.getPitch();
        float yaw = filter.getYaw();

        // Determine orientation (upside down or not)
        if (roll > 90.0f || roll < -90.0f) {
            orientation = 1; // Flipped
        } else {
            orientation = 0; // Normal
        }

        // Calculate True North heading
        heading = yaw + MAGNETIC_DECLINATION;
        if (heading < 0) {
            heading += 360;
        }
        if (heading >= 360) {
            heading -= 360;
        }
    }
}

/**
 * @brief Returns the calculated heading.
 */
float get_compass_heading() {
    return heading;
}

/**
 * @brief Returns the calculated orientation.
 */
int get_orientation() {
    return orientation;
}
