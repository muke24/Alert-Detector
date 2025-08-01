/**
 * @file      Compass.cpp
 * @author    Peter Thompson
 * @brief     Implementation of the Compass module, now using LIS3MDL Magnetometer.
 * @version   2.03 (Corrected Axis Remapping)
 * @date      2025-07-31
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "Compass.h"
#include <Wire.h>
#include "SensorQMI8658.hpp"
// Using the Adafruit LIS3MDL library for the new magnetometer.
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include "MadgwickAHRS.h"
#include "HWCDC.h" // For USBSerial printing

// --- Sensor Objects ---
static SensorQMI8658 qmi;
// Instantiate the LIS3MDL sensor object.
static Adafruit_LIS3MDL mag;
static Madgwick filter;

// --- Module State ---
static bool qmi_ready = false;
static bool mag_ready = false;
static float heading = 0.0f;
static int orientation = 0; // 0 = normal, 1 = flipped

// Magnetic declination for Springfield, NSW, Australia (approximate)
const float MAGNETIC_DECLINATION = 12.5;

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

    // Initialize the LIS3MDL magnetometer.
    // The begin_I2C() function returns true if the sensor is found and initialized.
    if (mag.begin_I2C()) {
        // Configure the LIS3MDL for best performance and to avoid saturation.
        // Set range to a high value to prevent magnetic saturation.
        mag.setRange(LIS3MDL_RANGE_16_GAUSS);
        // Set performance mode and data rate to match the IMU and filter.
        mag.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
        mag.setDataRate(LIS3MDL_DATARATE_1000_HZ);
        
        mag_ready = true;
        USBSerial.println("Compass module setup: LIS3MDL FOUND.");
    } else {
        mag_ready = false;
        USBSerial.println("Compass module setup: LIS3MDL NOT FOUND.");
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
            if(mag.begin_I2C()) {
                mag.setRange(LIS3MDL_RANGE_16_GAUSS);
                mag.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
                mag.setDataRate(LIS3MDL_DATARATE_1000_HZ);
                mag_ready = true;
                USBSerial.println("Compass module re-check: LIS3MDL FOUND.");
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
        // Read data from the LIS3MDL
        mag.getEvent(&mag_event);
        got_mag = true;
    }

    // --- RAW SENSOR VALUE DEBUGGING ---
    static unsigned long last_debug_print = 0;
    if (millis() - last_debug_print > 500) { // Print every 500ms
        if (got_accel) {
            USBSerial.printf("Acc X: %.2f, Y: %.2f, Z: %.2f g\n", acc.x, acc.y, acc.z);
        }
        if (got_gyro) {
            USBSerial.printf("Gyr X: %.2f, Y: %.2f, Z: %.2f dps\n", gyr.x, gyr.y, gyr.z);
        }
        if (got_mag) {
            USBSerial.printf("Mag X: %.2f, Y: %.2f, Z: %.2f uT\n", mag_event.magnetic.x, mag_event.magnetic.y, mag_event.magnetic.z);
        }
        last_debug_print = millis();
    }

    if (got_accel && got_gyro && got_mag) {
        // The Madgwick filter requires gyroscope data in radians/sec.
        float gyr_x_rad = gyr.x * DEG_TO_RAD;
        float gyr_y_rad = gyr.y * DEG_TO_RAD;
        float gyr_z_rad = gyr.z * DEG_TO_RAD;

        // --- AXIS REMAPPING to align LIS3MDL with QMI8658 ---
        // This mapping is based on the provided physical orientations.
        // QMI8658 (IMU) is the reference coordinate system for the filter.
        
        // Filter's X (Up/Down) must match QMI's X-axis.
        // QMI's X is Up/Down. LIS3MDL's Z is Up/Down.
        // Therefore, mag_x = LIS3MDL's Z reading.
        float mag_x = mag_event.magnetic.z;

        // Filter's Y (Left/Right) must match QMI's Y-axis.
        // QMI's Y is Left/Right. LIS3MDL's X is Left/Right.
        // Therefore, mag_y = LIS3MDL's X reading.
        float mag_y = mag_event.magnetic.x;

        // Filter's Z (Fwd/Bwd) must match QMI's Z-axis.
        // QMI's Z is Fwd/Bwd. LIS3MDL's Y is Fwd/Bwd.
        // Therefore, mag_z = LIS3MDL's Y reading.
        float mag_z = mag_event.magnetic.y;

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
