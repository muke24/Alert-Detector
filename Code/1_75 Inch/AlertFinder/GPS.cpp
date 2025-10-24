/**
 * @file      GPS.cpp
 * @author    Peter Thompson
 * @brief     Implementation for the LC76G GPS module driver.
 * @version   1.00
 * @date      2025-07-30
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "GPS.h"
#include <Wire.h>
#include <TinyGPS++.h>

// --- GPS Module Configuration ---
// According to the Waveshare Wiki for the ESP32-S3-Touch-AMOLED-1.75,
// the GPS module uses a separate I2C bus.
#define GPS_I2C_SDA         1
#define GPS_I2C_SCL         2
#define GPS_I2C_ADDRESS     0x10 // Default I2C address for LC76G

// --- I2C Bus and GPS Objects ---
static TwoWire I2C_GPS = TwoWire(0); // Use I2C bus 0 for the GPS
static TinyGPSPlus gps;
static GPSData current_gps_data;

/**
 * @brief Initializes the I2C bus for the GPS module.
 */
void gps_setup() {
    I2C_GPS.begin(GPS_I2C_SDA, GPS_I2C_SCL);
}

/**
 * @brief Reads NMEA sentences from the GPS module and parses them.
 */
void gps_loop() {
    // Request a chunk of data. The LC76G continuously streams NMEA sentences.
    // 128 bytes is a safe amount to read to capture a few sentences at a time.
    I2C_GPS.requestFrom((uint8_t)GPS_I2C_ADDRESS, (size_t)128);

    // Read all available bytes and feed them to the TinyGPS++ parser
    while (I2C_GPS.available()) {
        gps.encode(I2C_GPS.read());
    }

    // When the parser has processed a valid location sentence, update our data struct.
    // gps.location.isUpdated() is true for one loop cycle after a new RMC or GGA sentence.
    if (gps.location.isUpdated() && gps.location.isValid()) {
        current_gps_data.latitude = gps.location.lat();
        current_gps_data.longitude = gps.location.lng();
        current_gps_data.is_valid = true;
    }
}

/**
 * @brief Provides access to the last known valid GPS data.
 */
GPSData get_gps_data() {
    return current_gps_data;
}
