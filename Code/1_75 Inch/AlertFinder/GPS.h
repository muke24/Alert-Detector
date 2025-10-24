/**
 * @file      GPS.h
 * @author    Peter Thompson
 * @brief     Header file for the GPS module, designed for the LC76G on the
 * Waveshare 1.75" AMOLED display. It handles I2C communication
 * and NMEA parsing.
 * @version   1.00
 * @date      2025-07-30
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <Arduino.h>

// Struct to hold processed GPS data
struct GPSData {
    float latitude = 0.0;
    float longitude = 0.0;
    bool is_valid = false;
};

/**
 * @brief Initializes the GPS module on its dedicated I2C bus.
 */
void gps_setup();

/**
 * @brief Reads and processes available GPS data from the I2C bus.
 * This should be called frequently in the main loop to keep data fresh.
 */
void gps_loop();

/**
 * @brief Returns the most recent valid GPS data.
 * @return A GPSData struct containing latitude, longitude, and a validity flag.
 */
GPSData get_gps_data();

#endif // GPS_MODULE_H
