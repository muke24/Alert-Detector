/**
 * @file      Compass.h
 * @author    Peter Thompson
 * @brief     Header for the Compass module, which fuses data from a QMI8658
 * IMU and an HMC5883L magnetometer to provide stable heading
 * and orientation data.
 * @version   1.00
 * @date      2025-07-30
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef COMPASS_H
#define COMPASS_H

/**
 * @brief Initializes the QMI8658 IMU and HMC5883L magnetometer.
 * @return True if both sensors were initialized successfully, false otherwise.
 */
bool compass_setup();

/**
 * @brief Reads sensor data and updates the internal orientation calculations.
 * This should be called as frequently as possible in the main loop.
 */
void compass_loop();

/**
 * @brief Gets the current compass heading, corrected for magnetic declination.
 * @return The heading in degrees (0-360), where 0 is true North.
 */
float get_compass_heading();

/**
 * @brief Determines if the device is oriented upside down.
 * @return 1 if the device is flipped, 0 if it is in the normal orientation.
 */
int get_orientation();

#endif // COMPASS_H
