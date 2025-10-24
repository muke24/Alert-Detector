// compass.h
// Description: Manages the orientation sensor (LSM6DSOX + LIS3MDL).

#ifndef COMPASS_H
#define COMPASS_H

// Initializes the LSM6DSOX and LIS3MDL sensors.
// Must be called once in setup().
void initCompass();

// Checks for new data from the IMU, calculates the tilt-compensated true heading,
// and updates the global 'currentDirection' variable.
// Handles re-initialization attempts on failure.
// Must be called repeatedly in the main loop().
void updateIMU();

#endif // COMPASS_H