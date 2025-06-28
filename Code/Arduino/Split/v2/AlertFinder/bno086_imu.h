// bno086_imu.h

// BNO086 IS TRASH. USE ADAFRUIT LSM6DSOX + LIS3MDL INSTEAD!!!
// It is newer, way more accurate, and is also used in aviation.

#ifndef BNO086_IMU_H
#define BNO086_IMU_H

// Initializes the BNO086 sensor.
// Must be called once in setup().
void initBNO086();

// Checks for new data from the IMU, calculates the true heading,
// and updates the global 'currentDirection' variable.
// Handles sensor resets and re-initialization attempts.
// Must be called repeatedly in the main loop().
void updateIMU();

#endif // BNO086_IMU_H