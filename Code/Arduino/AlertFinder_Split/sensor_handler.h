// sensor_handler.h
#ifndef SENSOR_HANDLER_H
#define SENSOR_HANDLER_H

#include "globals.h"

// Function Declarations
void updateGPS();
void printGPS(unsigned long currentTime);
void updateIMU(unsigned long currentTime);

#endif // SENSOR_HANDLER_H