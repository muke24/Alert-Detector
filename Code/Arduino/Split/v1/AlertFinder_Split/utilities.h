// utilities.h
#ifndef UTILITIES_H
#define UTILITIES_H

#include <Arduino.h>
#include "globals.h" // We need access to the data structures like Location and Alert

// Function Declarations
BoundingArea boundingBox(Location location, float distanceInKm);
float calculateDistance(Location loc1, Location loc2);
float calculateAngle(Location from, Location to);
float normalizeAngle(float angle);
float calculateFacingDirection(Alert alert);
float calculateMultiplier(float distance);

#endif // UTILITIES_H