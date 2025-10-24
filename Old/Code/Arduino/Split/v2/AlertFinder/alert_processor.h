// alert_processor.h
#ifndef ALERT_PROCESSOR_H
#define ALERT_PROCESSOR_H

#include "global_types.h" // Required for Location, Alert, BoundingArea structs

// Processes the global 'currentAlerts' array. Finds the closest police alert,
// calculates the relative direction, and sends the result to the display ESP32.
// This is called by the network_manager after it successfully fetches new data.
void processAlerts();

// --- Utility Functions ---

// Calculates the distance in kilometers between two GPS coordinates using the Haversine formula.
float calculateDistance(Location loc1, Location loc2);

// Creates a square bounding box around a given location.
BoundingArea boundingBox(Location location, float distanceInKm);

// Calculates the initial bearing (angle) from a 'from' location to a 'to' location.
float calculateAngle(Location from, Location to);

// Normalizes an angle to the range of -180 to +180 degrees.
float normalizeAngle(float angle);

// Calculates the direction of an alert relative to the device's current heading.
float calculateFacingDirection(const Alert& alert);

// Calculates the LED animation speed multiplier based on the distance to an alert.
float calculateMultiplier(float distance);

#endif // ALERT_PROCESSOR_H