// network_manager.h
#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <Arduino.h>

// Initializes the network connection. Tries Wi-Fi first, then fails over to cellular.
// Call this once in setup().
void initNetwork();

// Maintains the network connection. If Wi-Fi is lost, it tries to reconnect or switch to cellular.
// Call this in the main loop().
void maintainNetworkConnection();

// Fetches Waze alert data for the current location.
// Call this in the main loop().
void fetchWazeData();

// Manages fetching and caching of the magnetic declination from the NOAA WMM API.
// Returns the declination in degrees for a given location.
// This is called by the IMU module.
float calculateDeclination(float latitude, float longitude);

#endif // NETWORK_MANAGER_H