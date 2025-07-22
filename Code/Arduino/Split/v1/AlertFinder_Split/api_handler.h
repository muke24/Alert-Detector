// api_handler.h
#ifndef API_HANDLER_H
#define API_HANDLER_H

#include "globals.h"

// Function Declarations

// --- Connection Management ---
void maintainWiFi();

// --- NOAA WMM API Functions ---
bool fetchWmmData(Location location, DeclinationData &declination);
float calculateDeclination(float latitude, float longitude);

// --- Waze API Functions ---
void fetchWazeData(unsigned long currentTime);
void processAlerts(unsigned long currentTime);

// --- Inter-device UART Communication ---
void receiveData(unsigned long currentTime);
void handleCommSerial(unsigned long currentTime);

#endif // API_HANDLER_H