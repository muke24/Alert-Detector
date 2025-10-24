// sim7000g_gps.h
#ifndef SIM7000G_GPS_H
#define SIM7000G_GPS_H

// Initializes the modem and enables the GPS receiver.
// This should be called once in the main setup().
void initGPS();

// Reads available data from the modem to update the global GPS location.
// This should be called repeatedly in the main loop().
void updateGPS();

// Prints the current GPS coordinates to the Serial monitor for debugging.
// This should be called repeatedly in the main loop().
void printGPS();

#endif // SIM7000G_GPS_H