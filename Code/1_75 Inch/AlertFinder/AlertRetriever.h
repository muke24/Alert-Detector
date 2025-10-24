/**
 * @file      AlertRetriever.h
 * @author    Peter Thompson
 * @brief     Header file for the AlertRetriever module. This module is responsible
 * for fetching, parsing, and processing alert data from web APIs.
 * @version   1.00
 * @date      2025-07-29
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef ALERT_RETRIEVER_H
#define ALERT_RETRIEVER_H

#include <Arduino.h>

// --- Data Structures ---

// Represents a geographical location.
struct Location {
  float latitude;
  float longitude;
};

// Represents a single alert from the API.
struct Alert {
  String type;
  String subtype;
  Location location;
  String street;
  float distance_km;
  float relative_angle;
};

// Holds the final, processed data ready for the UI renderer.
struct DisplayData {
  int alert_indices[5] = {-1, -1, -1, -1, -1}; // Indices for renderer (0=police)
  int heat_level = 0;                          // Number of active alerts
  float target_angle = 0;                      // Angle to the primary alert
  int target_distance_m = -1;                  // Distance in meters to primary alert
  bool data_is_new = false;                    // Flag for the main loop
};


// --- Public Function Declarations ---

/**
 * @brief Initializes the AlertRetriever module.
 */
void retriever_setup();

/**
 * @brief Main loop function for the retriever. Should be called repeatedly.
 * It internally manages when to fetch new data based on time and distance.
 */
void retriever_loop();

/**
 * @brief Updates the retriever with the device's current location.
 * @param lat The current latitude.
 * @param lon The current longitude.
 */
void set_current_location(float lat, float lon);

/**
 * @brief Updates the retriever with the device's current true heading.
 * @param heading The current direction in degrees (0-360, where 0 is North).
 */
void set_current_direction(float heading);

/**
 * @brief Informs the retriever about the network connection status.
 * @param is_connected True if a Wi-Fi or cellular connection is available.
 */
void set_network_status(bool is_connected);

/**
 * @brief Retrieves the latest processed data for the display.
 * @return A DisplayData struct containing all necessary UI information.
 */
DisplayData get_display_data();

#endif // ALERT_RETRIEVER_H
