/**
 * @file      AlertRetriever.cpp
 * @author    Peter Thompson
 * @brief     Implementation of the AlertRetriever module. Handles all logic
 * for API communication, data parsing, and alert processing.
 * @version   1.00
 * @date      2025-07-29
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "AlertRetriever.h"
#include <ArduinoJson.h>
#include <math.h>

#ifndef PI
#define PI 3.1415926535f
#endif

// --- Configuration Constants ---
const float MAX_DISTANCE_KM = 5.0f;       // Max distance for alerts (kilometers)
const float CHECK_INTERVAL_S = 45.0f;     // Interval to check for new alerts (seconds)
const float MOVEMENT_THRESHOLD_KM = 0.2f; // Distance to trigger alert check (kilometers)
const char* WAZE_HOST = "www.waze.com";
const char* WAZE_PATH = "/live-map/api/georss";

// --- Module-Internal State Variables ---
static Location currentLocation = {0, 0};
static Location lastCheckedLocation = {0, 0};
static float currentDirection = 0;
static bool isLocationInitialized = false;
static bool isNetworkConnected = false;
static unsigned long lastApiCallTime = 0;

static Alert all_alerts[20]; // Static array to hold up to 20 alerts
static int total_alert_count = 0;

static DisplayData displayData;

// --- Helper Structures ---
struct BoundingArea {
  float left, bottom, right, top;
};

// --- Forward Declarations for Internal Functions ---
static void fetchWazeData();
static void processAlerts();
static float calculateDistance(Location loc1, Location loc2);
static float calculateAngle(Location from, Location to);
static float normalizeAngle(float angle);
BoundingArea boundingBox(Location location, float distanceInKm);

/**
 * @brief Provides a weak implementation of the network GET function.
 * The main sketch (.ino) MUST provide a strong implementation of this function
 * using the specific networking library for the target hardware (e.g., WiFiClient, TinyGSM).
 * @param host The server host name.
 * @param path The API path.
 * @param payload A reference to a String to store the response body.
 * @return True on success, false on failure.
 */
extern "C" bool perform_http_get(const char* host, const char* path, String& payload) __attribute__((weak));
extern "C" bool perform_http_get(const char* host, const char* path, String& payload) {
    // This is a dummy function. The real implementation must be in the main sketch.
    return false;
}


// --- Public Function Implementations ---

void retriever_setup() {
    lastApiCallTime = - (long)(CHECK_INTERVAL_S * 1000); // Allows first check to run immediately
    displayData.data_is_new = false;
}

void retriever_loop() {
    if (!isLocationInitialized || !isNetworkConnected) {
        return; // Wait for valid location and network
    }

    unsigned long currentTime = millis();
    float distanceMoved = calculateDistance(currentLocation, lastCheckedLocation);
    float timeSinceLastCheck = (currentTime - lastApiCallTime) / 1000.0f;

    // Fetch data if check interval has passed or if we've moved enough
    if (timeSinceLastCheck >= CHECK_INTERVAL_S || distanceMoved > MOVEMENT_THRESHOLD_KM) {
        fetchWazeData();
        lastCheckedLocation = currentLocation;
        lastApiCallTime = currentTime;
    }
}

void set_current_location(float lat, float lon) {
    if (lat != 0 && lon != 0) {
        currentLocation.latitude = lat;
        currentLocation.longitude = lon;
        if (!isLocationInitialized) {
            lastCheckedLocation = currentLocation;
            isLocationInitialized = true;
        }
    }
}

void set_current_direction(float heading) {
    currentDirection = heading;
}

void set_network_status(bool is_connected) {
    isNetworkConnected = is_connected;
}

DisplayData get_display_data() {
    // Atomically get the data and reset the "new" flag
    noInterrupts();
    DisplayData dataToReturn = displayData;
    displayData.data_is_new = false;
    interrupts();
    return dataToReturn;
}


// --- Internal Logic Functions ---

/**
 * @brief Fetches alert data from the Waze API.
 */
static void fetchWazeData() {
    BoundingArea area = boundingBox(currentLocation, MAX_DISTANCE_KM);
    String path = String(WAZE_PATH) + "?top=" + String(area.top, 6) +
                  "&bottom=" + String(area.bottom, 6) +
                  "&left=" + String(area.left, 6) +
                  "&right=" + String(area.right, 6) +
                  "&env=row&types=alerts";

    String payload;
    if (!perform_http_get(WAZE_HOST, path.c_str(), payload)) {
        // Network call failed, handled in the implementation
        total_alert_count = 0; // Clear alerts on failure
        processAlerts();
        return;
    }

    DynamicJsonDocument doc(4096); // Increased size for more complex responses
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
        total_alert_count = 0; // Clear alerts on failure
        processAlerts();
        return;
    }

    JsonArray alerts = doc["alerts"];
    total_alert_count = 0;
    for (JsonObject alert_json : alerts) {
        if (total_alert_count >= 20) break; // Don't exceed array bounds

        // We only care about police alerts for now
        String type = alert_json["type"].as<String>();
        if (type == "POLICE") {
            Alert& current = all_alerts[total_alert_count];
            current.type = type;
            current.subtype = alert_json["subtype"].as<String>();
            current.location.latitude = alert_json["location"]["y"].as<float>();
            current.location.longitude = alert_json["location"]["x"].as<float>();
            current.street = alert_json["street"].as<String>();
            total_alert_count++;
        }
    }

    processAlerts();
}

/**
 * @brief Processes the raw alert data into a format for the UI.
 */
static void processAlerts() {
    // Reset display data
    DisplayData newDisplayData;
    newDisplayData.heat_level = 0;
    newDisplayData.target_distance_m = -1;

    if (total_alert_count == 0) {
        noInterrupts();
        displayData = newDisplayData;
        displayData.data_is_new = true;
        interrupts();
        return;
    }

    int closestIndex = -1;
    float minDistance = MAX_DISTANCE_KM + 1.0f;

    // First pass: calculate distances and angles for all alerts
    for (int i = 0; i < total_alert_count; i++) {
        all_alerts[i].distance_km = calculateDistance(currentLocation, all_alerts[i].location);

        // Calculate relative angle to the alert
        float angleToAlert = calculateAngle(currentLocation, all_alerts[i].location);
        all_alerts[i].relative_angle = normalizeAngle(angleToAlert - currentDirection);

        // Find the closest alert
        if (all_alerts[i].distance_km < minDistance) {
            minDistance = all_alerts[i].distance_km;
            closestIndex = i;
        }
    }

    // Populate the display data struct
    newDisplayData.heat_level = total_alert_count;
    
    // Fill alert indices for the renderer (0 for police)
    for(int i = 0; i < 5 && i < total_alert_count; ++i) {
        newDisplayData.alert_indices[i] = 0; // 0 is the type for "police"
    }

    if (closestIndex != -1) {
        newDisplayData.target_angle = all_alerts[closestIndex].relative_angle;
        newDisplayData.target_distance_m = (int)(all_alerts[closestIndex].distance_km * 1000);
    }

    noInterrupts();
    displayData = newDisplayData;
    displayData.data_is_new = true;
    interrupts();
}


// --- Math and Utility Functions ---

static float calculateDistance(Location loc1, Location loc2) {
    float lat1 = loc1.latitude * PI / 180.0;
    float lon1 = loc1.longitude * PI / 180.0;
    float lat2 = loc2.latitude * PI / 180.0;
    float lon2 = loc2.longitude * PI / 180.0;
    float dLat = lat2 - lat1;
    float dLon = lon2 - lon1;
    float a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return 6371.0 * c; // Distance in kilometers
}

static float calculateAngle(Location from, Location to) {
    float lat1 = from.latitude * PI / 180.0;
    float lon1 = from.longitude * PI / 180.0;
    float lat2 = to.latitude * PI / 180.0;
    float lon2 = to.longitude * PI / 180.0;
    float dLon = lon2 - lon1;
    float y = sin(dLon) * cos(lat2);
    float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    float bearing = atan2(y, x) * 180.0 / PI;
    return fmod((bearing + 360.0), 360.0); // Normalize to 0-360
}

static float normalizeAngle(float angle) {
    angle = fmod(angle, 360.0);
    if (angle > 180.0) {
        angle -= 360.0;
    } else if (angle <= -180.0) {
        angle += 360.0;
    }
    return angle;
}

BoundingArea boundingBox(Location location, float distanceInKm) {
    float latInRadians = location.latitude * PI / 180.0;
    float deltaLatitude = distanceInKm / 111.0;
    float deltaLongitude = distanceInKm / (111.0 * cos(latInRadians));
    BoundingArea area;
    area.left = location.longitude - deltaLongitude;
    area.bottom = location.latitude - deltaLatitude;
    area.right = location.longitude + deltaLongitude;
    area.top = location.latitude + deltaLatitude;
    return area;
}
