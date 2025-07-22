#include "alert_processor.h"
#include "config.h"
#include "global_types.h"

void processAlerts() {
    // We only stop if there are no alerts. We proceed even if the IMU is not ready.
    if (alertCount == 0) {
        Serial.println("Processing: No alerts in the area.");
        
        closestIndex = -1;
        
        taskENTER_CRITICAL(&timerMux);
        multiplier = 1.0f;
        alertAngle = 999.0f; // Set sentinel value for "no alert"
        taskEXIT_CRITICAL(&timerMux);
        
        return;
    }

    // Add a log if the IMU is not initialized, but don't stop.
    if (!imuInitialized) {
        Serial.println("Processing with default heading (0 North) as IMU is not ready.");
    }

    int foundPoliceIndex = -1;
    float minDistance = -1.0;

    // Find the closest "POLICE" alert
    for (int i = 0; i < alertCount; i++) {
        if (currentAlerts[i].type == "POLICE") {
            float distance = calculateDistance(currentLocation, currentAlerts[i].location);
            if (foundPoliceIndex == -1 || distance < minDistance) {
                minDistance = distance;
                foundPoliceIndex = i;
            }
        }
    }

    if (foundPoliceIndex >= 0) {
        closestIndex = foundPoliceIndex;
        Alert& alert = currentAlerts[closestIndex];
        // This will now use currentDirection (defaulting to 0.0f if IMU is offline)
        float relativeAngle = calculateFacingDirection(alert); 
        
        taskENTER_CRITICAL(&timerMux);
        multiplier = calculateMultiplier(minDistance);
        alertAngle = relativeAngle;
        taskEXIT_CRITICAL(&timerMux);

        Serial.println("Closest Police Alert Found:");
        Serial.println("  Type: " + alert.type + ", Subtype: " + alert.subtype);
        Serial.println("  Street: " + (alert.street.isEmpty() ? "N/A" : alert.street));
        Serial.println("  Distance: " + String(minDistance, 2) + " km");
        Serial.println("  Relative Angle: " + String(relativeAngle, 1) + "° (0° is ahead)");
        Serial.println("  LED Multiplier: " + String(multiplier, 2));

    } else {
        Serial.println("Processing: No police alerts found in the current data.");
        closestIndex = -1;
        
        taskENTER_CRITICAL(&timerMux);
        multiplier = 1.0f;
        alertAngle = 999.0f;
        taskEXIT_CRITICAL(&timerMux);
    }
}

// --- Utility Function Implementations ---

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

float calculateDistance(Location loc1, Location loc2) {
    if (loc1.latitude == 0 || loc2.latitude == 0) return 999999.0f;

    float lat1_rad = loc1.latitude * PI / 180.0;
    float lat2_rad = loc2.latitude * PI / 180.0;
    float dLat = lat2_rad - lat1_rad;
    float dLon = (loc2.longitude - loc1.longitude) * PI / 180.0;

    float a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1_rad) * cos(lat2_rad) * sin(dLon / 2) * sin(dLon / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    
    return 6371.0 * c;
}

float calculateAngle(Location from, Location to) {
    float phi1 = from.latitude * PI / 180.0;
    float phi2 = to.latitude * PI / 180.0;
    float deltaLambda = (to.longitude - from.longitude) * PI / 180.0;

    float y = sin(deltaLambda) * cos(phi2);
    float x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(deltaLambda);
    
    float theta = atan2(y, x);
    float bearing = fmod((theta * 180.0 / PI + 360.0), 360.0);
    return bearing;
}

float normalizeAngle(float angle) {
    while (angle <= -180) angle += 360;
    while (angle > 180) angle -= 360;
    return angle;
}

float calculateFacingDirection(const Alert& alert) {
    float bearingToAlert = calculateAngle(currentLocation, alert.location);
    float relativeAngle = bearingToAlert - currentDirection;
    return normalizeAngle(relativeAngle);
}

float calculateMultiplier(float distance) {
    if (distance >= MAX_ALERT_DISTANCE_KM) return 1.0f;
    if (distance <= 0) return 2.0f;
    return 2.0f - (distance / MAX_ALERT_DISTANCE_KM);
}