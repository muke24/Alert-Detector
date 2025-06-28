// utilities.cpp
#include "utilities.h"

/**
 * @brief Calculates a square bounding box around a given location.
 * @param location The center point.
 * @param distanceInKm The distance to the edges of the box from the center.
 * @return A BoundingArea struct.
 */
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

/**
 * @brief Calculates the distance between two GPS coordinates using the Haversine formula.
 * @param loc1 The first location.
 * @param loc2 The second location.
 * @return The distance in kilometers.
 */
float calculateDistance(Location loc1, Location loc2) {
  float lat1 = loc1.latitude * PI / 180.0;
  float lat2 = loc2.latitude * PI / 180.0;
  float lon1 = loc1.longitude * PI / 180.0;
  float lon2 = loc2.longitude * PI / 180.0;
  
  float dLat = lat2 - lat1;
  float dLon = lon2 - lon1;
  
  float a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  
  return 6371.0 * c; // Distance in kilometers
}

/**
 * @brief Calculates the initial bearing (angle) to travel from one coordinate to another.
 * @param from The starting location.
 * @param to The destination location.
 * @return The bearing in degrees (-180 to 180).
 */
float calculateAngle(Location from, Location to) {
  float phi1 = from.latitude * PI / 180.0;
  float phi2 = to.latitude * PI / 180.0;
  float deltaLambda = (to.longitude - from.longitude) * PI / 180.0;
  
  float y = sin(deltaLambda) * cos(phi2);
  float x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(deltaLambda);
  float theta = atan2(y, x);
  
  float bearing = (theta * 180.0 / PI + 360) - 180;
  return bearing;
}

/**
 * @brief Normalizes an angle to the range -180 to 180 degrees.
 * @param angle The angle to normalize.
 * @return The normalized angle.
 */
float normalizeAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

/**
 * @brief Calculates the direction of an alert relative to the device's current heading.
 * @param alert The alert to check.
 * @return The relative angle in degrees (-180 to 180, where 0 is straight ahead).
 */
float calculateFacingDirection(Alert alert) {
  float rawAngle = calculateAngle(currentLocation, alert.location);
  float relativeAngle = rawAngle - currentDirection;
  return normalizeAngle(relativeAngle);
}

/**
 * @brief Calculates an animation speed multiplier based on the distance to an alert.
 * @param distance The distance to the alert in kilometers.
 * @return A multiplier from 1.0 (far) to 2.0 (close).
 */
float calculateMultiplier(float distance) {
  if (distance >= maxDistanceKm) return 1.0f;
  if (distance <= 0) return 2.0f;
  
  // Linear interpolation: as distance decreases, multiplier increases.
  return 2.0f - (distance / maxDistanceKm); 
}