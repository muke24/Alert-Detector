// AlertFinder.ino
// This sketch fetches police alerts from the Waze API using GPS coordinates from a NEO-M8N module,
// calculates the relative angle to the closest police alert using BNO08X IMU data, and sends the angle
// via UART1 (loopback on GPIO18 TX to GPIO19 RX for testing). It connects to Wi-Fi for API access
// and logs data to the Serial Monitor for debugging.

// Libraries
#include <HardwareSerial.h> // For UART communication (GPS and loopback)
#include <TinyGPS++.h>      // For parsing GPS data from NEO-M8N
#include <Wire.h>           // For I2C communication with BNO08X IMU
#include <Adafruit_BNO08x.h> // For BNO08X IMU
#include <WiFi.h>           // For Wi-Fi connectivity
#include <HTTPClient.h>     // For HTTP requests to Waze API
#include <ArduinoJson.h>    // For parsing JSON responses from Waze API

// Configuration Constants
// Wi-Fi credentials
const char* ssid = "BigCock69";     // Wi-Fi SSID
const char* password = "GymBro69";  // Wi-Fi password

// Waze API settings (from Unity script)
const float maxDistanceKm = 1.0f;      // Max distance for alerts (kilometers)
const float checkInterval = 15.0f;     // Interval to check for new alerts (seconds)
const float movementThreshold = 0.2f;  // Distance to trigger alert check (kilometers)
const char* baseUrl = "https://www.waze.com/live-map/api/georss"; // Waze API endpoint

// Timing constants
const unsigned long printInterval = 500;         // Print GPS/IMU data every 500ms
const unsigned long retryInterval = 5000;        // Retry BNO08X initialization every 5 seconds
const unsigned long receivePrintInterval = 1000; // Print received UART data every 1000ms
const int maxImuFailures = 5;                   // Max consecutive IMU failures before reinitialization

// Hardware Objects
HardwareSerial gpsSerial(2);   // UART2 for GPS (GPIO16 RX, GPIO17 TX)
HardwareSerial commSerial(1);  // UART1 for loopback communication (GPIO19 RX, GPIO18 TX)
TinyGPSPlus gps;              // TinyGPS++ object for parsing GPS data
Adafruit_BNO08x bno08x;       // BNO08X IMU object for orientation data

// State Variables
bool bnoInitialized = false;            // Tracks BNO08X initialization status
int imuFailureCount = 0;                // Counts consecutive IMU data failures
unsigned long lastGpsPrint = 0;         // Timestamp for last GPS print
unsigned long lastImuPrint = 0;         // Timestamp for last IMU print
unsigned long lastBnoRetry = 0;         // Timestamp for last BNO08X retry
unsigned long lastApiCall = 0;          // Timestamp for last Waze API call
unsigned long lastReceivePrint = 0;     // Timestamp for last UART receive print
float timeSinceLastCheck = 0;           // Timer for periodic API checks (seconds)
bool isLocationInitialized = false;     // Tracks if GPS location is initialized
float currentDirection = 0;             // Current IMU heading (degrees)

// Data Structures
struct Location {
  float latitude;   // Latitude in degrees
  float longitude;  // Longitude in degrees
};
Location currentLocation = {0, 0};      // Current GPS location
Location lastCheckedLocation = {0, 0};  // Last location checked for alerts

struct Alert {
  String type;       // Alert type (e.g., "POLICE")
  String subtype;    // Alert subtype
  Location location; // Alert location (lat, lon)
  String street;     // Street name of alert
};
Alert* currentAlerts = nullptr; // Dynamic array of current alerts
int alertCount = 0;             // Number of alerts in currentAlerts

struct BoundingArea {
  float top;    // Top latitude of bounding box
  float bottom; // Bottom latitude of bounding box
  float left;   // Left longitude of bounding box
  float right;  // Right longitude of bounding box
};

// Utility Functions

// Calculates bounding box around a location for Waze API
// Parameters: location - Center location; distanceInKm - Radius in kilometers
// Returns: BoundingArea struct with top, bottom, left, right coordinates
BoundingArea boundingBox(Location location, float distanceInKm) {
  float latInRadians = location.latitude * PI / 180.0; // Convert latitude to radians for trigonometric calculations
  float deltaLatitude = distanceInKm / 111.0; // Approx 111 km per degree of latitude, calculates latitude span
  float deltaLongitude = distanceInKm / (111.0 * cos(latInRadians)); // Adjust longitude span based on latitude (shrinks near poles)

  BoundingArea area;
  area.left = location.longitude - deltaLongitude;  // Set left boundary of bounding box
  area.bottom = location.latitude - deltaLatitude;  // Set bottom boundary of bounding box
  area.right = location.longitude + deltaLongitude; // Set right boundary of bounding box
  area.top = location.latitude + deltaLatitude;     // Set top boundary of bounding box
  return area; // Return the computed bounding box
}

// Calculates distance between two locations using Haversine formula
// Parameters: loc1, loc2 - Locations to compare
// Returns: Distance in kilometers
float calculateDistance(Location loc1, Location loc2) {
  float lat1 = loc1.latitude * PI / 180.0;  // Convert loc1 latitude to radians
  float lat2 = loc2.latitude * PI / 180.0;  // Convert loc2 latitude to radians
  float lon1 = loc1.longitude * PI / 180.0; // Convert loc1 longitude to radians
  float lon2 = loc2.longitude * PI / 180.0; // Convert loc2 longitude to radians

  float dLat = lat2 - lat1; // Calculate latitude difference
  float dLon = lon2 - lon1; // Calculate longitude difference
  float a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2); // Haversine formula: compute great-circle distance
  float c = 2 * atan2(sqrt(a), sqrt(1 - a)); // Angular distance on sphere
  return 6371.0 * c; // Multiply by Earth's radius to get distance in kilometers
}

// Calculates bearing from one location to another
// Parameters: from - Starting location; to - Target location
// Returns: Bearing in degrees
float calculateAngle(Location from, Location to) {
  float phi1 = from.latitude * PI / 180.0; // Convert start latitude to radians
  float phi2 = to.latitude * PI / 180.0;   // Convert target latitude to radians
  float deltaLambda = (to.longitude - from.longitude) * PI / 180.0; // Convert longitude difference to radians

  float y = sin(deltaLambda) * cos(phi2); // Calculate y-component of bearing
  float x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(deltaLambda); // Calculate x-component of bearing
  float theta = atan2(y, x); // Compute bearing angle in radians

  float bearing = (theta * 180.0 / PI + 360) - 180; // Convert to degrees and shift to [-180, 180] range
  return bearing; // Return the bearing angle
}

// Normalizes an angle to [-180, 180] range
// Parameters: angle - Angle in degrees
// Returns: Normalized angle in degrees
float normalizeAngle(float angle) {
  while (angle > 180) angle -= 360; // Subtract 360 until angle is <= 180
  while (angle < -180) angle += 360; // Add 360 until angle is >= -180
  return angle; // Return normalized angle
}

// Calculates relative angle to an alert relative to current heading
// Parameters: alert - Alert to calculate angle for
// Returns: Relative angle in degrees (0° is ahead)
float calculateFacingDirection(Alert alert) {
  float rawAngle = calculateAngle(currentLocation, alert.location); // Get absolute bearing to alert
  float relativeAngle = rawAngle - currentDirection; // Subtract current IMU heading to get relative angle
  return normalizeAngle(relativeAngle); // Normalize to [-180, 180] range
}

// Hardware Initialization Functions

// Initializes Wi-Fi connection
void initWiFi() {
  WiFi.begin(ssid, password); // Start Wi-Fi connection with provided credentials
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) { // Loop until Wi-Fi is connected
    delay(500); // Wait 500ms between checks
    Serial.print("."); // Print progress dots
  }
  Serial.println("\nConnected to Wi-Fi"); // Confirm connection
  Serial.print("IP Address: "); // Print assigned IP address
  Serial.println(WiFi.localIP()); // Display IP for debugging
}

// Initializes GPS on UART2 (GPIO16 RX, GPIO17 TX)
void initGPS() {
  gpsSerial.begin(9600); // Start UART2 at 9600 baud for GPS communication
}

// Initializes IMU (BNO08X) on I2C (GPIO21 SDA, GPIO22 SCL)
void initIMU() {
  if (!bno08x.begin_I2C(0x4B)) { // Attempt to initialize BNO08X at I2C address 0x4B
    Serial.println("Failed to find BNO08X during setup. Will retry in loop..."); // Log failure
    bnoInitialized = false; // Mark IMU as uninitialized
  } else {
    Serial.println("BNO08X initialized successfully!"); // Confirm successful initialization
    bno08x.enableReport(SH2_ROTATION_VECTOR); // Enable rotation vector reports for orientation data
    bnoInitialized = true; // Mark IMU as initialized
  }
}

// Initializes UART1 for loopback communication (GPIO19 RX, GPIO18 TX)
void initCommSerial() {
  commSerial.begin(9600, SERIAL_8N1, 19, 18); // Start UART1 at 9600 baud, 8 data bits, no parity, 1 stop bit, on GPIO19 (RX) and GPIO18 (TX)
}

// DATA PROCESSING FUNCTIONS

// Updates GPS location from NEO-M8N
void updateGPS() {
  while (gpsSerial.available() > 0) { // Check if GPS data is available on UART2
    gps.encode(gpsSerial.read()); // Feed incoming bytes to TinyGPS++ for parsing
  }
  if (gps.location.isValid()) { // Check if parsed GPS data includes a valid location
    currentLocation.latitude = gps.location.lat(); // Update current latitude
    currentLocation.longitude = gps.location.lng(); // Update current longitude
    if (!isLocationInitialized) { // If this is the first valid GPS fix
      lastCheckedLocation = currentLocation; // Initialize last checked location
      isLocationInitialized = true; // Mark GPS as initialized
    }
  }
}

// Prints GPS data to Serial Monitor
void printGPS(unsigned long currentTime) {
  if (currentTime - lastGpsPrint >= printInterval) { // Check if 500ms has passed since last print
    if (gps.location.isValid()) { // If GPS data is valid
      Serial.print("GPS Lat: "); Serial.print(gps.location.lat(), 6); // Print latitude with 6 decimal places
      Serial.print(", Lon: "); Serial.println(gps.location.lng(), 6); // Print longitude with 6 decimal places
    } else {
      Serial.println("GPS: No fix"); // Indicate no valid GPS data
    }
    lastGpsPrint = currentTime; // Update timestamp for last GPS print
  }
}

// Updates IMU data and handles failures
void updateIMU(unsigned long currentTime) {
  if (!bnoInitialized && (currentTime - lastBnoRetry >= retryInterval)) { // If IMU is not initialized and 5s retry interval has passed
    initIMU(); // Attempt to reinitialize IMU
    lastBnoRetry = currentTime; // Update retry timestamp
  }

  if (bnoInitialized && (currentTime - lastImuPrint >= printInterval)) { // If IMU is initialized and 500ms has passed
    sh2_SensorValue_t sensorValue; // Structure to hold IMU sensor data
    if (bno08x.getSensorEvent(&sensorValue) && sensorValue.sensorId == SH2_ROTATION_VECTOR) { // If valid rotation vector data is received
      currentDirection = sensorValue.un.rotationVector.real * 180.0 / PI; // Convert quaternion real component to yaw angle in degrees
      Serial.print("IMU Yaw: "); Serial.print(currentDirection); // Print yaw angle
      Serial.print(", Pitch: "); Serial.print(sensorValue.un.rotationVector.i * 180.0 / PI); // Print pitch angle
      Serial.print(", Roll: "); Serial.println(sensorValue.un.rotationVector.j * 180.0 / PI); // Print roll angle
      imuFailureCount = 0; // Reset failure counter on successful read
    } else {
      Serial.println("IMU: No data"); // Log failure to read IMU data
      imuFailureCount++; // Increment failure counter
    }

    if (imuFailureCount >= maxImuFailures) { // If 5 consecutive failures occur
      Serial.println("BNO08X appears to be disconnected. Attempting to reinitialize..."); // Log reinitialization attempt
      bnoInitialized = false; // Mark IMU as uninitialized
      imuFailureCount = 0; // Reset failure counter
      lastBnoRetry = 0; // Allow immediate retry
    }
    lastImuPrint = currentTime; // Update timestamp for last IMU print
  }
}

// Maintains Wi-Fi connection
void maintainWiFi() {
  if (WiFi.status() != WL_CONNECTED) { // Check if Wi-Fi is disconnected
    Serial.println("Wi-Fi disconnected. Reconnecting..."); // Log reconnection attempt
    WiFi.reconnect(); // Attempt to reconnect to Wi-Fi
    while (WiFi.status() != WL_CONNECTED) { // Loop until reconnected
      delay(500); // Wait 500ms between checks
      Serial.print("."); // Print progress dots
    }
    Serial.println("\nReconnected to Wi-Fi"); // Confirm reconnection
  }
}

// Fetches and processes Waze API data
void fetchWazeData(unsigned long currentTime) {
  if (!isLocationInitialized) return; // Exit if GPS location is not yet valid

  float distanceMoved = calculateDistance(currentLocation, lastCheckedLocation); // Calculate distance moved since last API check
  if ((timeSinceLastCheck < checkInterval && distanceMoved <= movementThreshold) || WiFi.status() != WL_CONNECTED) { // Skip if too soon, not moved enough, or no Wi-Fi
    return; // Exit function to avoid unnecessary API calls
  }

  // Build API URL
  BoundingArea area = boundingBox(currentLocation, maxDistanceKm); // Create bounding box for current location
  String url = String(baseUrl) + "?top=" + String(area.top, 6) + "&bottom=" + String(area.bottom, 6) +
               "&left=" + String(area.left, 6) + "&right=" + String(area.right, 6) + "&env=row&types=alerts"; // Construct Waze API URL with bounding box parameters

  HTTPClient http; // Create HTTP client object
  http.begin(url); // Initialize HTTP request with URL
  int httpCode = http.GET(); // Send GET request to Waze API

  if (httpCode <= 0) { // Check for network or connection errors
    Serial.println("Waze API call failed, error: " + String(http.errorToString(httpCode).c_str())); // Log error
    http.end(); // Close HTTP connection
    return; // Exit function
  }

  if (httpCode != HTTP_CODE_OK) { // Check if HTTP response code is not 200
    Serial.printf("Waze API call failed, HTTP code: %d\n", httpCode); // Log HTTP error code
    http.end(); // Close HTTP connection
    return; // Exit function
  }

  String payload = http.getString(); // Get response payload as string
  DynamicJsonDocument doc(2048); // Allocate 2048-byte buffer for JSON parsing
  DeserializationError error = deserializeJson(doc, payload); // Parse JSON response
  if (error) { // Check for JSON parsing errors
    Serial.println("JSON parsing failed: " + String(error.c_str())); // Log parsing error
    http.end(); // Close HTTP connection
    return; // Exit function
  }

  JsonArray alerts = doc["alerts"]; // Extract alerts array from JSON
  alertCount = alerts.size(); // Store number of alerts

  // Free previous alerts
  if (currentAlerts != nullptr) { // If there are existing alerts in memory
    delete[] currentAlerts; // Free the dynamic array
    currentAlerts = nullptr; // Set pointer to null
  }

  // Store new alerts
  if (alertCount > 0) { // If there are new alerts
    currentAlerts = new Alert[alertCount]; // Allocate memory for new alerts
    for (int i = 0; i < alertCount; i++) { // Iterate through each alert
      currentAlerts[i].type = alerts[i]["type"].as<String>(); // Store alert type
      currentAlerts[i].subtype = alerts[i]["subtype"].as<String>(); // Store alert subtype
      currentAlerts[i].location.latitude = alerts[i]["location"]["y"].as<float>(); // Store alert latitude
      currentAlerts[i].location.longitude = alerts[i]["location"]["x"].as<float>(); // Store alert longitude
      currentAlerts[i].street = alerts[i]["street"].as<String>(); // Store alert street name
    }
  }

  processAlerts(); // Process the fetched alerts
  http.end(); // Close HTTP connection
  lastCheckedLocation = currentLocation; // Update last checked location
  timeSinceLastCheck = 0; // Reset API check timer
  lastApiCall = currentTime; // Update last API call timestamp
}

// Processes alerts to find closest police alert and send relative angle
void processAlerts() {
  if (alertCount == 0 || !bnoInitialized) { // Check if there are no alerts or IMU is not initialized
    Serial.println("No alerts found or IMU not initialized."); // Log status
    commSerial.println("999.0"); // Send sentinel value to indicate no alert
    return; // Exit function
  }

  int closestIndex = -1; // Initialize index of closest police alert
  float minDistance = 999999.0; // Initialize with large value to find minimum distance
  for (int i = 0; i < alertCount; i++) { // Iterate through all alerts
    if (currentAlerts[i].type == "POLICE") { // Check if alert is a police alert
      float distance = calculateDistance(currentLocation, currentAlerts[i].location); // Calculate distance to alert
      if (distance < minDistance) { // If this alert is closer than previous closest
        minDistance = distance; // Update minimum distance
        closestIndex = i; // Update closest alert index
      }
    }
  }

  if (closestIndex >= 0) { // If a police alert was found
    Alert& alert = currentAlerts[closestIndex]; // Reference closest alert
    float relativeAngle = calculateFacingDirection(alert); // Calculate relative angle to alert
    Serial.println("Closest Police Alert:"); // Log alert details
    Serial.println("  Type: " + alert.type); // Print alert type
    Serial.println("  Subtype: " + alert.subtype); // Print alert subtype
    Serial.println("  Location: Lat=" + String(alert.location.latitude, 6) +
                   ", Lon=" + String(alert.location.longitude, 6)); // Print alert coordinates
    Serial.println("  Street: " + alert.street); // Print alert street
    Serial.println("  Distance: " + String(minDistance, 2) + " km"); // Print distance to alert
    Serial.println("  Relative Angle: " + String(relativeAngle, 1) + "° (0° is ahead)"); // Print relative angle
    commSerial.println(String(relativeAngle, 1)); // Send relative angle via UART1
  } else {
    Serial.println("No police alerts found."); // Log if no police alerts
    commSerial.println("999.0"); // Send sentinel value to indicate no alert
  }
}

// Handles UART loopback communication (used for debugging sent data)
void handleCommSerial(unsigned long currentTime) {
  if (!commSerial.available()) return;            // Exit if no data is available on UART1

  if (currentTime - lastReceivePrint < receivePrintInterval) return; // Skip if less than 1000ms since last print

  String data = commSerial.readStringUntil('\n'); // Read data until newline
  float relativeAngle = data.toFloat();           // Convert received string to float
  if (relativeAngle <= 180.0 && relativeAngle >= -180.0) { // Check if angle is valid (within [-180, 180])
    Serial.print("Received Relative Angle: ");    // Log received angle
    Serial.print(relativeAngle, 1);               // Print angle with 1 decimal place
    Serial.println("° (0° is ahead)");            // Indicate angle reference
  } else {
    Serial.println("No valid police alert received."); // Log if invalid angle (e.g., sentinel value 999.0)
  }
  lastReceivePrint = currentTime; // Update timestamp for last receive print
}

// Main Setup and Loop
void setup() {
  Serial.begin(115200); // Initialize Serial Monitor at 115200 baud for debugging
  Serial.println("ESP32 Loopback Test is running!"); // Log startup message

  initCommSerial(); // Initialize UART1 for loopback
  initWiFi();       // Connect to Wi-Fi
  initGPS();        // Initialize GPS
  initIMU();        // Initialize IMU
  Wire.begin();     // Start I2C bus for IMU communication
}

void loop() {
  unsigned long currentTime = millis(); // Get current time in milliseconds
  timeSinceLastCheck += (currentTime - lastApiCall) / 1000.0; // Update API check timer (convert ms to seconds)

  updateGPS();                  // Update GPS coordinates
  printGPS(currentTime);        // Print GPS data if interval elapsed
  updateIMU(currentTime);       // Update IMU orientation data
  maintainWiFi();               // Check and maintain Wi-Fi connection
  fetchWazeData(currentTime);   // Fetch Waze alerts if conditions met
  handleCommSerial(currentTime);// Process received UART data
}