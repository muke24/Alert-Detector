// AlertFinder_LILYGO.ino
// This sketch uses a LILYGO T-SIM7000G to fetch police alerts from the Waze API using GPS coordinates from the SIM7000G module,
// calculates the relative angle to the closest police alert using BNO08X IMU data, and sends the angle
// via UART1 (loopback on GPIO18 TX to GPIO19 RX for testing). It prioritizes Wi-Fi for internet access,
// falling back to cellular (GPRS via SIM7000G) if Wi-Fi is unavailable, and logs data to the Serial Monitor for debugging.

// Libraries
#include <HardwareSerial.h> // For UART communication (loopback)
#include <TinyGsmClient.h>  // For SIM7000G modem communication
#include <TinyGPS++.h>      // For parsing GPS data (fallback)
#include <Wire.h>           // For I2C communication with BNO08X IMU
#include <Adafruit_BNO08x.h> // For BNO08X IMU
#include <WiFi.h>           // For Wi-Fi connectivity
#include <HTTPClient.h>     // For HTTP requests to Waze API
#include <ArduinoJson.h>    // For parsing JSON responses from Waze API

// Configuration Constants
// Wi-Fi credentials
const char* ssid = "BigCock69";     // Wi-Fi SSID
const char* password = "GymBro69";  // Wi-Fi password

// GPRS credentials for cellular connection
const char apn[] = ""; // SET TO YOUR APN
const char gprsUser[] = "";
const char gprsPass[] = "";

// Waze API settings
const float maxDistanceKm = 1.0f;      // Max distance for alerts (kilometers)
const float checkInterval = 15.0f;     // Interval to check for new alerts (seconds)
const float movementThreshold = 0.2f;  // Distance to trigger alert check (kilometers)
const char* baseUrl = "https://www.waze.com/live-map/api/georss"; // Waze API endpoint

// Timing constants
const unsigned long printInterval = 500;         // Print GPS/IMU data every 500ms
const unsigned long retryInterval = 5000;        // Retry BNO08X initialization every 5 seconds
const unsigned long receivePrintInterval = 1000; // Print received UART data every 1000ms
const int maxImuFailures = 5;                   // Max consecutive IMU failures before reinitialization

// SIM7000G Configuration
#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define SerialAT Serial1
#define UART_BAUD 115200
#define PIN_TX 27
#define PIN_RX 26
#define PWR_PIN 4

// Hardware Objects
HardwareSerial commSerial(1);  // UART1 for loopback communication (GPIO19 RX, GPIO18 TX)
TinyGsm modem(SerialAT);      // TinyGSM modem object for SIM7000G
TinyGsmClient gsmClient(modem); // TinyGSM client for cellular HTTP requests
TinyGPSPlus gps;              // TinyGPS++ object for parsing GPS data (fallback)
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
bool isGprsConnected = false;           // Tracks cellular connection status

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

// Converts quaternion to yaw angle in degrees
float quaternionToYaw(float q0, float q1, float q2, float q3) {
    return atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)) * 180.0 / PI;
}

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

// Initializes Wi-Fi and cellular connections
void initWiFi() {
  // Try Wi-Fi first
  WiFi.begin(ssid, password); // Start Wi-Fi connection with provided credentials
  Serial.print("Connecting to Wi-Fi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) { // Try for 10 seconds
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to Wi-Fi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    isGprsConnected = false; // Wi-Fi is active, no need for GPRS
    return;
  }

  // Wi-Fi failed, try cellular
  Serial.println("\nWi-Fi connection failed. Attempting cellular connection...");
  if (!modem.waitForNetwork(30000L)) { // Wait up to 30 seconds for network
    Serial.println("Failed to connect to cellular network");
    return;
  }
  Serial.println("Cellular network connected");

  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println("Failed to connect to GPRS");
    return;
  }
  isGprsConnected = true;
  Serial.println("GPRS connected");
  Serial.print("Local IP: ");
  Serial.println(modem.localIP());
}

// Initializes GPS and modem on Serial1 (GPIO26 RX, GPIO27 TX for SIM7000G)
void initGPS() {
  // Set PWR_PIN for GPS power control
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH); // Set high initially
  delay(300);
  digitalWrite(PWR_PIN, LOW); // Pulse low to power on
  delay(1000); // Hold low for 1 second
  digitalWrite(PWR_PIN, HIGH); // Release

  // Initialize serial for SIM7000G
  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(1000);

  // Initialize modem
  Serial.println("Initializing modem...");
  if (!modem.restart()) {
    Serial.println("Failed to restart modem, attempting to continue without restarting");
    modem.init(); // Fallback to init if restart fails
  }

  // Enable GPS
  modem.sendAT("+SGPIO=0,4,1,1"); // Turn on GPS power (GPIO4)
  if (modem.waitResponse(10000L) != 1) {
    Serial.println("Failed to power on GPS");
  } else {
    Serial.println("GPS powered on successfully");
  }
  modem.enableGPS();
}

// Initializes IMU (BNO08X) on I2C (GPIO21 SDA, GPIO22 SCL)
void initIMU() {
  if (!bno08x.begin_I2C(0x4B)) { // Attempt to initialize BNO08X at I2C address 0x4B
    Serial.println("Failed to find BNO08X during setup. Will retry in loop...");
    bnoInitialized = false; // Mark IMU as uninitialized
  } else {
    Serial.println("BNO08X initialized successfully!");
    bno08x.enableReport(SH2_ROTATION_VECTOR); // Enable rotation vector reports for orientation data
    bnoInitialized = true; // Mark IMU as initialized
  }
}

// Initializes UART1 for loopback communication (GPIO19 RX, GPIO18 TX)
void initCommSerial() {
  commSerial.begin(9600, SERIAL_8N1, 19, 18); // Start UART1 at 9600 baud, 8 data bits, no parity, 1 stop bit
}

// Data Processing Functions

// Updates GPS location from SIM7000G
void updateGPS() {
  // Try TinyGSM's getGPS method first
  float lat, lon;
  if (modem.getGPS(&lat, &lon)) {
    if (lat != 0 && lon != 0) {
      currentLocation.latitude = lat;
      currentLocation.longitude = lon;
      if (!isLocationInitialized) {
        lastCheckedLocation = currentLocation;
        isLocationInitialized = true;
      }
      return;
    }
  }

  // Fallback: Try parsing NMEA sentences with TinyGPS++
  while (SerialAT.available() > 0) {
    char c = SerialAT.read();
    gps.encode(c); // Feed incoming bytes to TinyGPS++ for parsing
    // Optionally log raw NMEA for debugging
    // Serial.write(c);
  }

  if (gps.location.isValid()) {
    currentLocation.latitude = gps.location.lat();
    currentLocation.longitude = gps.location.lng();
    if (!isLocationInitialized) {
      lastCheckedLocation = currentLocation;
      isLocationInitialized = true;
    }
  }
}

// Prints GPS data to Serial Monitor
void printGPS(unsigned long currentTime) {
  if (currentTime - lastGpsPrint >= printInterval) { // Check if 500ms has passed since last print
    if (gps.location.isValid() || (currentLocation.latitude != 0 && currentLocation.longitude != 0)) {
      Serial.print("GPS Lat: "); Serial.print(currentLocation.latitude, 6);
      Serial.print(", Lon: "); Serial.println(currentLocation.longitude, 6);
    } else {
      Serial.println("GPS: No fix");
    }
    lastGpsPrint = currentTime;
  }
}

// Updates IMU data and handles failures
void updateIMU(unsigned long currentTime) {
  if (!bnoInitialized && (currentTime - lastBnoRetry >= retryInterval)) {
    initIMU();
    lastBnoRetry = currentTime;
  }

  if (bnoInitialized && (currentTime - lastImuPrint >= printInterval)) {
    sh2_SensorValue_t sensorValue;
    if (bno08x.getSensorEvent(&sensorValue) && sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      // Use quaternionToYaw to calculate yaw from quaternion components
      currentDirection = quaternionToYaw(
        sensorValue.un.rotationVector.real,  // q0 (real component)
        sensorValue.un.rotationVector.i,     // q1
        sensorValue.un.rotationVector.j,     // q2
        sensorValue.un.rotationVector.k      // q3
      );
      Serial.print("IMU Yaw: "); Serial.print(currentDirection);
      Serial.print(", Pitch: "); Serial.print(sensorValue.un.rotationVector.i * 180.0 / PI);
      Serial.print(", Roll: "); Serial.println(sensorValue.un.rotationVector.j * 180.0 / PI);
      imuFailureCount = 0;
    } else {
      Serial.println("IMU: No data");
      imuFailureCount++;
    }

    if (imuFailureCount >= maxImuFailures) {
      Serial.println("BNO08X appears to be disconnected. Attempting to reinitialize...");
      bnoInitialized = false;
      imuFailureCount = 0;
      lastBnoRetry = 0;
    }
    lastImuPrint = currentTime;
  }
}

// Maintains internet connection (Wi-Fi or cellular)
void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (isGprsConnected) {
      modem.gprsDisconnect();
      isGprsConnected = false;
      Serial.println("Switched to Wi-Fi");
    }
    return; // Wi-Fi is active, no action needed
  }

  // Wi-Fi is disconnected, try reconnecting
  Serial.println("Wi-Fi disconnected. Reconnecting...");
  WiFi.reconnect();
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 5000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nReconnected to Wi-Fi");
    if (isGprsConnected) {
      modem.gprsDisconnect();
      isGprsConnected = false;
    }
    return;
  }

  // Wi-Fi reconnect failed, try cellular
  if (!isGprsConnected) {
    Serial.println("\nWi-Fi reconnect failed. Attempting cellular connection...");
    if (!modem.isNetworkConnected()) {
      if (!modem.waitForNetwork(30000L)) {
        Serial.println("Failed to connect to cellular network");
        return;
      }
    }
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
      Serial.println("Failed to connect to GPRS");
      return;
    }
    isGprsConnected = true;
    Serial.println("GPRS connected");
    Serial.print("Local IP: ");
    Serial.println(modem.localIP());
  }
}

// Fetches and processes Waze API data
void fetchWazeData(unsigned long currentTime) {
  if (!isLocationInitialized) return; // Exit if GPS location is not yet valid

  float distanceMoved = calculateDistance(currentLocation, lastCheckedLocation);
  if (timeSinceLastCheck < checkInterval && distanceMoved <= movementThreshold) return;

  // Check for internet connectivity
  if (WiFi.status() != WL_CONNECTED && !isGprsConnected) {
    Serial.println("No internet connection available");
    return;
  }

  // Build API URL
  BoundingArea area = boundingBox(currentLocation, maxDistanceKm);
  String url = String(baseUrl) + "?top=" + String(area.top, 6) + "&bottom=" + String(area.bottom, 6) +
               "&left=" + String(area.left, 6) + "&right=" + String(area.right, 6) + "&env=row&types=alerts";

  HTTPClient http;
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Using Wi-Fi for Waze API request");
    http.begin(url); // Use Wi-Fi
  } else {
    Serial.println("Using cellular for Waze API request");
    http.begin(gsmClient, url); // Use cellular
  }

  int httpCode = http.GET();
  if (httpCode <= 0) {
    Serial.println("Waze API call failed, error: " + String(http.errorToString(httpCode).c_str()));
    http.end();
    return;
  }

  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("Waze API call failed, HTTP code: %d\n", httpCode);
    http.end();
    return;
  }

  String payload = http.getString();
  DynamicJsonDocument doc(2048);
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Serial.println("JSON parsing failed: " + String(error.c_str()));
    http.end();
    return;
  }

  JsonArray alerts = doc["alerts"];
  alertCount = alerts.size();

  if (currentAlerts != nullptr) {
    delete[] currentAlerts;
    currentAlerts = nullptr;
  }

  if (alertCount > 0) {
    currentAlerts = new Alert[alertCount];
    for (int i = 0; i < alertCount; i++) {
      currentAlerts[i].type = alerts[i]["type"].as<String>();
      currentAlerts[i].subtype = alerts[i]["subtype"].as<String>();
      currentAlerts[i].location.latitude = alerts[i]["location"]["y"].as<float>();
      currentAlerts[i].location.longitude = alerts[i]["location"]["x"].as<float>();
      currentAlerts[i].street = alerts[i]["street"].as<String>();
    }
  }

  processAlerts();
  http.end();
  lastCheckedLocation = currentLocation;
  timeSinceLastCheck = 0;
  lastApiCall = currentTime;
}

// Processes alerts to find closest police alert and send relative angle
void processAlerts() {
  if (alertCount == 0 || !bnoInitialized) {
    Serial.println("No alerts found or IMU not initialized.");
    commSerial.println("999.0");
    return;
  }

  int closestIndex = -1;
  float minDistance = 999999.0;
  for (int i = 0; i < alertCount; i++) {
    if (currentAlerts[i].type == "POLICE") {
      float distance = calculateDistance(currentLocation, currentAlerts[i].location);
      if (distance < minDistance) {
        minDistance = distance;
        closestIndex = i;
      }
    }
  }

  if (closestIndex >= 0) {
    Alert& alert = currentAlerts[closestIndex];
    float relativeAngle = calculateFacingDirection(alert);
    Serial.println("Closest Police Alert:");
    Serial.println("  Type: " + alert.type);
    Serial.println("  Subtype: " + alert.subtype);
    Serial.println("  Location: Lat=" + String(alert.location.latitude, 6) +
                   ", Lon=" + String(alert.location.longitude, 6));
    Serial.println("  Street: " + alert.street);
    Serial.println("  Distance: " + String(minDistance, 2) + " km");
    Serial.println("  Relative Angle: " + String(relativeAngle, 1) + "° (0° is ahead)");
    commSerial.println(String(relativeAngle, 1));
  } else {
    Serial.println("No police alerts found.");
    commSerial.println("999.0");
  }
}

// Handles UART loopback communication (used for debugging sent data)
void handleCommSerial(unsigned long currentTime) {
  if (!commSerial.available()) return;

  if (currentTime - lastReceivePrint < receivePrintInterval) return;

  String data = commSerial.readStringUntil('\n');
  float relativeAngle = data.toFloat();
  if (relativeAngle <= 180.0 && relativeAngle >= -180.0) {
    Serial.print("Received Relative Angle: ");
    Serial.print(relativeAngle, 1);
    Serial.println("° (0° is ahead)");
  } else {
    Serial.println("No valid police alert received.");
  }
  lastReceivePrint = currentTime;
}

// Main Setup and Loop
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Loopback Test with SIM7000G is running!");

  initCommSerial();
  initWiFi();
  initGPS();
  initIMU();
  Wire.begin();
}

void loop() {
  unsigned long currentTime = millis();
  timeSinceLastCheck += (currentTime - lastApiCall) / 1000.0;

  updateGPS();
  printGPS(currentTime);
  updateIMU(currentTime);
  maintainWiFi();
  fetchWazeData(currentTime);
  handleCommSerial(currentTime);
}