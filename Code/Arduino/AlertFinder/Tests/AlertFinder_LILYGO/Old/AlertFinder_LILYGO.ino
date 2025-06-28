// AlertFinder_LILYGO.ino
// This sketch uses a LILYGO T-SIM7000G to fetch police alerts from the Waze API using GPS coordinates from the SIM7000G module,
// calculates the relative angle to the closest police alert using BNO08X IMU data via SPI, and sends the angle
// via UART1 (loopback on GPIO18 TX to GPIO19 RX for testing). It prioritizes Wi-Fi for internet access,
// falling back to cellular (GPRS via SIM7000G) if Wi-Fi is unavailable, and logs data to the Serial Monitor for debugging.

// **IMPORTANT**: For the BNO08X IMU, ensure it is set to SPI mode by connecting P0 to GND and P1 to VCC (for Adafruit breakouts).
// Check your specific breakout’s documentation for correct mode configuration.
// BNO08X INT pin is connected to GPIO33, and RST pin is connected to GPIO32.

// TODO: BNO08X IMU hardware was faulty, so we need to switch from the BNO08X to a HMC5883L.

// SIM7000G Configuration
#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define SerialAT Serial1
#define UART_BAUD 115200
#define PIN_TX 27
#define PIN_RX 26
#define PWR_PIN 4

// SPI Pins for BNO08X
#define BNO08X_CS   21  // Chip Select pin for SPI
#define BNO08X_SCK  18  // SPI Clock
#define BNO08X_MISO 19  // SPI MISO
#define BNO08X_MOSI 23  // SPI MOSI
#define BNO08X_INT  33  // Interrupt pin
#define BNO08X_RST  32  // Reset pin (changed from GPIO34 to GPIO32)

// Libraries
#include <HardwareSerial.h>  // For UART communication (loopback)
#include <TinyGsmClient.h>   // For SIM7000G modem communication
#include <TinyGPS++.h>       // For parsing GPS data (fallback)
#include <SPI.h>             // For SPI communication with BNO08X
#include <Adafruit_BNO08x.h> // For BNO08X IMU via SPI
#include <WiFi.h>            // For Wi-Fi connectivity
#include <HTTPClient.h>      // For HTTP requests to Waze API (Wi-Fi only)
#include <ArduinoJson.h>     // For parsing JSON responses from Waze API

// Configuration Constants
// Wi-Fi credentials
const char* ssid = "BigCock69";     // Replace with your Wi-Fi SSID
const char* password = "GymBro69";  // Replace with your Wi-Fi password

// GPRS credentials for cellular connection
const char apn[] = ""; // SET TO YOUR APN
const char gprsUser[] = "";
const char gprsPass[] = "";

// Waze API settings
const float maxDistanceKm = 1.0f;      // Max distance for alerts (kilometers)
const float checkInterval = 15.0f;     // Interval to check for new alerts (seconds)
const float movementThreshold = 0.2f;  // Distance to trigger alert check (kilometers)
const char* baseHost = "www.waze.com"; // Waze API host
const char* basePath = "/live-map/api/georss"; // Waze API path

// Timing constants
const unsigned long printInterval = 500;         // Print GPS/IMU data every 500ms
const unsigned long retryInterval = 5000;        // Retry BNO08X initialization every 5 seconds
const unsigned long receivePrintInterval = 1000; // Print received UART data every 1000ms
const int maxImuFailures = 5;                    // Max consecutive IMU failures before reinitialization

// Hardware Objects
HardwareSerial commSerial(1);  // UART1 for loopback communication (GPIO19 RX, GPIO18 TX)
TinyGsm modem(SerialAT);       // TinyGSM modem object for SIM7000G
TinyGsmClient gsmClient(modem); // TinyGSM client for cellular HTTP requests
TinyGPSPlus gps;               // TinyGPS++ object for parsing GPS data (fallback)
Adafruit_BNO08x bno08x;        // BNO08X IMU object for orientation data via SPI

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

// Calculates bounding box around a location for Waze API
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

// Calculates distance between two locations using Haversine formula
float calculateDistance(Location loc1, Location loc2) {
  float lat1 = loc1.latitude * PI / 180.0;
  float lat2 = loc2.latitude * PI / 180.0;
  float lon1 = loc1.longitude * PI / 180.0;
  float lon2 = loc2.longitude * PI / 180.0;

  float dLat = lat2 - lat1;
  float dLon = lon2 - lon1;
  float a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return 6371.0 * c;
}

// Calculates bearing from one location to another
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

// Normalizes an angle to [-180, 180] range
float normalizeAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

// Calculates relative angle to an alert relative to current heading
float calculateFacingDirection(Alert alert) {
  float rawAngle = calculateAngle(currentLocation, alert.location);
  float relativeAngle = rawAngle - currentDirection;
  return normalizeAngle(relativeAngle);
}

// Hardware Initialization Functions

// Initializes Wi-Fi and cellular connections
void initWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to Wi-Fi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    isGprsConnected = false;
    return;
  }

  Serial.println("\nWi-Fi connection failed. Attempting cellular connection...");
  if (!modem.waitForNetwork(30000L)) {
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

// Initializes GPS and modem on Serial1
void initGPS() {
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  delay(300);
  digitalWrite(PWR_PIN, LOW);
  delay(1000);
  digitalWrite(PWR_PIN, HIGH);

  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(1000);

  Serial.println("Initializing modem...");
  if (!modem.restart()) {
    Serial.println("Failed to restart modem, attempting to continue without restarting");
    modem.init();
  }

  modem.sendAT("+SGPIO=0,4,1,1");
  if (modem.waitResponse(10000L) != 1) {
    Serial.println("Failed to power on GPS");
  } else {
    Serial.println("GPS powered on successfully");
  }
  modem.enableGPS();
}

// Initializes IMU (BNO08X) on SPI
void initIMU() {
  // Configure RST and INT pins
  pinMode(BNO08X_RST, OUTPUT);
  pinMode(BNO08X_INT, INPUT);
  
  // Perform reset sequence
  digitalWrite(BNO08X_RST, LOW);
  delay(10); // Hold low for at least 10ms
  digitalWrite(BNO08X_RST, HIGH);
  delay(50); // Wait for sensor to stabilize

  SPI.begin(BNO08X_SCK, BNO08X_MISO, BNO08X_MOSI, BNO08X_CS);

  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT, &SPI)) {
    Serial.println("Failed to find BNO08X during setup. Will retry in loop...");
    bnoInitialized = false;
  } else {
    Serial.println("BNO08X initialized successfully via SPI!");
    bno08x.enableReport(SH2_ROTATION_VECTOR);
    bnoInitialized = true;
  }
}

// Initializes UART1 for loopback communication
void initCommSerial() {
  commSerial.begin(9600, SERIAL_8N1, 19, 18);
}

// Data Processing Functions

// Updates GPS location from SIM7000G
void updateGPS() {
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

  while (SerialAT.available() > 0) {
    gps.encode(SerialAT.read());
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
  if (currentTime - lastGpsPrint >= printInterval) {
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
    // Check if new data is available via INT pin (active low)
    if (digitalRead(BNO08X_INT) == LOW) {
      sh2_SensorValue_t sensorValue;
      if (bno08x.getSensorEvent(&sensorValue) && sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        float q0 = sensorValue.un.rotationVector.real;
        float q1 = sensorValue.un.rotationVector.i;
        float q2 = sensorValue.un.rotationVector.j;
        float q3 = sensorValue.un.rotationVector.k;
        currentDirection = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) * 180.0 / PI;
        Serial.print("IMU Yaw: "); Serial.print(currentDirection);
        Serial.print(", Pitch: "); Serial.print(atan2(2 * (q0 * q2 - q3 * q1), 1 - 2 * (q2 * q2 + q1 * q1)) * 180.0 / PI);
        Serial.print(", Roll: "); Serial.println(asin(2 * (q0 * q1 + q2 * q3)) * 180.0 / PI);
        imuFailureCount = 0;
      } else {
        Serial.println("IMU: No data");
        imuFailureCount++;
      }
    } else {
      Serial.println("IMU: Waiting for interrupt");
      return; // Skip reading if no interrupt
    }

    if (imuFailureCount >= maxImuFailures) {
      Serial.println("BNO08X appears to be disconnected. Attempting to reinitialize...");
      digitalWrite(BNO08X_RST, LOW);
      delay(10); // Reset sensor
      digitalWrite(BNO08X_RST, HIGH);
      delay(50); // Wait for stabilization
      bnoInitialized = false;
      imuFailureCount = 0;
      lastBnoRetry = 0;
    }
    lastImuPrint = currentTime;
  }
}

// Maintains internet connection
void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (isGprsConnected) {
      modem.gprsDisconnect();
      isGprsConnected = false;
      Serial.println("Switched to Wi-Fi");
    }
    return;
  }

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

// Performs HTTP GET request over cellular using TinyGsmClient
bool performCellularHttpGet(const String& host, const String& path, String& response) {
  // Connect to the server
  if (!gsmClient.connect(host.c_str(), 443)) {
    Serial.println("Failed to connect to Waze server via cellular");
    return false;
  }

  // Build the HTTP GET request
  String request = "GET " + path + " HTTP/1.1\r\n";
  request += "Host: " + host + "\r\n";
  request += "Connection: close\r\n";
  request += "\r\n";

  // Send the request
  gsmClient.print(request);

  // Read response
  unsigned long timeout = millis();
  while (gsmClient.connected() && millis() - timeout < 10000L) {
    if (gsmClient.available()) {
      response = gsmClient.readString();
      break;
    }
  }

  // Close connection
  gsmClient.stop();

  // Extract payload (skip HTTP headers)
  int headerEnd = response.indexOf("\r\n\r\n");
  if (headerEnd != -1) {
    response = response.substring(headerEnd + 4);
  } else {
    Serial.println("Failed to parse cellular HTTP response");
    return false;
  }

  return true;
}

// Fetches and processes Waze API data
void fetchWazeData(unsigned long currentTime) {
  if (!isLocationInitialized) return;

  float distanceMoved = calculateDistance(currentLocation, lastCheckedLocation);
  if (timeSinceLastCheck < checkInterval && distanceMoved <= movementThreshold) return;

  if (WiFi.status() != WL_CONNECTED && !isGprsConnected) {
    Serial.println("No internet connection available");
    return;
  }

  // Build API path
  BoundingArea area = boundingBox(currentLocation, maxDistanceKm);
  String path = String(basePath) + "?top=" + String(area.top, 6) + "&bottom=" + String(area.bottom, 6) +
                "&left=" + String(area.left, 6) + "&right=" + String(area.right, 6) + "&env=row&types=alerts";

  String payload;
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Using Wi-Fi for Waze API request");
    HTTPClient http;
    WiFiClient client;
    String url = "https://" + String(baseHost) + path;
    http.begin(client, url);
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
    payload = http.getString();
    http.end();
  } else {
    Serial.println("Using cellular for Waze API request");
    String response;
    if (!performCellularHttpGet(baseHost, path, response)) {
      Serial.println("Waze API call failed via cellular");
      return;
    }
    payload = response;
  }

  DynamicJsonDocument doc(2048);
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Serial.println("JSON parsing failed: " + String(error.c_str()));
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
  lastCheckedLocation = currentLocation;
  timeSinceLastCheck = 0;
  lastApiCall = currentTime;
}

// Processes alerts to find closest police alert
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

// Handles UART loopback communication
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