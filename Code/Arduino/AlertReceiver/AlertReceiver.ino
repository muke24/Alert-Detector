// AlertFinder_LILYGO.ino
// This sketch uses a LILYGO T-SIM7000G to fetch police alerts from the Waze API using GPS coordinates from the SIM7000G module,
// calculates the relative angle to the closest police alert using HMC5883L magnetometer data via I2C, and sends the angle
// via UART2 (loopback on GPIO32 TX to GPIO33 RX for testing). It prioritizes Wi-Fi for internet access,
// falling back to cellular (GPRS via SIM7000G) if Wi-Fi is unavailable, and logs data to the Serial Monitor for debugging.

// SIM7000G Configuration
#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb
#define SerialAT Serial1
#define UART_BAUD 115200
#define PIN_TX 27
#define PIN_RX 26
#define PWR_PIN 4

// ESP32 Communication
#define COMM_TX 32
#define COMM_RX 33
// Alternative pins for testing (uncomment to use)
// #define COMM_TX 16
// #define COMM_RX 17

// Libraries
#include <HardwareSerial.h>  // For UART communication (loopback)
#include <TinyGsmClient.h>   // For SIM7000G modem communication
#include <TinyGPS++.h>       // For parsing GPS data (fallback)
#include <Wire.h>            // For I2C communication with HMC5883L
#include <Adafruit_HMC5883_U.h> // For HMC5883L magnetometer via I2C
#include <WiFi.h>            // For Wi-Fi connectivity
#include <WiFiClientSecure.h> // For secure HTTPS requests
#include <HTTPClient.h>      // For HTTP requests to Waze API (Wi-Fi only)
#include <ArduinoJson.h>     // For parsing JSON responses from Waze API

// Configuration Constants
const char* ssid = "BigCock69";     // Replace with your Wi-Fi SSID
const char* password = "GymBro69";  // Replace with your Wi-Fi password
const char apn[] = ""; // SET TO YOUR APN
const char gprsUser[] = "";
const char gprsPass[] = "";
const float maxDistanceKm = 15.0f;      // Max distance for alerts (kilometers)
const float checkInterval = 15.0f;     // Interval to check for new alerts (seconds)
const float movementThreshold = 0.2f;  // Distance to trigger alert check (kilometers)
const char* baseHost = "www.waze.com"; // Waze API host
const char* basePath = "/live-map/api/georss"; // Waze API path
const unsigned long printInterval = 500;         // Print GPS/IMU data every 500ms
const unsigned long retryInterval = 5000;        // Retry HMC5883L initialization every 5 seconds
const unsigned long receivePrintInterval = 1000; // Print received UART data every 1000ms
const int maxImuFailures = 5;                    // Max consecutive IMU failures before reinitialization

// Hardware Objects
HardwareSerial commSerial(2);  // UART1 for loopback communication
TinyGsm modem(SerialAT);       // TinyGSM modem object for SIM7000G
TinyGsmClient gsmClient(modem); // TinyGSM client for cellular HTTP requests
TinyGPSPlus gps;               // TinyGPS++ object for parsing GPS data (fallback)
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345); // HMC5883L magnetometer object for heading data via I2C

// State Variables
bool bnoInitialized = false;
int imuFailureCount = 0;
unsigned long lastGpsPrint = 0;
unsigned long lastImuPrint = 0;
unsigned long lastBnoRetry = 0;
unsigned long lastApiCall = 0;
unsigned long lastReceivePrint = 0;
float timeSinceLastCheck = 0;
bool isLocationInitialized = false;
float currentDirection = 0;
bool isGprsConnected = false;

// Data Structures
struct Location {
  float latitude;
  float longitude;
};
Location currentLocation = {0, 0};
Location lastCheckedLocation = {0, 0};

struct Alert {
  String type;
  String subtype;
  Location location;
  String street;
};
Alert* currentAlerts = nullptr;
int alertCount = 0;

struct BoundingArea {
  float top;
  float bottom;
  float left;
  float right;
};

// Utility Functions
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

float normalizeAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

float calculateFacingDirection(Alert alert) {
  float rawAngle = calculateAngle(currentLocation, alert.location);
  float relativeAngle = rawAngle - currentDirection;
  return normalizeAngle(relativeAngle);
}

// Hardware Initialization Functions
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

void initHMC5883L() {
  Wire.begin(21, 22);
  if (!mag.begin()) {
    Serial.println("Failed to find HMC5883L. Check wiring!");
    bnoInitialized = false;
  } else {
    Serial.println("HMC5883L initialized successfully!");
    bnoInitialized = true;
  }
}

void initCommSerial() {
  commSerial.begin(115200, SERIAL_8N1, COMM_RX, COMM_TX);
  Serial.print("UART1 initialized for loopback: GPIO");
  Serial.print(COMM_TX);
  Serial.print(" (TX) to GPIO");
  Serial.print(COMM_RX);
  Serial.println(" (RX)");
  // Clear any stale data
  while (commSerial.available()) commSerial.read();
}

// Data Processing Functions
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

void updateIMU(unsigned long currentTime) {
  if (!bnoInitialized && (currentTime - lastBnoRetry >= retryInterval)) {
    initHMC5883L();
    lastBnoRetry = currentTime;
  }
  if (bnoInitialized && (currentTime - lastImuPrint >= printInterval)) {
    sensors_event_t event;
    if (mag.getEvent(&event)) {
      float heading = atan2(event.magnetic.y, event.magnetic.x);
      if (heading < 0) heading += 2 * PI;
      currentDirection = heading * 180 / PI;
      Serial.print("HMC5883L Heading: ");
      Serial.println(currentDirection);
      imuFailureCount = 0;
    } else {
      Serial.println("HMC5883L: No data");
      imuFailureCount++;
    }
    if (imuFailureCount >= maxImuFailures) {
      Serial.println("HMC5883L appears to be disconnected. Attempting to reinitialize...");
      bnoInitialized = false;
      imuFailureCount = 0;
    }
    lastImuPrint = currentTime;
  }
}

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

bool performCellularHttpGet(const String& host, const String& path, String& response) {
  if (!gsmClient.connect(host.c_str(), 443)) {
    Serial.println("Failed to connect to Waze server via cellular");
    return false;
  }
  String request = "GET " + path + " HTTP/1.1\r\n";
  request += "Host: " + host + "\r\n";
  request += "Connection: close\r\n";
  request += "\r\n";
  gsmClient.print(request);
  unsigned long timeout = millis();
  while (gsmClient.connected() && millis() - timeout < 10000L) {
    if (gsmClient.available()) {
      response = gsmClient.readString();
      break;
    }
  }
  gsmClient.stop();
  int headerEnd = response.indexOf("\r\n\r\n");
  if (headerEnd != -1) {
    response = response.substring(headerEnd + 4);
  } else {
    Serial.println("Failed to parse cellular HTTP response");
    return false;
  }
  return true;
}

void fetchWazeData(unsigned long currentTime) {
  if (!isLocationInitialized) return;
  float distanceMoved = calculateDistance(currentLocation, lastCheckedLocation);
  if (timeSinceLastCheck < checkInterval && distanceMoved <= movementThreshold) return;
  if (WiFi.status() != WL_CONNECTED && !isGprsConnected) {
    Serial.println("No internet connection available");
    return;
  }
  BoundingArea area = boundingBox(currentLocation, maxDistanceKm);
  String path = String(basePath) + "?top=" + String(area.top, 6) + "&bottom=" + String(area.bottom, 6) +
                "&left=" + String(area.left, 6) + "&right=" + String(area.right, 6) + "&env=row&types=alerts";
  String payload;
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Using Wi-Fi for Waze API request");
    Serial.print("Wi-Fi Status: ");
    Serial.println(WiFi.status());
    HTTPClient http;
    WiFiClientSecure client;
    client.setInsecure();
    String url = "https://" + String(baseHost) + path;
    http.setTimeout(10000);
    if (!http.begin(client, url)) {
      Serial.println("Failed to begin HTTP connection");
      http.end();
      return;
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
  processAlerts(currentTime);
  lastCheckedLocation = currentLocation;
  timeSinceLastCheck = 0;
  lastApiCall = currentTime;
}

void processAlerts(unsigned long currentTime) {
  if (alertCount == 0 || !bnoInitialized) {
    Serial.println("No alerts found or IMU not initialized.");
    float noAlert = 999.0;
    while (commSerial.available()) commSerial.read(); // Clear buffer
    commSerial.write((uint8_t*)&noAlert, sizeof(float));
    commSerial.flush(); // Ensure data is sent
    Serial.print("Sent float: ");
    Serial.println(noAlert, 1);
    // Immediate receive check with retries
    receiveData(currentTime);
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
    while (commSerial.available()) commSerial.read(); // Clear buffer
    commSerial.write((uint8_t*)&relativeAngle, sizeof(float));
    commSerial.flush(); // Ensure data is sent
    Serial.print("Sent float: ");
    Serial.println(relativeAngle, 1);
    // Immediate receive check with retries
    receiveData(currentTime);
  } else {
    Serial.println("No police alerts found.");
    float noAlert = 999.0;
    while (commSerial.available()) commSerial.read(); // Clear buffer
    commSerial.write((uint8_t*)&noAlert, sizeof(float));
    commSerial.flush(); // Ensure data is sent
    Serial.print("Sent float: ");
    Serial.println(noAlert, 1);
    // Immediate receive check with retries
    receiveData(currentTime);
  }
}

void receiveData(unsigned long currentTime) {
  const int maxRetries = 3;
  for (int retry = 0; retry < maxRetries; retry++) {
    unsigned long startTime = millis();
    while (commSerial.available() < sizeof(float) && millis() - startTime < 100) {
    ; // Wait for enough bytes
  }
    Serial.print("Receive attempt ");
    Serial.print(retry + 1);
    Serial.print(" - Available bytes: ");
    Serial.println(commSerial.available());
    if (commSerial.available() >= sizeof(float)) {
      float receivedFloat;
      commSerial.readBytes((uint8_t*)&receivedFloat, sizeof(receivedFloat));
      Serial.print("Received float: ");
      Serial.println(receivedFloat, 1);
      if (receivedFloat <= 180.0f && receivedFloat >= -180.0f) {
        Serial.print("Received Relative Angle (loopback): ");
        Serial.print(receivedFloat, 1);
        Serial.println("° (0° is ahead)");
      } else {
        Serial.println("No valid police alert received (loopback): 999.0");
      }
      lastReceivePrint = currentTime;
      return; // Success, exit function
    }
    // Clear buffer before retry
    while (commSerial.available()) commSerial.read();
    delay(10); // Brief pause before retry
  }
  Serial.println("Error: No data received after retries!");
  // Dump raw buffer for debugging
  Serial.print("Raw buffer: ");
  while (commSerial.available()) {
    Serial.print(commSerial.read(), HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void handleCommSerial(unsigned long currentTime) {
  // Optional: Check for stray data outside processAlerts
  if (commSerial.available() >= sizeof(float)) {
    receiveData(currentTime);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Loopback Test with SIM7000G is running!");
  Serial.print("Sizeof float: ");
  Serial.println(sizeof(float)); // Debug float size
  configTime(0, 0, "pool.ntp.org");
  Serial.println("Synchronizing time with NTP server...");
  initCommSerial();
  initWiFi();
  initGPS();
  initHMC5883L();
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