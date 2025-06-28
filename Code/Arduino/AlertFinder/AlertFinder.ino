// Project: Alert Finder V1.0

// PARTS (V1.0):
// - LILYGO T-SIM7000G V1.1 ESP32 (this script runs on this device)
// - Viewe ESP32 1.28 Inch 240×240 IOT Smart Display Screen Rotate and Press Circular Knob Screen with WiFi BLE
// - Adafruit Stemma Speaker
// - WS2812B LED strip (46 in total)
// - BNO086 IMU (replacing HMC5883L)

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

// LED Strip Configuration
#define LED_PIN 14    // GPIO14 on LILYGO ESP32
#define NUM_LEDS 46   // Total number of LEDs

// Speaker Configuration
#define SPEAKER_PIN 25 // GPIO25 for Adafruit STEMMA Speaker signal (DAC1)

// BNO086 Configuration
#define BNO08X_INT  19  // GPIO19 for interrupt
#define BNO08X_RST  23  // GPIO23 for reset
#define BNO08X_ADDR 0x4B  // Default I2C address (0x4A if ADR jumper closed)

// Libraries
#include <HardwareSerial.h>     // For UART communication
#include <TinyGsmClient.h>      // For SIM7000G modem
#include <TinyGPS++.h>          // For parsing GPS data
#include <Wire.h>               // For I2C with BNO086
#include "SparkFun_BNO08x_Arduino_Library.h"  // For BNO086 IMU
#include <WiFi.h>               // For Wi-Fi connectivity
#include <WiFiClientSecure.h>   // For secure HTTPS requests
#include <HTTPClient.h>         // For HTTP requests to Waze API and NOAA WMM
#include <ArduinoJson.h>        // For parsing JSON responses
#include <FastLED.h>            // For WS2812B LED strip control
#include <freertos/FreeRTOS.h>  // For FreeRTOS tasks
#include <freertos/task.h>      // For task management
#include <AudioFileSourcePROGMEM.h> // For playing audio from PROGMEM
#include <AudioGeneratorWAV.h>      // For WAV file playback
#include <AudioOutputI2S.h>         // For audio output to DAC
#include "alert_wav.h"              // Include the custom sound file byte array

// Configuration Constants
const char* ssid = "BigCock69";     // Replace with your Wi-Fi SSID
const char* password = "GymBro69";  // Replace with your Wi-Fi password
const char apn[] = "";              // SET TO YOUR APN
const char gprsUser[] = "";
const char gprsPass[] = "";
const float maxDistanceKm = 5.0f;      // Max distance for alerts (kilometers)
const float checkInterval = 45.0f;      // Interval to check for new alerts (seconds)
const float movementThreshold = 0.2f;   // Distance to trigger alert check (kilometers)
const float wmmUpdateDistance = 100.0f; // Distance to trigger WMM update (kilometers)
const unsigned long wmmUpdateInterval = 24 * 60 * 60 * 1000UL; // Check WMM daily (ms)
const unsigned long wmmRetryDelay = 5 * 60 * 1000UL; // Retry after 5 minutes on API error
const char* baseHost = "www.waze.com";  // Waze API host
const char* basePath = "/live-map/api/georss"; // Waze API path
const char* wmmHost = "www.ngdc.noaa.gov"; // NOAA WMM API host
const char* wmmPath = "/geomag-web/calculators/calculateDeclination"; // NOAA WMM API path
const unsigned long printInterval = 500;         // Print GPS/IMU data every 500ms
const unsigned long retryInterval = 5000;        // Retry BNO086 initialization every 5s
const unsigned long receivePrintInterval = 1000; // Print received UART data every 1000ms
const unsigned long ledUpdateInterval = 50;      // Update LEDs every 50ms
const int maxImuFailures = 5;                    // Max consecutive IMU failures

// Hardware Objects
HardwareSerial commSerial(2);  // UART2 for communication
TinyGsm modem(SerialAT);       // TinyGSM modem object
TinyGsmClient gsmClient(modem); // TinyGSM client
TinyGPSPlus gps;               // TinyGPS++ object
BNO08x myIMU;                  // BNO086 IMU
CRGB leds[NUM_LEDS];           // LED strip array

// Audio Objects
AudioGeneratorWAV *wav = nullptr;
AudioFileSourcePROGMEM *file = nullptr;
AudioOutputI2S *out = nullptr;

// LED Row Definitions
int row1[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};              // Row 1: LEDs 0 to 11
int row2[] = {22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12};        // Row 2: LEDs 12 to 22
int row3[] = {23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33};        // Row 3: LEDs 23 to 33
int row4[] = {45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34};    // Row 4: LEDs 34 to 45

// Number of LEDs in Each Row
const int num_leds_row1 = 12;
const int num_leds_row2 = 11;
const int num_leds_row3 = 11;
const int num_leds_row4 = 12;

// LED Color and Multiplier
CRGB color = CRGB(128, 255, 0); // R:128, G:255, B:0 (lime green)
volatile float multiplier = 1.0f; // Animation speed multiplier (1.0 to 2.0), volatile for task safety

// State Variables
bool bnoInitialized = false;
int imuFailureCount = 0;
unsigned long lastGpsPrint = 0;
unsigned long lastImuPrint = 0;
unsigned long lastBnoRetry = 0;
unsigned long lastApiCall = 0;
unsigned long lastReceivePrint = 0;
unsigned long lastWmmUpdate = 0; // Timestamp of last WMM update
unsigned long lastWmmAttempt = 0; // Timestamp of last WMM API attempt
float timeSinceLastCheck = 0;
bool isLocationInitialized = false;
float currentDirection = 0;
bool isGprsConnected = false;
volatile int alertCount = 0; // Volatile for task safety
volatile int closestIndex = -1; // Volatile for task safety
volatile bool playSound = false; // Flag to trigger sound in audio task

// Data Structures
struct Location {
  float latitude;
  float longitude;
};
Location currentLocation = {0, 0};
Location lastCheckedLocation = {0, 0};
Location lastWmmLocation = {0, 0}; // Location of last WMM update

// Declination Data Structure
struct DeclinationData {
  float latitude;
  float longitude;
  float declination;
  String modelVersion; // e.g., "WMM2025"
  bool isValid;
};
DeclinationData currentDeclination = {0, 0, 0, "WMM2025", false};

// Data Structures for Alerts
struct Alert {
  String type;
  String subtype;
  Location location;
  String street;
};
Alert* currentAlerts = nullptr;

struct BoundingArea {
  float left;
  float bottom;
  float right;
  float top;
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
  return 6371.0 * c; // Distance in kilometers
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

float calculateMultiplier(float distance) {
  if (distance >= maxDistanceKm) return 1.0f;
  if (distance <= 0) return 2.0f;
  return 2.0f - (distance / maxDistanceKm); // Linear interpolation
}

bool fetchWmmData(Location location, DeclinationData &declination) {
  if (!isLocationInitialized || (WiFi.status() != WL_CONNECTED && !isGprsConnected)) {
    Serial.println("Cannot fetch WMM data: No internet or invalid location");
    return false;
  }

  // Check if we're in a retry delay period due to previous API error
  if (millis() - lastWmmAttempt < wmmRetryDelay) {
    Serial.println("WMM API retry delay active. Using cached declination.");
    return false;
  }
  lastWmmAttempt = millis();

  // Get date from GPS or use default (2025-06-27)
  int year = 2025;
  int month = 6;
  int day = 27;
  if (gps.date.isValid()) {
    year = gps.date.year();
    month = gps.date.month();
    day = gps.date.day();
    // Validate year for WMM (2024-2029)
    if (year < 2024 || year > 2029) {
      Serial.println("GPS year " + String(year) + " outside WMM range (2024-2029). Using default 2025.");
      year = 2025;
    }
    Serial.print("Using GPS date: ");
    Serial.print(year);
    Serial.print("-");
    Serial.print(month);
    Serial.print("-");
    Serial.println(day);
  } else {
    Serial.println("Invalid GPS date. Using default date: 2025-06-27");
  }

  // Construct API request with GPS date
  String path = String(wmmPath) + "?lat1=" + String(location.latitude, 6) +
                "&lon1=" + String(location.longitude, 6) +
                "&model=WMM" +
                "&startYear=" + String(year) +
                "&startMonth=" + String(month) +
                "&startDay=" + String(day) +
                "&key=zNEw7" +
                "&resultFormat=json";
  String payload;

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Fetching WMM data via Wi-Fi...");
    HTTPClient http;
    WiFiClientSecure client;
    client.setInsecure(); // Note: For production, use proper certificates
    String url = "https://" + String(wmmHost) + path;
    http.setTimeout(10000);
    if (!http.begin(client, url)) {
      Serial.println("Failed to begin HTTP connection for WMM");
      http.end();
      return false;
    }
    int httpCode = http.GET();
    if (httpCode <= 0 || httpCode != HTTP_CODE_OK) {
      Serial.println("WMM API call failed, HTTP code: " + String(httpCode));
      if (httpCode == 429) {
        Serial.println("Rate limit exceeded. Retrying after 5 minutes.");
      }
      http.end();
      return false;
    }
    payload = http.getString();
    http.end();
  } else {
    Serial.println("Fetching WMM data via cellular...");
    String response;
    if (!gsmClient.connect(wmmHost, 443)) {
      Serial.println("Failed to connect to NOAA WMM server via cellular");
      return false;
    }
    String request = "GET " + path + " HTTP/1.1\r\n";
    request += "Host: " + String(wmmHost) + "\r\n";
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
      payload = response.substring(headerEnd + 4);
    } else {
      Serial.println("Failed to parse cellular WMM HTTP response");
      return false;
    }
  }

  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Serial.println("WMM JSON parsing failed: " + String(error.c_str()));
    return false;
  }

  if (!doc.containsKey("result") || !doc["result"][0].containsKey("declination")) {
    Serial.println("WMM JSON response missing declination data");
    return false;
  }

  declination.latitude = location.latitude;
  declination.longitude = location.longitude;
  declination.declination = doc["result"][0]["declination"].as<float>();
  declination.modelVersion = doc["model"].as<String>();
  declination.isValid = true;

  Serial.print("Fetched WMM Declination: ");
  Serial.print(declination.declination);
  Serial.print("° for Lat: ");
  Serial.print(location.latitude, 6);
  Serial.print(", Lon: ");
  Serial.print(location.longitude, 6);
  Serial.print(", Model: ");
  Serial.println(declination.modelVersion);

  return true;
}

float calculateDeclination(float latitude, float longitude) {
  if (!currentDeclination.isValid || 
      calculateDistance(currentLocation, lastWmmLocation) > wmmUpdateDistance ||
      millis() - lastWmmUpdate >= wmmUpdateInterval) {
    DeclinationData newDeclination;
    if (fetchWmmData(currentLocation, newDeclination)) {
      if (!currentDeclination.isValid || 
          newDeclination.modelVersion != currentDeclination.modelVersion ||
          calculateDistance(currentLocation, {currentDeclination.latitude, currentDeclination.longitude}) > wmmUpdateDistance) {
        currentDeclination = newDeclination;
        lastWmmLocation = currentLocation;
        lastWmmUpdate = millis();
      }
    } else if (currentDeclination.isValid) {
      Serial.println("Using cached declination due to fetch failure");
    } else {
      Serial.println("No valid declination data available. Using 0.0");
      return 0.0;
    }
  }
  return currentDeclination.declination;
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

void initBNO086() {
  Wire.begin(21, 22); // SDA=21, SCL=22 on LILYGO ESP32
  if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST)) {
    Serial.println("BNO086 initialized successfully!");
    bnoInitialized = true;
    setReports();
  } else {
    Serial.println("Failed to find BNO086. Check wiring!");
    bnoInitialized = false;
  }
}

void setReports() {
  if (myIMU.enableGeomagneticRotationVector(100)) {  // Report every 100ms
    Serial.println("Geomagnetic rotation vector enabled");
  } else {
    Serial.println("Failed to enable geomagnetic rotation vector");
  }
}

void initCommSerial() {
  commSerial.begin(115200, SERIAL_8N1, COMM_RX, COMM_TX);
  Serial.print("UART2 initialized: GPIO");
  Serial.print(COMM_TX);
  Serial.print(" (TX) to GPIO");
  Serial.print(COMM_RX);
  Serial.println(" (RX)");
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
        lastWmmLocation = currentLocation;
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
      lastWmmLocation = currentLocation;
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
  if (!bnoInitialized) {
    if (currentTime - lastBnoRetry >= retryInterval) {
      initBNO086();
      lastBnoRetry = currentTime;
    }
    return;
  }

  if (myIMU.wasReset()) {
    Serial.println("Sensor reset detected");
    setReports();
  }

  if (myIMU.getSensorEvent()) {
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR) {
      float w = myIMU.getQuatReal();
      float x = myIMU.getQuatI();
      float y = myIMU.getQuatJ();
      float z = myIMU.getQuatK();

      // Calculate yaw (heading) in radians
      float yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
      float headingMagnetic = yaw * (180.0 / PI);
      if (headingMagnetic < 0) headingMagnetic += 360;

      // Adjust for true north with magnetic declination from GPS
      float declination = calculateDeclination(currentLocation.latitude, currentLocation.longitude);
      float headingTrue = headingMagnetic + declination;
      if (headingTrue >= 360) headingTrue -= 360;
      if (headingTrue < 0) headingTrue += 360;

      currentDirection = headingTrue;

      if (currentTime - lastImuPrint >= printInterval) {
        Serial.print("BNO086 Magnetic Heading: ");
        Serial.print(headingMagnetic);
        Serial.print("° | True Heading: ");
        Serial.print(headingTrue);
        Serial.print("° | Declination: ");
        Serial.print(declination);
        Serial.println("°");
        lastImuPrint = currentTime;
      }
    }
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
    Serial.println("Failed to connect to server via cellular");
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
    HTTPClient http;
    WiFiClientSecure client;
    client.setInsecure(); // Note: For production, use proper certificates
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
    while (commSerial.available()) commSerial.read();
    commSerial.write((uint8_t*)&noAlert, sizeof(float));
    commSerial.flush();
    Serial.print("Sent float: ");
    Serial.println(noAlert, 1);
    closestIndex = -1;
    receiveData(currentTime);
    return;
  }
  closestIndex = -1;
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
    multiplier = calculateMultiplier(minDistance);
    Serial.println("Closest Police Alert:");
    Serial.println("  Type: " + alert.type);
    Serial.println("  Subtype: " + alert.subtype);
    Serial.println("  Location: Lat=" + String(alert.location.latitude, 6) +
                   ", Lon=" + String(alert.location.longitude, 6));
    Serial.println("  Street: " + alert.street);
    Serial.println("  Distance: " + String(minDistance, 2) + " km");
    Serial.println("  Relative Angle: " + String(relativeAngle, 1) + "° (0° is ahead)");
    Serial.println("  LED Multiplier: " + String(multiplier, 2));
    while (commSerial.available()) commSerial.read();
    commSerial.write((uint8_t*)&relativeAngle, sizeof(float));
    commSerial.flush();
    Serial.print("Sent float: ");
    Serial.println(relativeAngle, 1);
    receiveData(currentTime);
  } else {
    Serial.println("No police alerts found.");
    float noAlert = 999.0;
    while (commSerial.available()) commSerial.read();
    commSerial.write((uint8_t*)&noAlert, sizeof(float));
    commSerial.flush();
    Serial.print("Sent float: ");
    Serial.println(noAlert, 1);
    closestIndex = -1;
    receiveData(currentTime);
  }
}

void receiveData(unsigned long currentTime) {
  const int maxRetries = 3;
  for (int retry = 0; retry < maxRetries; retry++) {
    unsigned long startTime = millis();
    while (commSerial.available() < sizeof(float) && millis() - startTime < 100) {
      ;
    }
    if (commSerial.available() >= sizeof(float)) {
      float receivedFloat;
      commSerial.readBytes((uint8_t*)&receivedFloat, sizeof(receivedFloat));
      if (currentTime - lastReceivePrint >= receivePrintInterval) {
        Serial.print("Received float: ");
        Serial.println(receivedFloat, 1);
        if (receivedFloat <= 180.0f && receivedFloat >= -180.0f) {
          Serial.print("Received Relative Angle: ");
          Serial.print(receivedFloat, 1);
          Serial.println("° (0° is ahead)");
        } else {
          Serial.println("No valid police alert received: 999.0");
        }
        lastReceivePrint = currentTime;
      }
      return;
    }
    while (commSerial.available()) commSerial.read();
    delay(10);
  }
  if (currentTime - lastReceivePrint >= receivePrintInterval) {
    Serial.println("Error: No data received after retries!");
    lastReceivePrint = currentTime;
  }
}

void handleCommSerial(unsigned long currentTime) {
  if (commSerial.available() >= sizeof(float)) {
    receiveData(currentTime);
  }
}

// LED Animation Task
void ledTask(void *pvParameters) {
  unsigned long lastLedUpdate = 0;
  while (1) {
    unsigned long currentTime = millis();
    if (currentTime - lastLedUpdate >= ledUpdateInterval) {
      lastLedUpdate = currentTime;
      if (alertCount > 0 && closestIndex >= 0) {
        float period = 1000.0 * multiplier; // Period in ms, adjusted by multiplier
        float elapsed = fmod(currentTime, period);
        static float prevElapsed = 0;
        if (elapsed < prevElapsed) {
          playSound = true; // Signal to play sound at cycle start
        }
        prevElapsed = elapsed;
        float fraction = elapsed / period;
        FastLED.clear();
        for (int i = 0; i < num_leds_row1; i++) {
          if (i < fraction * num_leds_row1) leds[row1[i]] = color;
        }
        for (int i = 0; i < num_leds_row2; i++) {
          if (i < fraction * num_leds_row2) leds[row2[i]] = color;
        }
        for (int i = 0; i < num_leds_row3; i++) {
          if (i < fraction * num_leds_row3) leds[row3[i]] = color;
        }
        for (int i = 0; i < num_leds_row4; i++) {
          if (i < fraction * num_leds_row4) leds[row4[i]] = color;
        }
        FastLED.show();
      } else {
        FastLED.clear();
        FastLED.show();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Yield control for 10ms
  }
}

// Audio Task
void audioTask(void *pvParameters) {
  while (1) {
    if (playSound) {
      if (wav == nullptr || !wav->isRunning()) {
        if (wav != nullptr) {
          wav->stop();
          delete wav;
          delete file;
        }
        file = new AudioFileSourcePROGMEM(alert_wav, alert_wav_len);
        wav = new AudioGeneratorWAV();
        wav->begin(file, out);
        playSound = false; // Reset flag after starting playback
      }
    }
    if (wav != nullptr && wav->isRunning()) {
      if (!wav->loop()) {
        wav->stop();
        delete wav;
        delete file;
        wav = nullptr;
        file = nullptr;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1)); // Yield control frequently for smooth playback
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("AlertFinder_LILYGO starting...");
  initCommSerial();
  initWiFi();
  initGPS();
  initBNO086();
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS); // Initialize LED strip

  // Initialize audio output to DAC on GPIO25
  out = new AudioOutputI2S();
  out->SetPinout(0, 0, SPEAKER_PIN); // Use DAC on GPIO25
  out->SetGain(0.5); // Set volume (0.0 to 1.0)

  // Create LED task on core 0
  xTaskCreatePinnedToCore(
    ledTask,     // Task function
    "LED Task",  // Task name
    10000,       // Stack size in words
    NULL,        // Parameters
    1,           // Priority
    NULL,        // Task handle
    0            // Core 0
  );

  // Create Audio task on core 1
  xTaskCreatePinnedToCore(
    audioTask,   // Task function
    "Audio Task",// Task name
    10000,       // Stack size in words
    NULL,        // Parameters
    1,           // Priority
    NULL,        // Task handle
    1            // Core 1
  );
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