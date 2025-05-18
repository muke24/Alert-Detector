#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// Wi-Fi credentials
const char* ssid = "BigCock69"; // Your Wi-Fi SSID
const char* password = "GymBro69"; // Your Wi-Fi password

// Waze API settings (from Unity script)
const float maxDistanceKm = 1.0f; // Max distance for alerts (kilometers)
const float checkInterval = 15.0f; // How often to check for new alerts (seconds)
const float movementThreshold = 0.2f; // Distance to trigger alert check (kilometers)
const char* baseUrl = "https://www.waze.com/live-map/api/georss";

// Define serial ports
HardwareSerial gpsSerial(2); // UART2 for GPS
HardwareSerial commSerial(1); // UART1 for loopback communication

// TinyGPS++ object for parsing GPS data
TinyGPSPlus gps;

// Adafruit BNO08x object for IMU data
Adafruit_BNO08x bno08x;

// Flag to track BNO08X initialization
bool bnoInitialized = false;

// Counter for consecutive IMU data failures
int imuFailureCount = 0;
const int maxImuFailures = 5; // Number of consecutive failures before reinitializing

// Timestamps for controlling print and retry intervals
unsigned long lastGpsPrint = 0;
unsigned long lastImuPrint = 0;
unsigned long lastBnoRetry = 0;
unsigned long lastApiCall = 0; // For API call interval
unsigned long lastReceivePrint = 0; // For printing received data
float timeSinceLastCheck = 0; // Timer for periodic API checks (seconds)
const unsigned long printInterval = 500; // Print every 500ms
const unsigned long retryInterval = 5000; // Retry BNO08X every 5 seconds
const unsigned long receivePrintInterval = 1000; // Print received data every 1000ms

// Location tracking
struct Location {
  float latitude;
  float longitude;
};
Location currentLocation = {0, 0};
Location lastCheckedLocation = {0, 0};
bool isLocationInitialized = false;

// Alert structure to store Waze API data
struct Alert {
  String type;
  String subtype;
  Location location;
  String street;
};
Alert* currentAlerts = nullptr;
int alertCount = 0;
float currentDirection = 0; // Current heading from IMU (degrees)

// Function to calculate bounding box for Waze API
struct BoundingArea {
  float top;
  float bottom;
  float left;
  float right;
};
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

// Function to calculate distance between two locations (Haversine formula)
float calculateDistance(Location loc1, Location loc2) {
  float lat1 = loc1.latitude * PI / 180.0;
  float lat2 = loc2.latitude * PI / 180.0;
  float lon1 = loc1.longitude * PI / 180.0;
  float lon2 = loc2.longitude * PI / 180.0;

  float dLat = lat2 - lat1;
  float dLon = lon2 - lon1;
  float a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return 6371.0 * c; // Earth's radius in kilometers
}

// Function to calculate bearing from one location to another
float calculateAngle(Location from, Location to) {
  float phi1 = from.latitude * PI / 180.0; // Latitude of current location
  float phi2 = to.latitude * PI / 180.0;   // Latitude of alert location
  float deltaLambda = (to.longitude - from.longitude) * PI / 180.0; // Longitude difference

  float y = sin(deltaLambda) * cos(phi2);
  float x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(deltaLambda);
  float theta = atan2(y, x);

  float bearing = (theta * 180.0 / PI + 360) - 180;
  return bearing;
}

// Function to normalize angle to [-180, 180]
float normalizeAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

// Function to calculate relative angle to alert
float calculateFacingDirection(Alert alert) {
  float rawAngle = calculateAngle(currentLocation, alert.location);
  float relativeAngle = rawAngle - currentDirection;
  return normalizeAngle(relativeAngle);
}

void setup() {
  // Initialize the console serial port (UART0)
  Serial.begin(115200);
  Serial.println("ESP32 Loopback Test is running!");

  // Initialize communication serial port (UART1) on GPIO19 (RX), GPIO18 (TX)
  commSerial.begin(9600, SERIAL_8N1, 19, 18); // RX=GPIO19, TX=GPIO18

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Initialize GPS serial port on GPIO16 (RX) and GPIO17 (TX)
  gpsSerial.begin(9600);

  // Initialize I2C bus on default pins (GPIO21 SDA, GPIO22 SCL)
  Wire.begin();

  // Attempt to initialize BNO08X IMU at address 0x4B
  if (!bno08x.begin_I2C(0x4B)) {
    Serial.println("Failed to find BNO08X during setup. Will retry in loop...");
    bnoInitialized = false;
  } else {
    Serial.println("BNO08X initialized successfully!");
    bno08x.enableReport(SH2_ROTATION_VECTOR);
    bnoInitialized = true;
  }
}

void loop() {
  // Read GPS data continuously
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // Read from commSerial (UART1) for loopback testing
  if (commSerial.available()) {
    unsigned long currentTime = millis();
    if (currentTime - lastReceivePrint >= receivePrintInterval) {
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
  }

  // Get current time
  unsigned long currentTime = millis();
  timeSinceLastCheck += (currentTime - lastApiCall) / 1000.0; // Update timer in seconds

  // Update current location from GPS
  if (gps.location.isValid()) {
    currentLocation.latitude = gps.location.lat();
    currentLocation.longitude = gps.location.lng();
    if (!isLocationInitialized) {
      lastCheckedLocation = currentLocation;
      isLocationInitialized = true;
    }
  }

  // Print GPS data every printInterval (500ms)
  if (currentTime - lastGpsPrint >= printInterval) {
    if (gps.location.isValid()) {
      Serial.print("GPS Lat: "); Serial.print(gps.location.lat(), 6);
      Serial.print(", Lon: "); Serial.println(gps.location.lng(), 6);
    } else {
      Serial.println("GPS: No fix");
    }
    lastGpsPrint = currentTime;
  }

  // If BNO08X is not initialized, try to initialize it every retryInterval (5 seconds)
  if (!bnoInitialized && (currentTime - lastBnoRetry >= retryInterval)) {
    if (bno08x.begin_I2C(0x4B)) {
      Serial.println("BNO08X initialized successfully!");
      bno08x.enableReport(SH2_ROTATION_VECTOR);
      bnoInitialized = true;
      imuFailureCount = 0; // Reset failure count on successful initialization
    } else {
      Serial.println("BNO08X still not found. Retrying...");
    }
    lastBnoRetry = currentTime;
  }

  // Update IMU data and print every printInterval (500ms) if initialized
  if (bnoInitialized && (currentTime - lastImuPrint >= printInterval)) {
    sh2_SensorValue_t sensorValue;
    if (bno08x.getSensorEvent(&sensorValue) && sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      currentDirection = sensorValue.un.rotationVector.real * 180.0 / PI; // Convert quaternion to yaw (degrees)
      Serial.print("IMU Yaw: "); Serial.print(currentDirection);
      Serial.print(", Pitch: "); Serial.print(sensorValue.un.rotationVector.i * 180.0 / PI);
      Serial.print(", Roll: "); Serial.println(sensorValue.un.rotationVector.j * 180.0 / PI);
      imuFailureCount = 0; // Reset failure count on successful read
    } else {
      Serial.println("IMU: No data");
      imuFailureCount++; // Increment failure count on failed read
    }

    // If too many consecutive failures, assume BNO08X is disconnected and reinitialize
    if (imuFailureCount >= maxImuFailures) {
      Serial.println("BNO08X appears to be disconnected. Attempting to reinitialize...");
      bnoInitialized = false;
      imuFailureCount = 0;
      lastBnoRetry = 0; // Force immediate retry
    }

    lastImuPrint = currentTime;
  }

  // Check Wi-Fi status and reconnect if disconnected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi disconnected. Reconnecting...");
    WiFi.reconnect();
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nReconnected to Wi-Fi");
  }

  // Check if it's time to fetch Waze API data
  if (isLocationInitialized) {
    float distanceMoved = calculateDistance(currentLocation, lastCheckedLocation);
    if ((timeSinceLastCheck >= checkInterval || distanceMoved > movementThreshold) && WiFi.status() == WL_CONNECTED) {
      // Build Waze API URL
      BoundingArea area = boundingBox(currentLocation, maxDistanceKm);
      String url = String(baseUrl) + "?top=" + String(area.top, 6) + "&bottom=" + String(area.bottom, 6) +
                   "&left=" + String(area.left, 6) + "&right=" + String(area.right, 6) + "&env=row&types=alerts";

      HTTPClient http;
      http.begin(url);
      int httpCode = http.GET();

      if (httpCode > 0) {
        if (httpCode == HTTP_CODE_OK) {
          String payload = http.getString();
          // Parse JSON response
          DynamicJsonDocument doc(2048); // Adjust size based on expected response
          DeserializationError error = deserializeJson(doc, payload);
          if (error) {
            Serial.println("JSON parsing failed: " + String(error.c_str()));
          } else {
            JsonArray alerts = doc["alerts"];
            alertCount = alerts.size();
            // Free previous alerts array if it exists
            if (currentAlerts != nullptr) {
              delete[] currentAlerts;
              currentAlerts = nullptr;
            }
            // Allocate new alerts array
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
            // Find and process closest police alert
            if (alertCount > 0 && bnoInitialized) {
              int closestIndex = -1;
              float minDistance = 999999.0; // Large initial value
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
                // Send relative angle via UART1
                commSerial.println(String(relativeAngle, 1));
              } else {
                Serial.println("No police alerts found.");
                // Send a sentinel value (e.g., 999.0) to indicate no alert
                commSerial.println("999.0");
              }
            } else {
              Serial.println("No alerts found or IMU not initialized.");
              // Send a sentinel value (e.g., 999.0) to indicate no alert
              commSerial.println("999.0");
            }
          }
        } else {
          Serial.printf("Waze API call failed, HTTP code: %d\n", httpCode);
        }
      } else {
        Serial.println("Waze API call failed, error: " + String(http.errorToString(httpCode).c_str()));
      }
      http.end();
      lastCheckedLocation = currentLocation;
      timeSinceLastCheck = 0;
      lastApiCall = currentTime;
    }
  }
}