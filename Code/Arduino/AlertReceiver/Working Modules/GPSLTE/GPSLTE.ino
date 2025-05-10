#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <ArduinoJson.h>


// Serial connection to SIM7080G (UART2: RX=16, TX=17 on ESP32)
HardwareSerial simSerial(2);

// Serial connection to NEO-M8N GPS module (UART1: RX=18, TX=19)
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

// Variables for GPS coordinates and alert fetching logic
float currentLat = 0.0;
float currentLon = 0.0;
float lastCheckedLat = 0.0;
float lastCheckedLon = 0.0;

// Configuration settings
float maxDistanceKm = 1.0;      // Search radius for alerts in kilometers
float checkInterval = 60.0;     // Check alerts every 60 seconds
float movementThreshold = 0.2;  // Trigger fetch if moved 0.2 km
unsigned long lastCheckTime = 0;

void setup() {
  // Initialize serial communication for debugging and SIM7080G
  Serial.begin(115200);
  simSerial.begin(115200, SERIAL_8N1, 16, 17);

  // Initialize GPS serial for NEO-M8N
  gpsSerial.begin(9600, SERIAL_8N1, 18, 19);

  // Optional: Uncomment and set SIM PIN if required
  // simSerial.println("AT+CPIN=\"your_pin\"");
  // delay(1000);

  Serial.println("Setup complete: SIM7080G and NEO-M8N initialized");
}

void loop() {
  // Read available GPS data from NEO-M8N
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // Get current GPS location
  getGPSLocation();

  // Proceed only if valid GPS fix is obtained
  if (currentLat != 0.0 && currentLon != 0.0) {
    unsigned long currentTime = millis();
    float timeSinceLastCheck = (currentTime - lastCheckTime) / 1000.0;
    float distanceMoved = calculateDistance(currentLat, currentLon, lastCheckedLat, lastCheckedLon);

    // Fetch alerts if enough time has passed or significant movement detected
    if (timeSinceLastCheck >= checkInterval || distanceMoved > movementThreshold) {
      fetchAlerts();
      lastCheckedLat = currentLat;
      lastCheckedLon = currentLon;
      lastCheckTime = currentTime;
    }
  }
  delay(1000); // Prevent loop from running too quickly
}

/**
 * Retrieve GPS location from NEO-M8N using TinyGPS++
 */
void getGPSLocation() {
  if (gps.location.isValid()) {
    currentLat = gps.location.lat();
    currentLon = gps.location.lng();
    Serial.println("GPS Location: Lat=" + String(currentLat, 6) + ", Lon=" + String(currentLon, 6));
  } else {
    currentLat = 0.0;
    currentLon = 0.0;
    Serial.println("Waiting for GPS fix...");
  }
}

/**
 * Calculate bounding box coordinates based on current position and distance
 */
void calculateBoundingBox(float lat, float lon, float distanceKm, float& top, float& bottom, float& left, float& right) {
  float latInRadians = lat * PI / 180.0;
  float deltaLatitude = distanceKm / 111.0; // 1 degree latitude ≈ 111 km
  float deltaLongitude = distanceKm / (111.0 * cos(latInRadians)); // Adjust for longitude
  top = lat + deltaLatitude;
  bottom = lat - deltaLatitude;
  left = lon - deltaLongitude;
  right = lon + deltaLongitude;
}

/**
 * Calculate approximate distance between two points in kilometers
 */
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  float dLat = (lat2 - lat1) * 111.0; // Latitude difference in km
  float dLon = (lon2 - lon1) * 111.0 * cos(lat1 * PI / 180.0); // Longitude difference in km
  return sqrt(dLat * dLat + dLon * dLon); // Euclidean distance approximation
}

/**
 * Fetch Waze alerts using SIM7080G cellular connection
 */
void fetchAlerts() {
  float top, bottom, left, right;
  calculateBoundingBox(currentLat, currentLon, maxDistanceKm, top, bottom, left, right);

  Serial.println("Now attempting to fetch Waze request...");

  // Construct Waze API URL
  String url = "https://www.waze.com/live-map/api/georss?top=" + String(top, 6) +
               "&bottom=" + String(bottom, 6) +
               "&left=" + String(left, 6) +
               "&right=" + String(right, 6) +
               "&env=row&types=alerts";

  // Check Network Registration
  String response;
  if (sendATCommand("AT+CEREG?", "+CEREG:", response, 5000)) {
    Serial.println("Network Registration: " + response);
    if (!(response.indexOf("+CEREG: 0,1") >= 0 || response.indexOf("+CEREG: 0,5") >= 0)) {
      Serial.println("Error: Module not registered on network!");
      return; // Exit if not registered
    }
  } else {
    Serial.println("Error: Failed to check network registration");
    return;
  }

  // Check Signal Strength
  if (sendATCommand("AT+CSQ", "+CSQ:", response, 5000)) {
    Serial.println("Signal Strength: " + response);
    int signalQuality;
    sscanf(response.c_str(), "+CSQ: %d,", &signalQuality);
    if (signalQuality == 99) {
      Serial.println("Error: No signal detected!");
      return; // Exit if no signal
    } else if (signalQuality < 10) {
      Serial.println("Warning: Weak signal strength");
    }
  } else {
    Serial.println("Error: Failed to check signal strength");
    return;
  }

  // Configure APN for AldiMobile SIM card
  if (!sendATCommand("AT+CGDCONT=1,\"IP\",\"telstra.internet\"", "OK", response, 5000)) {
    Serial.println("Error: Failed to set APN");
    return;
  }

  // Activate Packet Data Protocol (PDP) context
  if (!sendATCommand("AT+CGACT=1,1", "OK", response, 10000)) {
    Serial.println("Error: Failed to activate PDP context");
    return;
  }

  // Verify PDP Context
  if (sendATCommand("AT+CGACT?", "+CGACT:", response, 5000)) {
    Serial.println("PDP Context Status: " + response);
    if (response.indexOf("+CGACT: 1,1") == -1) {
      Serial.println("Error: PDP context not active!");
      return;
    }
  } else {
    Serial.println("Error: Failed to verify PDP context");
    return;
  }

  // Initialize HTTP service
  if (!sendATCommand("AT+HTTPINIT", "OK", response, 5000)) {
    Serial.println("Error: Failed to initialize HTTP service");
    return;
  }

  // Enable HTTPS
  if (!sendATCommand("AT+HTTPSSL=1", "OK", response, 5000)) {
    Serial.println("Error: Failed to enable HTTPS");
    simSerial.println("AT+HTTPTERM"); // Clean up
    delay(1000);
    return;
  }

  // Set URL for HTTP request
  if (!sendATCommand("AT+HTTPPARA=\"URL\",\"" + url + "\"", "OK", response, 5000)) {
    Serial.println("Error: Failed to set HTTP URL");
    simSerial.println("AT+HTTPTERM");
    delay(1000);
    return;
  }

  // Send HTTP GET request and process response
  if (sendATCommand("AT+HTTPACTION=0", "+HTTPACTION:", response, 30000)) { // Extended timeout
    int method, status, data_length;
    sscanf(response.c_str(), "+HTTPACTION: %d,%d,%d", &method, &status, &data_length);
    if (status == 200) {
      if (sendATCommand("AT+HTTPREAD", "+HTTPREAD:", response, 10000)) {
        int read_status, read_length;
        sscanf(response.c_str(), "+HTTPREAD: %d,%d", &read_status, &read_length);
        if (read_status == 0) {
          String jsonData = readHTTPData(read_length);
          DynamicJsonDocument doc(2048);
          DeserializationError error = deserializeJson(doc, jsonData);
          if (!error) {
            JsonArray alerts = doc["alerts"];
            if (alerts.isNull() || alerts.size() == 0) {
              Serial.println("No alerts found.");
            } else {
              Serial.println("Alerts received:");
              for (JsonVariant alert : alerts) {
                String type = alert["type"] | "Unknown";
                String subtype = alert["subtype"] | "N/A";
                float x = alert["location"]["x"] | 0.0;
                float y = alert["location"]["y"] | 0.0;
                String street = alert["street"] | "Unknown";
                Serial.println("- Type: " + type +
                              ", Subtype: " + subtype +
                              ", Location: (" + String(x, 6) + ", " + String(y, 6) +
                              "), Street: " + street);
              }
            }
          } else {
            Serial.println("JSON parsing failed: " + String(error.c_str()));
          }
        }
      }
    } else {
      Serial.println("HTTP request failed with status: " + String(status));
    }
  } else {
    Serial.println("Error: HTTP request timed out or failed");
  }

  // Terminate HTTP service
  simSerial.println("AT+HTTPTERM");
  delay(1000);

  // Deactivate PDP context
  simSerial.println("AT+CGACT=0,1");
  delay(1000);
}

/**
 * Read a single line from the SIM7080G serial port
 */
String readLine(unsigned long timeout = 1000) {
  String line = "";
  unsigned long startTime = millis();
  while (millis() - startTime < timeout) {
    if (simSerial.available()) {
      char c = simSerial.read();
      if (c == '\n') {
        return line;
      } else if (c != '\r') {
        line += c;
      }
    }
  }
  return line;
}

/**
 * Send an AT command and wait for a specific response
 */
bool sendATCommand(String command, const char* expectedStart, String& response, unsigned long timeout) {
  Serial.println("Sending AT command: " + command);
  simSerial.println(command);
  unsigned long startTime = millis();
  while (millis() - startTime < timeout) {
    String line = readLine();
    Serial.println("Received: " + line);
    if (line.startsWith(expectedStart)) {
      response = line;
      return true;
    }
  }
  Serial.println("Timeout waiting for response to: " + command);
  return false;
}

/**
 * Read HTTP data of specified length from SIM7080G
 */
String readHTTPData(int length) {
  String data = "";
  for (int i = 0; i < length; i++) {
    while (!simSerial.available()) {
      // Wait for data
    }
    data += (char)simSerial.read();
  }
  return data;
}