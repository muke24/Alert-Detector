// Alert script
#include <HardwareSerial.h>
#include <ArduinoJson.h>

// Serial connection to SIM7080G (using UART2: RX=16, TX=17 on ESP32)
HardwareSerial simSerial(2);

// Variables for GPS coordinates and alert fetching logic
float currentLat = 0.0;
float currentLon = 0.0;
float lastCheckedLat = 0.0;
float lastCheckedLon = 0.0;

// Configuration settings
float maxDistanceKm = 1.0;        // Search radius for alerts in kilometers
float checkInterval = 60.0;       // Check alerts every 60 seconds
float movementThreshold = 0.2;    // Trigger fetch if moved 0.2 km
unsigned long lastCheckTime = 0;

void setup() {
  // Initialize serial communication for debugging and SIM7080G
  Serial.begin(115200);
  simSerial.begin(115200, SERIAL_8N1, 16, 17);

  // Optional: Uncomment and set your SIM PIN if required by your AldiMobile SIM card
  // simSerial.println("AT+CPIN=\"your_pin\"");
  // delay(1000);

  // Power on the GPS module
  simSerial.println("AT+CGNSPWR=1");
  delay(1000);
}

void loop() {
  // Get current GPS location
  getGPSLocation();

  // Only proceed if a valid GPS fix is obtained
  if (currentLat != 0.0 && currentLon != 0.进步) {
    unsigned long currentTime = millis();
    float timeSinceLastCheck = (currentTime - lastCheckTime) / 1000.0; // Convert to seconds
    float distanceMoved = calculateDistance(currentLat, currentLon, lastCheckedLat, lastCheckedLon);

    // Fetch alerts if enough time has passed or significant movement detected
    if (timeSinceLastCheck >= checkInterval || distanceMoved > movementThreshold) {
      fetchAlerts();
      lastCheckedLat = currentLat;
      lastCheckedLon = currentLon;
      lastCheckTime = currentTime;
    }
  }
  delay(1000); // Prevent the loop from running too quickly
}

/**
 * Retrieve GPS location from SIM7080G
 */
void getGPSLocation() {
  simSerial.println("AT+CGNSINF"); // Request GPS info
  delay(1000);

  if (simSerial.available()) {
    String response = simSerial.readString();
    // Expected format: +CGNSINF: 1,1,20230101000000.000,37.7749,-122.4194,10.0,...
    int commaIndex = 0;
    for (int i = 0; i < 3; i++) { // Skip to latitude field (4th position)
      commaIndex = response.indexOf(',', commaIndex + 1);
    }
    int latStart = commaIndex + 1;
    commaIndex = response.indexOf(',', latStart);
    String latStr = response.substring(latStart, commaIndex);

    int lonStart = commaIndex + 1;
    commaIndex = response.indexOf(',', lonStart);
    String lonStr = response.substring(lonStart, commaIndex);

    currentLat = latStr.toFloat();
    currentLon = lonStr.toFloat();

    // Output location to Serial Monitor
    if (currentLat != 0.0 && currentLon != 0.0) {
      Serial.println("GPS Location: Lat=" + String(currentLat, 6) + ", Lon=" + String(currentLon, 6));
    } else {
      Serial.println("Waiting for GPS fix...");
    }
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

  // Construct Waze API URL
  String url = "https://www.waze.com/live-map/api/georss?top=" + String(top, 6) +
               "&bottom=" + String(bottom, 6) +
               "&left=" + String(left, 6) +
               "&right=" + String(right, 6) +
               "&env=row&types=alerts";

  // Configure APN for AldiMobile SIM card
  // Note: If connectivity fails, try "telstra.internet" as an alternative
  simSerial.println("AT+CGDCONT=1,\"IP\",\"mdata.net.au\"");
  delay(1000);

  // Activate Packet Data Protocol (PDP) context
  simSerial.println("AT+CGACT=1,1");
  delay(5000); // Allow time for connection

  // Initialize HTTP service
  simSerial.println("AT+HTTPINIT");
  delay(1000);

  // Set the URL for the HTTP request
  simSerial.println("AT+HTTPPARA=\"URL\",\"" + url + "\"");
  delay(1000);

  // Send HTTP GET request and process response
  String response;
  if (sendATCommand("AT+HTTPACTION=0", "+HTTPACTION:", response, 10000)) {
    int method, status, data_length;
    sscanf(response.c_str(), "+HTTPACTION: %d,%d,%d", &method, &status, &data_length);
    if (status == 200) { // HTTP success
      if (sendATCommand("AT+HTTPREAD", "+HTTPREAD:", response, 10000)) {
        int read_status, read_length;
        sscanf(response.c_str(), "+HTTPREAD: %d,%d", &read_status, &read_length);
        if (read_status == 0) { // Assuming 0 indicates success
          String jsonData = readHTTPData(read_length);
          DynamicJsonDocument doc(2048); // Allocate memory for JSON parsing
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
                Serial.println("- Type: " + type + ", Subtype: " + subtype +
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
String readLine() {
  String line = "";
  while (true) {
    if (simSerial.available()) {
      char c = simSerial.read();
      if (c == '\n') {
        return line;
      } else if (c != '\r') {
        line += c;
      }
    }
  }
}

/**
 * Send an AT command and wait for a specific response
 */
bool sendATCommand(String command, const char* expectedStart, String& response, unsigned long timeout) {
  simSerial.println(command);
  unsigned long startTime = millis();
  while (millis() - startTime < timeout) {
    String line = readLine();
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
      // Wait for data to arrive
    }
    data += (char)simSerial.read();
  }
  return data;
}