// api_handler.cpp
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "api_handler.h"
#include "utilities.h" // For boundingBox, calculateDistance, etc.

/**
 * @brief Helper function for making HTTPS GET requests over cellular.
 */
bool performCellularHttpGet(const String& host, const String& path, String& response) {
  if (!gsmClient.connect(host.c_str(), 443)) {
    Serial.println("Failed to connect to server via cellular");
    return false;
  }

  String request = "GET " + path + " HTTP/1.1\r\n";
  request += "Host: " + host + "\r\n";
  request += "Connection: close\r\n\r\n";
  
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
    return true;
  } else {
    Serial.println("Failed to parse cellular HTTP response");
    return false;
  }
}


/**
 * @brief Fetches magnetic declination data from the NOAA WMM API.
 */
bool fetchWmmData(Location location, DeclinationData &declination) {
  if (!isLocationInitialized || (WiFi.status() != WL_CONNECTED && !isGprsConnected)) {
    Serial.println("Cannot fetch WMM data: No internet or invalid location");
    return false;
  }

  if (millis() - lastWmmAttempt < wmmRetryDelay) {
    Serial.println("WMM API retry delay active. Using cached declination.");
    return false;
  }
  lastWmmAttempt = millis();

  int year = 2025, month = 6, day = 28; // Default date
  if (gps.date.isValid() && gps.date.year() >= 2024 && gps.date.year() <= 2029) {
    year = gps.date.year();
    month = gps.date.month();
    day = gps.date.day();
    Serial.println("Using GPS date for WMM query: " + String(year) + "-" + String(month) + "-" + String(day));
  } else {
    Serial.println("Using default date for WMM query: 2025-06-28");
  }

  String path = String(wmmPath) + "?lat1=" + String(location.latitude, 6) +
                "&lon1=" + String(location.longitude, 6) +
                "&model=WMM" +
                "&startYear=" + String(year) + "&startMonth=" + String(month) + "&startDay=" + String(day) +
                "&key=zNEw7" + "&resultFormat=json";
  
  String payload;

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    WiFiClientSecure client;
    client.setInsecure(); // For development; use proper certs for production
    String url = "https://" + String(wmmHost) + path;
    http.setTimeout(10000);
    if (!http.begin(client, url)) {
      Serial.println("Failed to begin HTTP connection for WMM");
      return false;
    }
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      payload = http.getString();
    } else {
      Serial.println("WMM API call failed, HTTP code: " + String(httpCode));
      http.end();
      return false;
    }
    http.end();
  } else { // Cellular connection
    if (!performCellularHttpGet(wmmHost, path, payload)) {
      Serial.println("WMM API call failed via cellular");
      return false;
    }
  }

  DynamicJsonDocument doc(1024);
  if (deserializeJson(doc, payload)) {
    Serial.println("WMM JSON parsing failed.");
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
  Serial.println("Fetched WMM Declination: " + String(declination.declination) + "°, Model: " + declination.modelVersion);
  return true;
}


/**
 * @brief Provides magnetic declination, fetching new data if necessary.
 */
float calculateDeclination(float latitude, float longitude) {
  bool needsUpdate = !currentDeclination.isValid ||
                     calculateDistance(currentLocation, lastWmmLocation) > wmmUpdateDistance ||
                     millis() - lastWmmUpdate >= wmmUpdateInterval;

  if (needsUpdate) {
    Serial.println("WMM data is stale or invalid. Attempting to fetch new data.");
    DeclinationData newDeclination;
    if (fetchWmmData(currentLocation, newDeclination)) {
      currentDeclination = newDeclination;
      lastWmmLocation = currentLocation;
      lastWmmUpdate = millis();
    } else if (currentDeclination.isValid) {
      Serial.println("Using cached declination due to fetch failure.");
    } else {
      Serial.println("No valid declination data available. Using 0.0");
      return 0.0;
    }
  }
  return currentDeclination.declination;
}


/**
 * @brief Fetches alert data from the Waze API based on the current location.
 */
void fetchWazeData(unsigned long currentTime) {
  if (!isLocationInitialized) return;

  float distanceMoved = calculateDistance(currentLocation, lastCheckedLocation);
  if (timeSinceLastCheck < checkInterval && distanceMoved <= movementThreshold) return;
  
  if (WiFi.status() != WL_CONNECTED && !isGprsConnected) {
    Serial.println("No internet connection available for Waze API");
    return;
  }

  BoundingArea area = boundingBox(currentLocation, maxDistanceKm);
  String path = String(basePath) + "?top=" + String(area.top, 6) + "&bottom=" + String(area.bottom, 6) +
                "&left=" + String(area.left, 6) + "&right=" + String(area.right, 6) + "&env=row&types=alerts";
  String payload;

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    WiFiClientSecure client;
    client.setInsecure();
    String url = "https://" + String(baseHost) + path;
    http.setTimeout(10000);
    if (!http.begin(client, url)) {
      Serial.println("Failed to begin Waze HTTP connection");
      return;
    }
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      payload = http.getString();
    } else {
      Serial.println("Waze API call failed, HTTP code: " + String(httpCode));
      http.end();
      return;
    }
    http.end();
  } else { // Cellular connection
    if (!performCellularHttpGet(baseHost, path, payload)) {
      Serial.println("Waze API call failed via cellular");
      return;
    }
  }

  DynamicJsonDocument doc(2048);
  if (deserializeJson(doc, payload)) {
    Serial.println("Waze JSON parsing failed");
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


/**
 * @brief Processes the downloaded alerts to find the closest police alert and sends data.
 */
void processAlerts(unsigned long currentTime) {
  if (alertCount == 0 || !bnoInitialized) {
    if (alertCount == 0) Serial.println("No alerts found.");
    if (!bnoInitialized) Serial.println("IMU not initialized.");
    
    closestIndex = -1;
    float noAlert = 999.0;
    while (commSerial.available()) commSerial.read(); // Clear buffer
    commSerial.write((uint8_t*)&noAlert, sizeof(float));
    commSerial.flush();
    Serial.println("Sent float: 999.0 (No Alert)");
    receiveData(currentTime); // Listen for a response
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
    
    Serial.println("Closest Police Alert: " + alert.subtype + " on " + alert.street);
    Serial.println("  Distance: " + String(minDistance, 2) + " km");
    Serial.println("  Relative Angle: " + String(relativeAngle, 1) + "°");
    
    while (commSerial.available()) commSerial.read(); // Clear buffer
    commSerial.write((uint8_t*)&relativeAngle, sizeof(float));
    commSerial.flush();
    Serial.println("Sent float: " + String(relativeAngle, 1));
    receiveData(currentTime); // Listen for a response
  } else {
    Serial.println("No POLICE alerts found in the current batch.");
    closestIndex = -1;
    float noAlert = 999.0;
    while (commSerial.available()) commSerial.read(); // Clear buffer
    commSerial.write((uint8_t*)&noAlert, sizeof(float));
    commSerial.flush();
    Serial.println("Sent float: 999.0 (No Alert)");
    receiveData(currentTime); // Listen for a response
  }
}

/**
 * @brief Listens for a response from the other ESP32 with retries.
 */
void receiveData(unsigned long currentTime) {
  const int maxRetries = 3;
  for (int retry = 0; retry < maxRetries; retry++) {
    unsigned long startTime = millis();
    while (commSerial.available() < sizeof(float) && millis() - startTime < 100) {
      delay(1); // Small delay to prevent busy-waiting
    }

    if (commSerial.available() >= sizeof(float)) {
      float receivedFloat;
      commSerial.readBytes((uint8_t*)&receivedFloat, sizeof(receivedFloat));
      if (currentTime - lastReceivePrint >= receivePrintInterval) {
          Serial.print("Received float from display ESP: ");
          Serial.println(receivedFloat, 1);
          lastReceivePrint = currentTime;
      }
      return; // Success
    }
    delay(10); // Wait before retrying
  }
  
  if (currentTime - lastReceivePrint >= receivePrintInterval) {
    Serial.println("Error: No data received from display ESP after retries!");
    lastReceivePrint = currentTime;
  }
}

/**
 * @brief Checks for any unsolicited incoming data from the other ESP32.
 */
void handleCommSerial(unsigned long currentTime) {
    if (commSerial.available() >= sizeof(float)) {
        receiveData(currentTime);
    }
}

/**
 * @brief Maintains internet connectivity, switching between Wi-Fi and Cellular.
 */
void maintainWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (isGprsConnected) {
      modem.gprsDisconnect();
      isGprsConnected = false;
      Serial.println("Switched from Cellular to Wi-Fi");
    }
    return;
  }

  Serial.println("Wi-Fi disconnected. Attempting to reconnect...");
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
      modem.waitForNetwork(30000L);
    }
    if (modem.isNetworkConnected()) {
        if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
            Serial.println("Failed to connect to GPRS");
        } else {
            isGprsConnected = true;
            Serial.println("GPRS connected");
        }
    } else {
        Serial.println("Failed to connect to cellular network");
    }
  }
}