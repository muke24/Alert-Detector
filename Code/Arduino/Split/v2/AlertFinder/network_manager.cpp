// network_manager.cpp
#include "network_manager.h"
#include "config.h"
#include "global_types.h"
#include "alert_processor.h" // For boundingBox() and processAlerts()
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h> // Required for gps.date

// The TinyGPS++ object, declared here to be accessible for date info
extern TinyGPSPlus gps;

// Timestamps for WMM API logic, local to this module
static unsigned long lastWmmUpdate = 0;
static unsigned long lastWmmAttempt = 0;

// Internal helper function for fetching WMM data
static bool fetchWmmData(Location location, DeclinationData &declination);

void initNetwork() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to Wi-Fi");
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected to Wi-Fi.");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
        isGprsConnected = false;
        return;
    }

    Serial.println("\nWi-Fi connection failed. Attempting cellular connection...");
    if (!modem.waitForNetwork(30000L)) {
        Serial.println("Failed to find cellular network.");
        return;
    }
    Serial.println("Cellular network found.");

    if (!modem.gprsConnect(APN, GPRS_USER, GPRS_PASS)) {
        Serial.println("Failed to connect to GPRS.");
        return;
    }
    isGprsConnected = true;
    Serial.println("GPRS connected.");
    Serial.print("Modem IP: ");
    Serial.println(modem.localIP());
}

void maintainNetworkConnection() {
    if (WiFi.status() == WL_CONNECTED) {
        if (isGprsConnected) {
            modem.gprsDisconnect();
            isGprsConnected = false;
            Serial.println("Switched from Cellular to Wi-Fi.");
        }
        return;
    }

    // If WiFi is not connected, and we aren't already on GPRS, try to connect.
    if (!isGprsConnected) {
        Serial.println("Wi-Fi disconnected. Attempting to switch to cellular...");
        // First try to reconnect to WiFi quickly
        WiFi.reconnect();
        delay(2000); // Give it a moment
        if (WiFi.status() == WL_CONNECTED) {
             Serial.println("Reconnected to Wi-Fi.");
             return;
        }

        // If WiFi reconnect fails, move to cellular
        if (!modem.isNetworkConnected()) {
            if (!modem.waitForNetwork(30000L)) {
                Serial.println("Failed to connect to cellular network.");
                return;
            }
        }
        if (!modem.gprsConnect(APN, GPRS_USER, GPRS_PASS)) {
            Serial.println("Failed to connect to GPRS.");
            return;
        }
        isGprsConnected = true;
        Serial.println("GPRS connected.");
    }
}

void fetchWazeData() {
    if (!isLocationInitialized || (WiFi.status() != WL_CONNECTED && !isGprsConnected)) {
        return; // No location or no internet
    }

    BoundingArea area = boundingBox(currentLocation, MAX_ALERT_DISTANCE_KM);
    String path = String(WAZE_PATH) + "?top=" + String(area.top, 6) + "&bottom=" + String(area.bottom, 6) +
                  "&left=" + String(area.left, 6) + "&right=" + String(area.right, 6) + "&env=row&types=alerts";

    String payload = "";
    bool success = false;

    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        WiFiClientSecure client;
        client.setInsecure(); // For simplicity. For production, use certificates.
        String url = "https://" + String(WAZE_HOST) + path;
        http.setTimeout(10000);
        if (http.begin(client, url)) {
            int httpCode = http.GET();
            if (httpCode == HTTP_CODE_OK) {
                payload = http.getString();
                success = true;
            } else {
                Serial.printf("[HTTP] Waze GET failed, error: %d %s\n", httpCode, http.errorToString(httpCode).c_str());
            }
            http.end();
        }
    } else if (isGprsConnected) {
        Serial.println("Fetching Waze data via cellular...");
        gsmClient.connect(WAZE_HOST, 443);
        gsmClient.print(String("GET ") + path + " HTTP/1.1\r\n" +
                        "Host: " + WAZE_HOST + "\r\n" +
                        "Connection: close\r\n\r\n");
        
        // Basic response handling
        unsigned long timeout = millis();
        while (gsmClient.connected() && millis() - timeout < 10000L) {
            if (gsmClient.available()) {
                String response = gsmClient.readString();
                int bodyPos = response.indexOf("\r\n\r\n");
                if(bodyPos > 0) {
                    payload = response.substring(bodyPos + 4);
                    success = true;
                }
                break;
            }
        }
        gsmClient.stop();
    }

    if (success && payload.length() > 0) {
        DynamicJsonDocument doc(4096); // Increased size for Waze alerts
        DeserializationError error = deserializeJson(doc, payload);
        if (error) {
            Serial.println("Waze JSON parsing failed!");
            return;
        }

        JsonArray wazeAlerts = doc["alerts"];
        if (currentAlerts != nullptr) {
            delete[] currentAlerts;
            currentAlerts = nullptr;
        }

        alertCount = wazeAlerts.size();
        if (alertCount > 0) {
            currentAlerts = new Alert[alertCount];
            for (int i = 0; i < alertCount; i++) {
                currentAlerts[i].type = wazeAlerts[i]["type"].as<String>();
                currentAlerts[i].subtype = wazeAlerts[i]["subtype"].as<String>();
                currentAlerts[i].location.latitude = wazeAlerts[i]["location"]["y"].as<float>();
                currentAlerts[i].location.longitude = wazeAlerts[i]["location"]["x"].as<float>();
                currentAlerts[i].street = wazeAlerts[i]["street"].as<String>();
            }
        }
        processAlerts(); // Process the newly fetched alerts
    }
}


float calculateDeclination(float latitude, float longitude) {
    if (!isLocationInitialized) return 0.0f;

    bool needsUpdate = !currentDeclination.isValid ||
                       calculateDistance(currentLocation, lastWmmLocation) > WMM_UPDATE_DISTANCE_KM ||
                       millis() - lastWmmUpdate >= WMM_UPDATE_INTERVAL_MS;

    if (needsUpdate) {
        Serial.println("Updating magnetic declination data from NOAA...");
        DeclinationData newDeclination;
        if (fetchWmmData(currentLocation, newDeclination)) {
            currentDeclination = newDeclination;
            lastWmmLocation = currentLocation;
            lastWmmUpdate = millis();
        } else if (currentDeclination.isValid) {
            Serial.println("Using cached declination due to fetch failure.");
        } else {
            Serial.println("No valid declination data available. Using 0.0.");
            return 0.0;
        }
    }
    return currentDeclination.declination;
}


static bool fetchWmmData(Location location, DeclinationData &declination) {
    if (millis() - lastWmmAttempt < WMM_RETRY_DELAY_MS) {
        Serial.println("WMM API retry delay active.");
        return false;
    }
    lastWmmAttempt = millis();

    int year = 2025;
    int month = 6;
    int day = 27;

    if (gps.date.isValid() && gps.date.year() >= 2024 && gps.date.year() <= 2029) {
        year = gps.date.year();
        month = gps.date.month();
        day = gps.date.day();
    }

    String path = String(WMM_PATH) + "?lat1=" + String(location.latitude, 6) +
                  "&lon1=" + String(location.longitude, 6) +
                  "&model=WMM" +
                  "&startYear=" + String(year) +
                  "&startMonth=" + String(month) +
                  "&startDay=" + String(day) +
                  "&key=" + String(WMM_API_KEY) +
                  "&resultFormat=json";

    String payload = "";
    bool success = false;

    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        WiFiClientSecure client;
        client.setInsecure();
        String url = "https://" + String(WMM_HOST) + path;
        http.setTimeout(10000);
        if (http.begin(client, url)) {
            int httpCode = http.GET();
            if (httpCode == HTTP_CODE_OK) {
                payload = http.getString();
                success = true;
            } else {
                 Serial.printf("[HTTP] WMM GET failed, error: %d %s\n", httpCode, http.errorToString(httpCode).c_str());
            }
            http.end();
        }
    } else if (isGprsConnected) {
        Serial.println("Fetching WMM data via cellular...");
        gsmClient.connect(WMM_HOST, 443);
        gsmClient.print(String("GET ") + path + " HTTP/1.1\r\n" +
                        "Host: " + WMM_HOST + "\r\n" +
                        "Connection: close\r\n\r\n");

        unsigned long timeout = millis();
        while (gsmClient.connected() && millis() - timeout < 10000L) {
             if (gsmClient.available()) {
                String response = gsmClient.readString();
                int bodyPos = response.indexOf("\r\n\r\n");
                if(bodyPos > 0) {
                    payload = response.substring(bodyPos + 4);
                    success = true;
                }
                break;
            }
        }
        gsmClient.stop();
    }
    
    if (success && payload.length() > 0) {
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, payload);
        if (error) {
            Serial.println("WMM JSON parsing failed!");
            return false;
        }

        if (!doc.containsKey("result") || !doc["result"][0].containsKey("declination")) {
            Serial.println("WMM JSON response missing declination data.");
            return false;
        }
        declination.latitude = location.latitude;
        declination.longitude = location.longitude;
        declination.declination = doc["result"][0]["declination"].as<float>();
        declination.modelVersion = doc["model"].as<String>();
        declination.isValid = true;

        Serial.print("Fetched WMM Declination: ");
        Serial.print(declination.declination, 2);
        Serial.print("°, Model: ");
        Serial.println(declination.modelVersion);
        return true;
    }
    return false;
}