// network_manager.cpp (Corrected with User-Agent)
#include "network_manager.h"
#include "config.h"
#include "global_types.h"
#include "alert_processor.h" // For boundingBox() and processAlerts()
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h> // Required for gps.date

// --- Root CA Certificate for www.waze.com (DigiCert Global Root G2) ---
// This is used to verify the server's identity for a secure HTTPS connection.
const char* waze_root_ca = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIDjjCCAnagAwIBAgIQAzRxT6uOqsQd4X03ADK58jANBgkqhkiG9w0BAQsFADBh\n" \
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n" \
"d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n" \
"MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT\n" \
"MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n" \
"b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG\n" \
"9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiAljMo2vAf\n" \
"yzfxpZtgZkYwLS7P4AADsACr4zBMyqMvEVma1U64r1vHwZfERsNFo9HnSYLwUKsV\n" \
"9uGDBOI4TozRd12J1LryPO4mIrbcIFtO+VLczqV5MAzKksNbX25xiecQDvWnsG2o\n" \
"pMEdd4ivtlx8kP2iPMPLRT+5V4zLoQ59aEK9scQi2hv2/i4JCDi+RVuwjwJcFj8k\n" \
"dTuPym3BEV2dStiBSrU7FhZ2D5jQ2Z7/lZ0+t9mq3GmtcoCj7vR26vxU1Ym1rHRE\n" \
"A9oXyQkDFCIvR0UqWoKfiixKPxGnvcH0sYxGqXyQjO6bW0Mn/DRS4fLqgLEGFK3x\n" \
"Q+hA5hM4vL0tH3YA1+2v/yM9Wml8b0vr9jR5AZ0x9hY2yQodgfn2eB9stReIAWOK\n" \
"JkAM+s2xU70HD3m42NOC+j5s8h9b7/BMEb2AzCSwhMh/omw3uH44EDiYd1aV5j9K\n" \
"VfVp4dsS2bY62b+gT39VfX6912pL02gR8MAe4V+3S/pC3eTO9M2gS62sJPv1U28a\n" \
"OqsuPAgJvK1sWHV4v3V4262A7M7bM3o+2wK+jT44L9dKekYg44lE+iD+9VPimR9C\n" \
"GmxN4w7x1gCnskd1Wfgp4S1i3rGNd+yKxAhFW21J+d3b4OgjLzMhHqfD+g3S2fA6\n" \
"iQIDAQABo2YwZDAOBgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNV\n" \
"HQ4EFgQUA5qI0d2TvU75P63sMyComFkDZXQwHwYDVR0jBBgwFoAUA5qI0d2TvU75\n" \
"P63sMyComFkDZXQwDQYJKoZIhvcNAQELBQADggEBAA+p5BLyGv0d47s+d/XW0i/k\n" \
"AQMhD5xG8Gz/M0a9v5wMPyZfM3r5S2dKAHPq/0IL+6H2i4F12d4gR90g3J+s0c4P\n" \
"lP/kl2/g7Gq7+4v9Ld7v3r8o26g3V12d/k6a2sLAn4C3Yl/a3sOYSZTjM9L25vva\n" \
"AAn0ebn7s7d2UvXy00m8yW/2uV88x0k32vjL/MoK3JtP5f/N4s3gA1/GcvY2sW2C\n" \
"i2dp34Qjgyh1Qp2h2wWfPITHYc0EybtW3k3aI214s8A0z4w1zTRb2b2/tXoPCk4s\n" \
"H3b2W22n1yH26pTTsWvHhQyR23oFKA==\n" \
"-----END CERTIFICATE-----\n";

extern TinyGPSPlus gps;
static unsigned long lastWmmUpdate = 0;
static unsigned long lastWmmAttempt = 0;

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

    if (!isGprsConnected) {
        Serial.println("Wi-Fi disconnected. Attempting to switch to cellular...");
        WiFi.reconnect();
        delay(2000);
        if (WiFi.status() == WL_CONNECTED) {
             Serial.println("Reconnected to Wi-Fi.");
             return;
        }

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
        return; 
    }

    BoundingArea area = boundingBox(currentLocation, MAX_ALERT_DISTANCE_KM);
    String path = String(WAZE_PATH) + "?top=" + String(area.top, 6) + "&bottom=" + String(area.bottom, 6) +
                  "&left=" + String(area.left, 6) + "&right=" + String(area.right, 6) + "&env=row&types=alerts";

    String payload = "";
    bool success = false;

    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        WiFiClientSecure client;
        client.setCACert(waze_root_ca);
        String url = "https://" + String(WAZE_HOST) + path;
        
        if (http.begin(client, url)) {
            // <<< FIX: Add all headers to make the request as browser-like as possible
            http.addHeader("User-Agent", "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/58.0.3029.110 Safari/537.36");
            http.addHeader("Referer", "https://www.waze.com/"); // Corrected Referer
            http.addHeader("Accept", "application/json, text/plain, */*");
            
            int httpCode = http.GET();
            
            if (httpCode > 0) { 
                if (httpCode == HTTP_CODE_OK) {
                    payload = http.getString();
                    success = true;
                } else {
                    Serial.printf("[HTTP] Waze GET failed, unexpected HTTP code: %d\n", httpCode);
                }
            } else {
                Serial.printf("[HTTP] Waze GET failed, error: %s\n", http.errorToString(httpCode).c_str());
            }
            http.end();
        } else {
            Serial.println("Failed to begin HTTP connection.");
        }

    } else if (isGprsConnected) {
        Serial.println("Fetching Waze data via cellular...");
        if (gsmClient.connect(WAZE_HOST, 443)) {
            // <<< FIX: Add Referer to cellular request as well
            gsmClient.print(String("GET ") + path + " HTTP/1.1\r\n" +
                            "Host: " + String(WAZE_HOST) + "\r\n" +
                            "User-Agent: Mozilla/5.0\r\n" + 
                            "Referer: https://www.waze.com/live-map/\r\n" +
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
        } else {
            Serial.println("Cellular client failed to connect.");
        }
    }

    if (success && payload.length() > 0) {
        DynamicJsonDocument doc(4096);
        DeserializationError error = deserializeJson(doc, payload);
        if (error) {
            Serial.print("Waze JSON parsing failed: ");
            Serial.println(error.c_str());
            return;
        }

        JsonArray wazeAlerts = doc["alerts"];
        if (currentAlerts != nullptr) {
            delete[] currentAlerts;
            currentAlerts = nullptr;
        }

        taskENTER_CRITICAL(&timerMux);
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
        taskEXIT_CRITICAL(&timerMux);
        
        processAlerts();
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