// sim7000g_gps.cpp
#include "sim7000g_gps.h"
#include "config.h"
#include "global_types.h"
#include <TinyGsmClient.h>
#include <TinyGPS++.h>

// GPS parsing object
//static TinyGPSPlus gps;

// Timestamp for printing GPS data
static unsigned long lastGpsPrint = 0;

void initGPS() {
    // Power on the SIM7000G module
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, HIGH);
    delay(300);
    digitalWrite(PWR_PIN, LOW);
    delay(1000);
    digitalWrite(PWR_PIN, HIGH);

    // Begin serial communication with the modem
    SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
    delay(1000);

    Serial.println("Initializing modem...");
    if (!modem.restart()) {
        Serial.println("Failed to restart modem, attempting to continue without restarting.");
        // Try to init instead of restart
        if (!modem.init()) {
            Serial.println("Failed to initialize modem!");
            return;
        }
    }

    // Explicitly power on the GPS chip on the SIM7000G
    // This sends the AT command: AT+SGPIO=0,4,1,1
    modem.sendAT("+SGPIO=0,4,1,1");
    if (modem.waitResponse(10000L) != 1) {
        Serial.println("Failed to send GPS power on command.");
    } else {
        Serial.println("GPS power on command sent successfully.");
    }

    // Enable GPS/GNSS
    if (modem.enableGPS()) {
        Serial.println("GPS enabled successfully.");
    } else {
        Serial.println("Failed to enable GPS.");
    }
}

void updateGPS() {
    // First, try the high-level TinyGSM function to get GPS data
    float lat, lon;
    if (modem.getGPS(&lat, &lon)) {
        // Check for valid, non-zero coordinates
        if (lat != 0.0 && lon != 0.0) {
            currentLocation.latitude = lat;
            currentLocation.longitude = lon;

            if (!isLocationInitialized) {
                // Set initial locations for distance checks
                lastCheckedLocation = currentLocation;
                lastWmmLocation = currentLocation;
                isLocationInitialized = true;
                Serial.println("GPS location initialized!");
            }
            return; // Got location, no need to parse NMEA manually
        }
    }

    // If getGPS() fails or returns 0,0, fall back to parsing NMEA stream
    while (SerialAT.available() > 0) {
        gps.encode(SerialAT.read());
    }

    if (gps.location.isUpdated() && gps.location.isValid()) {
        currentLocation.latitude = gps.location.lat();
        currentLocation.longitude = gps.location.lng();

        if (!isLocationInitialized) {
            // Set initial locations for distance checks
            lastCheckedLocation = currentLocation;
            lastWmmLocation = currentLocation;
            isLocationInitialized = true;
            Serial.println("GPS location initialized via NMEA parsing!");
        }
    }
}

void printGPS() {
    if (millis() - lastGpsPrint >= GPS_IMU_PRINT_INTERVAL_MS) {
        if (isLocationInitialized) {
            Serial.print("GPS Lat: "); Serial.print(currentLocation.latitude, 6);
            Serial.print(", Lon: "); Serial.println(currentLocation.longitude, 6);
        } else {
            Serial.println("GPS: No fix yet...");
        }
        lastGpsPrint = millis();
    }
}