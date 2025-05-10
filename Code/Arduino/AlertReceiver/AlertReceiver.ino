#include <HardwareSerial.h>
#include <TinyGPS++.h>

// Define the UART pins for GPS
#define RXD2 16  // GPIO16 for U2-RXD
#define TXD2 17  // GPIO17 for U2-TXD

// Initialize UART2 for GPS and TinyGPS++ object
HardwareSerial gpsSerial(2);  // UART2
TinyGPSPlus gps;

void setup() {
  // Start Serial Monitor for debugging
  Serial.begin(115200);
  // Start UART2 for GPS communication
  gpsSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);  // 9600 baud is typical for Neo M8N
  
  Serial.println("GPS Module Initialized");
}

void loop() {
  // Read data from GPS module
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      // Check if a valid location fix is available
      if (gps.location.isValid()) {
        Serial.print("Latitude: ");
        Serial.println(gps.location.lat(), 6);  // Print latitude with 6 decimal places
        Serial.print("Longitude: ");
        Serial.println(gps.location.lng(), 6);  // Print longitude with 6 decimal places
        Serial.print("Altitude: ");
        Serial.println(gps.altitude.meters());  // Print altitude in meters
      } else {
        Serial.println("Waiting for GPS fix...");
      }
    }
  }
  delay(1000);  // Delay to avoid overwhelming the Serial Monitor
}