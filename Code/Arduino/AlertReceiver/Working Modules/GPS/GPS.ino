#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// Define the serial port for GPS (UART2 on ESP32)
HardwareSerial gpsSerial(2);

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
const unsigned long printInterval = 500; // Print every 500ms
const unsigned long retryInterval = 5000; // Retry BNO08X every 5 seconds

void setup() {
  // Initialize the console serial port
  Serial.begin(115200);
  Serial.println("ESP32 is running!");
  
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

  // Get current time
  unsigned long currentTime = millis();

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

  // Print IMU data every printInterval (500ms) if initialized
  if (bnoInitialized && (currentTime - lastImuPrint >= printInterval)) {
    sh2_SensorValue_t sensorValue;
    if (bno08x.getSensorEvent(&sensorValue) && sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      Serial.print("IMU Yaw: "); Serial.print(sensorValue.un.rotationVector.real);
      Serial.print(", Pitch: "); Serial.print(sensorValue.un.rotationVector.i);
      Serial.print(", Roll: "); Serial.println(sensorValue.un.rotationVector.j);
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
}