// =================================================================
// AlertFinder_Split.ino
// Project: Alert Finder V1.0 (Refactored)
// =================================================================
// This is the main project file. It defines global objects and calls
// functions from the other scripts to orchestrate the program flow.
// =================================================================


// =================================================================
// 1. INCLUDES
// =================================================================
// (A) Our custom header files
#include "config.h"
#include "globals.h"
#include "utilities.h"
#include "initialization.h"
#include "sensor_handler.h"
#include "api_handler.h"
#include "tasks.h"

// (B) All necessary libraries for the project
// (It's best practice to include them all in the main .ino file for the Arduino IDE)
#include <HardwareSerial.h>
#include <TinyGsmClient.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <FastLED.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "AudioFileSourcePROGMEM.h"
#include "AudioGeneratorWAV.h"
#include "AudioOutputI2S.h"
#include "alert_wav.h"

// =================================================================
// CONFIGURATION VARIABLE DEFINITIONS (NEW SECTION)
// =================================================================
// This is where we DEFINE the variables declared in config.h
const char* ssid = "TelstraE6FB15";
const char* password = "thebigtent";
const char apn[] = "";
const char gprsUser[] = "";
const char gprsPass[] = "";

const char* baseHost = "www.waze.com";
const char* basePath = "/live-map/api/georss";
const char* wmmHost = "www.ngdc.noaa.gov";
const char* wmmPath = "/geomag-web/calculators/calculateDeclination";

const float maxDistanceKm = 5.0f;
const float checkInterval = 45.0f;
const float movementThreshold = 0.2f;
const float wmmUpdateDistance = 100.0f;
const unsigned long wmmUpdateInterval = 24 * 60 * 60 * 1000UL;
const unsigned long wmmRetryDelay = 5 * 60 * 1000UL;
const unsigned long printInterval = 500;
const unsigned long retryInterval = 5000;
const unsigned long receivePrintInterval = 1000;
const unsigned long ledUpdateInterval = 50;
const int maxImuFailures = 5;

// =================================================================
// 2. GLOBAL VARIABLE DEFINITIONS
// =================================================================
// This is where all the 'extern' variables from globals.h are actually created.

// --- Hardware Objects ---
HardwareSerial commSerial(2);
TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);
TinyGPSPlus gps;
BNO08x myIMU;
CRGB leds[NUM_LEDS];

// --- Audio Objects ---
AudioGeneratorWAV *wav = nullptr;
AudioFileSourcePROGMEM *file = nullptr;
AudioOutputI2S *out = nullptr;

// --- LED Row Definitions ---
int row1[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
int row2[] = {22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12};
int row3[] = {23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33};
int row4[] = {45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34};
const int num_leds_row1 = sizeof(row1) / sizeof(int);
const int num_leds_row2 = sizeof(row2) / sizeof(int);
const int num_leds_row3 = sizeof(row3) / sizeof(int);
const int num_leds_row4 = sizeof(row4) / sizeof(int);

// --- LED Color and Multiplier ---
CRGB color = CRGB(128, 255, 0); // Lime green
volatile float multiplier = 1.0f;

// --- State Variables ---
bool bnoInitialized = false;
int imuFailureCount = 0;
unsigned long lastGpsPrint = 0;
unsigned long lastImuPrint = 0;
unsigned long lastBnoRetry = 0;
unsigned long lastApiCall = 0;
unsigned long lastReceivePrint = 0;
unsigned long lastWmmUpdate = 0;
unsigned long lastWmmAttempt = 0;
float timeSinceLastCheck = 0;
bool isLocationInitialized = false;
float currentDirection = 0;
bool isGprsConnected = false;
volatile int alertCount = 0;
volatile int closestIndex = -1;
volatile bool playSound = false;

// --- Data Structure Instances ---
Location currentLocation = {0, 0};
Location lastCheckedLocation = {0, 0};
Location lastWmmLocation = {0, 0};
DeclinationData currentDeclination = {0, 0, 0, "WMM2025", false};
Alert* currentAlerts = nullptr;


// =================================================================
// 3. SETUP & LOOP
// =================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("\n\nAlertFinder starting...");

  // Call our organized initialization functions
  initCommSerial();
  initLeds();
  initAudio();
  initGPS();
  initWiFi(); // Init GPS first to give modem time to start, then connect
  initBNO086();
  
  Serial.println("Core initializations complete. Starting tasks...");
  createTasks();

  Serial.println("Setup complete. Entering main loop.");
}

void loop() {
  // Get current time once at the start of the loop
  unsigned long currentTime = millis();

  // Update the time since the last API call
  timeSinceLastCheck += (currentTime - lastApiCall) / 1000.0;
  
  // Call our organized handler functions
  updateGPS();
  printGPS(currentTime);
  updateIMU(currentTime);
  maintainWiFi();
  fetchWazeData(currentTime);
  handleCommSerial(currentTime);
}