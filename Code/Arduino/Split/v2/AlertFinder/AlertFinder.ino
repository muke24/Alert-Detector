// AlertFinder.ino (Corrected)
// Project: Alert Finder V1.0 (Modular)
// Description: Main application sketch that initializes and orchestrates all modules.

// =========================================================================
// INCLUDE ALL MODULE HEADERS
// =========================================================================
#include "config.h"
#include "global_types.h"
#include "sim7000g_gps.h"
#include "network_manager.h"
#include "bno086_imu.h"
#include "esp32_comm.h"
#include "led_controller.h"
#include "audio_controller.h"
#include "alert_processor.h"

// =========================================================================
// DEFINE GLOBAL OBJECTS & VARIABLES
// =========================================================================

// <<< FIX: Define configuration constants that were declared 'extern' in config.h >>>
const char* WIFI_SSID      = "TelstraE6FB15";
const char* WIFI_PASSWORD  = "thebigtent";
const char* APN            = "";
const char* GPRS_USER      = "";
const char* GPRS_PASS      = "";
const char* WAZE_HOST      = "www.waze.com";
const char* WAZE_PATH      = "/live-map/api/georss";
const char* WMM_HOST       = "www.ngdc.noaa.gov";
const char* WMM_PATH       = "/geomag-web/calculators/calculateDeclination";
const char* WMM_API_KEY    = "zNEw7";

// Hardware & Library Objects
TinyGsm          modem(SerialAT);
TinyGsmClient    gsmClient(modem);
TinyGPSPlus      gps; // <<< FIX: Define the global gps object
BNO08x           myIMU;
CRGB             leds[NUM_LEDS];
HardwareSerial   commSerial(2);

// Audio Objects
AudioGeneratorWAV* wav = nullptr;
AudioFileSourcePROGMEM* file = nullptr;
AudioOutputI2S* out = nullptr;

// Mutex
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// State Variables
Location currentLocation      = {0, 0};
Location lastCheckedLocation  = {0, 0};
Location lastWmmLocation      = {0, 0};
float    currentDirection     = 0;
bool     isLocationInitialized= false;
bool     bnoInitialized       = false;
bool     isGprsConnected      = false;
Alert* currentAlerts        = nullptr;
volatile int alertCount       = 0;
volatile int closestIndex     = -1;
volatile bool playSound       = false;
volatile float multiplier     = 1.0f;
DeclinationData currentDeclination = {0, 0, 0, "", false};

// Timing for main loop
static unsigned long lastApiCallTimestamp = 0;


// =========================================================================
// SETUP and LOOP (No changes from previous version)
// =========================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("\n\nAlertFinder Modular V1.0 Starting...");

  initCommSerial();
  initNetwork();
  initGPS();
  initBNO086();
  initLEDs();
  initAudio();

  Serial.println("All modules initialized. Starting tasks...");

  startLedTask();
  startAudioTask();

  lastApiCallTimestamp = millis();
  Serial.println("System setup complete. Entering main loop.");
}

void loop() {
  updateGPS();
  updateIMU();
  maintainNetworkConnection();

  float distanceMoved = calculateDistance(currentLocation, lastCheckedLocation);
  float timeSinceLastCheck = (millis() - lastApiCallTimestamp) / 1000.0f;

  if (isLocationInitialized && (timeSinceLastCheck >= ALERT_CHECK_INTERVAL_S || distanceMoved > MOVEMENT_THRESHOLD_KM)) {
    Serial.println("----------------------------------------");
    Serial.print("Triggering Waze check. Reason: ");
    if (timeSinceLastCheck >= ALERT_CHECK_INTERVAL_S) {
      Serial.println("Time interval elapsed.");
    } else {
      Serial.print("Moved ");
      Serial.print(distanceMoved * 1000, 0);
      Serial.println(" meters.");
    }
    
    fetchWazeData();
    
    lastApiCallTimestamp = millis();
    lastCheckedLocation = currentLocation;
  }

  handleCommSerial();
  printGPS();
}