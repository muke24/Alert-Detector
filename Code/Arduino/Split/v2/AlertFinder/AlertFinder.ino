// AlertFinder.ino (Corrected)

// =========================================================================
// DEVELOPMENT & DEMO OPTIONS
// =========================================================================
#define USE_DUMMY_DATA // Comment out this line to use the real network manager


// =========================================================================
// INCLUDE ALL MODULE HEADERS
// =========================================================================
#include "config.h"
#include "global_types.h"
#include "sim7000g_gps.h"

// Conditionally include the correct data provider
#ifdef USE_DUMMY_DATA
  #include "dummy_data_manager.h"
#else
  #include "network_manager.h"
#endif

#include "compass.h"
#include "display.h"
#include "led_controller.h"
#include "audio_controller.h"
#include "alert_processor.h"

// =========================================================================
// DEFINE GLOBAL OBJECTS & VARIABLES
// =========================================================================

// Define configuration constants
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
TinyGsm              modem(SerialAT);
TinyGsmClient        gsmClient(modem);
TinyGPSPlus          gps;
Adafruit_LSM6DSOX    dsox;
Adafruit_LIS3MDL     lis3mdl;
CRGB                 leds[NUM_LEDS];

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
bool     imuInitialized       = false;
bool     isGprsConnected      = false;
Alert* currentAlerts          = nullptr;
volatile int alertCount       = 0;
volatile int closestIndex     = -1;
volatile bool playSound       = false;
volatile float multiplier     = 1.0f;
volatile float alertAngle     = 999.0f;
DeclinationData currentDeclination = {0, 0, 0, "", false};

// Timing for main loop
static unsigned long lastApiCallTimestamp = 0;
static unsigned long gpsSearchStartTime = 0;

// =========================================================================
// SETUP and LOOP
// =========================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial);
    Serial.println("\n\nAlertFinder Modular V1.0 Starting...");

    initDisplay();
    
    // <<< FIX: Only initialize the network if we are NOT using dummy data
    #ifndef USE_DUMMY_DATA
      initNetwork();
    #endif

    initGPS();
    initCompass();
    initLEDs();
    initAudio();

    Serial.println("All modules initialized. Starting tasks...");

    startDisplayTask();
    startLedTask();
    startAudioTask();

    lastApiCallTimestamp = millis();
    gpsSearchStartTime = millis();
    Serial.println("System setup complete. Entering main loop.");
    Serial.print("Waiting for GPS fix (Timeout: ");
    Serial.print(GPS_FIX_TIMEOUT_MS / 1000 / 60);
    Serial.println(" minutes)...");
}

void loop() {
    updateGPS();
    updateIMU();

    #ifndef USE_DUMMY_DATA
      maintainNetworkConnection();
    #endif

    if (!isLocationInitialized && (millis() - gpsSearchStartTime > GPS_FIX_TIMEOUT_MS)) {
        Serial.println("!!! GPS FIX TIMEOUT !!!");
        Serial.print("Proceeding with default location: Lat ");
        Serial.print(DEFAULT_LATITUDE, 4);
        Serial.print(", Lon ");
        Serial.println(DEFAULT_LONGITUDE, 4);
        
        currentLocation = { DEFAULT_LATITUDE, DEFAULT_LONGITUDE };
        lastCheckedLocation = currentLocation;
        lastWmmLocation = currentLocation;
        isLocationInitialized = true;
    }

    float distanceMoved = calculateDistance(currentLocation, lastCheckedLocation);
    float timeSinceLastCheck = (millis() - lastApiCallTimestamp) / 1000.0f;

    if (isLocationInitialized && (timeSinceLastCheck >= ALERT_CHECK_INTERVAL_S || distanceMoved > MOVEMENT_THRESHOLD_KM)) {
        
        #ifdef USE_DUMMY_DATA
          fetchDummyWazeData();
        #else
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
        #endif
        
        lastApiCallTimestamp = millis();
        lastCheckedLocation = currentLocation;
    }

    printGPS();
}