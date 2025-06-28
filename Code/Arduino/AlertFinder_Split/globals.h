// globals.h
#ifndef GLOBALS_H
#define GLOBALS_H

#include "config.h" // Include our configuration constants

// =================================================================
// LIBRARY INCLUDES FOR OBJECT TYPES
// =================================================================
// We include these here because this file declares objects of these types.

// CORRECTED SECTION:
#include <HardwareSerial.h>
#include <TinyGsmClient.h>
#include <TinyGPS++.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include <FastLED.h>
#include "AudioOutputI2S.h"
#include "AudioGeneratorWAV.h"
#include "AudioFileSourcePROGMEM.h"


// =================================================================
// DATA STRUCTURES
// =================================================================

struct Location {
  float latitude;
  float longitude;
};

struct DeclinationData {
  float latitude;
  float longitude;
  float declination;
  String modelVersion;
  bool isValid;
};

struct Alert {
  String type;
  String subtype;
  Location location;
  String street;
};

struct BoundingArea {
  float left;
  float bottom;
  float right;
  float top;
};


// =================================================================
// EXTERNAL VARIABLE AND OBJECT DECLARATIONS
// =================================================================
// Using 'extern' tells the compiler these are defined in another file (our main .ino)

// --- Hardware Objects ---
extern HardwareSerial commSerial;
extern TinyGsm modem;
extern TinyGsmClient gsmClient;
extern TinyGPSPlus gps;
extern BNO08x myIMU;
extern CRGB leds[NUM_LEDS];

// --- Audio Objects ---
extern AudioGeneratorWAV *wav;
extern AudioFileSourcePROGMEM *file;
extern AudioOutputI2S *out;

// --- LED Row Definitions ---
extern int row1[];
extern int row2[];
extern int row3[];
extern int row4[];
extern const int num_leds_row1;
extern const int num_leds_row2;
extern const int num_leds_row3;
extern const int num_leds_row4;

// --- LED Color and Multiplier ---
extern CRGB color;
extern volatile float multiplier; // Volatile for safe access from different tasks

// --- State Variables ---
extern bool bnoInitialized;
extern int imuFailureCount;
extern unsigned long lastGpsPrint;
extern unsigned long lastImuPrint;
extern unsigned long lastBnoRetry;
extern unsigned long lastApiCall;
extern unsigned long lastReceivePrint;
extern unsigned long lastWmmUpdate;
extern unsigned long lastWmmAttempt;
extern float timeSinceLastCheck;
extern bool isLocationInitialized;
extern float currentDirection;
extern bool isGprsConnected;
extern volatile int alertCount;     // Volatile for task safety
extern volatile int closestIndex;   // Volatile for task safety
extern volatile bool playSound;      // Flag to trigger sound in audio task

// --- Data Structure Instances ---
extern Location currentLocation;
extern Location lastCheckedLocation;
extern Location lastWmmLocation;
extern DeclinationData currentDeclination;
extern Alert* currentAlerts;

#endif // GLOBALS_H