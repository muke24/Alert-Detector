// global_types.h (Corrected)
#ifndef GLOBAL_TYPES_H
#define GLOBAL_TYPES_H

#include <Arduino.h>
#include "config.h" // <<< FIX: Include config.h FIRST to define the modem type

// Now include library headers that define types we use globally
#include <TinyGsmClient.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include <FastLED.h>
#include <TinyGPS++.h> // <<< FIX: Include TinyGPS++ header

// =========================================================================
// FORWARD DECLARATIONS (for types only used as pointers)
// =========================================================================
class AudioGeneratorWAV;
class AudioFileSourcePROGMEM;
class AudioOutputI2S;

// =========================================================================
// SHARED DATA STRUCTURES
// =========================================================================
// (No changes in this section)
struct Location {
  float latitude;
  float longitude;
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

struct DeclinationData {
  float latitude;
  float longitude;
  float declination;
  String modelVersion;
  bool isValid;
};

// =========================================================================
// GLOBAL HARDWARE & OBJECT DECLARATIONS
// =========================================================================
extern TinyGsm modem;
extern TinyGsmClient gsmClient;
extern TinyGPSPlus gps; // <<< FIX: Add extern declaration for gps object
extern BNO08x myIMU;
extern CRGB leds[NUM_LEDS];
extern HardwareSerial commSerial;

// Audio Objects
extern AudioGeneratorWAV* wav;
extern AudioFileSourcePROGMEM* file;
extern AudioOutputI2S* out;

// =========================================================================
// GLOBAL STATE VARIABLE DECLARATIONS
// =========================================================================
// (No changes in this section, but a new variable is added below)
extern Location currentLocation;
extern Location lastCheckedLocation;
extern Location lastWmmLocation;
extern float currentDirection;
extern bool isLocationInitialized;
extern bool bnoInitialized;
extern Alert* currentAlerts;
extern volatile int alertCount;
extern volatile int closestIndex;
extern bool isGprsConnected;
extern volatile bool playSound;
extern DeclinationData currentDeclination;
extern volatile float multiplier;

// <<< FIX: Declare the mutex for FreeRTOS critical sections
extern portMUX_TYPE timerMux;


#endif // GLOBAL_TYPES_H