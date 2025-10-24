// global_types.h (Corrected & Modified)
#ifndef GLOBAL_TYPES_H
#define GLOBAL_TYPES_H

#include "config.h" // Include config.h FIRST

// Now include library headers that define types we use globally
#include <TinyGsmClient.h>
#include <TinyGPS++.h>
#include <FastLED.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h> // <<< FIX: Changed "..." to <...> for correct library path

// =========================================================================
// FORWARD DECLARATIONS
// =========================================================================
class AudioGeneratorWAV;
class AudioFileSourcePROGMEM;
class AudioOutputI2S;

// =========================================================================
// SHARED DATA STRUCTURES
// =========================================================================
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
extern TinyGPSPlus gps;
extern Adafruit_LSM6DSOX dsox;
extern Adafruit_LIS3MDL lis3mdl;
extern CRGB leds[NUM_LEDS];
//extern HardwareSerial commSerial;

// Audio Objects
extern AudioGeneratorWAV* wav;
extern AudioFileSourcePROGMEM* file;
extern AudioOutputI2S* out;

// =========================================================================
// GLOBAL STATE VARIABLE DECLARATIONS
// =========================================================================
extern Location currentLocation;
extern Location lastCheckedLocation;
extern Location lastWmmLocation;
extern float currentDirection;
extern bool isLocationInitialized;
extern bool imuInitialized; // Renamed from bnoInitialized
extern Alert* currentAlerts;
extern volatile int alertCount;
extern volatile int closestIndex;
extern bool isGprsConnected;
extern volatile bool playSound;
extern DeclinationData currentDeclination;
extern volatile float multiplier;
extern volatile float alertAngle; // <-- ADD THIS LINE

extern portMUX_TYPE timerMux;

#endif // GLOBAL_TYPES_H