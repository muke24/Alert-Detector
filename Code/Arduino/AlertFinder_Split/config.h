// config.h
#ifndef CONFIG_H
#define CONFIG_H

// =================================================================
// HARDWARE PIN & CONFIGURATION DEFINITIONS
// =================================================================
// These are pre-processor defines and are safe to keep here.
#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024
#define SerialAT Serial1
#define UART_BAUD 115200
#define PIN_TX 27
#define PIN_RX 26
#define PWR_PIN 4
#define COMM_TX 32
#define COMM_RX 33
#define LED_PIN 14
#define NUM_LEDS 46
#define SPEAKER_PIN 25
#define BNO08X_INT 19
#define BNO08X_RST 23
#define BNO08X_ADDR 0x4B

// =================================================================
// GLOBAL VARIABLE DECLARATIONS
// =================================================================
// Using 'extern' DECLARES the variable without DEFINING it.
// The actual definition will be in the main .ino file.

// --- WIFI & CELLULAR CREDENTIALS ---
extern const char* ssid;
extern const char* password;
extern const char apn[];
extern const char gprsUser[];
extern const char gprsPass[];

// --- API CONFIGURATION ---
extern const char* baseHost;
extern const char* basePath;
extern const char* wmmHost;
extern const char* wmmPath;

// --- OPERATIONAL CONSTANTS ---
extern const float maxDistanceKm;
extern const float checkInterval;
extern const float movementThreshold;
extern const float wmmUpdateDistance;
extern const unsigned long wmmUpdateInterval;
extern const unsigned long wmmRetryDelay;
extern const unsigned long printInterval;
extern const unsigned long retryInterval;
extern const unsigned long receivePrintInterval;
extern const unsigned long ledUpdateInterval;
extern const int maxImuFailures;

#endif // CONFIG_H