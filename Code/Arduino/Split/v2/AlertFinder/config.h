// config.h (Corrected)
#ifndef CONFIG_H
#define CONFIG_H

// =========================================================================
// SIM7000G MODEM CONFIGURATION (No changes here)
// =========================================================================
#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 1024
#define SerialAT Serial1
#define UART_BAUD          115200
#define PIN_TX             27
#define PIN_RX             26
#define PWR_PIN            4

// =========================================================================
// OTHER HARDWARE & PERIPHERALS (No changes here)
// =========================================================================
#define COMM_TX            32
#define COMM_RX            33
#define COMM_BAUD          115200
#define BNO08X_SDA         21
#define BNO08X_SCL         22
#define BNO08X_INT         19
#define BNO08X_RST         23
#define BNO08X_ADDR        0x4B
#define LED_PIN            14
#define NUM_LEDS           46
#define SPEAKER_PIN        25

// =========================================================================
// NETWORKING & API CONFIGURATION
// <<< FIX: Change definitions to extern declarations >>>
// =========================================================================
extern const char* WIFI_SSID;
extern const char* WIFI_PASSWORD;
extern const char* APN;
extern const char* GPRS_USER;
extern const char* GPRS_PASS;
extern const char* WAZE_HOST;
extern const char* WAZE_PATH;
extern const char* WMM_HOST;
extern const char* WMM_PATH;
extern const char* WMM_API_KEY;

// =========================================================================
// APPLICATION & LOGIC CONSTANTS (No changes here)
// =========================================================================
const float MAX_ALERT_DISTANCE_KM = 5.0f;
const float ALERT_CHECK_INTERVAL_S = 45.0f;
const float MOVEMENT_THRESHOLD_KM = 0.2f;
const float WMM_UPDATE_DISTANCE_KM = 100.0f;
const unsigned long WMM_UPDATE_INTERVAL_MS = 24 * 60 * 60 * 1000UL;
const unsigned long WMM_RETRY_DELAY_MS = 5 * 60 * 1000UL;
const unsigned long GPS_IMU_PRINT_INTERVAL_MS = 500;
const unsigned long BNO_RETRY_INTERVAL_MS = 5000;
const unsigned long UART_PRINT_INTERVAL_MS = 1000;
const unsigned long LED_UPDATE_INTERVAL_MS = 50;
const int MAX_IMU_FAILURES = 5;

#include <FastLED.h>
const CRGB LED_COLOR = CRGB(128, 255, 0);

#endif // CONFIG_H