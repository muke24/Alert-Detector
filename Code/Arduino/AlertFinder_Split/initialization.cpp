// initialization.cpp
#include <WiFi.h>
#include "initialization.h"
#include "globals.h"
#include "tasks.h" // Needed to create the LED and Audio tasks

/**
 * @brief Initializes the UART communication with the other ESP32.
 */
void initCommSerial() {
  commSerial.begin(115200, SERIAL_8N1, COMM_RX, COMM_TX);
  Serial.print("UART2 initialized: GPIO");
  Serial.print(COMM_TX);
  Serial.print(" (TX) <--> GPIO");
  Serial.print(COMM_RX);
  Serial.println(" (RX)");
  // Clear any garbage data in the buffer on startup
  while (commSerial.available()) commSerial.read();
}

/**
 * @brief Initializes and connects to Wi-Fi, with cellular as a fallback.
 */
void initWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to Wi-Fi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    isGprsConnected = false;
    return;
  }

  Serial.println("\nWi-Fi connection failed. Attempting cellular connection...");
  if (!modem.waitForNetwork(30000L)) {
    Serial.println("Failed to connect to cellular network");
    return;
  }
  Serial.println("Cellular network connected");

  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println("Failed to connect to GPRS");
    return;
  }
  isGprsConnected = true;
  Serial.println("GPRS connected");
  Serial.print("Local IP: ");
  Serial.println(modem.localIP());
}

/**
 * @brief Initializes the SIM7000G modem and enables its GPS module.
 */
void initGPS() {
  // Power on the modem
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  delay(300);
  digitalWrite(PWR_PIN, LOW);
  delay(1000);
  digitalWrite(PWR_PIN, HIGH);

  SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  delay(1000);
  
  Serial.println("Initializing modem...");
  if (!modem.restart()) {
    Serial.println("Failed to restart modem, attempting to continue without restarting");
    if(!modem.init()) {
      Serial.println("Failed to init modem");
      return;
    }
  }

  // Power on the GPS unit using a specific AT command for the T-SIM7000G
  modem.sendAT("+SGPIO=0,4,1,1");
  if (modem.waitResponse(10000L) != 1) {
    Serial.println("Failed to power on GPS");
  } else {
    Serial.println("GPS powered on successfully");
  }

  // Enable GPS
  modem.enableGPS();
}

/**
 * @brief Configures the BNO086 IMU to send orientation data.
 */
void setReports() {
  Serial.println("Setting BNO086 reports...");
  // Use the Geomagnetic Rotation Vector for a magnetic-north-referenced heading
  if (myIMU.enableGeomagneticRotationVector(100)) { // Report every 100ms
    Serial.println("Geomagnetic rotation vector enabled");
  } else {
    Serial.println("Failed to enable geomagnetic rotation vector");
  }
}

/**
 * @brief Initializes the BNO086 IMU sensor over I2C.
 */
void initBNO086() {
  Wire.begin(21, 22); // SDA=21, SCL=22 on LILYGO ESP32
  if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST)) {
    Serial.println("BNO086 initialized successfully!");
    bnoInitialized = true;
    setReports(); // Now set the reports we want to receive
  } else {
    Serial.println("Failed to find BNO086. Check wiring!");
    bnoInitialized = false;
  }
}

/**
 * @brief Initializes the FastLED library for the WS2812B strip.
 */
void initLeds() {
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    Serial.println("FastLED initialized.");
}

/**
 * @brief Initializes the I2S audio output for the speaker.
 */
void initAudio() {
    out = new AudioOutputI2S();
    out->SetPinout(0, 0, SPEAKER_PIN); // Use DAC on GPIO25
    out->SetGain(0.5); // Set volume (0.0 to 1.0)
    Serial.println("Audio output initialized.");
}

/**
 * @brief Creates and starts the FreeRTOS tasks for LEDs and audio.
 */
void createTasks() {
  // Create LED task on core 0
  xTaskCreatePinnedToCore(
    ledTask,     // Task function
    "LED Task",  // Task name
    10000,       // Stack size in words
    NULL,        // Parameters
    1,           // Priority
    NULL,        // Task handle
    0            // Core 0
  );

  // Create Audio task on core 1
  xTaskCreatePinnedToCore(
    audioTask,   // Task function
    "Audio Task",// Task name
    10000,       // Stack size in words
    NULL,        // Parameters
    1,           // Priority
    NULL,        // Task handle
    1            // Core 1
  );
  Serial.println("LED and Audio tasks created.");
}