#include <Wire.h>                // I2C library for ESP32
#include <Adafruit_Sensor.h>     // Base sensor library
#include <Adafruit_HMC5883_U.h>  // HMC5883L library

#define DRDY_PIN 2  // DRDY pin connected to GPIO 2 (D2) on ESP32

// Create an instance of the HMC5883L sensor with a unique ID
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Volatile flag to indicate when data is ready
volatile bool dataReady = false;

// Interrupt service routine (ISR) to set the dataReady flag
void IRAM_ATTR handleDRDY() {
  dataReady = true;
}

void setup() {
  Serial.begin(115200);  // Start serial communication at 115200 baud
  delay(500);            // Short delay to allow serial to initialize

  // Initialize the HMC5883L sensor
  if (!mag.begin()) {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    while (1);  // Halt if sensor isn’t detected
  }

  // Set DRDY pin as input and attach interrupt on rising edge
  pinMode(DRDY_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(DRDY_PIN), handleDRDY, RISING);
}

void loop() {
  if (dataReady) {
    // Get a new sensor event
    sensors_event_t event;
    mag.getEvent(&event);

    // Calculate heading in radians
    float heading = atan2(event.magnetic.y, event.magnetic.x);

    // Convert to degrees and correct for negative values
    if (heading < 0) {
      heading += 2 * PI;
    }
    float headingDegrees = heading * 180 / PI;

    // Print the heading to the serial monitor
    Serial.print("Heading (degrees): ");
    Serial.println(headingDegrees);

    // Reset the flag
    dataReady = false;
  }
  // No delay needed; the loop will wait for the next interrupt
}