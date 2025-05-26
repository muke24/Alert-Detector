#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// Create an instance of the HMC5883L sensor
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

void setup() {
  // Start serial communication at 115200 baud
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Wait for serial port to connect
  }
  
  // Initialize I2C communication with SDA on Pin 21 and SCL on Pin 22
  Wire.begin(21, 22);
  
  // Initialize the HMC5883L sensor
  if (!mag.begin()) {
    Serial.println("Error: HMC5883L not detected. Check your wiring!");
    while (1); // Halt if sensor is not found
  }
  
  Serial.println("HMC5883L detected successfully!");
}

void loop() {
  // Get a new sensor event
  sensors_event_t event;
  mag.getEvent(&event);
  
  // Calculate heading using X and Y magnetic field values
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Convert radians to degrees
  float headingDegrees = heading * 180 / M_PI;
  
  // Adjust to 0-360 degree range
  if (headingDegrees < 0) {
    headingDegrees += 360;
  }
  
  // Print the heading to the serial monitor
  Serial.print("Heading: ");
  Serial.print(headingDegrees);
  Serial.println(" degrees");
  
  // Delay for readability
  delay(500);
}