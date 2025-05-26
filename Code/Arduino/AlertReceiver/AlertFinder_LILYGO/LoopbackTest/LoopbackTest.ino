#include <Arduino.h>

HardwareSerial mySerial(1); // Use UART1 for loopback test

void setup() {
  // Initialize USB Serial for debugging (uses GPIO1 TX, GPIO3 RX)
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial to initialize
  }

  // Initialize UART1 with GPIO33 as RX and GPIO32 as TX
  mySerial.begin(115200, SERIAL_8N1, 33, 32);
  Serial.println("Loopback Test Started: Connect Pin 32 (TX) to Pin 33 (RX)");
}

void loop() {
  // Float to send
  float sentFloat = 123.45;

  // Send the float as bytes
  mySerial.write((uint8_t*)&sentFloat, sizeof(sentFloat));
  Serial.print("Sent float: ");
  Serial.println(sentFloat, 2); // Print with 2 decimal places

  // Wait for data to be available (timeout after 100ms)
  unsigned long startTime = millis();
  while (mySerial.available() < sizeof(float) && millis() - startTime < 100) {
    ; // Wait for enough bytes
  }

  // Check if data was received
  if (mySerial.available() >= sizeof(float)) {
    float receivedFloat;
    mySerial.readBytes((uint8_t*)&receivedFloat, sizeof(receivedFloat));
    Serial.print("Received float: ");
    Serial.println(receivedFloat, 2);

    // Compare sent and received values
    if (abs(sentFloat - receivedFloat) < 0.01) { // Allow small floating-point error
      Serial.println("Success: Sent and received floats match!");
    } else {
      Serial.println("Error: Sent and received floats do not match!");
    }
  } else {
    Serial.println("Error: No data received within timeout!");
  }

  // Wait before next test
  delay(1000);
}