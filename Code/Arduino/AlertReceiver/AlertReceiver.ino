#include <HardwareSerial.h>

HardwareSerial SerialUART(0); // UART2 on GPIO17 (TX), GPIO16 (RX)

// Prompt: How can I test that the data is being sent correctly over GPIO ports?

void setup() {
  SerialUART.begin(115200, SERIAL_8N1, 1, 3); // Baud rate 115200, RX=16, TX=17
  Serial.begin(115200);
}

void loop() {
  static float angle = 0;
  SerialUART.print("angle:");
  SerialUART.println(angle); // Send data in "angle:xx.x" format
  angle += 10;              // Increment angle by 10 degrees
  if (angle >= 360) angle = 0; // Reset to 0 after 360 degrees
  String data = "angle: " + String(angle);
  Serial.println(data);
  delay(1000);              // Update every second
}