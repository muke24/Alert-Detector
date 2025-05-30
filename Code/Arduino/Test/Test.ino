#include <HardwareSerial.h>
// tx: 17, rx: 16
HardwareSerial mySerial(1); // Use UART2

void setup() {
  Serial.begin(115200); // For debugging via USB
  while (!Serial) {
    ; // Wait for Serial to initialize
  }
  // Initialize UART2: RX on GPIO33 (unused), TX on GPIO32
  mySerial.begin(115200, SERIAL_8N1, 16, 17);
  Serial.println("ESP32-WROOM-32D Test Sender Started");
}

void loop() {
  //float angle = 123.45; // Fixed test value
  // Optional dynamic test: Uncomment to simulate a moving angle
   static float angle = -180.0;
   angle += 10.0;
   if (angle > 180.0) angle = -180.0;

  mySerial.write((uint8_t*)&angle, sizeof(angle)); // Send float as 4 bytes
  Serial.print("Sent: ");
  Serial.println(angle, 2); // Debug output to Serial Monitor
  delay(5000); // Send every second
}