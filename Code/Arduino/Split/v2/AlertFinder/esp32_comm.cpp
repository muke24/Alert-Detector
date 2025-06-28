// esp32_comm.cpp
#include "esp32_comm.h"
#include "config.h"
#include "global_types.h"

// Timestamp for throttling debug prints of received data
static unsigned long lastReceivePrint = 0;

void initCommSerial() {
    // Begin UART2 on the specified pins and baud rate
    commSerial.begin(COMM_BAUD, SERIAL_8N1, COMM_RX, COMM_TX);
    Serial.print("UART2 initialized: GPIO");
    Serial.print(COMM_TX);
    Serial.print(" (TX) <-> GPIO");
    Serial.print(COMM_RX);
    Serial.println(" (RX)");

    // Clear any leftover data in the buffer on startup
    while (commSerial.available()) {
        commSerial.read();
    }
}

void sendDataToDisplay(float data) {
    // Clear any old data from the RX buffer before sending to prevent sync issues.
    while (commSerial.available()) {
        commSerial.read();
    }

    // Write the raw bytes of the float variable to the serial port.
    commSerial.write(reinterpret_cast<const uint8_t*>(&data), sizeof(data));

    // Wait for the transmission of outgoing serial data to complete.
    commSerial.flush();

    // Debug print to the main serial monitor
    Serial.print("Sent float via UART: ");
    Serial.println(data, 1);
}

void handleCommSerial() {
    // Check if enough bytes are available to constitute a float
    if (commSerial.available() >= sizeof(float)) {
        float receivedFloat;
        commSerial.readBytes(reinterpret_cast<uint8_t*>(&receivedFloat), sizeof(receivedFloat));

        // Periodically print the received data for debugging
        if (millis() - lastReceivePrint >= UART_PRINT_INTERVAL_MS) {
            Serial.print("Received float via UART: ");

            // Interpret the received value
            if (receivedFloat <= 180.0f && receivedFloat >= -180.0f) {
                Serial.print(receivedFloat, 1);
                Serial.println("°");
            } else {
                Serial.println(receivedFloat, 1); // Print it as-is if not in the angle range
            }
            lastReceivePrint = millis();
        }
    }
}