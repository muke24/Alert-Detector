// esp32_comm.h
#ifndef ESP32_COMM_H
#define ESP32_COMM_H

// Initializes the hardware serial port (UART2) for communication.
// Call this once in setup().
void initCommSerial();

// Sends a float value to the display ESP32.
// This is the primary method for communicating the alert angle.
void sendDataToDisplay(float data);

// Checks for and processes any incoming data from the display ESP32.
// Call this in the main loop().
void handleCommSerial();

#endif // ESP32_COMM_H