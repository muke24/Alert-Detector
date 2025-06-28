// initialization.h
#ifndef INITIALIZATION_H
#define INITIALIZATION_H

// Function Declarations
void initCommSerial();
void initWiFi();
void initGPS();
void initBNO086();
void setReports(); // Helper for BNO086 initialization
void initLeds();
void initAudio();
void createTasks();

#endif // INITIALIZATION_H