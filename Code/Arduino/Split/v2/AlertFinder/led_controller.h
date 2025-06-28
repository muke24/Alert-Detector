// led_controller.h
#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

// Initializes the FastLED library for the connected LED strip.
// Call this once in setup().
void initLEDs();

// Creates and starts the dedicated FreeRTOS task for running LED animations.
// Call this once in setup() after initLEDs().
void startLedTask();

#endif // LED_CONTROLLER_H