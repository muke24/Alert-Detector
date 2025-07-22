// tasks.h
#ifndef TASKS_H
#define TASKS_H

#include <Arduino.h> // Required for FreeRTOS

// RTOS Task Declarations
// Task functions must have this specific signature.
void ledTask(void *pvParameters);
void audioTask(void *pvParameters);

#endif // TASKS_H