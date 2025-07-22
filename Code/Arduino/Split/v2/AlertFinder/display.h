#ifndef DISPLAY_H
#define DISPLAY_H

// Initializes the display hardware and the LVGL library.
void initDisplay();

// Starts the dedicated FreeRTOS task for handling LVGL rendering.
void startDisplayTask();

#endif // DISPLAY_H