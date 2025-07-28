/**
 * @file      AlertRenderer.h
 * @author    Peter Thompson
 * @brief     Header file for the AlertRenderer module. It declares the public
 * functions for creating and controlling the UI, and declares
 * extern variables for hardware access.
 * @version   3.00 (Refactored)
 * @date      2025-07-29
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef ALERT_RENDERER_H
#define ALERT_RENDERER_H

#include <lvgl.h>
#include "Arduino_GFX_Library.h"
#include "TouchDrvCSTXXX.hpp"

// --- Extern Hardware & Screen Variables (defined in AlertFinder.ino) ---
// These allow the .cpp file to access the hardware objects from the main sketch.
extern Arduino_GFX *gfx;
extern TouchDrvCSTXXX touch;
extern uint32_t screenWidth;
extern uint32_t screenHeight;

// --- Extern for Touch Coordinates and Orientation ---
// These are needed by the my_touchpad_read callback in the main .ino file.
extern int16_t touch_x[5], touch_y[5];
extern int orientation;


// --- Public Function Declarations ---

/**
 * @brief Initializes LVGL, its display buffers, and its drivers.
 */
void renderer_init_lvgl();

/**
 * @brief Creates all the graphical elements of the user interface.
 */
void create_arc_gui();

/**
 * @brief Sets the heat level display.
 * @param heatLevel The heat level from 0 (off) to 5 (max).
 */
void setHeat(int heatLevel);

/**
 * @brief Updates the distance text label.
 * @param meters The distance in meters. -1 to hide.
 */
void updateDistance(int meters);

/**
 * @brief Sets the target angle for the rotating arrow.
 * @param angle The target angle in degrees (0-359).
 */
void updateAngle(int angle);

/**
 * @brief Updates the alert icons shown on the display.
 * @param alertIndex0 Index for the 1st icon (-1 for none).
 * @param alertIndex1 Index for the 2nd icon (-1 for none).
 * @param alertIndex2 Index for the 3rd icon (-1 for none).
 * @param alertIndex3 Index for the 4th icon (-1 for none).
 * @param alertIndex4 Index for the 5th icon (-1 for none).
 */
void updateAlerts(int alertIndex0, int alertIndex1, int alertIndex2, int alertIndex3, int alertIndex4);

/**
 * @brief Sets the UI orientation.
 * @param ori 0 for normal (top half is main), 1 for flipped (bottom half is main).
 */
void setOrientation(int ori);


// --- Callback Declarations (for use within the module) ---
// These are called by LVGL but their logic is part of the renderer.
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data);
void my_print(const char *buf);

#endif // ALERT_RENDERER_H
