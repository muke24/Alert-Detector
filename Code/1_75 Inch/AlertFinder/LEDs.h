/**
 * @file      LEDs.h
 * @author    Peter Thompson
 * @brief     Header file for the WS2812B LED strip controller.
 * @version   1.00
 * @date      2025-07-30
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef LEDS_H
#define LEDS_H

/**
 * @brief Initializes the LED strip on its dedicated GPIO pin.
 */
void leds_setup();

/**
 * @brief Updates the LED animation based on the current alert status.
 * @param alert_count The number of active alerts.
 * @param distance_km The distance in kilometers to the closest alert.
 */
void leds_update(int alert_count, float distance_km);

#endif // LEDS_H
