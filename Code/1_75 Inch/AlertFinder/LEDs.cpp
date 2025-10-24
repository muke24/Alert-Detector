/**
 * @file      LEDs.cpp
 * @author    Peter Thompson
 * @brief     Implementation for the WS2812B LED strip controller.
 * @version   1.01 (Pin Definition Update)
 * @date      2025-07-30
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "LEDs.h"
#include <FastLED.h>

// --- LED Configuration ---
#define LED_PIN         18  // Using native GPIO18 for the LED data line
#define NUM_LEDS        48
#define BRIGHTNESS      150 // Brightness from 0-255
#define MAX_ALERT_KM    5.0f // The max distance used for speed calculation

static CRGB leds[NUM_LEDS];
static const CRGB ALERT_COLOR = CRGB(128, 255, 0); // Lime green

/**
 * @brief Initializes the FastLED library for the WS2812B strip.
 */
void leds_setup() {
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);
    FastLED.clear();
    FastLED.show();
}

/**
 * @brief Calculates the animation speed multiplier based on distance.
 * @param distance The distance in kilometers.
 * @return A multiplier from 1.0 (slowest) to 2.0 (fastest).
 */
static float calculate_multiplier(float distance) {
    if (distance >= MAX_ALERT_KM) return 1.0f;
    if (distance <= 0) return 2.0f;
    return 2.0f - (distance / MAX_ALERT_KM); // Linear interpolation
}

/**
 * @brief Updates the LED animation frame.
 */
void leds_update(int alert_count, float distance_km) {
    if (alert_count > 0) {
        float multiplier = calculate_multiplier(distance_km);
        float period = 1000.0 / multiplier; // Period in ms, adjusted by multiplier
        float elapsed = fmod(millis(), period);
        float fraction = elapsed / period;

        // Simple "chaser" animation
        for (int i = 0; i < NUM_LEDS; i++) {
            // Calculate the brightness based on a sine wave moving along the strip
            uint8_t brightness = (uint8_t)(128 * (1 + sin(2 * PI * (i / (float)NUM_LEDS - fraction))));
            leds[i] = ALERT_COLOR;
            leds[i].nscale8(brightness);
        }
    } else {
        FastLED.clear();
    }
    FastLED.show();
}
