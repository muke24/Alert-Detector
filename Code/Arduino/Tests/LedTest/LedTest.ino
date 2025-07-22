#include <FastLED.h>

#define LED_PIN 14    // GPIO14 on LILIGO ESP32
#define NUM_LEDS 46   // Total number of LEDs

CRGB leds[NUM_LEDS];

// Define strip indices for each row based on your custom indexing
int row1[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};              // Row 1: LEDs 1 to 12
int row2[] = {23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12};    // Row 2: LEDs 13 to 24
int row3[] = {24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34};        // Row 3: LEDs 25 to 35
int row4[] = {45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35};        // Row 4: LEDs 36 to 46

// Number of LEDs in each row
const int num_leds_row1 = 12;
const int num_leds_row2 = 12;
const int num_leds_row3 = 11;
const int num_leds_row4 = 11;

// Define the color (R:128, G:255, B:0)
CRGB color = CRGB(128, 255, 0);

// Multiplier to adjust animation duration (1.0 = 1 second)
float multiplier = 1.0; // Change this value to speed up or slow down (e.g., 2.0 for 2 seconds)

void setup() {
  // Initialize FastLED with WS2812B LEDs on GPIO14
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
}

void loop() {
  unsigned long current_time = millis();         // Get current time in milliseconds
  float period = 1000.0 * multiplier;            // Animation period in milliseconds
  float elapsed = fmod(current_time, period);    // Time elapsed in current cycle
  float fraction = elapsed / period;             // Fraction of period completed (0 to <1)

  // Clear all LEDs before setting new states
  FastLED.clear();

  // Light up LEDs in each row based on fraction
  for (int i = 0; i < num_leds_row1; i++) {
    if (i < fraction * num_leds_row1) leds[row1[i]] = color;
  }
  for (int i = 0; i < num_leds_row2; i++) {
    if (i < fraction * num_leds_row2) leds[row2[i]] = color;
  }
  for (int i = 0; i < num_leds_row3; i++) {
    if (i < fraction * num_leds_row3) leds[row3[i]] = color;
  }
  for (int i = 0; i < num_leds_row4; i++) {
    if (i < fraction * num_leds_row4) leds[row4[i]] = color;
  }

  // Update the LED strip
  FastLED.show();
}