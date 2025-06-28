// led_controller.cpp (Corrected)
#include "led_controller.h"
#include "config.h"
#include "global_types.h"
#include <Arduino.h> // For FreeRTOS
#include <FastLED.h>

// --- LED Strip Physical Layout ---
static const int row1[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
static const int row2[] = {22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12};
static const int row3[] = {23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33};
static const int row4[] = {45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34};

static const int num_leds_row1 = sizeof(row1) / sizeof(row1[0]);
static const int num_leds_row2 = sizeof(row2) / sizeof(row2[0]);
static const int num_leds_row3 = sizeof(row3) / sizeof(row3[0]);
static const int num_leds_row4 = sizeof(row4) / sizeof(row4[0]);


static void ledTask(void *pvParameters) {
    unsigned long lastLedUpdate = 0;

    while (1) {
        unsigned long currentTime = millis();
        if (currentTime - lastLedUpdate >= LED_UPDATE_INTERVAL_MS) {
            lastLedUpdate = currentTime;

            bool hasAlert;
            float currentMultiplier;
            
            // Read volatile variables safely inside a critical section
            taskENTER_CRITICAL(&timerMux); // <<< FIX: Pass mutex argument
            hasAlert = (alertCount > 0 && closestIndex >= 0);
            currentMultiplier = multiplier;
            taskEXIT_CRITICAL(&timerMux);  // <<< FIX: Pass mutex argument

            if (hasAlert) {
                float period = 2000.0f / currentMultiplier;
                float elapsed = fmod(currentTime, period);

                static float prevElapsed = 0;
                if (elapsed < prevElapsed) {
                    playSound = true;
                }
                prevElapsed = elapsed;

                float fraction = elapsed / period;

                FastLED.clear();
                for (int i = 0; i < num_leds_row1; i++) {
                    if (i < fraction * num_leds_row1) leds[row1[i]] = LED_COLOR;
                }
                for (int i = 0; i < num_leds_row2; i++) {
                    if (i < fraction * num_leds_row2) leds[row2[i]] = LED_COLOR;
                }
                for (int i = 0; i < num_leds_row3; i++) {
                    if (i < fraction * num_leds_row3) leds[row3[i]] = LED_COLOR;
                }
                for (int i = 0; i < num_leds_row4; i++) {
                    if (i < fraction * num_leds_row4) leds[row4[i]] = LED_COLOR;
                }
                FastLED.show();

            } else {
                FastLED.clear();
                FastLED.show();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void initLEDs() {
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(100);
    FastLED.clear();
    FastLED.show();
    Serial.println("LED strip initialized.");
}

void startLedTask() {
    xTaskCreatePinnedToCore(
        ledTask, "LED Task", 10000, NULL, 1, NULL, 0);
    Serial.println("LED animation task started on Core 0.");
}