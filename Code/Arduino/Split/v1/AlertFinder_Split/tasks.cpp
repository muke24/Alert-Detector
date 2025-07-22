// tasks.cpp
#include "tasks.h"
#include "globals.h" // Needed for access to LEDs, audio objects, and state variables

// Include the sound data
#include "alert_wav.h" 

/**
 * @brief FreeRTOS task to control the WS2812B LED strip animation.
 * Runs independently on a dedicated core.
 */
void ledTask(void *pvParameters) {
  unsigned long lastLedUpdate = 0;

  while (1) { // A task's main loop never exits
    unsigned long currentTime = millis();
    if (currentTime - lastLedUpdate >= ledUpdateInterval) {
      lastLedUpdate = currentTime;

      // Only run the animation if there is a valid, close alert
      if (alertCount > 0 && closestIndex >= 0) {
        // Period is in milliseconds, adjusted by the proximity multiplier
        float period = 1000.0 * multiplier; 
        float elapsed = fmod(currentTime, period);
        
        static float prevElapsed = 0;
        // Check if a full animation cycle has just completed
        if (elapsed < prevElapsed) {
          playSound = true; // Signal the audio task to play the sound
        }
        prevElapsed = elapsed;
        
        float fraction = elapsed / period;
        
        FastLED.clear(); // Clear all LEDs first

        // Light up each row based on the animation progress
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
        
        FastLED.show();
      } else {
        // If no alerts, ensure the LEDs are off
        FastLED.clear();
        FastLED.show();
      }
    }
    
    // Yield control to the scheduler for a short time
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}

/**
 * @brief FreeRTOS task to handle audio playback.
 * Runs independently on a dedicated core.
 */
void audioTask(void *pvParameters) {
  while (1) { // A task's main loop never exits
    // Check if the LED task has signaled to play a sound
    if (playSound) {
      // Only start a new sound if one isn't already playing
      if (wav == nullptr || !wav->isRunning()) {
        if (wav != nullptr) { // Clean up previous instances if they exist
          wav->stop();
          delete wav;
          delete file;
        }
        file = new AudioFileSourcePROGMEM(alert_wav, alert_wav_len);
        wav = new AudioGeneratorWAV();
        wav->begin(file, out);
        playSound = false; // Reset the flag after starting playback
      }
    }

    // If a sound is currently playing, we need to continuously feed the DAC
    if (wav != nullptr && wav->isRunning()) {
      if (!wav->loop()) { // loop() returns false when playback is complete
        wav->stop();
        // Clean up memory
        delete wav;
        delete file;
        wav = nullptr;
        file = nullptr;
      }
    }

    // Yield control very frequently to ensure smooth audio playback
    vTaskDelay(pdMS_TO_TICKS(1)); 
  }
}