// audio_controller.cpp
#include "audio_controller.h"
#include "config.h"
#include "global_types.h"
#include <Arduino.h> // For FreeRTOS

// Include the audio libraries
#include "AudioFileSourcePROGMEM.h"
#include "AudioGeneratorWAV.h"
#include "AudioOutputI2S.h"

// Include the header file containing the alert sound WAV data
#include "alert_wav.h"

// The FreeRTOS task that handles audio playback.
// It runs independently on Core 1 to ensure smooth, non-blocking sound.
static void audioTask(void *pvParameters) {
    while (1) { // Loop forever
        // Check if the LED task has signaled to play a sound
        if (playSound) {
            // Ensure we are not already playing something
            if (wav == nullptr || !wav->isRunning()) {
                // Clean up any previous instances, just in case
                if (wav != nullptr) {
                    wav->stop();
                    delete wav;
                    delete file;
                    wav = nullptr;
                    file = nullptr;
                }

                // Create a new audio source from the byte array in PROGMEM
                file = new AudioFileSourcePROGMEM(alert_wav, alert_wav_len);
                
                // Create a new WAV generator
                wav = new AudioGeneratorWAV();
                
                // Start the playback
                wav->begin(file, out);
                
                // Reset the flag immediately after starting
                playSound = false;
            }
        }

        // If a sound is currently playing, we need to keep feeding the buffer.
        if (wav != nullptr && wav->isRunning()) {
            if (!wav->loop()) {
                // The loop() function returns false when playback is complete
                wav->stop();
                delete wav;
                delete file;
                wav = nullptr;
                file = nullptr;
            }
        }

        // This task can yield for a very short time.
        // A small delay is crucial to allow the audio buffers to be managed
        // without starving other processes. 1-2ms is typical.
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}


void initAudio() {
    // Initialize the I2S audio output object
    out = new AudioOutputI2S();
    
    // Use the internal DAC on the specified speaker pin.
    // For ESP32 DAC, BCLK and LRC are not used, so set to 0.
    out->SetPinout(0, 0, SPEAKER_PIN);
    
    // Set the volume/gain. 0.0 to 1.0 is a reasonable range.
    out->SetGain(0.5);
    
    Serial.println("Audio output initialized.");
}

void startAudioTask() {
    // Create the audio playback task and pin it to Core 1
    xTaskCreatePinnedToCore(
        audioTask,       // Task function
        "Audio Task",    // Name of the task
        10000,           // Stack size in words
        NULL,            // Task input parameter
        2,               // Priority of the task (can be higher than LED)
        NULL,            // Task handle
        1);              // Core where the task should run

    Serial.println("Audio playback task started on Core 1.");
}