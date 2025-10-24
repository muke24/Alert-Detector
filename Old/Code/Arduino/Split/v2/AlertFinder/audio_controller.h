// audio_controller.h
#ifndef AUDIO_CONTROLLER_H
#define AUDIO_CONTROLLER_H

// Initializes the I2S audio output for the speaker.
// Call this once in setup().
void initAudio();

// Creates and starts the dedicated FreeRTOS task for audio playback.
// Call this once in setup() after initAudio().
void startAudioTask();

#endif // AUDIO_CONTROLLER_H