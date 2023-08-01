#ifndef STATUSLED_H
#define STATUSLED_H

#include "Pinout.h"
#include "CameraState.h"
#include "Arduino.h"
#include "ErrorCodes.h"


#define LED_ON_DURATION_MS   600000 // Duration of time that the status LED will be on for after the camera status has changed. 10 minutes 10 * 60 * 1000 = 600000
#define LED_FLASH_DURATION_MS 500   // Duration between LED turning ON and OFF when flashing. 500ms = 0.5s 
#define ERROR_LED_DURATION_MS 20000 // Duration of time that the error status will be shown for. 20 seconds
#define LED_FLASH_ON_DURATION_MS 500 // Duration of time that the LED will be on for when flashing.
#define LED_FLASH_OFF_DURATION_MS 500 // Duration of time that the LED will be off for when flashing.
#define LED_BETWEEN_FLASHES_MS 2000 // Times between LED flash sequences.

class StatusLED {
    public:
        void error(ErrorCode);
        void updateLEDs(CameraState, CameraConnectionState);
        void show();
        void writeColor(uint32_t color);
        void writeColor(uint8_t r, uint8_t g, uint8_t b);
        void showError(ErrorCode);
        void flash(uint8_t, unsigned long, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
    private:
        CameraState cameraState;
        CameraConnectionState connectionState;
        unsigned long ledOnTime = 0;
        unsigned long lastLEDFlashUpdateTime = 0;
        bool ledFlashState = LOW;
        uint8_t ledFlashCount = 0;
        
        unsigned long flashDelay;
        // Flashing is used to flash between two different colors for a number of times
        // to show a what state the attiny is in.
        uint8_t color1R = 0;
        uint8_t color1G = 0;
        uint8_t color1B = 0;
        uint8_t color2R = 0;
        uint8_t color2G = 0;
        uint8_t color2B = 0;
        bool flashing = false;
        uint8_t flashSequenceCount = 0; // Number of times the error code should be flashed.
        uint8_t flashLength;          // Number of flashes 
        unsigned long flashStartTime;
        unsigned long flashDuration;
};

#endif