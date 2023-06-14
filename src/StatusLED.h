#ifndef STATUSLED_H
#define STATUSLED_H

#include "Pinout.h"
#include "CameraState.h"
#include "Arduino.h"
#include "ErrorCodes.h"


#define LED_ON_DURATION_MS   600000 // Duration of time that the status LED will be on for after the camera status has changed. 10 minutes 10 * 60 * 1000 = 600000
#define LED_FLASH_DURATION_MS 500   // Duration between LED turning ON and OFF when flashing. 500ms = 0.5s 


class StatusLED {
    public:
        using ErrorFunctionType = void (*)(ErrorCode);
        ErrorFunctionType error;
        void updateLEDs(CameraState cameraState);
    private:
        void writeColor(uint8_t r, uint8_t g, uint8_t b);
        unsigned long lastStateUpdateTime = 0;
        unsigned long lastLEDFlashUpdateTime = 0;
        bool ledFlashState = LOW;
        CameraState cameraState;
};

#endif