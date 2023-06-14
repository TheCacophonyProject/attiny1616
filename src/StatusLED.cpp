#include "Arduino.h"
#include "StatusLED.h"

void StatusLED::updateLEDs(CameraState newState) {
  if (newState != cameraState) {
    lastStateUpdateTime = millis();
    cameraState = newState;
  }
  if (millis() - lastStateUpdateTime > LED_ON_DURATION_MS) {
    writeColor(0, 0, 0);
    return;
  }
  switch (cameraState) {
    case CameraState::POWERING_ON:
      writeColor(255, 0, 0);
      break;
    case CameraState::POWERED_ON:
      writeColor(0, 0, 255);
      break;
    case CameraState::POWERED_OFF:
      writeColor(0, 0, 0);
      break;
    case CameraState::CONNECTED_TO_NETWORK:
      writeColor(0, 255, 0);
      break;
    case CameraState::POWER_ON_TIMEOUT:
      if (millis() - lastLEDFlashUpdateTime > LED_FLASH_DURATION_MS) {
        if (ledFlashState) {
          ledFlashState = LOW;
        } else {
          ledFlashState = HIGH;
        }
      }
      if (ledFlashState) {
        writeColor(255, 0, 0);
      } else {
        writeColor(0, 0, 0);
      }
      break;
    default:
      error(ErrorCode::INVALID_CAMERA_STATE);
      //TODO make LED error blink
      break;
  }
}

void StatusLED::writeColor(uint8_t r, uint8_t g, uint8_t b) {
  // Low is on and High is off so am flipping the bits.
  analogWrite(LED_R, uint8_t(~r));
  analogWrite(LED_G, uint8_t(~g));
  analogWrite(LED_B, uint8_t(~b));
}
