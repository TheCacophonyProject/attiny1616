#include "Arduino.h"
#include "StatusLED.h"

void StatusLED::updateLEDs(CameraState newState) {
  if (newState != cameraState) {
    ledOnTime = millis();
    cameraState = newState;
  }

  if (flashing) {  
    if (millis() - lastLEDFlashUpdateTime > flashDelay) {
      lastLEDFlashUpdateTime = millis();
      flashDelay = LED_FLASH_ON_DURATION_MS;
      ledFlashState = !ledFlashState;
      if (ledFlashState) { 
        flashSequenceCount += 1;
        writeColor(color1R, color1B, color1G);
      } else {
        writeColor(color2R, color2B, color2G);
      }
    }
    if (flashSequenceCount > flashLength) {
      flashDelay = LED_FLASH_OFF_DURATION_MS*4; // Wait four times longer between flash sequences.
      writeColor(0, 0, 0);
      flashSequenceCount = 0;
      ledFlashState = LOW;

      if (millis() - flashStartTime > flashDuration) {
        flashing = false;
      }
    }
    return;
  }
  
  if (millis() - ledOnTime > LED_ON_DURATION_MS) {
    writeColor(0, 0, 0);
    return;
  }
  switch (cameraState) {
    case CameraState::POWERING_ON:
      writeColor(255, 0, 0);  // Red
      break;
    case CameraState::POWERED_ON:
      writeColor(0, 0, 255); // Blue
      break;
    case CameraState::POWERING_OFF:
      writeColor(255, 255, 0);  // Yellow
      break;
    case CameraState::POWERED_OFF:
      writeColor(0, 0, 0);  // OFF
      break;
    case CameraState::CONNECTED_TO_NETWORK:
      writeColor(0, 255, 0);  // Green
      break;
    case CameraState::POWER_ON_TIMEOUT:
      if (millis() - lastLEDFlashUpdateTime > LED_FLASH_DURATION_MS) {
        ledFlashState = !ledFlashState;
        lastLEDFlashUpdateTime = millis();
      }
      if (ledFlashState) {
        writeColor(255, 0, 0);
      } else {
        writeColor(0, 0, 0);
      }
      break;
    default:
      error(ErrorCode::INVALID_CAMERA_STATE);
      break;
  }
}

void StatusLED::flash(uint8_t flashLen, unsigned long duration, uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2) {
  if (flashing) {
    return; // Don't update flash if already flashing.
  }
  
  flashLength = flashLen;
  flashDuration = duration;
  
  color1R = r1;
  color1G = g1;
  color1B = b1;
  color2R = r2;
  color2G = g2;
  color2B = b2;

  flashing = true;
  flashSequenceCount = 0;
  flashStartTime = millis();
}

void StatusLED::error(ErrorCode error) {
  flash(static_cast<int>(error), ERROR_LED_DURATION_MS, 255, 0, 0, 0, 0, 0);
}

void StatusLED::show() {
  ledOnTime = millis();
}

void StatusLED::writeColor(uint8_t r, uint8_t g, uint8_t b) {
  // Low is on and High is off so am flipping the bits.
  analogWrite(LED_R, uint8_t(~r));
  analogWrite(LED_G, uint8_t(~g));
  analogWrite(LED_B, uint8_t(~b));
}
