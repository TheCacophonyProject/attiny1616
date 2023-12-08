#include "Arduino.h"
#include "StatusLED.h"
#include "timer.h"

#define GREEN 0x00FF00
#define RED 0xFF0000
#define BLUE 0x0000FF
#define YELLOW 0xFF8300
#define LED_OFF 0x000000

void StatusLED::updateLEDs(CameraState newState, CameraConnectionState newConnectionState, bool auxTerminalEnabled) {
  if (newState != cameraState) {
    ledOnTime = getPitTimeMillis();
    cameraState = newState;
    ledOn = true;
  }
  if (newConnectionState != connectionState) {
    ledOnTime = getPitTimeMillis();
    connectionState = newConnectionState;
    ledOn = true;
  }

  if (flashing) {
    if (getPitTimeMillis() - lastLEDFlashUpdateTime > flashDelay) {
      lastLEDFlashUpdateTime = getPitTimeMillis();
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

      if (getPitTimeMillis() - flashStartTime > flashDuration) {
        flashing = false;
      }
    }
    return;
  }

  if (getPitTimeMillis() - ledOnTime > LED_ON_DURATION_MS) {
    ledOn = false;
  }

  if (!ledOn) {
    writeColor(LED_OFF);
    return;
  }

  switch (cameraState) {
    case CameraState::POWERING_ON:
      writeColor(0, sawTooth(220, 1, ledOnTime, 2000), 0);
      break;
    case CameraState::POWERING_OFF:
      writeColor(255-sawTooth(254, 35, ledOnTime, 2000), 0, 0);
      break;
    case CameraState::POWERED_OFF:
      writeColor(RED);
      break;
    case CameraState::POWER_ON_TIMEOUT:
      if (getPitTimeMillis() - lastLEDFlashUpdateTime > LED_FLASH_DURATION_MS) {
        ledFlashState = !ledFlashState;
        lastLEDFlashUpdateTime = getPitTimeMillis();
      }
      if (ledFlashState) {
        writeColor(RED);
      } else {
        writeColor(LED_OFF);
      }
      break;
    
    case CameraState::POWERED_ON:
      // Flash light when aux terminal is enabled.
      if (auxTerminalEnabled && getPitTimeMillis() % 1000 < 500) {
        writeColor(LED_OFF);
        break;
      }
      switch (connectionState) {
        case CameraConnectionState::NO_CONNECTION:
          writeColor(BLUE);
          break;
        case CameraConnectionState::CONNECTED_TO_WIFI:
          writeColor(GREEN);
          break;
        case CameraConnectionState::HOSTING_HOTSPOT:
          writeColor(YELLOW);
          break;
        default:
          error(ErrorCode::INVALID_CAMERA_STATE);
          break;
      }
      break;
    case CameraState::REBOOTING:
      if (getPitTimeMillis() - ledOnTime < 8000) {
        writeColor(255-sawTooth(254, 35, ledOnTime, 2000), 0, 0);
      } else {
        writeColor(0, sawTooth(220, 1, ledOnTime, 2000), 0);
      }
      break;
    default:
      error(ErrorCode::INVALID_CAMERA_STATE);
      break;
  }
}

uint8_t sawTooth(uint8_t max, uint8_t min, long startTime, unsigned long period) {
    if (period == 0) {
        return 0;
    }
    unsigned long i = (getPitTimeMillis() - startTime) % period;
    // Calculate i^3 / period^3 using double to avoid overflow and precision loss.
    // This is to make the led brightness look more linear to the human eye.
    double ratio = ((double)i * i * i) / ((double)period * period * period);
    return (uint8_t)(min+((max-min) * ratio));
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
  flashStartTime = getPitTimeMillis();
}

void StatusLED::error(ErrorCode error) {
  flash(static_cast<int>(error), ERROR_LED_DURATION_MS, 255, 0, 0, 0, 0, 0);
}

bool StatusLED::isOn() {
  return ledOn;
}

void StatusLED::show() {
  ledOnTime = getPitTimeMillis();
  ledOn = true;
}

void StatusLED::off() {
  ledOn = false;
}

void StatusLED::writeColor(uint32_t color) {
  writeColor((color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF);
}

void StatusLED::writeColor(uint8_t r, uint8_t g, uint8_t b) {
  // Low is on and High is off so am flipping the bits.
  analogWrite(LED_R, uint8_t(~r));
  analogWrite(LED_G, uint8_t(~g));
  analogWrite(LED_B, uint8_t(~b));
}

