#include "Arduino.h"
#include "StatusLED.h"
#include "timer.h"

#define GREEN 0x00FF00
#define RED 0xFF0000
#define BLUE 0x0000FF
#define YELLOW 0xFF8300
#define LED_OFF 0x000000

void StatusLED::updateLEDs(RPiState newState, CameraConnectionState newConnectionState, bool auxTerminalEnabled) {
  if (ledOn) { // Only update if the LED is already on. This is so once the LED is off it will stay off.
    if (newState != cameraState) {
      ledChange = getPitTimeMillis();
      cameraState = newState;
      ledOn = true;
    }
    if (newConnectionState != connectionState) {
      ledChange = getPitTimeMillis();
      connectionState = newConnectionState;
      ledOn = true;
    }
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
      writeColor(LED_OFF);
      flashSequenceCount = 0;
      ledFlashState = LOW;

      if (getPitTimeMillis() - flashStartTime > flashDuration) {
        flashing = false;
      }
    }
    return;
  }

  if (getPitTimeMillis() - ledChange > LED_ON_DURATION_MS) {
    ledOn = false;
  }

  if (!ledOn) {
    writeColor(LED_OFF);
    return;
  }

  switch (cameraState) {
    case RPiState::POWERING_ON:
      writeColor(0, 0, increasingSawTooth(220, 1, ledChange, 2000));
      break;
    case RPiState::POWERING_OFF:
      writeColor(decreasingSawTooth(220, 1, ledChange, 2000), 0, 0);
      break;
    case RPiState::POWERED_OFF:
      writeColor(RED);
      break;
    case RPiState::POWER_ON_TIMEOUT:
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
    
    case RPiState::POWERED_ON:
      // Flash light when aux terminal is enabled.
      if (auxTerminalEnabled && getPitTimeMillis() % 1000 < 200) {
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
        case CameraConnectionState::WIFI_SETUP:
          writeColor(0, increasingSawTooth(220, 1, ledChange, 2000), 0);
          break;
        case CameraConnectionState::HOTSPOT_SETUP:
          writeColor(increasingSawTooth(0xFF, 1, ledChange, 2000), increasingSawTooth(0x83, 1, ledChange, 2000), 0);
          break;
        default:
          error(ErrorCode::INVALID_CAMERA_STATE);
          break;
      }
      break;
    case RPiState::REBOOTING:
      if (getPitTimeMillis() - ledChange < 8000) {
        writeColor(decreasingSawTooth(220, 1, ledChange, 2000), 0, 0);
      } else {
        writeColor(0, 0, increasingSawTooth(220, 1, ledChange, 2000));
      }
      break;
    default:
      error(ErrorCode::INVALID_CAMERA_STATE);
      break;
  }
}

uint8_t increasingSawTooth(uint8_t max, uint8_t min, long startTime, unsigned long period) {
    if (period == 0) {
        return 0;
    }
    unsigned long i = (getPitTimeMillis() - startTime) % period;
    // Calculate i^3 / period^3 using double to avoid overflow and precision loss.
    // This is to make the led brightness look more linear to the human eye.
    double ratio = ((double)i * i * i) / ((double)period * period * period);
    return (uint8_t)(min+((max-min) * ratio));
}

uint8_t decreasingSawTooth(uint8_t max, uint8_t min, long startTime, unsigned long period) {
    if (period == 0) {
        return max;
    }
    unsigned long i = (getPitTimeMillis() - startTime) % period;
    i = period - i; // Make it decreasing instead of increasing.
    // Calculate i^3 / period^3 using double to avoid overflow and precision loss.
    // This is to make the led brightness look more linear to the human eye.
    double ratio = ((double)i * i * i) / ((double)period * period * period);
    return (uint8_t)(min + ((max - min) * ratio));
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
  ledChange = getPitTimeMillis();
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
  if (b > 0) {
    TCA0.SPLIT.LCMP0 = uint8_t(~b);
    TCA0.SPLIT.CTRLB |=  TCA_SPLIT_LCMP0EN_bm;
  } else {
    TCA0.SPLIT.CTRLB &= ~TCA_SPLIT_LCMP0EN_bm;
    digitalWrite(LED_B, HIGH);
  }
}

