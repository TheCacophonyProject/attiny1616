#include <Arduino.h>
#include <avr/sleep.h>
#include <Wire.h>
#include "main.h"

#define VERSION 1

//=====DEFINITIONS=====//
#define POWER_ON_DURATION_MS 180000 // Duration of time to give the camera to power on and send a signal to the camera. 3 minutes 3 * 60 * 1000 = 180000


//=====I2C DEFINITIONS=====//
#define I2C_ADDRESS 0x25
#define REG_LEN     0x24

// Check registers.md for details on registers functions.
#define REG_VERSION             0x00
#define REG_TYPE                0x01
#define REG_CAMERA_STATE        0x02
#define REG_RESET_WATCHDOG      0x03
#define REG_TRIGGER_SLEEP       0x04
#define REG_TRIGGER_SLEEP_DELAY 0x05
#define REG_CAMERA_WAKEUP       0x06
#define REG_SLEEP_DURATION_MSB  0x07
#define REG_SLEEP_DURATION_MID  0x08
#define REG_SLEEP_DURATION_LSB  0x09
#define REG_BATTERY1            0x0A
#define REG_BATTERY2            0x0B
#define REG_BATTERY3            0x0C
#define REG_BATTERY4            0x0D
#define REG_BATTERY5            0x0E
#define REG_RTC_BATTERY1        0x0F
#define REG_RTC_BATTERY2        0x10
#define REG_ERRORS1             0x20
#define REG_ERRORS2             0x21
#define REG_ERRORS3             0x22
#define REG_ERRORS4             0x23


uint8_t registers[REG_LEN] = {0};
uint8_t writeMasks[REG_LEN] = {}; // Should all be initialised to 0xFF
uint8_t registerAddress = 0;

//=====GLOBAL VARIABLES=====//
volatile unsigned long poweredOnTime = 0;       // Time from millis() from when the camera was powered on (5V enabled).
//volatile unsigned long lastStateUpdateTime = 0; // Time from millis() when the camera state was last changed.
volatile unsigned long sleepTimeStart = 0;      // Time when the camera was powered off.
volatile unsigned long sleepCountdown = 0;      // Remaining time to sleep (in milliseconds).
volatile uint16_t mainBatteryVoltage = 0;       // Raw reading value from ADC
volatile uint16_t rtcBatteryVoltage = 0;        // Raw reading value from ADC
volatile bool lowBatteryCheck = false;
volatile uint16_t lowBatteryValue = 0;
//volatile unsigned long lastLEDFlashUpdateTime = 0;  // Time from millis() when the LED was last updated when in a flash sequence.
//volatile bool ledFlashState = LOW;                  // In an LED flash sequence is the LED ON or OFF.

// Different states that the camera is in.
volatile CameraState cameraState = CameraState::POWERING_ON;
StatusLED statusLED;


void setup() {
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(EN_5V, OUTPUT);
  pinMode(RTC_BAT_SENSE, INPUT);
  pinMode(MAIN_BAT_SENSE, INPUT);
  pinMode(RTC_INT, INPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  digitalWrite(LED_R, LOW);
  digitalWrite(LED_G, LOW);
  digitalWrite(LED_B, LOW);
  
  //digitalWrite(EN_5V, LOW); //TODO

  //TODO read in values from memory
  //lowBatteryCheck
  //lowBatteryValue
  
  //Check for low battery
  //
  

  checkMainBattery();
  for (int i = 0; i < REG_LEN; i++) {
    writeMasks[i] = 0xFF;
  }
  writeMasks[REG_VERSION] = 0x00; 
  writeMasks[REG_BATTERY3] = 0x03; // Only allow writing to bits to turn on or off battery check.
  writeMasks[REG_BATTERY1] = 0x01 << 7;
  writeMasks[REG_BATTERY2] = 0x00;
  writeMasks[REG_RTC_BATTERY1] = 0x01 << 7;
  writeMasks[REG_RTC_BATTERY2] = 0x00;
  
  registers[REG_TYPE] = 0xCA;
  registers[REG_VERSION] = VERSION;

  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  //set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  //sleep_enable();
}

volatile bool registersWrittenTo = false; // Flag to indicate that registers have been written to.

void loop1() {

}

void loop() {
  if (registersWrittenTo) { // Function that only get called when there has been an update to the registers.
    registersWrittenTo = false;
    lowBatteryRegUpdate();
    mainBatteryRegUpdate();
    rtcBatteryRegUpdate();
    checkRegSleep();
  }
  checkMainBattery();
  checkCameraState();
  statusLED.updateLEDs(cameraState);

  //sleep_cpu();  // put the MCU to sleep. It will wake up when an I2C request is received.
}

void mainBatteryRegUpdate() {
  if (registers[REG_BATTERY1] & 0x01 << 0x07) {
    checkMainBattery();
    registers[REG_BATTERY2] = mainBatteryVoltage & 0xFF;
    registers[REG_BATTERY1] = mainBatteryVoltage >> 8 & ~(0x01 << 0x07); // Writing the MSB of the battery voltage and clear bit 7.
  }
}

void rtcBatteryRegUpdate() {
  if (registers[REG_RTC_BATTERY1] & 0x01 << 0x07) {
    checkRTCBattery();
    registers[REG_RTC_BATTERY2] = rtcBatteryVoltage & 0xFF;
    registers[REG_RTC_BATTERY1] = rtcBatteryVoltage >> 8 & ~(0x01 << 0x07); // Writing the MSB of the battery voltage and clear bit 7.
  }
}

void checkRegSleep() {
  if (registers[REG_TRIGGER_SLEEP] != 0) {
    long delayDurationMs = uint32_t(registers[REG_TRIGGER_SLEEP_DELAY]) * 1000;
    delay(delayDurationMs);
    digitalWrite(EN_5V, LOW);
    uint32_t sleepDurationSec = uint32_t(registers[REG_SLEEP_DURATION_MSB] << 16 | registers[REG_SLEEP_DURATION_MID] << 8 | registers[REG_SLEEP_DURATION_LSB]);
    sleepTimeStart = millis();
    sleepCountdown = sleepDurationSec * 1000;
  }
}

void checkWakeUp() {
  if (registers[REG_CAMERA_WAKEUP] != 0) {
    registers[REG_CAMERA_WAKEUP] = 0;
    digitalWrite(EN_5V, HIGH);
    writeCameraState(CameraState::POWERING_ON);
  }
}

//REG_BATTERY3
// 7-3, reserved
// bit 2: read only. 0 = no low battery, 1 = low battery
// bit 1: write high to disable low battery.
// bit 0: write high to enable low battery.
void lowBatteryRegUpdate() {
  // Check if low battery check is wanting to be enabled.
  if (registers[REG_BATTERY3] & 0x01) {
    uint16_t newLowBatteryVal = (registers[REG_BATTERY1] << 8) | registers[REG_BATTERY2];
    checkMainBattery();
    // Check that the new value is not less than the current battery value with a bit of a buffer.
    // This is to prevent the case when it might be set too high and the device will never turn on fully as it always has a "low" battery.
    // 31 should be about 0.1V
    if (newLowBatteryVal < mainBatteryVoltage-31) {
      registers[REG_BATTERY3] = 0x00;
      // TODO Make warning LED signal.
      return;
    }
    registers[REG_BATTERY3] = 0x01 << 2; // Set bit 2 to 1, signaling that it is enabled.
    return;
  }
  // Check if low battery check is wanting to be disabled.
  if (registers[REG_BATTERY3] & 0x01 << 1) {
    registers[REG_BATTERY3] = 0x00;
  }
}

void checkMainBattery() {
  int samples = 10;
  int batteryVoltage = 0;
  for (int i = 0; i < samples; i++) {
    batteryVoltage += analogRead(MAIN_BAT_SENSE);
  }
  mainBatteryVoltage = batteryVoltage / samples;
  //TODO Check against low battery value if set to do so.
}

void checkRTCBattery() {
  int samples = 10;
  int batteryVoltage = 0;
  for (int i = 0; i < samples; i++) {
    batteryVoltage += analogRead(RTC_BAT_SENSE);
  }
  rtcBatteryVoltage = batteryVoltage / samples;
}

void checkCameraState() {
  // Check that it is a valid state.
  if (registers[REG_CAMERA_STATE] <= static_cast<uint8_t>(CameraState::POWER_ON_TIMEOUT)) {
    writeCameraState(static_cast<CameraState>(registers[REG_CAMERA_STATE]));
  }
  return;
  // Check if the camera has finished sleeping.
  if (cameraState == CameraState::POWERED_OFF && millis() - sleepTimeStart > sleepCountdown) {
    writeCameraState(CameraState::POWERING_ON);
    digitalWrite(EN_5V, HIGH);
  }

  // Check if the camera has timedout to power on.
  if (cameraState == CameraState::POWERING_ON && millis() - poweredOnTime > POWER_ON_DURATION_MS) {
    writeCameraState(CameraState::POWER_ON_TIMEOUT);
  }
}

void writeCameraState(CameraState newCameraState) {
  if (newCameraState != cameraState) {
    cameraState = newCameraState;
    
    //TODO lastStateUpdateTime = millis();
  }
  registers[REG_CAMERA_STATE] = static_cast<uint8_t>(cameraState);
}

void receiveEvent(int howMany) {
  while(Wire.available()) {
    uint8_t address = Wire.read();
    // Prevent from writing/reading to registers outside of the range.
    if (address >= REG_LEN) {
      while(Wire.available()) {
        Wire.read();
        //TODO Make warning LED signal.
        return;
      }
    }

    registerAddress = address;
    
    // Write data to register.
    if (Wire.available()) {
      // Disable the low battery check if changing the low battery value.
      if (registerAddress == REG_BATTERY5 || registerAddress == REG_BATTERY4) {
        registers[REG_BATTERY3] = 0;
      }

      uint8_t data = Wire.read();
      uint8_t readOnlyData = registers[registerAddress] & ~writeMasks[registerAddress]; // Register read only values.
      uint8_t writeData = data & writeMasks[registerAddress];                           // Register new values to write
      // If writeData and data are not the same then there was an attempt to write to a read only register, so cancel write.
      if (writeData != data) {
        // TODO Make warning LED signal
        return;
      }

      // Combine read only bits and new write bits into the register.
      registers[registerAddress] = readOnlyData | writeData;
      registersWrittenTo = true;
    }
  }
}

void requestEvent() {
  Wire.write(registers[registerAddress]);
}
