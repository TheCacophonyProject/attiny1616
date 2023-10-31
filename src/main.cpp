#include <Arduino.h>
#include <avr/sleep.h>
#include <Wire.h>
#include "main.h"
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <timer.h>
#include <avr/io.h>

#define VERSION 5

//=====DEFINITIONS=====//
#define BATTERY_HYSTERESIS 10
#define BUTTON_LONG_PRESS_DURATION 2000

//=====I2C DEFINITIONS=====//
#define I2C_ADDRESS 0x25
#define REG_LEN     0x24

// Check registers.md for details on registers functions. //TODO, update registers.md
#define REG_TYPE                0x00
#define REG_VERSION             0x01
#define REG_CAMERA_STATE        0x02
#define REG_CAMERA_CONNECTION   0x03
#define REG_PI_COMMANDS         0x04
#define REG_TRIGGER_SLEEP       0x05
#define REG_PI_WAKEUP           0x06

#define REG_BATTERY1            0x10
#define REG_BATTERY2            0x11
#define REG_BATTERY3            0x12
#define REG_BATTERY4            0x13
#define REG_BATTERY5            0x14
#define REG_RTC_BATTERY1        0x15
#define REG_RTC_BATTERY2        0x16

#define REG_ERRORS1             0x20
#define REG_ERRORS2             0x21
#define REG_ERRORS3             0x22
#define REG_ERRORS4             0x23

#define WRITE_CAMERA_STATE_FLAG 0x01
#define READ_ERRORS_FLAG        0x01 << 1
#define ENABLE_WIFI_FLAG        0x01 << 2


uint8_t registers[REG_LEN] = {0};
uint8_t writeMasks[REG_LEN] = {}; // Should all be initialised to 0xFF
uint8_t registerAddress = 0;

//=====GLOBAL VARIABLES=====//
volatile uint16_t mainBatteryVoltage = 0; // Raw reading value from ADC
volatile uint16_t rtcBatteryVoltage = 0;  // Raw reading value from ADC
volatile bool lowBatteryCheck = false;    // Check main battery for a low battery condition.
volatile uint16_t lowBatteryValue = 0;    // Value that triggers a low battery condition.
volatile bool registersWrittenTo = false; // Flag to indicate that registers have been written to.

//======== TIMERS ==========//
// Time from getPitTimeMillis() of then the camera was powered on.
// After 5 minutes a error code shoule be shown on the LED of the camera.
// After 30 minutes the camera is reset and a MAX_POWERING_ON_DURATION_MS error flag is set in the I2C register.
volatile unsigned long poweringOnTime = 0;
#define MAX_POWERING_ON_DURATION_MS 300000
//#define MAX_POWERING_ON_DURATION_MS 10000

// Time from getPitTimeMillis() of the last time the ATtiny was communicated with the RPi.
// If it has not been communicated for over PI_COMMS_INTERVAL it will request comms from the RPi.
volatile unsigned long lastPiCommsTime = 0;
//#define WDT_RESET_INTERVAL 30000
#define PI_COMMS_INTERVAL 10000

// Time from getPitTimeMillis() of when the camera asked to be turned off.
// After a set about of time in ms the camera is defined from POWER_OFF_DELAY_MS it will then power off the camera.
volatile unsigned long poweringOffTime = 0;
#define POWER_OFF_DELAY_MS 60000

// Timer to check that the ATtiny is woken up by the RTC_ALARM interrupt after at least 24 hours. 
// If reaches MAX_POWERED_OFF_DURATION_MS then the camera is reset and a SLEEP_ error flag is set in the I2C register
volatile unsigned long poweredOffTime = 0; 
#define MAX_POWERED_OFF_DURATION_MS 86400000

// Time from getPitTimeMillis() of when the ATtiny requested communications from the Raspberry Pi.
// If the Raspberry Pi is not hear from after PI_COMMAND_TIMEOUT a error flag will be set in the I2C register.
//volatile unsigned long pingPiTime = 0;
volatile unsigned long piCommandRequestTime = 0;
#define PI_COMMAND_TIMEOUT 5000


volatile CameraState cameraState = CameraState::POWERING_ON;
StatusLED statusLED;

volatile bool quickFlash = false;

void setup() {
  // Initialize Pins
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  digitalWrite(EN_5V, HIGH);
  pinMode(EN_5V, OUTPUT);
  digitalWrite(PI_SHUTDOWN, HIGH);
  pinMode(PI_SHUTDOWN, OUTPUT);
  pinMode(EN_RP2040, OUTPUT);
  pinMode(RTC_BAT_SENSE, INPUT);
  pinMode(MAIN_BAT_SENSE, INPUT);
  pinMode(RTC_ALARM, INPUT_PULLUP);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(PI_POWERED_OFF, INPUT_PULLUP);
  pinMode(PI_COMMAND_PIN, OUTPUT);
  digitalWrite(PI_COMMAND_PIN, HIGH);
  statusLED.writeColor(0, 0, 0);

  // Check for a low battery.
  checkMainBattery();

  statusLED.writeColor(255, 0, 0);
  delay(100);
  statusLED.writeColor(0, 255, 0);
  delay(100);
  statusLED.writeColor(0, 0, 255);
  delay(100);

  // Write I2C register write masks.
  for (int i = 0; i < REG_LEN; i++) {
    writeMasks[i] = 0xFF;
  }
  writeMasks[REG_VERSION] = 0x00; 
  writeMasks[REG_TYPE] = 0x00;
  writeMasks[REG_BATTERY3] = 0x03; // Only allow writing to bits to turn on or off battery check.
  writeMasks[REG_BATTERY1] = 0x01 << 7;
  writeMasks[REG_BATTERY2] = 0x00;
  writeMasks[REG_RTC_BATTERY1] = 0x01 << 7;
  writeMasks[REG_RTC_BATTERY2] = 0x00;
  
  // Write I2C initial register values.
  registers[REG_TYPE] = 0xCA;
  registers[REG_VERSION] = VERSION;

  // Setup I2C
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  // Setup interrupts
  attachInterrupt(digitalPinToInterrupt(RTC_ALARM), rtcWakeUp, FALLING);
  setupPIT();

  set_sleep_mode(SLEEP_MODE_IDLE);
  // set_sleep_mode(SLEEP_MODE_PWR_DOWN); // PWM LED signals don't work in this power mode. //TODO Use this power mode when LEDs are solid or not on.
  sleep_enable();
  powerOnRPi();
}

void loop() {
  if (quickFlash) {
    statusLED.writeColor(255, 255, 255);
    delay(50);
    statusLED.writeColor(0, 0, 0);
    quickFlash = false;
  }

  // TODO Check this only when needed.
  // Check updates from I2C registers
  //if (registersWrittenTo) {
  //  registersWrittenTo = false;
  lowBatteryRegUpdate();
  mainBatteryRegUpdate();
  rtcBatteryRegUpdate();
  checkRegSleep();
  checkWakeUpPiReg();
  //}

  buttonWakeUp();

  processButtonPress();
  checkMainBattery();
  checkCameraState();
  checkPiCommands();
  checkPiCommsCountdown();

  updateLEDs();
  //delay(50);
  sleep_cpu();
}

void updateLEDs() {
  statusLED.updateLEDs(cameraState, static_cast<CameraConnectionState>(registers[REG_CAMERA_CONNECTION]));
}

void writeErrorFlag(ErrorCode errorCode, bool flash = true) {
  uint8_t errorCodeUint = static_cast<uint8_t>(errorCode);
  uint8_t regOffset = errorCodeUint/8;
  uint8_t reg = REG_ERRORS1 + regOffset;
  registers[reg] |= (1 << (errorCodeUint%8));
  // TODO save error registers to EEPROM so can be restored on reboot 
  if (flash) {
    statusLED.error(errorCode);
  }
  requestPiCommand(READ_ERRORS_FLAG);
}

void checkPiCommsCountdown() {
  if (cameraState != CameraState::POWERED_ON) {
    return;
  }
  if (getPitTimeMillis() - lastPiCommsTime > PI_COMMS_INTERVAL) {
    requestPiCommand(WRITE_CAMERA_STATE_FLAG);
  }
}

void checkPiCommands() {
  // Check if the Raspberry Pi is responding to command requests
  if (registers[REG_PI_COMMANDS] != 0 &&  getPitTimeMillis() - piCommandRequestTime > PI_COMMAND_TIMEOUT) {
    writeErrorFlag(ErrorCode::PI_COMMAND_TIMEDOUT);
    registers[REG_PI_COMMANDS] = 0;
  }

  // Drive PI_COMMAND_PIN low to get Raspberry Pi to check what commands to run
  if (registers[REG_PI_COMMANDS] == 0) {
    digitalWrite(PI_COMMAND_PIN, HIGH);
  } else {
    digitalWrite(PI_COMMAND_PIN, LOW);
  }
}

void checkMainBattery() {
  int samples = 10;
  int batteryVoltage = 0;
  for (int i = 0; i < samples; i++) {
    batteryVoltage += analogRead(MAIN_BAT_SENSE);
  }
  mainBatteryVoltage = batteryVoltage / samples;

  // Check if battery voltage is OK.
  if (!lowBatteryCheck || mainBatteryVoltage > lowBatteryValue || mainBatteryVoltage <= 1) {
    return; // Battery is OK.
  }

  powerRPiOffNow();
  
  // Flash LED to show battery is low
  for (int i = 0; i < 3; i++) {
    statusLED.writeColor(0xFF, 0x00, 0x00);
    delay(300);
    statusLED.writeColor(0x00, 0x00, 0x00);
    delay(300);  
  }

  // Loop checking battery voltage until battery is better
  while (1) {
    // Check main battery, making sure it reaches the threshold at least 10 times in a row.
    bool lowBattery = false;
    for (int i = 0; i < 10; i++) {
      if (analogRead(MAIN_BAT_SENSE) < lowBatteryValue + BATTERY_HYSTERESIS) {
        lowBattery = true;
        break;
      }
    }
    if (!lowBattery) {
      return; // Battery is better again, return to normal loop.
    }

    // TODO go to deep sleep.
    // Wait for PIT to wake up again and then check battery. 
  }
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
  // Update camera state from register if valid.
  if (registers[REG_CAMERA_STATE] <= static_cast<uint8_t>(CameraState::POWER_ON_TIMEOUT)) {
    writeCameraState(static_cast<CameraState>(registers[REG_CAMERA_STATE]));
  } else {
    writeErrorFlag(ErrorCode::INVALID_CAMERA_STATE);
  }

  // Check if the camera has had a power on timeout.
  if (cameraState == CameraState::POWERING_ON && getPitTimeMillis() - poweringOnTime > MAX_POWERING_ON_DURATION_MS) {
    writeCameraState(CameraState::POWER_ON_TIMEOUT);
    writeErrorFlag(ErrorCode::POWER_ON_FAILED);
    return;
  }

  // Check if the camera has powered off.
  if (cameraState == CameraState::POWERING_OFF) {
    // gpio-poweroff in config.txt for the RPi will drive a pin low when it powers off.
    // TODO Check if you need to wait here for a second or two for the flash to finish writing.
    if (digitalRead(PI_POWERED_OFF) == LOW) {
      powerRPiOffNow();
      statusLED.off();
    }
  }

  if (cameraState == CameraState::POWERED_ON && digitalRead(PI_POWERED_OFF) == LOW) {
    powerRPiOffNow();
    delay(1000);
    powerOnRPi();
  }

  // TODO replace this with a shutdown timeout, as power off will be triggered from the PI_POWERED_OFF pin.
  // Check if the camera has had enough time to power off.
  if (cameraState == CameraState::POWERING_OFF && getPitTimeMillis() - poweringOffTime > POWER_OFF_DELAY_MS) {
    powerRPiOffNow();
    return;
  }

  // Check if the device has been in sleep for too long.
  if (cameraState == CameraState::POWERED_OFF && getPitTimeMillis() - poweredOffTime > MAX_POWERED_OFF_DURATION_MS) {
    writeErrorFlag(ErrorCode::RTC_TIMEOUT);
    powerOnRPi();
    return;
  }
}

void writeCameraState(CameraState newCameraState) {
  if (newCameraState != cameraState) {
    cameraState = newCameraState;
    updateLEDs();
  }
  registers[REG_CAMERA_STATE] = static_cast<uint8_t>(cameraState);
}
//============================== I2C Register FUNCTIONS ==============================//

void checkWakeUpPiReg() {
  if (registers[REG_PI_WAKEUP] != 0) {
    registers[REG_PI_WAKEUP] = 0;
    powerOnRPi();
  }
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
    registers[REG_TRIGGER_SLEEP] = 0;
    poweringOffRPi();
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
      writeErrorFlag(ErrorCode::LOW_BATTERY_LEVEL_SET_TOO_LOW);
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


//============================== I2C FUNCTIONS ==============================//
void receiveEvent(int howMany) {
  while(Wire.available()) {
    uint8_t address = Wire.read();
    // Prevent from writing/reading to registers outside of the range.
    if (address >= REG_LEN) {
      while(Wire.available()) {
        Wire.read();
        writeErrorFlag(ErrorCode::INVALID_REG_ADDRESS);
        return;
      }
    }
    if (address == REG_CAMERA_STATE) {
      lastPiCommsTime = getPitTimeMillis();
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
        writeErrorFlag(ErrorCode::WRITE_TO_READ_ONLY);
        return;
      }

      // Combine read only bits and new write bits into the register.
      registers[registerAddress] = readOnlyData | writeData;
      registersWrittenTo = true;
    }
  }
}

// Best to just read one register at a time, this is explained further in this example.
// https://github.com/SpenceKonde/megaTinyCore/blob/master/megaavr/libraries/Wire/examples/register_model/register_model.ino
// TODO Set it up so it 
void requestEvent() {
  Wire.write(registers[registerAddress]);
  //quickFlash = true;
}

//=============================ISR DEFINITIONS==================================//
// Function attached to the pin connected to the alarm pin on the RTC.
void rtcWakeUp() {
  if (cameraState == CameraState::POWERED_OFF) {
    checkMainBattery();
    powerOnRPi();
  }
  // TODO Wake up RP2040 
}

//======================= RASPBERRY_PI FUNCTIONS ============================//
void powerOnRPi() {
  checkMainBattery(); // Will stay in checkMainBattery until battery is good.
  digitalWrite(EN_5V, HIGH);
  digitalWrite(EN_RP2040, LOW);
  writeCameraState(CameraState::POWERING_ON);
  poweringOnTime = getPitTimeMillis();
}

void poweringOffRPi() {
  writeCameraState(CameraState::POWERING_OFF);
  poweringOffTime = getPitTimeMillis();
}

void powerRPiOffNow() {
  writeCameraState(CameraState::POWERED_OFF);
  digitalWrite(EN_5V, LOW);
}

void powerOffRPi() {
  digitalWrite(PI_SHUTDOWN, LOW);
  digitalWrite(EN_RP2040, HIGH);
  poweringOffRPi();
  delay(100);
  digitalWrite(PI_SHUTDOWN, HIGH);
}

void requestPiCommand(uint8_t command) {
  if (registers[REG_PI_COMMANDS] == 0) {
    piCommandRequestTime = getPitTimeMillis();
  }
  registers[REG_PI_COMMANDS] = registers[REG_PI_COMMANDS] | command;
}

//============================ BUTTON FUNCTIONS =============================//
// Records the duration that the button was pressed.
// If less than 50 then ignore, probably debounce.
// From 50 to 2999, ask the RPi to start communications.
// 3000 or above, shut down RPi then restart ATtiny1616 after 5 seconds.
// After being processed the buttonPressDuration is reset to 0.
volatile unsigned long buttonPressDuration = 0;

// buttonWakeUp is function called by the interrupt of the falling edge of the button.
void buttonWakeUp() {
  if (digitalRead(BUTTON) == HIGH) {{
    return;
  }}
  unsigned long start = getPitTimeMillis();
  while (digitalRead(BUTTON) == LOW) {
    if (getPitTimeMillis() - start > BUTTON_LONG_PRESS_DURATION) {
      statusLED.writeColor(0x000000);
    } else {
      statusLED.writeColor(0xFFFFFF);
    }
    delay(1);
  }
  updateLEDs();
  buttonPressDuration = getPitTimeMillis() - start;
}

void processButtonPress() {
  if (buttonPressDuration < 10) {
    buttonPressDuration = 0;
    return;
  } else if (buttonPressDuration < BUTTON_LONG_PRESS_DURATION) {
    if (!statusLED.isOn()) {
      statusLED.show();
    } else if (cameraState == CameraState::POWERED_ON) {
      requestPiCommand(ENABLE_WIFI_FLAG);
    }
    updateLEDs();
  } else {
    if (cameraState == CameraState::POWERED_OFF) {
      powerOnRPi();
    } else {
      powerOffRPi();
      //requestPiCommand(ENABLE_WIFI_FLAG);
    }
  }
  buttonPressDuration = 0;
}
