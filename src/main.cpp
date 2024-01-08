#include <Arduino.h>
#include <avr/sleep.h>
#include <Wire.h>
#include "main.h"
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <timer.h>
#include <avr/io.h>

#define VERSION 9

//=====DEFINITIONS=====//
#define BATTERY_HYSTERESIS 10
#define BUTTON_LONG_PRESS_DURATION 1200

//=====I2C DEFINITIONS=====//
#define I2C_ADDRESS 0x25
#define REG_LEN     0x24

// Check registers.md for details on registers functions. //TODO, update registers.md
#define REG_TYPE                 0x00
#define REG_VERSION              0x01
#define REG_CAMERA_STATE         0x02
#define REG_CAMERA_CONNECTION    0x03
#define REG_PI_COMMANDS          0x04
#define REG_RP2040_PI_POWER_CTRL 0x05
#define REG_AUX_TERMINAL         0x06
#define REG_TC2_AGENT_READY      0x07

#define REG_BATTERY_CHECK_CTRL 0x10 //CTRL
#define REG_BATTERY_LOW_VAL1   0x11 //LOW 1
#define REG_BATTERY_LOW_VAL2   0x12 //LOW 2
#define REG_BATTERY_LV_DIV_VAL1    0x13 //HV1
#define REG_BATTERY_LV_DIV_VAL2    0x14 //HV2
#define REG_BATTERY_HV_DIV_VAL1    0x15 //LV1
#define REG_BATTERY_HV_DIV_VAL2    0x16 //LV2
#define REG_BATTERY_RTC_VAL1   0x17 //LV1
#define REG_BATTERY_RTC_VAL2   0x18 //LV2

#define REG_ERRORS1 0x20
#define REG_ERRORS2 0x21
#define REG_ERRORS3 0x22
#define REG_ERRORS4 0x23

#define WRITE_CAMERA_STATE_FLAG  0x01
#define READ_ERRORS_FLAG         0x01 << 1
#define ENABLE_WIFI_FLAG         0x01 << 2
#define POWER_OFF_RPI            0x01 << 3
#define TOGGLE_AUX_TERMINAL_FLAG 0x01 << 4


uint8_t registers[REG_LEN] = {0};
uint8_t writeMasks[REG_LEN] = {}; // Should all be initialised to 0xFF
uint8_t registerAddress = 0;

//=====GLOBAL VARIABLES=====//
volatile uint16_t battLowVoltageDiv = 0;      // Raw reading value from ADC
volatile uint16_t battHighVoltageDiv = 0;     // Raw reading value from ADC
volatile uint16_t battRTC = 0;      // Raw reading value from ADC
volatile bool lowBatteryCheck = false;        // Check main battery for a low battery condition.
volatile uint16_t lowBatteryValue = 0;        // Value that triggers a low battery condition.
volatile bool registersWrittenTo = false;     // Flag to indicate that registers have been written to.
volatile bool rp2040ReadyToPowerOff = false;  // Need to wait for RPi to power off so this flag is set by the RP2040 when it is ready.

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
#define PI_COMMS_INTERVAL 100000

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

// Timer to check if the button is being pressed in quick succession.
volatile unsigned long previousButtonPressTime = 0;
volatile uint8_t quickButtonPressCount = 0;
#define BUTTON_QUICK_PRESS_DURATION 500

volatile CameraState cameraState = CameraState::POWERING_ON;
StatusLED statusLED;

volatile bool quickFlash = false;

void setup() {
  PORTMUX_CTRLC |= PORTMUX_TCA00_bm;
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
  pinMode(HV_BAT_SENSE, INPUT);
  pinMode(LV_BAT_SENSE, INPUT);
  pinMode(RTC_ALARM, INPUT_PULLUP);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(PI_POWERED_OFF, INPUT_PULLUP);
  pinMode(PI_COMMAND_PIN, OUTPUT);
  digitalWrite(PI_COMMAND_PIN, HIGH);
  statusLED.writeColor(0, 0, 0);

  // Check for a low battery.
  checkForLowBattery();
  statusLED.writeColor(255, 0, 0);
  delay(100);
  statusLED.writeColor(0, 255, 0);
  delay(100);
  statusLED.writeColor(0, 0, 255);
  delay(100);
  statusLED.writeColor(0, 0, 0);

  // Write I2C register write masks.
  for (int i = 0; i < REG_LEN; i++) {
    writeMasks[i] = 0xFF;
  }
  writeMasks[REG_VERSION] = 0x00; 
  writeMasks[REG_TYPE] = 0x00;
  writeMasks[REG_BATTERY_CHECK_CTRL] = 0x03; // Only allow writing to bits to turn on or off battery check.
  writeMasks[REG_BATTERY_LV_DIV_VAL1] = 0x01 << 7;
  writeMasks[REG_BATTERY_LV_DIV_VAL2] = 0x00;
  writeMasks[REG_BATTERY_HV_DIV_VAL1] = 0x01 << 7;
  writeMasks[REG_BATTERY_HV_DIV_VAL2] = 0x00;
  writeMasks[REG_BATTERY_RTC_VAL1] = 0x01 << 7;
  writeMasks[REG_BATTERY_RTC_VAL2] = 0x00;
  
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
  regBatteryLVDivUpdate();
  regBatteryHVDivUpdate();
  regBatteryRTCUpdate();
  //checkRegSleep();
  checkRegRP2040PiPowerCtrl();
  //checkWakeUpPiReg();
  //}

  buttonWakeUp();

  processButtonPress();
  checkForLowBattery();
  checkCameraState();
  checkPiCommands();
  checkPiCommsCountdown();

  // Only power off the RP2040 when the RPi has been power off first.
  if (rp2040ReadyToPowerOff && registers[REG_CAMERA_STATE] == uint8_t(CameraState::POWERED_OFF)) {
    powerOffRP2040();
    rp2040ReadyToPowerOff = false;
    //TODO Check that this won't power off the RP2040 when unwanted, might need more logic around the rp2040ReadyToPowerOff variable.
  }

  if (statusLED.isOn()) {
    set_sleep_mode(SLEEP_MODE_IDLE);
  } else {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  }

  updateLEDs();
  //delay(50);
  sleep_cpu();
}

void updateLEDs() {
  bool auxEnabled = registers[REG_AUX_TERMINAL] & 0x01;
  statusLED.updateLEDs(cameraState, static_cast<CameraConnectionState>(registers[REG_CAMERA_CONNECTION]), auxEnabled);
}

void writeErrorFlag(ErrorCode errorCode, bool flash = true) {
  uint8_t errorCodeUint = static_cast<uint8_t>(errorCode);
  uint8_t regOffset = errorCodeUint/8;
  uint8_t regBit = errorCodeUint%8;
  uint8_t reg = REG_ERRORS1 + regOffset;
  registers[reg] |= 1 << regBit;
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
    //TODO Only drive the pin high when the pi is powered on.
    digitalWrite(PI_COMMAND_PIN, HIGH);
  } else {
    digitalWrite(PI_COMMAND_PIN, LOW);
  }
}


// ================ BATTERY CHECKS ==================

uint16_t sampleBattery(int batteryVoltagePin) {
  int samples = 10;
  int batteryVoltageSum = 0;
  for (int i = 0; i < samples; i++) {
    batteryVoltageSum += analogRead(batteryVoltagePin);
  }
  return batteryVoltageSum / samples;
}

void checkBatteryHighVoltageDivider() {
  battHighVoltageDiv = sampleBattery(HV_BAT_SENSE);
}

void checkBatteryLowVoltageDivider() {
  checkBatteryHighVoltageDivider();
  if (battHighVoltageDiv > 340) { // TODO Find proper value for this.
    // If battery voltage is too high the low voltage divider won't work and 
    // should be set to an LOW output to prevent high voltages on the pin.
    digitalWrite(LV_BAT_SENSE, LOW);
    pinMode(LV_BAT_SENSE, OUTPUT);
    battLowVoltageDiv = 1023;
    return;
  }
  pinMode(LV_BAT_SENSE, INPUT);
  delay(1);
  battLowVoltageDiv = sampleBattery(LV_BAT_SENSE);
}

void checkBatteryRTC() {
  battRTC = sampleBattery(RTC_BAT_SENSE);
}

void checkForLowBattery() {
  //TODO
  /*
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
  */
}

void checkCameraState() {
  // Update camera state from register if valid.
  if (registers[REG_CAMERA_STATE] <= static_cast<uint8_t>(CameraState::REBOOTING)) {
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

  if ((cameraState == CameraState::POWERED_ON || cameraState == CameraState::REBOOTING) && digitalRead(PI_POWERED_OFF) == LOW) {
    powerRPiOffNow();
    delay(1000);
    powerOnRPi();
  }

  //TODO Check if PI_POWERED_OFF pin is low and not in the proper state for it.

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

void regBatteryLVDivUpdate() {
  if (registers[REG_BATTERY_LV_DIV_VAL1] & 0x01 << 0x07) {
    checkBatteryLowVoltageDivider();
    registers[REG_BATTERY_LV_DIV_VAL1] = battLowVoltageDiv >> 8 & ~(0x01 << 0x07); // Writing the MSB of the battery voltage and clear bit 7.
    registers[REG_BATTERY_LV_DIV_VAL2] = battLowVoltageDiv & 0xFF;
  }
}

void regBatteryHVDivUpdate() {
  if (registers[REG_BATTERY_HV_DIV_VAL1] & 0x01 << 0x07) {
    checkBatteryHighVoltageDivider();
    registers[REG_BATTERY_HV_DIV_VAL1] = battHighVoltageDiv >> 8 & ~(0x01 << 0x07); // Writing the MSB of the battery voltage and clear bit 7.
    registers[REG_BATTERY_HV_DIV_VAL2] = battHighVoltageDiv & 0xFF;
  }
}

void regBatteryRTCUpdate() {
  if (registers[REG_BATTERY_RTC_VAL1] & 0x01 << 0x07) {
    checkBatteryRTC();
    registers[REG_BATTERY_RTC_VAL1] = battRTC >> 8 & ~(0x01 << 0x07); // Writing the MSB of the battery voltage and clear bit 7.
    registers[REG_BATTERY_RTC_VAL2] = battRTC & 0xFF;
  }
}

void checkRegRP2040PiPowerCtrl() {
  if (registers[REG_RP2040_PI_POWER_CTRL] & (0x01 << 0) == 0) {
    // RP2040 does not require the RPi to be powered on.
    // Do not directly power off the RPi, instead the RPi will read this (//TODO) to check if it can power off.
  } else {
    // RP2040 requires the RPi to be powered on. (RPi will read this so it knows to stay on, //TODO)
    // If RPi is off, then power it on.
    if (cameraState == CameraState::POWERED_OFF) {
      powerOnRPi();
    }
  }

  if (registers[REG_RP2040_PI_POWER_CTRL] & (0x01 << 1) == 0) {
    // RP2040 needs to be powered on.
    powerOnRP2040();
  } else {
    // Power off RP2040 if the Raspberry Pi is powered off
    if (cameraState == CameraState::POWERED_OFF) {
      powerOffRP2040();
    } else {
      powerOnRP2040();
    }
  }
}

//REG_BATTERY3
// 7-3, reserved
// bit 2: read only. 0 = no low battery, 1 = low battery
// bit 1: write high to disable low battery.
// bit 0: write high to enable low battery.
void lowBatteryRegUpdate() {
  //TODO
  /*
  // Check if low battery check is wanting to be enabled.
  if (registers[REG_BATTERY3] & 0x01) {
    uint16_t newLowBatteryVal = (registers[REG_BATTERY4] << 8) | registers[REG_BATTERY5];
    checkForLowBattery();
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
  */
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
      if (registerAddress == REG_BATTERY_LOW_VAL2 || registerAddress == REG_BATTERY_LOW_VAL1) {
        registers[REG_BATTERY_CHECK_CTRL] = 0;
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
  registers[REG_RP2040_PI_POWER_CTRL] = 0;
  if (cameraState == CameraState::POWERED_OFF) {
    checkForLowBattery();
    powerOnRP2040();
  }
}

//======================= RP2040 FUNCTIONS ============================//
void powerOnRP2040() {
  rp2040ReadyToPowerOff = false;
  checkForLowBattery(); // Will stay in checkForLowBattery until battery is good.
  digitalWrite(EN_RP2040, LOW);
}

void powerOffRP2040() {
  digitalWrite(EN_RP2040, HIGH);
}

//======================= RASPBERRY_PI FUNCTIONS ============================//
void powerOnRPi() {
  checkForLowBattery(); // Will stay in checkForLowBattery until battery is good.
  digitalWrite(EN_5V, HIGH);
  powerOnRP2040();
  writeCameraState(CameraState::POWERING_ON);
  poweringOnTime = getPitTimeMillis();
}

void poweringOffRPi() {
  writeCameraState(CameraState::POWERING_OFF);
  registers[REG_TC2_AGENT_READY] = 0;
  poweringOffTime = getPitTimeMillis();
}

void powerRPiOffNow() {
  writeCameraState(CameraState::POWERED_OFF);
  registers[REG_TC2_AGENT_READY] = 0;
  digitalWrite(EN_5V, LOW);
}

void powerOffRPi() {
  digitalWrite(PI_SHUTDOWN, LOW);
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
// From 5 to 2999, ask the RPi to start communications.
// 3000 or above, shut down RPi then restart ATtiny1616 after 5 seconds.
// After being processed the buttonPressDuration is reset to 0.
volatile unsigned long buttonPressDuration = 0;

// buttonWakeUp is function called by the interrupt of the falling edge of the button.
void buttonWakeUp() {
  if (digitalRead(BUTTON) == HIGH) {{
    return;
  }}
  unsigned long start = getPitTimeMillis();
  buttonPressDuration = 0;
  while (digitalRead(BUTTON) == LOW) {
    if (buttonPressDuration > BUTTON_LONG_PRESS_DURATION) {
      statusLED.writeColor(0x000000);
    } else {
      statusLED.writeColor(0xFFFFFF);
    }
    delay(1);
    buttonPressDuration++;
  }
  if (buttonPressDuration > 5) {
    if (getPitTimeMillis() - previousButtonPressTime < BUTTON_QUICK_PRESS_DURATION) {
      quickButtonPressCount++;
    } else {
      quickButtonPressCount = 0;
    }
    previousButtonPressTime = getPitTimeMillis();
  }
  updateLEDs();
}

void processButtonPress() {
  if (buttonPressDuration < 5) {
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
    }
  }
  if (quickButtonPressCount > 10) {
    quickButtonPressCount = 0;
    statusLED.writeColor(0xFFFFFF);
    delay(2000);
    requestPiCommand(TOGGLE_AUX_TERMINAL_FLAG);
  }
  buttonPressDuration = 0;
}
