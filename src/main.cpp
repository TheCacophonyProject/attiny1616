#include <Arduino.h>
#include <avr/sleep.h>
#include <Wire.h>
#include "main.h"
#include <avr/wdt.h>
#include <avr/interrupt.h> // Include the AVR interrupt library
#include <avr/wdt.h> // Include the AVR watchdog timer library
#include <avr/wdt.h>
#include <avr/interrupt.h>

#define VERSION 1

//=====DEFINITIONS=====//
#define BATTERY_HYSTERESIS 10          // The hysteresis for the battery voltage.

//=====I2C DEFINITIONS=====//
#define I2C_ADDRESS 0x25
#define REG_LEN     0x24

// Check registers.md for details on registers functions.
#define REG_TYPE                0x00
#define REG_VERSION             0x01
#define REG_CAMERA_STATE        0x02
#define REG_CAMERA_CONNECTION   0x03
#define REG_RESET_WATCHDOG      0x04
#define REG_TRIGGER_SLEEP       0x05
#define REG_CAMERA_WAKEUP       0x06
#define REG_REQUEST_COMMUNICATION 0x07
#define REG_PING_PI               0x08

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
// Time from millis() of then the camera was powered on.
// After 5 minutes a error code shoule be shown on the LED of the camera.
// After 30 minutes the camera is reset and a MAX_POWERING_ON_DURATION_MS error flag is set in the I2C register.
volatile unsigned long poweringOnTime = 0;
#define MAX_POWERING_ON_DURATION_MS 300000
//#define MAX_POWERING_ON_DURATION_MS 10000

// Time from millis() of when the camera WDT was reset.
// If camera WDT is not reset for more than WDT_RESET_INTERVAL (5 minutes) then 
// the camera is reset, assuming something went wrong on the camera causing it to freeze.
// A error flag will also be set in the I2C register, so the camera can see and report the error.
volatile unsigned long poweredOnWDTResetTime = 0;
//#define WDT_RESET_INTERVAL 30000
#define WDT_RESET_INTERVAL 3000000

// Time from millis() of when the camera asked to be turned off.
// After a set about of time in ms the camera is defined from POWER_OFF_DELAY_MS it will then power off the camera.
volatile unsigned long poweringOffTime = 0;
#define POWER_OFF_DELAY_MS 30000

// Timer to check that the ATtiny is woken up by the RTC_ALARM interrupt after at least 24 hours. 
// If reaches MAX_POWERED_OFF_DURATION_MS then the camera is reset and a SLEEP_ error flag is set in the I2C register
volatile unsigned long poweredOffTime = 0; 
#define MAX_POWERED_OFF_DURATION_MS 86400000

// Time from millis() of when the ATtiny requested communications from the Raspberry Pi.
// If the Raspberry Pi is not hear from after PING_PI_TIMEOUT a error flag will be set in the I2C register.
volatile unsigned long pingPiTime = 0;
#define PING_PI_TIMEOUT 30000


volatile CameraState cameraState = CameraState::POWERING_ON;
StatusLED statusLED;

void setup() {
  // Initialize Pins
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(EN_5V, OUTPUT);
  digitalWrite(EN_5V, HIGH);
  pinMode(RTC_BAT_SENSE, INPUT);
  pinMode(MAIN_BAT_SENSE, INPUT);
  pinMode(RTC_ALARM, INPUT_PULLUP);
  pinMode(BUTTON, INPUT_PULLUP);
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
  attachInterrupt(digitalPinToInterrupt(BUTTON), buttonWakeUp, FALLING);
  setupPIT(); // Wake up every 1 second.

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  powerOnRPi();
}

void loop() {
  // Check updates from I2C registers
  //if (registersWrittenTo) {
  //  registersWrittenTo = false;
  lowBatteryRegUpdate();
  mainBatteryRegUpdate();
  rtcBatteryRegUpdate();
  checkRegSleep();
  wdtRegUpdate();
  checkWakeUpReg();
  //}

  processButtonPress();
  checkMainBattery();
  checkCameraState();
  checkPing();
  checkWDTCountdown();

  updateLEDs();
  //sleep_cpu();  //TODO Enable, enabling this seams to muck with millis(), maybe, maybe not.
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
}

void checkWDTCountdown() {
  if (cameraState != CameraState::POWERED_ON) {
    return;
  }
  if (millis() - poweredOnWDTResetTime > WDT_RESET_INTERVAL) {
    writeErrorFlag(ErrorCode::WATCHDOG_TIMEOUT);
    powerRPiOffNow();
  }
}

void checkPing() {
  if (registers[REG_PING_PI] != 0 &&  millis() - pingPiTime > PING_PI_TIMEOUT) {
    // Timeout for raspberry pi communicating to attiny.
    writeErrorFlag(ErrorCode::NO_PING_RESPONSE);
    registers[REG_PING_PI] = 0;
  }

  // Update Ping Pi pin
  if (registers[REG_PING_PI] == 1) {
    digitalWrite(PING_PIN, HIGH);
  } else {
    digitalWrite(PING_PIN, LOW);
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
    bool lowbattery = false;
    for (int i = 0; i < 10; i++) {
      if (analogRead(MAIN_BAT_SENSE) < lowBatteryValue + BATTERY_HYSTERESIS) {
        lowbattery = true;
        break;
      }
    }
    if (!lowbattery) {
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
  if (cameraState == CameraState::POWERING_ON && millis() - poweringOnTime > MAX_POWERING_ON_DURATION_MS) {
    writeCameraState(CameraState::POWER_ON_TIMEOUT);
    writeErrorFlag(ErrorCode::POWER_ON_FAILED);
    return;
  }

  // Check if the camera has had enough time to power off.
  if (cameraState == CameraState::POWERING_OFF && millis() - poweringOffTime > POWER_OFF_DELAY_MS) {
    powerRPiOffNow();
    return;
  }

  // Check if the device has been in sleep for too long.
  if (cameraState == CameraState::POWERED_OFF && millis() - poweredOffTime > MAX_POWERED_OFF_DURATION_MS) {
    writeErrorFlag(ErrorCode::RTC_TIMEOUT);
    powerOnRPi();
    return;
  }
}

void writeCameraState(CameraState newCameraState) {
  if (newCameraState != cameraState) {
    poweredOnWDTResetTime = millis();
    cameraState = newCameraState;
    updateLEDs();
  }
  registers[REG_CAMERA_STATE] = static_cast<uint8_t>(cameraState);
}
//============================== I2C Register FUNCTIONS ==============================//

void checkWakeUpReg() {
  if (registers[REG_CAMERA_WAKEUP] != 0) {
    registers[REG_CAMERA_WAKEUP] = 0;
    powerOnRPi();
  }
}

void wdtRegUpdate() {
  if (registers[REG_RESET_WATCHDOG] != 0) {
    registers[REG_RESET_WATCHDOG] = 0;
    poweredOnWDTResetTime = millis();
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
    poweringOffRPi();
    registers[REG_TRIGGER_SLEEP] = 0;
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
}

//=============================ISR DEFINITIONS==================================//
ISR(RTC_PIT_vect) {
  RTC.PITINTFLAGS = RTC_PI_bm; // Clear interrupt flag, otherwise it will constantly trigger.
}

// Setup the RTC_PIT_interrupt to trigger every 1 second.
void setupPIT() {
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc; // Set clock source to the internal 32.768kHz oscillator
  RTC.PITCTRLA = RTC_PITEN_bm | RTC_PERIOD_CYC32768_gc; // Enable PIT and set period to 1 second
  RTC.PITINTCTRL = RTC_PI_bm; // Enable PIT interrupt
}

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
  writeCameraState(CameraState::POWERING_ON);
  poweringOnTime = millis();
}

void poweringOffRPi() {
  writeCameraState(CameraState::POWERING_OFF);
  poweringOffTime = millis();
}

void powerRPiOffNow() {
  writeCameraState(CameraState::POWERED_OFF);
  digitalWrite(EN_5V, LOW);
}

// request raspberry pi to start up wifi communications.
void startWifiRPi() {
  pingPiTime = millis();
  registers[REG_PING_PI] = 0x01;                // Will be reset by raspberry pi to 0x00
  registers[REG_REQUEST_COMMUNICATION] = 0x01;  // Will be reset by raspberry pi to 0x00
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
  unsigned long start = millis();
  while (digitalRead(BUTTON) == LOW) {}
  buttonPressDuration = millis() - start;
}

void processButtonPress() {
  if (buttonPressDuration < 50) {
    return;
  } else if (buttonPressDuration < 2000) {
    statusLED.show();
    if (cameraState == CameraState::POWERED_OFF) {
      powerOnRPi();
    } else {
      startWifiRPi();
    }
    updateLEDs();
  } else {
    wdt_enable(WDTO_15MS);  // enable the watchdog with shortest available timeout, this will make the attiny reboot.
    while(1) {}
  }
}
