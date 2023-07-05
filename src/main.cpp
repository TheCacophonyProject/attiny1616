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
#define BATTERY_HYSTERESIS 10          // The hysteresis for the battery voltage.  // TODO Make this configurable in I2C reg.

//=====I2C DEFINITIONS=====//
#define I2C_ADDRESS 0x25
#define REG_LEN     0x24

// Check registers.md for details on registers functions.
#define REG_VERSION             0x00
#define REG_TYPE                0x01
#define REG_CAMERA_STATE        0x02
#define REG_RESET_WATCHDOG      0x03
#define REG_TRIGGER_SLEEP       0x04
//#define REG_TRIGGER_SLEEP_DELAY 0x05
#define REG_CAMERA_WAKEUP       0x06
//#define REG_SLEEP_DURATION_MSB  0x07
//#define REG_SLEEP_DURATION_MID  0x08
//#define REG_SLEEP_DURATION_LSB  0x09
#define REG_BATTERY1            0x0A
#define REG_BATTERY2            0x0B
#define REG_BATTERY3            0x0C
#define REG_BATTERY4            0x0D
#define REG_BATTERY5            0x0E
#define REG_RTC_BATTERY1        0x0F
#define REG_RTC_BATTERY2        0x10
#define REG_REQUEST_COMMUNICATION 0x11
#define REG_ERRORS1             0x20
#define REG_ERRORS2             0x21
#define REG_ERRORS3             0x22
#define REG_ERRORS4             0x23



uint8_t registers[REG_LEN] = {0};
uint8_t writeMasks[REG_LEN] = {}; // Should all be initialised to 0xFF
uint8_t registerAddress = 0;

//=====GLOBAL VARIABLES=====//
volatile unsigned long poweredOnTime = 0; // Time when the camera was powered on. Used to check for camera power on timeout.
volatile uint16_t mainBatteryVoltage = 0; // Raw reading value from ADC
volatile uint16_t rtcBatteryVoltage = 0;  // Raw reading value from ADC
volatile bool lowBatteryCheck = false;    // Check main battery for a low battery condition.
volatile uint16_t lowBatteryValue = 0;    // Value that triggers a low battery condition.
volatile bool registersWrittenTo = false; // Flag to indicate that registers have been written to.


//======== TIMERS ==========//
// Time from millis() of then the camera was powered on.
// After 5 minutes a error code shoule be shown on the LED of the camera.
// After 30 minutes the camera is reset and a POWERING_ON_TIMEOUT error flag is set in the I2C register.
volatile unsigned long poweringOnTime = 0;
#define MAX_POWERING_ON_DURATION_MS 300000

// Time from millis() of when the camera WDT was reset.
// If camera WDT is not reset for more than WDT_RESET_INTERVAL (5 minutes) then 
// the camera is reset, assuming something went wrong on the camera causing it to freeze.
// A error flag will also be set in the I2C register, so the camera can see and report the error.
volatile unsigned long poweredOnWDTResetTime = 0;
#define WDT_RESET_INTERVAL 300000

// Time from millis() of when the camera asked to be turned off.
// After a set about of time in ms the camera is defined from POWER_OFF_DELAY_MS it will then power off the camera.
volatile unsigned long poweringOffTime = 0;
#define POWER_OFF_DELAY_MS 30000

// Timer to check that the ATtiny is woken up by the RTC_ALARM interrupt after at least 24 hours. 
// If reaches MAX_POWERED_OFF_DURATION_MS then the camera is reset and a SLEEP_ error flag is set in the I2C register
volatile unsigned long poweredOffTime = 0; 
#define MAX_POWERED_OFF_DURATION_MS 86400000

// Time from millis() of when the ATtiny requested communications from the Raspberry Pi.
// If the Raspberry Pi is not hear from after REQUEST_COMMUNICATION_TIMEOUT_MS a error flag will be set in the I2C register.
volatile unsigned long requestCommunicationTime = 0;
#define REQUEST_COMMUNICATION_TIMEOUT_MS 300000


volatile CameraState cameraState = CameraState::POWERING_ON;
StatusLED statusLED;

void setup() {
  // Initialize Pins
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(EN_5V, OUTPUT);
  pinMode(RTC_BAT_SENSE, INPUT);
  pinMode(MAIN_BAT_SENSE, INPUT);
  pinMode(RTC_ALARM, INPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  statusLED.writeColor(0, 0, 0);
  
  // Check for a low battery.
  checkMainBattery();

  // Write I2C register write masks.
  for (int i = 0; i < REG_LEN; i++) {
    writeMasks[i] = 0xFF;
  }
  writeMasks[REG_VERSION] = 0x00; 
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

  // TODO initial wakeup sequence 
  //set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  //sleep_enable();
}

void loop() {
  // Check updates from I2C registers
  if (registersWrittenTo) {
    registersWrittenTo = false;
    lowBatteryRegUpdate();
    mainBatteryRegUpdate();
    rtcBatteryRegUpdate();
    checkRegSleep();
    // TODO checkRegCommunication();
  }

  processButtonPress();

  //checkWakeUpRegisters(); 
  checkMainBattery();
  checkCameraState();
  checkWDTCountdown();

  statusLED.updateLEDs(cameraState);
  sleep_cpu();
  // TODO detach interrupts then just enable before sleep?
  //detachInterrupt(digitalPinToInterrupt(RTC_INT));
}



void checkWDTCountdown() {
  //TODO Check if poweringOnWDTCountdown or poweredOnWDTCountdown or poweredOffWDTCountdown has reached 0.
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
    cameraState = CameraState::POWERING_OFF;
  }
}

void checkRegCommunication() {
  if (registers[REG_REQUEST_COMMUNICATION] != 0 &&  millis() - requestCommunicationTime > REQUEST_COMMUNICATION_TIMEOUT_MS) {
    // Timeout for raspberry pi communicating to attiny.
    // TODO Set error flag in I2C register
    registers[REG_REQUEST_COMMUNICATION] = 0;
  }
  if (registers[REG_REQUEST_COMMUNICATION] == 0) {
    // If register is reset the ping pin can be reset also.
    // TODO reset ping pin.
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

  // Check if battery voltage is OK.
  if (lowBatteryCheck && (mainBatteryVoltage > lowBatteryValue || mainBatteryRegUpdate == 0)) {
    return; // Battery is OK.
  }

  cameraState = CameraState::POWERED_OFF;
  // TODO Disable 5V
  
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

    // go to deep sleep.
    // Wait for PIT to wake up again and then check battery. 
  }
}

void waitForBatteryCharged() {
  
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

  // Check if the camera has timedout to power on.
  if (cameraState == CameraState::POWERING_ON && millis() - poweredOnTime > MAX_POWERING_ON_DURATION_MS) {
    writeCameraState(CameraState::POWER_ON_TIMEOUT);
    return;
  }

  // Check if the camera has had enough time to power off.
  if (cameraState == CameraState::POWERING_OFF && millis() - poweringOffTime > POWER_OFF_DELAY_MS) {
    // TODO disable 5V
    writeCameraState(CameraState::POWERED_OFF);
    return;
  }

  // Check if the sleep has timedout
  if (cameraState == CameraState::POWERED_OFF && millis() - poweredOffTime > MAX_POWERED_OFF_DURATION_MS) {
    // TODO error LED for RTC timeout
    powerOnRPi();
    return;
  }
}

void writeCameraState(CameraState newCameraState) {
  if (newCameraState != cameraState) {
    cameraState = newCameraState;
    
    //TODO lastStateUpdateTime = millis();
  }
  registers[REG_CAMERA_STATE] = static_cast<uint8_t>(cameraState);
}


//============================== I2C FUNCTIONS ==============================//
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
  checkMainBattery();
  powerOnRPi();
  // TODO Wake up RP2040 
}

//======================= RASPBERRY_PI FUNCTIONS ============================//
void powerOnRPi() {
  checkMainBattery(); // Will stay in checkMainBattery until battery is good.
  // TODO Enable 5V
  cameraState = CameraState::POWERING_ON;
  poweringOnTime = millis();
}

void poweringOffRPi() {
  cameraState = CameraState::POWERING_OFF;
  poweringOffTime = millis();
}

// request raspberry pi to start up wifi communications.
void startWifiRPi() {
  requestCommunicationTime = millis();
  registers[REG_REQUEST_COMMUNICATION] = 0x01;
  // TODO Drive RPi ping pin to low, how it triggers the RPi to read the registers.
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
  } else if (buttonPressDuration < 3000) {
    statusLED.show();
    if (cameraState == CameraState::POWERED_OFF) {
      powerOnRPi();
    } else {
      startWifiRPi();
    }
    statusLED.updateLEDs(cameraState);
  } else {
    wdt_enable(WDTO_15MS);  // enable the watchdog with shortest available timeout
    while(1) {}
    delay(5000);
  }
}
