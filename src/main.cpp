#include <Arduino.h>
#include <avr/sleep.h>
#include <Wire.h>
#include "main.h"
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <timer.h>
#include <avr/io.h>

//=====DEFINITIONS=====//
#define BATTERY_HYSTERESIS 10
#define BUTTON_LONG_PRESS_DURATION 1200

//=====I2C DEFINITIONS=====//
#define I2C_ADDRESS 0x25
#define REG_LEN     0x24

// Check registers.md for details on registers functions. //TODO, update registers.md
#define REG_TYPE                 0x00
#define REG_MAJOR_VERSION        0x01
#define REG_CAMERA_STATE         0x02
#define REG_CAMERA_CONNECTION    0x03
#define REG_PI_COMMANDS          0x04
#define REG_RP2040_PI_POWER_CTRL 0x05
#define REG_AUX_TERMINAL         0x06
#define REG_TC2_AGENT_READY      0x07
#define REG_MINOR_VERSION        0x08
#define REG_FLASH_ERRORS         0x09
#define REG_CLEAR_ERRORS         0x0A
#define REG_PATCH_VERSION        0x0B
#define REG_BOOT_DURATION_1      0x0C
#define REG_BOOT_DURATION_2      0x0D
#define REG_WDT_RP2040           0x0E

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

#define WDT_DURATION WDTO_2S

uint8_t writeMasks[REG_LEN] = {}; // Should all be initialised to 0xFF

//=====GLOBAL VARIABLES=====//
volatile uint8_t registers[REG_LEN] = {0};
volatile uint8_t registerAddress = 0;
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
// This will note how long it took the camera to power on.
volatile unsigned long bootDuration = 0;

// Time from getPitTimeMillis() of the last time the ATtiny was communicated with the RPi.
// If it has not been communicated for over PI_COMMS_INTERVAL it will request comms from the RPi.
volatile unsigned long lastPiCommsTime = 0;
//#define WDT_RESET_INTERVAL 30000
#define PI_COMMS_INTERVAL 100000

// WDT timer for RP2040 (15 minutes)
volatile unsigned long rp2040WdtResetTime = 0;
#define RP2040_WDT_RESET_INTERVAL 900000

// Time from getPitTimeMillis() of when the RP2040 was turned off for a reset
// After a set amount of time (RP2040_POWER_OFF_RESET_DURATION) the RP2040 should be powered back on.
volatile unsigned long rp2040PoweringOffResetTime = 0;
#define RP2040_POWER_OFF_RESET_DURATION 5000

// Time from getPitTimeMillis() of when the camera asked to be turned off.
// After a set about of time in ms the camera is defined from POWER_OFF_DELAY_MS it will then power off the camera.
volatile unsigned long poweringOffTime = 0;
#define POWER_OFF_DELAY_MS 60000

// Timer to check that the ATtiny is woken up by the RTC_ALARM interrupt after at least 24 hours. 
// If reaches MAX_POWERED_OFF_DURATION_MS then the camera is reset and a SLEEP_ error flag is set in the I2C register
volatile unsigned long poweredOffTime = 0; 
#define MAX_POWERED_OFF_DURATION_MS 129'600'000 // 36 hours

// Time from getPitTimeMillis() of when the ATtiny requested communications from the Raspberry Pi.
// If the Raspberry Pi is not hear from after PI_COMMAND_TIMEOUT a error flag will be set in the I2C register.
//volatile unsigned long pingPiTime = 0;
volatile unsigned long piCommandRequestTime = 0;
#define PI_COMMAND_TIMEOUT 5000

// Timer to check if the button is being pressed in quick succession.
volatile unsigned long previousButtonPressTime = 0;
volatile uint8_t quickButtonPressCount = 0;
#define BUTTON_QUICK_PRESS_DURATION 500


volatile unsigned long poweredOnTime = 0;

volatile RPiState cameraState = RPiState::POWERING_ON;
StatusLED statusLED;

volatile bool quickFlash = false; // A debugging tool to quickly flash the LED.

void setup() {
  wdt_disable();

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
  digitalWrite(PI_COMMAND_PIN, LOW);
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
  writeMasks[REG_MAJOR_VERSION] = 0x00;
  writeMasks[REG_MINOR_VERSION] = 0x00;
  writeMasks[REG_PATCH_VERSION] = 0x00;
  writeMasks[REG_TYPE] = 0x00;
  writeMasks[REG_BATTERY_CHECK_CTRL] = 0x03; // Only allow writing to bits to turn on or off battery check.
  writeMasks[REG_BATTERY_LV_DIV_VAL1] = 0x01 << 7;
  writeMasks[REG_BATTERY_LV_DIV_VAL2] = 0x00;
  writeMasks[REG_BATTERY_HV_DIV_VAL1] = 0x01 << 7;
  writeMasks[REG_BATTERY_HV_DIV_VAL2] = 0x00;
  writeMasks[REG_BATTERY_RTC_VAL1] = 0x01 << 7;
  writeMasks[REG_BATTERY_RTC_VAL2] = 0x00;
  writeMasks[REG_BOOT_DURATION_1] = 0x00;
  writeMasks[REG_BOOT_DURATION_2] = 0x00;

  // Write I2C initial register values.
  registers[REG_TYPE] = 0xCA;
  // When running `pio run` MAJOR, MINOR, and PATCH are set using environment variables.
  registers[REG_MAJOR_VERSION] = MAJOR_VERSION; // If getting error set environment variable with `export MAJOR_VERSION=0`
  registers[REG_MINOR_VERSION] = MINOR_VERSION; // If getting error set environment variable with `export MINOR_VERSION=0`
  registers[REG_PATCH_VERSION] = PATCH_VERSION; // If getting error set environment variable with `export PATCH_VERSION=0`

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
  powerOnRP2040();

  /*
  // Checking for the reason for booting doesn't seam to work.
  if (RSTCTRL.RSTFR & RSTCTRL_WDRF_bm) {
    RSTCTRL.RSTFR = RSTCTRL_WDRF_bm; // Clear the watchdog reset flag
    writeErrorFlag(ErrorCode::WDT_TRIGGERED, true);
    delay(10000);
  }

  if (RSTCTRL.RSTFR & RSTCTRL_BORF_bm) {
    RSTCTRL.RSTFR = RSTCTRL_BORF_bm; // Clear the brown out reset flag
    writeErrorFlag(ErrorCode::BOR_TRIGGERED, true);
    delay(10000);
  }
  */
  wdt_reset();
  wdt_enable(WDT_DURATION);
}

volatile uint8_t sleepMode = SLEEP_MODE_IDLE;

void updateSleepMode() {
  uint8_t newSleepMode = statusLED.isOn() ? SLEEP_MODE_IDLE : SLEEP_MODE_PWR_DOWN;

  if (newSleepMode != sleepMode) {
    noInterrupts();

    // Check if the I2C is currently active
    if (TWI0.SSTATUS & (TWI_APIF_bm | TWI_DIF_bm)) {
      interrupts();
      return;
    }

    sleepMode = newSleepMode;

    sleep_disable();
    set_sleep_mode(sleepMode);
    sleep_enable();

    interrupts();
  }
}

// The ATtiny gets into a state where it will hold the I2C SDA low. Not too sure on the reasons for this but this is a bit of a hack to recover from it. //TODO Find out why this happens.
volatile uint8_t sdaLowCount = 0;

void checkHoldingSDALow() {
  if (digitalRead(I2C_SDA) == LOW) {
    sdaLowCount++;
  } else {
    sdaLowCount = 0;
  }
  if (sdaLowCount > 32) { // Will check about 16 times a second when called in the main loop so will take 2 second to recover.
    sdaLowCount = 0;
    writeErrorFlag(ErrorCode::HOLDING_SDA_LOW, true);
    noInterrupts();
    Wire.end();
    Wire.begin(I2C_ADDRESS);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);
    interrupts();
  }
}

void loop() {
  wdt_reset();
  if (quickFlash) {
    statusLED.writeColor(255, 255, 255);
    delay(100);
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
  checkRP2040State();

  // Only power off the RP2040 when the RPi has been power off first.
  if (rp2040ReadyToPowerOff && registers[REG_CAMERA_STATE] == uint8_t(RPiState::POWERED_OFF)) {
    powerOffRP2040();
    rp2040ReadyToPowerOff = false;
    //TODO Check that this won't power off the RP2040 when unwanted, might need more logic around the rp2040ReadyToPowerOff variable.
  }

  checkHoldingSDALow();

  updateLEDs();
  updateSleepMode();
  //delay(50);
  sleep_mode();
}

void updateLEDs() {
  bool auxEnabled = registers[REG_AUX_TERMINAL] & 0x01;
  statusLED.updateLEDs(cameraState, static_cast<CameraConnectionState>(registers[REG_CAMERA_CONNECTION]), auxEnabled);
}

void writeErrorFlag(ErrorCode errorCode, bool flash = true) {
  flash = flash && (registers[REG_FLASH_ERRORS] & 0x01);
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
  if (cameraState != RPiState::POWERED_ON) {
    return;
  }
  if (getPitTimeMillis() - lastPiCommsTime > PI_COMMS_INTERVAL) {
    requestPiCommand(WRITE_CAMERA_STATE_FLAG);
  }
}

void checkPiCommands() {
  // Check if the Raspberry Pi is responding to command requests
  if (registers[REG_PI_COMMANDS] != 0  &&
      getPitTimeMillis() - piCommandRequestTime > PI_COMMAND_TIMEOUT &&
      cameraState == RPiState::POWERED_ON &&
      getPitTimeMillis() - poweredOnTime > PI_COMMAND_TIMEOUT) {
    writeErrorFlag(ErrorCode::PI_COMMAND_TIMEDOUT);
    registers[REG_PI_COMMANDS] = 0;
    // TODO, should we be restarting the Raspberry Pi here?
  }

  // Drive PI_COMMAND_PIN low if there is a command the RPi should be checking.
  if (registers[REG_PI_COMMANDS] == 0 && cameraState == RPiState::POWERED_ON) {
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
  // Make a reading first, this is because the RTC battery first reading is often bad.
  analogRead(RTC_BAT_SENSE);
  delayMicroseconds(200);
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
  if (registers[REG_CAMERA_STATE] <= static_cast<uint8_t>(RPiState::REBOOTING)) {
    writeCameraState(static_cast<RPiState>(registers[REG_CAMERA_STATE]));
  } else {
    writeErrorFlag(ErrorCode::INVALID_CAMERA_STATE);
  }

  // Check if the camera has just finished booting. If so set the boot duration.
  if (cameraState == RPiState::POWERED_ON && registers[REG_BOOT_DURATION_1] == 0 && registers[REG_BOOT_DURATION_2] == 0) {
    uint16_t powerOnDurationSeconds = (getPitTimeMillis() - poweringOnTime)/1000;
    registers[REG_BOOT_DURATION_1] = powerOnDurationSeconds >> 8;
    registers[REG_BOOT_DURATION_2] = powerOnDurationSeconds & 0xFF;
  }

  // Check if the camera has had a power on timeout.
  if (cameraState == RPiState::POWERING_ON && getPitTimeMillis() - poweringOnTime > MAX_POWERING_ON_DURATION_MS) {
    powerRPiOffNow();
    delay(1000);
    powerOnRPi();
    //writeCameraState(CameraState::POWER_ON_TIMEOUT);  //TODO do we need this state?
    writeErrorFlag(ErrorCode::POWER_ON_FAILED);
    return;
  }

  // Check if the camera has powered off.
  if (cameraState == RPiState::POWERING_OFF) {
    // gpio-poweroff in config.txt for the RPi will drive a pin low when it powers off.
    // TODO Check if you need to wait here for a second or two for the flash to finish writing.
    if (digitalRead(PI_POWERED_OFF) == LOW) {
      powerRPiOffNow();
      statusLED.off();
    }
  }

  if ((cameraState == RPiState::POWERED_ON || cameraState == RPiState::REBOOTING) && digitalRead(PI_POWERED_OFF) == LOW) {
    powerRPiOffNow();
    delay(1000);
    powerOnRPi();
  }

  //TODO Check if PI_POWERED_OFF pin is low and not in the proper state for it.

  // TODO replace this with a shutdown timeout, as power off will be triggered from the PI_POWERED_OFF pin.
  // Check if the camera has had enough time to power off.
  if (cameraState == RPiState::POWERING_OFF && getPitTimeMillis() - poweringOffTime > POWER_OFF_DELAY_MS) {
    powerRPiOffNow();
    return;
  }

  // Check if the device has been in sleep for too long.
  if (cameraState == RPiState::POWERED_OFF && getPitTimeMillis() - poweredOffTime > MAX_POWERED_OFF_DURATION_MS) {
    writeErrorFlag(ErrorCode::RTC_TIMEOUT);
    powerOnRPi();
    return;
  }
}

void writeCameraState(RPiState newCameraState) {
  if (newCameraState != cameraState) {
    // This is to skip setting the state to POWERED_ON on when the camera is powering off.
    // In the time that the camera takes to power off the tc2-hat-attiny service might set the state to POWERED_ON
    // because it doesn't know the camera is powering off.
    if (cameraState  == RPiState::POWERING_OFF &&
      newCameraState == RPiState::POWERED_ON &&
      poweringOffTime - getPitTimeMillis() > 10000) {
      return;
    }

    cameraState = newCameraState;
    updateLEDs();
    if (cameraState == RPiState::POWERED_ON) {
      poweredOnTime = getPitTimeMillis();
    }
    if (cameraState == RPiState::POWERING_OFF) {
      poweringOffTime = getPitTimeMillis();
    }
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

bool isBitSet(uint8_t reg, uint8_t bitPos) {
    return (reg & (1 << bitPos)) != 0;
}

void checkRegRP2040PiPowerCtrl() {
  if (isBitSet(registers[REG_RP2040_PI_POWER_CTRL], 0)) {
    // RP2040 requires the RPi to be powered on. (RPi will read this so it knows to stay on, //TODO)
    // If RPi is off, then power it on.
    if (cameraState == RPiState::POWERED_OFF) {
      powerOnRPi();
    }
  } // If this bit is not set it is up to the RPi to decide when to power itself off.

  if (isBitSet(registers[REG_RP2040_PI_POWER_CTRL], 1)) {
    // Keep RP2040 on while RPi is powered on
    if (cameraState == RPiState::POWERED_ON) {
      powerOnRP2040();
    } else {
      powerOffRP2040();
    }
  } else {
    // If bit is not set the RP2040 should be powered on.
    powerOnRP2040();
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
uint16_t crcCalc(const uint8_t *data, size_t length) {
    uint16_t crc = 0x1D0F;
    for (size_t i = 0; i < length; i++) {
        crc = crc ^ ((uint16_t)data[i] << 8);
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

void receiveEvent(int howMany) {
  #define BUFFER_SIZE 10
  uint8_t buffer[BUFFER_SIZE];
  // Read all incoming data into buffer
  int size = 0;
  while (Wire.available() && size < BUFFER_SIZE) {
    buffer[size++] = uint8_t(Wire.read());
  }
  
  // Ignore empty data
  if (size == 0){
    // Clear data
    while (Wire.available()) {
      Wire.read();
    }
    return;
  }

  if (size < 3) {
    //writeErrorFlag(ErrorCode::BAD_I2C_LENGTH_SMALL);
    // Clear data
    while (Wire.available()) {
      Wire.read();
    }
    return;
  }

  if (size > 4) {    
    // writeErrorFlag(ErrorCode::BAD_I2C_LENGTH_BIG);
    // Clear data
    while (Wire.available()) {
      Wire.read();
    }
    return;
  }

  if (Wire.available()) {
    //writeErrorFlag(ErrorCode::BAD_I2C);
    // Clear data
    while (Wire.available()) {
      Wire.read();
    }
    return;
  }
  
  uint16_t receivedCRC = ((uint16_t)buffer[size - 2] << 8) | buffer[size - 1];
  uint16_t calculatedCRC = crcCalc(buffer, size - 2);
  if (receivedCRC != calculatedCRC) {
    //writeErrorFlag(ErrorCode::CRC_ERROR);
    return;
  }

  // Process data if CRC is valid
  //for (int i = 0; i < index - 2; ++i) {
  uint8_t address = buffer[0];
  //uint8_t address = Wire.read();
  // Prevent from writing/reading to registers outside of the range.
  if (address >= REG_LEN) {
    while(Wire.available()) {
      Wire.read();
      writeErrorFlag(ErrorCode::INVALID_REG_ADDRESS);
      return;
    }
  }
  
  // WDT for RPi
  if (address == REG_CAMERA_STATE) {
    lastPiCommsTime = getPitTimeMillis();
  }

  // WDT for RP2040
  if (address == REG_WDT_RP2040) {
    rp2040WdtResetTime = getPitTimeMillis();
  }

  registerAddress = address;
  
  // Write data to register.
  if (size == 4) {
    // Disable the low battery check if changing the low battery value.
    if (registerAddress == REG_BATTERY_LOW_VAL2 || registerAddress == REG_BATTERY_LOW_VAL1) {
      registers[REG_BATTERY_CHECK_CTRL] = 0;
    }

    // Clear errors
    if (registerAddress == REG_CLEAR_ERRORS && buffer[1] == 0x01) {
      registers[REG_ERRORS1] = 0x00;
      registers[REG_ERRORS2] = 0x00;
      registers[REG_ERRORS3] = 0x00;
      registers[REG_ERRORS4] = 0x00;
      return;
    }

    uint8_t data = buffer[1];
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

// Best to just read one register at a time, this is explained further in this example.
// https://github.com/SpenceKonde/megaTinyCore/blob/master/megaavr/libraries/Wire/examples/register_model/register_model.ino
// TODO Set it up so it 
// TODO use Wire.getBytesRead();
void requestEvent() {
  uint8_t data[] = {registers[registerAddress]};
  uint16_t crc = crcCalc(data, 1);
  uint8_t payload[3] = {registers[registerAddress], crc >> 8, crc & 0xff};
  Wire.write(payload, 3);
}

//=============================ISR DEFINITIONS==================================//
// Function attached to the pin connected to the alarm pin on the RTC.
void rtcWakeUp() {
  registers[REG_RP2040_PI_POWER_CTRL] = 0;
  if (cameraState == RPiState::POWERED_OFF) {
    checkForLowBattery();
    powerOnRP2040();
  }
}

//======================= RP2040 FUNCTIONS ============================//

RP2040State rp2040State = RP2040State::POWERED_OFF;

void checkRP2040State() {
  switch (rp2040State) {
    case RP2040State::POWERED_OFF:
      break;
    case RP2040State::POWERED_ON:
      // Check if the WDT has triggered.
      if (getPitTimeMillis() - rp2040WdtResetTime > RP2040_WDT_RESET_INTERVAL) {
        powerOffRP2040();
        rp2040State = RP2040State::WDT_REBOOT;
        rp2040WdtResetTime = getPitTimeMillis();
        // Write an error.
        writeErrorFlag(ErrorCode::RP2040_WDT_TIMEOUT);
      }
      break;
    case RP2040State::WDT_REBOOT:
      // Check if enough time has passed to turn the RP2040 back on.  
      if (getPitTimeMillis() - rp2040WdtResetTime > RP2040_POWER_OFF_RESET_DURATION) {
        powerOnRP2040(false);
      }
      break;
  }
}

void powerOnRP2040(bool wdtRebootCheck) {
  if (wdtRebootCheck && rp2040State == RP2040State::WDT_REBOOT) {
    // In the process of rebooting from a WDT failure, don't turn the RP2040 back on.
    return;
  }
  if (rp2040State == RP2040State::POWERED_ON) {
    // Already powered on, don't need to do anything.
    return;
  }
  rp2040WdtResetTime = getPitTimeMillis();  // Reset the WDT for the RP2040.
  rp2040ReadyToPowerOff = false;
  checkForLowBattery(); // Will stay in checkForLowBattery until battery is good.
  digitalWrite(EN_RP2040, LOW);
  rp2040State = RP2040State::POWERED_ON;
}

void powerOffRP2040() {
  if (rp2040State == RP2040State::POWERED_OFF) {
    // Already powered off, don't need to do anything.
    return;
  }
  rp2040State = RP2040State::POWERED_OFF;
  digitalWrite(EN_RP2040, HIGH);
}

//======================= RASPBERRY_PI FUNCTIONS ============================//
void powerOnRPi() {
  checkForLowBattery(); // Will stay in checkForLowBattery until battery is good.
  digitalWrite(EN_5V, HIGH);
  powerOnRP2040();
  writeCameraState(RPiState::POWERING_ON);
  poweringOnTime = getPitTimeMillis();
}

void powerRPiOffNow() {
  poweredOffTime = getPitTimeMillis();  //TODO Tets this
  digitalWrite(EN_5V, LOW);
  writeCameraState(RPiState::POWERED_OFF);
  registers[REG_TC2_AGENT_READY] = 0;
  registers[REG_PI_COMMANDS] = 0;
  registers[REG_CAMERA_CONNECTION] = 0;
  registers[REG_BOOT_DURATION_1] = 0;
  registers[REG_BOOT_DURATION_2] = 0;
}

void requestRPiPowerOff() {
  digitalWrite(PI_SHUTDOWN, LOW);
  writeCameraState(RPiState::POWERING_OFF);
  registers[REG_TC2_AGENT_READY] = 0;
  poweringOffTime = getPitTimeMillis();
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
    wdt_reset();
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
  }
  if (buttonPressDuration < BUTTON_LONG_PRESS_DURATION) {
    // Button short press
    if (statusLED.isOn()) {
      statusLED.off();
    } else {
      statusLED.show();
    }
    updateLEDs();
  } else {
    // Button long press
    if (cameraState == RPiState::POWERED_OFF) {
      powerOnRPi();
      registers[REG_RP2040_PI_POWER_CTRL] = 0;
    } else {
      powerOffRP2040();
      requestRPiPowerOff();
      delay(200);
      registers[REG_RP2040_PI_POWER_CTRL] = 0x02;
    }
    statusLED.show();
  }
  if (quickButtonPressCount > 20) {
    quickButtonPressCount = 0;
    statusLED.writeColor(0xFFFFFF);
    wdt_disable();
    delay(3000);
    requestPiCommand(TOGGLE_AUX_TERMINAL_FLAG);
    wdt_reset();
    wdt_enable(WDT_DURATION);
    //while (true) {
    //  CCP = 0xD8;
    //  RSTCTRL.SWRR |= RSTCTRL_SWRE_bm;  // Writing this bit will reset the ATtiny1616
    //}
  }
  buttonPressDuration = 0;
}
