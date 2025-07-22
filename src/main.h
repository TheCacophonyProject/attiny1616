#ifndef MAIN_H
#define MAIN_H

#include "CameraState.h"
#include "Pinout.h"
#include "StatusLED.h"
#include "ErrorCodes.h"
#include "timer.h"

void checkMainBattery();
void lowBatteryRegUpdate();
void mainBatteryRegUpdate();
void rtcBatteryRegUpdate();
void checkRegSleep();
void checkCameraState();
void checkRTCBattery();
void writeCameraState(RPiState);
void receiveEvent(int);
void requestEvent();
void error(ErrorCode);
void powerOnPRi();
void processButtonPress();
void buttonWakeUp();
void powerOnRPi();
void requestRPiPowerOff();
void rtcWakeUp();
void powerRPiOffNow();
void checkWakeUpPiReg();
void poweringOffRPi();
void updateLEDs();
void writeErrorFlag(ErrorCode, bool);
void requestPiCommand(uint8_t);
void checkPiCommands();
void checkPiCommsCountdown();
void powerOnRP2040(bool ignoreRebootCheck = false);
void powerOffRP2040();
void regBatteryRTCUpdate();
void regBatteryHVDivUpdate();
void regBatteryLVDivUpdate();
void checkForLowBattery();
void checkRegRP2040PiPowerCtrl();
void checkRP2040State();

#endif // MAIN_H
