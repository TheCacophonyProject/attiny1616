#ifndef MAIN_H
#define MAIN_H

#include "CameraState.h"
#include "Pinout.h"
#include "StatusLED.h"
#include "ErrorCodes.h"

void checkMainBattery();
void lowBatteryRegUpdate();
void mainBatteryRegUpdate();
void rtcBatteryRegUpdate();
void checkRegSleep();
void checkCameraState();
void checkRTCBattery();
void writeCameraState(CameraState);
void receiveEvent(int);
void requestEvent();
void error(ErrorCode);
void wdtCountdown();
void checkWDTCountdown();
void powerOnPRi();
void processButtonPress();
void buttonWakeUp();
void powerOnRPi();
void powerOffRPi();
void rtcWakeUp();
void setupPIT();
void checkPing();
void powerRPiOffNow();
void wdtRegUpdate();
void checkWakeUpReg();
void poweringOffRPi();

#endif // MAIN_H
