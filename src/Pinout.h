#ifndef PINOUT_H
#define PINOUT_H

//=====PIN DEFINITIONS=====//
#define LED_R           PIN_PA5 
#define LED_G           PIN_PB2
#define LED_B           PIN_PB3
#define EN_5V           PIN_PA2   // Enable the 5V voltage rail.
#define RTC_BAT_SENSE   PIN_PB4   // Connected to the RTC battery. Connected through a diode so will expect a voltage drop from that.
#define MAIN_BAT_SENSE  PIN_PB5   // Connected to the main battery.
#define RTC_ALARM       PIN_PC1   // Connected to the RTC interrupt pin.
#define BUTTON          PIN_PA6   // Connected to the button on the case.
#define PI_POWERED_OFF  PIN_PA7   // Raspberry Pi will drive this pin low when it has finished powering off.
#define PI_SHUTDOWN     PIN_PC3   // Trigger Raspberry Pi to shutdown.
#define EN_RP2040       PIN_PA1   // Drive low to power on rp2040.
#define PI_COMMAND_PIN  PIN_PC0   // Drive low to get the Raspberry Pi to check what commands to run.

#endif