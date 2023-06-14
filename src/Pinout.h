#ifndef PINOUT_H
#define PINOUT_H

//=====PIN DEFINITIONS=====//
#define LED_R           PIN_PA5 
#define LED_G           PIN_PA3
#define LED_B           PIN_PA4
#define EN_5V           PIN_PA2   // Enable the 5V voltage rail.
#define RTC_BAT_SENSE   PIN_PB4   // Connected to the RTC battery. Connected through a diode so will expect a voltage drop from that.
#define MAIN_BAT_SENSE  PIN_PB5   // Connected to the main battery.
#define RTC_INT         PIN_PC1   // Connected to the RTC interrupt pin.
#define BUTTON          PIN_PA7   // Connected to the button on the case.

#endif