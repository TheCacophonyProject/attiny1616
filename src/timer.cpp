#include <Arduino.h>
#include "timer.h"


volatile unsigned long pitTime = 0;

// Setup the RTC_PIT_interrupt to trigger every 1/16 of a second.
void setupPIT() {
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc; // Set clock source to the internal 32.768kHz oscillator
  RTC.PITCTRLA = RTC_PITEN_bm | RTC_PERIOD_CYC2048_gc; // Enable PIT and set period to 1/16 second
  RTC.PITINTCTRL = RTC_PI_bm; // Enable PIT interrupt
}

ISR(RTC_PIT_vect) {
  RTC.PITINTFLAGS = RTC_PI_bm; // Clear interrupt flag, otherwise it will constantly trigger.
  pitTime += 1;
}

unsigned long getPitTime() {
  noInterrupts();
  unsigned long time = pitTime;
  interrupts();
  return time;
}

unsigned long getPitTimeMillis() {
  return millis();
  noInterrupts();
  unsigned long time = pitTime;
  interrupts();
  time = time * 63;   // Multiply by 63 to get milliseconds (1000/16 = 62.5)
  return time;
}