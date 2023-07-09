# Testing for ATtiny1616
## Normal states
[] Red when first powered on.

[] Blue LED for on but not connected to/hosting wifi.

[] Green LED for on and connected to/hosting wifi.

[] Low power sleep between main loops, waking up from PIT interrupt

[] Power on RPi when RTC alarm triggers.

[] Power off RPi after 30 seconds when requested.

[] Long button press restart ATtiny1616

[] Short button press trigger RPi to enable wifi again/wake up RPi.

### Extra 
[] Power off RPi with low battery voltage.

[] Power on RPi when battery voltage is normal again.

[] Read battery voltage.

[] Read RTC battery voltage.



### Error states, 
[] Each of these errors triggering a flag in the I2C register.
[] Each having a unique LED flash sequence.
[] Wake up RPi when RTC alarm didn't trigger for 24 hours.
 
