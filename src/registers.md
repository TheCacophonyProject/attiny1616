## I2C Register Map

### Register 0x00 - REG_VERSION
Coded [x]
Tested []
- **Read-only**
- Stores the firmware version.

### Register 0x01 - REG_CAMERA_STATE
Coded [x]
Tested []
- **Read-write**
- 0x00: Waiting for camera to power on.
- 0x01: Camera is powered on.
- 0x02: Camera is powered off.
- 0x03: Camera is connected to a network.
- 0x04: Power on timeout, took too long for the camera to power on and notify the ATTiny.

### Register 0x02 - REG_TRIGGER_SLEEP
Coded [x]
Tested []
- **Read-write**
- Bit 0: Write high to trigger sleep sequence.
- Bit 1-7: Reserved for future use.

### Register 0x03 - REG_TRIGGER_SLEEP_DELAY
Coded [x]
Tested []
- **Read-write**
- Bit 0-7: Delay in seconds for trigger sleep to powering off 5V.

### Register 0x04 - REG_CAMERA_WAKE_UP
Coded []
Tested []
- **Read-write**
- Bit 0: Write high to enable 5V to wake up camera.
- Bit 1-7: Reserved for future use.

### Register 0x05 - REG_SLEEP_DURATION_MSB
Coded [x]
Tested []
- **Read-write**
- Bit 0-7: MSB of sleep duration in seconds

### Register 0x06 - REG_SLEEP_DURATION_MID
Coded [x]
Tested []
- **Read-write**
- Bit 0-7: MID of sleep duration in seconds

### Register 0x07 - REG_SLEEP_DURATION_LSB
Coded [x]
Tested []
- **Read-write**
- Bit 0-7: LSB of sleep duration in seconds

### Register 0x08 REG_BATTERY1
Coded [x]
Tested []
- Bit 7: **Read-write** Write high to trigger voltage reading, will be set low after voltage reading has been taken. The raw 10 bit reading will be written.
- Bit 6-2: Reserved for future use.
- Bit 1-0: MSB of 10 bit voltage reading.

### Register 0x09 - REG_BATTERY2
Coded [x]
Tested []
- **Read only**
- Bit 7-0: LSB of 10 bit voltage reading

### Register 0x0A - REG_BATTERY3
Coded [x]
Tested []
- Bit 7-3: Reserved for future use.
- Bit 2: **Read-only** High if low battery check enabled, low if disabled.
- Bit 1: **Read-write** Write high to disable low battery check.
- Bit 0: **Read-write** Write high to enable low battery check. 

### Register 0x0B - REG_BATTERY4
Coded []
Tested []
- **Read-write**
- Bit 7-2: Reserved for future use.
- Bit 1-0: MSB of low battery voltage

### Register 0x0C - REG_BATTERY5
Coded []
Tested []
- **Read-write**
- Bit 7-0: LSB of low battery voltage.

### Register 0x0D REG_RTC_BATTERY1
Coded [x]
Tested []
- Bit 7: **Read-write** Write high to trigger voltage reading of RTC battery, will be set low after voltage reading has been taken and the raw 10 bit reading will be written.
- Bit 6-2: Reserved for future use.
- Bit 1-0: **Read only** MSB of 10 bit voltage reading.

### Register 0x0E - REG_RTC_BATTERY2
Coded [x]
Tested []
- **Read only**
- Bit 7-0: LSB of 10 bit RTC battery voltage reading.

### Register 0x0F - REG_RESET_WATCHDOG
Coded []
Tested []
- **Read-write**
- Bit 7-1: Reserved for future use.
- Bit 0: Write high to reset watchdog.



