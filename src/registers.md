# I2C Register Map

## General Control Registers

### Register 0x00 - REG_TYPE
Coded [x]
Tested []
**Description** This register has the value `0xCA`. It can be read to check that the I2C comms are working to the ATtiny1616.
- **Read-only**
- Bit Details:
  - 7-0: `0xCA`

### Register 0x01 - REG_VERSION
Coded [x]
Tested []
**Description** This register has the version number of the firmware for the ATtiny1616. When first connecting to the ATtiny (from RP2040 or RPi) this should be checked against to make sure that it is the same version that the host is expecting as no compatibility between versions are guaranteed.
- **Read-only**
- Bit Details:
  - 7-0: Version of software running on the ATtiny.

### Register 0x02 - REG_CAMERA_STATE
Coded [x]
Tested []
- **Read-write**
- Bit Details:
  - 7-0: Value of `CameraState`. See `CameraState.h` for details

### Register 0x03 - REG_CAMERA_CONNECTION
Coded [x]
Tested []
- **Read-write**
- Bit Details:
  - 7-0: Value of `CameraConnectionState` in `CameraState.h` for details
- See 

### Register 0x04 - REG_PI_COMMANDS
Coded [x]
Tested []
- **Read-write**
- Bit 7-4: Reserved.
- Bit 3: POWER_OFF_RPI, request RPi to power off.
state register.
- Bit 2: ENABLE_WIFI_FLAG, request RPi to enable it's wifi or hotspot.
- Bit 1: READ_ERRORS_FLAG, request RPi to read the errors from the ATtiny.
- Bit 0: WRITE_CAMERA_STATE_FLAG, request RPi to update the camera 

### Register 0x05 - REG_RP2040_PI_POWER_CTRL
**Description**: This register controls the power states between the RP2040 and the Raspberry Pi. It is used to indicate whether the RP2040 requires the RPi to be powered on and whether the RP2040 itself needs to remain powered on.

The RP2040 will update this register on events like, end/start of recording window, CPTV uploading. 

The ATtiny will set them both to 0 on camera wake up events.

- **Read-write**
- Bit Details:
  - Bit 7-2: Reserved.
  - Bit 1 (RP2040 Power Status):
    - 0 (default): RP2040 needs to remain powered on.
    - 1: RP2040 can be powered off. (The ATtiny will only power off the RP2040 after the RPi has powered off itself, until then the RP2040 should keep on sending frames to the RPi for the camera preview)
  - Bit 0 (RPi Power Request):
    - 0 (default): RP2040 does not require the RPi to be powered on.
    - 1: RP2040 requires the RPi to be powered on. (RPi will read this so it knows to stay on)  

### Register 0x07 - REG_TC2_AGENT_READY
Coded  []
Tested []
- Bit Details:
  - Bit 7-1: Reserved.
  - Bit 0: 1 = tc2-agent running. 0 = tc2-agent not running.

## Battery Voltage Registers

### Register 0x10 - REG_BATTERY_CHECK_CTRL
Coded []
Tested []
- Bit Details:
  - Bit 7-3: Reserved.
  - Bit 2: Read only. 0 = low battery check disabled, 1 = low battery check enabled.
  - Bit 1: Disable low battery check
  - Bit 0: Enable low battery check

### Register 0x11 - REG_BATTERY_LOW_VAL1
Coded []
Tested []
- **Read-write**
- Bit Details:
  - Bit 7: 0 = HV battery reading, 1 = LV battery reading.
  - Bit 6-2: Reserved.
  - Bit 1-0: MSB of low battery voltage.

### Register 0x12 - REG_BATTERY_LOW_VAL2 
Coded []
Tested []
- **Read-write**
- Bit Details:
  - Bit 7-0: LSB of low battery voltage.

### Register 0x13 - REG_BATTERY_LV_DIV_VAL1
**Description** Battery reading with the low voltage divider, used for reading the voltage of low voltage batteries, up to //TODO (14V?).
Coded []
Tested []
- **Read-write**
- Bit Details:
  - Bit 7: **Read-write** Write high to trigger voltage reading of LV battery, will be set low after voltage reading has been taken and the raw 10 bit reading will be written.
  - Bit 6-2: Reserved for future use.
  - Bit 1-0: **Read only** MSB of 10 bit voltage reading.

### Register 0x14 - REG_BATTERY_LV_DIV_VAL2 
Coded []
Tested []
- **Read-write**
- Bit Details:
  - Bit 7-0: LSB of low battery voltage.

### Register 0x15 - REG_BATTERY_HV_DIV_VAL1
**Description** Battery reading with the high voltage divider, used for reading the voltage of high voltage batteries, up to 42V.
Coded []
Tested []
- **Read-write**
- Bit Details:
  - Bit 7: **Read-write** Write high to trigger voltage reading of HV battery, will be set low after voltage reading has been taken and the raw 10 bit reading will be written.
  - Bit 6-2: Reserved for future use.
  - Bit 1-0: **Read only** MSB of 10 bit voltage reading.

### Register 0x16 - REG_BATTERY_HV_DIV_VAL2 
Coded []
Tested []
- **Read-write**
- Bit Details:
  - Bit 7-0: LSB of high battery voltage.

### Register 0x17 - REG_BATTERY_RTC_VAL1
Coded []
Tested []
- **Read-write**
- Bit Details:
  - Bit 7: **Read-write** Write high to trigger voltage reading of RTC battery, will be set low after voltage reading has been taken and the raw 10 bit reading will be written.
  - Bit 6-2: Reserved for future use.
  - Bit 1-0: **Read only** MSB of 10 bit voltage reading.

### Register 0x18 - REG_BATTERY_RTC_VAL2 
Coded []
Tested []
- **Read-write**
- Bit Details:
  - Bit 7-0: LSB of low battery voltage.

## Error registers

### Register 0x20
- See ErrorCodes.h for list of errors.
