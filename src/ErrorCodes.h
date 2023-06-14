#ifndef ERRORCODE_H
#define ERRORCODE_H


enum class ErrorCode {
  POWER_ON_FAILED               = 0x00,  // Timeout in camera powering on.
  WATCHDOG_TIMEOUT              = 0x01,  // Watchdog was triggered.
  INVALID_CAMERA_STATE          = 0x02,  // Camera state was set to an invalid value.
  WRITE_TO_READ_ONLY            = 0x03,  // Attempted to write to read only bits.
  LOW_BATTERY_LEVEL_SET_TOO_LOW = 0x04,  // Low battery level was attempted to be set too low.
  INVALID_REG_ADDRESS           = 0x05,  // Attempted to write to an invalid register address.
  INVALID_ERROR_CODE            = 0x06   // Invalid error code.
};


#endif