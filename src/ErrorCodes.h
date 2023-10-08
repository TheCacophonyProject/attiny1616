#ifndef ERRORCODE_H
#define ERRORCODE_H


enum class ErrorCode {
  POWER_ON_FAILED               = 0x02,  // Timeout in camera powering on.
  WATCHDOG_TIMEOUT              = 0x03,  // Watchdog was triggered.
  INVALID_CAMERA_STATE          = 0x04,  // Camera state was set to an invalid value.
  WRITE_TO_READ_ONLY            = 0x05,  // Attempted to write to read only bits.
  LOW_BATTERY_LEVEL_SET_TOO_LOW = 0x06,  // Low battery level was attempted to be set too low.
  INVALID_REG_ADDRESS           = 0x07,  // Attempted to write to an invalid register address.
  INVALID_ERROR_CODE            = 0x08,  // Invalid error code.
  PI_COMMAND_TIMEDOUT           = 0x09,  // No ping response.
  RTC_TIMEOUT                   = 0x0A   // RTC timeout.
};


#endif