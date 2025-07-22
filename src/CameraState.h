#ifndef CAMERA_STATE_H
#define CAMERA_STATE_H

enum class RPiState {
  POWERING_ON          = 0x00,  // Waiting to hear from camera.
  POWERED_ON           = 0x01,  // Heard from camera.
  POWERING_OFF         = 0x02,  // Waiting to power off the Raspberry Pi.
  POWERED_OFF          = 0x03,  // Camera is powered off.
  POWER_ON_TIMEOUT     = 0x04,  // Didn't hear from the camera after x amount of time after it being powered on.
  REBOOTING            = 0x05,  // Raspberry Pi is rebooting.
};

enum class CameraConnectionState {
  NO_CONNECTION         = 0x00, // blue
  CONNECTED_TO_WIFI     = 0x01, // green
  HOSTING_HOTSPOT       = 0x02, // yellow
  WIFI_SETUP            = 0x03, // pulsing green
  HOTSPOT_SETUP         = 0x04, // pulsing yellow
};

enum class RP2040State {
  POWERED_ON            = 0x00,
  POWERED_OFF           = 0x01,
  WDT_REBOOT            = 0x02, // The WDT got triggered so the RP2040 needs to be rebooted.
};

#endif