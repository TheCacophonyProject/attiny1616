#ifndef CAMERASTATE_H
#define CAMERASTATE_H

enum class CameraState {
  POWERING_ON          = 0x00,  // Waiting to hear from camera.
  POWERED_ON           = 0x01,  // Heard from camera.
  POWERED_OFF          = 0x02,  // Camera is powered off.
  CONNECTED_TO_NETWORK = 0x03,  // Camera is connected to a network.
  POWER_ON_TIMEOUT     = 0x04   // Didn't hear from the camera after x amount of time after it being powered on.
};

#endif