#!/bin/bash

set -e

device=$1

if [[ -z $device ]]; then
  echo "please provide device name. Usage \`install.sh [devicename]\`"
  exit 1
fi

pio run
scp .pio/build/ATtiny1616/firmware.hex $device:
ssh $device "sudo systemctl stop tc2-hat-attiny"
if ! ssh $device "sudo pymcuprog -d attiny1616 -t uart -u /dev/serial0 ping"; then
  ssh $device "sudo systemctl start tc2-hat-attiny"
  echo "Failed to connect to the ATtiny over UART. Nothing was flashed. This could be because:"
  echo "- The UART isn't routed to the ATtiny. On the device run:"
  echo "    sudo pinctrl set 6 op dl; sudo pinctrl set 12 op dl; sudo pinctrl set 14 a0; sudo pinctrl set 15 a0"
  echo "- Another service is using /dev/serial0, usually tc2-hat-comms. Check with \`sudo fuser -v /dev/serial0\`"
  echo "  and stop it with \`sudo systemctl stop tc2-hat-comms\`."
  echo "- The serial console is enabled (console=serial0 in /boot/firmware/cmdline.txt). Run"
  echo "  \`sudo disable-aux-uart\` on the device and reboot."
  exit 1
fi
ssh $device "sudo pymcuprog -d attiny1616 -t uart -u /dev/serial0 erase"
ssh $device "sudo pymcuprog -d attiny1616 -t uart -u /dev/serial0 write -f firmware.hex"
ssh $device "sudo systemctl start tc2-hat-attiny"

echo "Installed on device."
echo "Run this in the tc2-hat-controller so it can be used with this software."
echo 'export ATTINY_MAJOR='$MAJOR_VERSION
echo 'export ATTINY_MINOR='$MINOR_VERSION
echo 'export ATTINY_PATCH='$PATCH_VERSION
echo 'export ATTINY_HASH=0'
