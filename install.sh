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
ssh $device "sudo pymcuprog -d attiny1616 -t uart -u /dev/serial0 erase"
ssh $device "sudo pymcuprog -d attiny1616 -t uart -u /dev/serial0 write -f firmware.hex"
ssh $device "sudo systemctl start tc2-hat-attiny"

echo "Installed on device."
echo "Run this in the tc2-hat-controller so it can be used with this software."
echo 'export ATTINY_MAJOR='$MAJOR_VERSION
echo 'export ATTINY_MINOR='$MINOR_VERSION
echo 'export ATTINY_PATCH='$PATCH_VERSION
echo 'export ATTINY_HASH=0'
