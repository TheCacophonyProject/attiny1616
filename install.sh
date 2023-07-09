#!/bin/bash
pio run
scp .pio/build/ATtiny1616/firmware.hex tc2test:
ssh tc2test "sudo pymcuprog -d attiny1616 -t uart -u /dev/serial0 erase"
ssh tc2test "sudo pymcuprog -d attiny1616 -t uart -u /dev/serial0 write -f firmware.hex"
