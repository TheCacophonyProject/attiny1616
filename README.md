# ATtiny1616

## PlatformIO
- Install VSCode or VSCodium and then install the PlatformIO project.
- Open the plugin and then import this project.
- Don't have to use platformIO but that might be the easiest way to set up things.


### Programming from RPi
- `sudo apt install python3-pip -y`
- `sudo pip install pymcuprog`
- Free up UART to be used for UPDI connection. Add `dtoverlay=disable-bt` to the end of `/boot/config.txt`
- Disable UART outputting to console: Remove `console=serial0,115200` from `/boot/cmdline.txt` and reboot.
- Ping `sudo pymcuprog -d attiny1616 -t uart -u /dev/serial0 ping`
- Erase `sudo pymcuprog -d attiny1616 -t uart -u /dev/serial0 erase`
- Program through UPDI `sudo pymcuprog -d attiny1616 -t uart -u /dev/serial0 write -f ./Blink-attiny1616.ino.t1616.20c0.mD0.v261.hex`
- Note that you should Erase before programming each time, I think..


