# ATtiny1616

Firmware for the ATtiny1616 on the TC2 (thermal camera 2) main PCB. It controls the power for the
Raspberry Pi and RP2040, the status LED, the button, battery and RTC battery readings, and the
enable pin of the PCA9517A I2C repeater that connects the Pi's I2C to the rest of the bus.
The Pi talks to it over I2C (address `0x25`) through `tc2-hat-attiny` from
[tc2-hat-controller](https://github.com/TheCacophonyProject/tc2-hat-controller).
See `src/registers.md` for the I2C registers and `testing.md` for a manual test checklist.

## PlatformIO

- Install VSCode or VSCodium and then install the PlatformIO plugin.
- Open the plugin and then import this project.
- You don't have to use PlatformIO, but it's probably the easiest way to set things up.
  The plugin installs `pio` in `~/.platformio/penv/bin`.

## Building

The version is set from environment variables when building:

``` bash
export MAJOR_VERSION=1 MINOR_VERSION=0 PATCH_VERSION=10
pio run
```

The firmware is written to `.pio/build/ATtiny1616/firmware.hex`.

## Installing on a camera

1. Find the version `tc2-hat-attiny` expects by running this on the camera. It also prints the
   `export` line for the version variables:

   ``` bash
   tc2-hat-attiny --attiny-firmware-info
   ```

2. Set the version variables on your machine and run:

   ``` bash
   ./install.sh pi@<camera-hostname>.local
   ```

   This builds the firmware, copies it to the camera, stops `tc2-hat-attiny`, flashes the ATtiny
   over UPDI and starts `tc2-hat-attiny` again.
3. Check the log on the camera. It should show `Reset reason: UPDI reset` and
   `ATtiny Running correct firmware version`:

   ``` bash
   journalctl -u tc2-hat-attiny -n 30 --no-pager
   ```

### Why the version matters

When `tc2-hat-attiny` starts it reads the ATtiny's version over I2C. If it isn't the version
tc2-hat-controller was built for, it reflashes the ATtiny with the firmware bundled in its
package (`/etc/cacophony/attiny-firmware.hex`), replacing yours.

To ship new firmware in tc2-hat-controller instead, copy the hex file to its
`_release/attiny-firmware.hex` and set `ATTINY_MAJOR`, `ATTINY_MINOR`, `ATTINY_PATCH` and
`ATTINY_HASH` (`sha256sum _release/attiny-firmware.hex`).

### If it can't connect to the ATtiny over UART

`install.sh` pings the ATtiny before erasing it, and stops with a message if that fails. A failed
ping or `UPDI initialisation failed` can mean:

- **The UART isn't routed to the ATtiny.** Run on the camera:

  ``` bash
  sudo pinctrl set 6 op dl    # UART multiplexer: select the ATtiny UPDI pin
  sudo pinctrl set 12 op dl
  sudo pinctrl set 14 a0      # GPIO14/15 to UART
  sudo pinctrl set 15 a0
  ```

- **Another service is using `/dev/serial0`,** usually `tc2-hat-comms`. Check with
  `sudo fuser -v /dev/serial0` and stop it with `sudo systemctl stop tc2-hat-comms`.
  Start it again after flashing.
- **The serial console is enabled** (`console=serial0` in `/boot/firmware/cmdline.txt`).
  Run `sudo disable-aux-uart` on the camera and reboot.

Flashing also restarts the RP2040, so `tc2-hat-attiny` stops `tc2-agent` when it updates the ATtiny.
To do the same, run `sudo systemctl stop tc2-agent` before flashing and start it again afterwards.

## Releases

Pushing a tag like `v1.0.11` runs the CI workflow (`.github/workflows/release.yaml`), which
builds the firmware with that version and attaches `firmware.hex` and `firmware.hex.sha256`
to the GitHub release. tc2-hat-controller's release downloads these.
