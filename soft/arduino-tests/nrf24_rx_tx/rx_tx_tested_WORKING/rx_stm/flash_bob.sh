#!/bin/sh

echo "Flashing BOB ttyACM0 ttyUSB0"
~/bin/toolchain/arm-gnu-toolchain-14.2.rel1/bin/arm-none-eabi-gdb bob.elf \
  -ex "target extended-remote /dev/serial/by-id/usb-Black_Magic_Debug_Black_Magic_Probe__ST-Link_v2__v1.10.0-229-g98f11d47_A7864BB1-if00" \
-ex "monitor swdp_scan" -ex "attach 1" -ex "load" -ex "b main"
