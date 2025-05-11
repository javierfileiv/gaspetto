#!/bin/sh

echo "Flashing ALICE ttyACM2 ttyUSB1"
~/bin/toolchain/arm-gnu-toolchain-14.2.rel1/bin/arm-none-eabi-gdb alice.elf \
  -ex "target extended-remote /dev/serial/by-id/usb-Black_Magic_Debug_Black_Magic_Probe__ST-Link_v2__v2.0.0-rc2-63-gbae24105_816841A7-if00" \
  -ex "monitor swdp_scan" -ex "attach 1" -ex "load" -ex "b main"
