#!/bin/sh
picocom /dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0 --baud 115200 --omap crcrlf --echo
