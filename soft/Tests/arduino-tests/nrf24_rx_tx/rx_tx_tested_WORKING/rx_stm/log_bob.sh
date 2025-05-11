#!/bin/sh
picocom /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0 --baud 115200 --omap crcrlf --echo
