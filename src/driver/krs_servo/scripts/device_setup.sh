#!/bin/bash

modprobe ftdi-sio vendor=0x165C product=0007
chmod 777 /sys/bus/usb-serial/drivers/ftdi_sio/new_id
echo 165c 0008 > /sys/bus/usb-serial/drivers/ftdi_sio/new_id
