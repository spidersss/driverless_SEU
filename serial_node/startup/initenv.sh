#!/bin/bash
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout",  SYMLINK+="stm32"' >/etc/udev/rules.d/stm32.rules

service udev reload
sleep 2
service udev restart

