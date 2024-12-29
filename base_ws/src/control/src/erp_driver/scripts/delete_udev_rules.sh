#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to  rplidar"
echo "sudo rm   /etc/udev/rules.d/erp.rules"
sudo rm   /etc/udev/rules.d/erp.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"
