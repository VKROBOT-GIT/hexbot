#!/bin/bash

echo "***************"
echo "delete the remap device serial port to vk_hexbot"
sudo rm   /etc/udev/rules.d/hexbot.rules
echo "delete the remap device serial port to vk_ydllidar"
sudo rm   /etc/udev/rules.d/ydlidar.rules
echo "delete the remap device serial port to vk_imu"
sudo rm   /etc/udev/rules.d/imu.rules
echo "Restarting udev"
sudo service udev reload
sudo service udev restart
echo "finish"
echo "***************"
