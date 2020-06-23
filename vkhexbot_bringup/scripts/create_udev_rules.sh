#!/bin/bash

echo "***************"
echo "remap the device serial port(ttyUSBX) to vk_hexbot"
echo "start copy hexbot.rules to  /etc/udev/rules.d/"
sudo cp `rospack find vkhexbot_bringup`/scripts/rules/hexbot.rules  /etc/udev/rules.d

echo "remap the device serial port(ttyUSBX) to vk_ydlidar"
echo "start copy ydlidar.rules to  /etc/udev/rules.d/"
sudo cp `rospack find vkhexbot_bringup`/scripts/rules/ydlidar.rules  /etc/udev/rules.d

echo "remap the device serial port(ttyUSBX) to vk_imu"
echo "start copy imu.rules to  /etc/udev/rules.d/"
sudo cp `rospack find vkhexbot_bringup`/scripts/rules/imu.rules  /etc/udev/rules.d

echo "Restarting udev"
sudo service udev reload
sudo service udev restart
echo "finish"
echo "***************"
