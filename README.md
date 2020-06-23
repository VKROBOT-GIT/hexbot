# 智能六足机器人HEXBOT ROS源码包
使用步骤
----
1、安装ROS

Melodic/Ubuntu 18.04 [安装步骤](http://wiki.ros.org/melodic/Installation/Ubuntu) 

2、配置开发环境 [配置方法](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

3、获取源码
```
$ mkdir -p ~/catkin/src
$ cd ~/catkin_ws/src/
$ catkin_init_workspace
$ git clone https://github.com/VKROBOT-GIT/hexbot.git
```
4、安装依赖项

Kinetic/Ubuntu 16.0.4
```
$ cd ~/catkin_ws/src/hexbot/vkhexbot_bringup/scripts
$ ./install_for_kinetic.sh
```

Melodic/Ubuntu 18.0.4
```
$ cd ~/catkin_ws/src/hexbot/vkhexbot_bringup/scripts
$ ./install_for_melodic.sh
```
5、编译
```
$ cd ~/catkin_ws
$ catkin_make
```

6、设置设备权限
```
$ roscd vkhexbot_bringup
$ cd scripts
$ chmod +x create_udev_rules.sh
$ ./create_udev_rules.sh 
```
