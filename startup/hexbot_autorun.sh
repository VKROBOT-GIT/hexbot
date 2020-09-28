#! /bin/bash
source /opt/ros/melodic/setup.bash
source /home/vkrobot/hexbot_ws/devel/setup.bash
source /home/vkrobot/.bashrc
sleep 2s
gnome-terminal --geometry=80x10+350+0 --title="ROS Core" -x roscore  #start core
sleep 15s                                        #waitting core start
gnome-terminal --geometry=80x10+1200+0 --title="usb cam" -x rosrun usb_cam usb_cam_node _pixel_format:=yuyv _camera_frame_id:=usb_cam _io_method:=mmap #start  camera 
sleep 2s
gnome-terminal --geometry=80x10+350+320 --title="Rosbridge Node" -x roslaunch rosbridge_server rosbridge_websocket.launch  #start rosbridge for android mobile phone
sleep 2s
sleep 3s

gnome-terminal --geometry=80x10+1200+320 --title="gmapping" -x  roslaunch vkhexbot_bringup gmapping_slam.launch #gmapping 
 
gnome-terminal --geometry=80x10+350+600 --title="Web cam" -x rosrun web_video_server web_video_server  #start core
sleep 2s
#gnome-terminal --geometry=80x10+12000+600 --title="test" -x rosnode list && rostopic list  #

