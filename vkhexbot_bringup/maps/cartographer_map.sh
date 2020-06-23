rm -rf ${HOME}/vkrobot_ws/src/hexapod_bringup/maps/house.pgm 
rm -rf ${HOME}/vkrobot_ws/src/hexapod_bringup/maps/house.yaml

rosservice call /finish_trajectory 0
rosservice call /write_state ${HOME}/vkrobot_ws/src/hexapod_bringup/maps/map.bag.pbstream

rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename=${HOME}/vkrobot_ws/src/hexapod_bringup/maps/map.bag.pbstream -map_filestem=${HOME}/vkrobot_ws/src/hexapod_bringup/maps/cartographer_map_20191201 -resolution=0.05

#cp  /home/rikirobot/catkin_ws/src/rikirobot_project/rikirobot/maps/map.pgm /home/rikirobot/catkin_ws/src/rikirobot_project/rikirobot/maps/house.pgm

#cp  /home/rikirobot/catkin_ws/src/rikirobot_project/rikirobot/maps/map.yaml /home/rikirobot/catkin_ws/src/rikirobot_project/rikirobot/maps/house.yaml

