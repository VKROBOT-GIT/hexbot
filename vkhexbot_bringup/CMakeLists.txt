cmake_minimum_required(VERSION 2.8.3)
project(vkhexbot_bringup)

find_package( catkin REQUIRED roslaunch )

catkin_package()

roslaunch_add_file_check(launch)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES vkhexbot_bringup.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
  
 
