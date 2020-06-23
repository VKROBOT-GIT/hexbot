cmake_minimum_required(VERSION 2.8.3)
project( vkhexbot_teleop )

find_package( catkin REQUIRED COMPONENTS nodelet roscpp geometry_msgs nav_msgs sensor_msgs std_msgs  vkhexbot_msgs)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hexbot_teleop_key
  CATKIN_DEPENDS nodelet roscpp geometry_msgs sensor_msgs std_msgs 
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(hexbot_teleop_key src/hexbot_teleop_key.cpp )
target_link_libraries( hexbot_teleop_key ${catkin_LIBRARIES})


install(TARGETS hexbot_teleop_key
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
