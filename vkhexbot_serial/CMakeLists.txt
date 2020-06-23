cmake_minimum_required(VERSION 2.8.3)
project( vkhexbot_serial )

find_package( catkin REQUIRED COMPONENTS nodelet serial roscpp geometry_msgs nav_msgs sensor_msgs std_msgs tf vkhexbot_msgs)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vkhexbot_serial 
  CATKIN_DEPENDS nodelet roscpp geometry_msgs sensor_msgs std_msgs tf
)

include_directories(include ${catkin_INCLUDE_DIRS})


add_library( hexbotProtocolPack src/hexbotProtocolPack.cpp )
add_library( hexbotProtocol src/hexbotProtocol.cpp )

add_executable( vkhexbot_serial src/vkhexbot_serial.cpp )
target_link_libraries( vkhexbot_serial ${catkin_LIBRARIES} hexbotProtocolPack hexbotProtocol)
add_dependencies( vkhexbot_serial vkhexbot_serial_generate_messages_cpp )

install(TARGETS vkhexbot_serial
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
