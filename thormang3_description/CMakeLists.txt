cmake_minimum_required(VERSION 2.8.3)
project(thormang3_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS
  roscpp
  sensor_msgs
)

add_executable(thormang3_rviz_pub 
	src/thormang3_rviz_pub.cpp
)

target_link_libraries(thormang3_rviz_pub
  ${catkin_LIBRARIES}
)
