cmake_minimum_required(VERSION 2.8.3)
project(gps_navigation)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

set(${PROJECT_NAME}_INCLUDE_EXPORTS
  include
)

catkin_package(
	INCLUDE_DIRS ${${PROJECT_NAME}_INCLUDE_EXPORTS}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${${PROJECT_NAME}_INCLUDE_EXPORTS}
)


add_executable(navigation_node 
	src/gps_navigation.cpp 
	src/GpsConversion.cpp
)
target_link_libraries(navigation_node
	${catkin_LIBRARIES}
)
