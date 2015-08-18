cmake_minimum_required(VERSION 2.8.3)
project(gps_example)

set(${PROJECT_NAME}_CATKIN_DEPS
  geometry_msgs
  dynamic_reconfigure
  visualization_msgs
  roscpp
  nav_msgs
  tf
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

set(${PROJECT_NAME}_INCLUDE_EXPORTS
  include
)

catkin_package(
	INCLUDE_DIRS ${${PROJECT_NAME}_INCLUDE_EXPORTS}
	LIBRARIES gps_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${${PROJECT_NAME}_INCLUDE_EXPORTS}
)

add_executable(gps_example_node 
	src/gps_example_node.cpp 
	src/GpsConversion.cpp
)
target_link_libraries(gps_example_node
	${catkin_LIBRARIES}
)

add_executable(sensor_integration_node 
	src/sensor_integration_node.cpp 
)
target_link_libraries(sensor_integration_node
	${catkin_LIBRARIES}
)

add_library(gps_lib src/GpsConversion.cpp)
target_link_libraries(gps_lib ${catkin_LIBRARIES})
