cmake_minimum_required(VERSION 2.8.3)
project(ekf_example)

set(${PROJECT_NAME}_CATKIN_DEPS
  geometry_msgs
  roscpp
  sensor_msgs
  gps_example
  tf
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(unicycle_ekf src/UnicycleEkf.cpp)
target_link_libraries(unicycle_ekf
  ${catkin_LIBRARIES}
)

add_executable(ekf_example_node src/ekf_example_node.cpp)
target_link_libraries(ekf_example_node
  ${catkin_LIBRARIES}
  unicycle_ekf
)
