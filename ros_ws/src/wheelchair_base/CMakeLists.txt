cmake_minimum_required(VERSION 3.0.2)
project(wheelchair_base)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  geometry_msgs
  wheelchair_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp tf2 tf2_ros tf2_geometry_msgs nav_msgs geometry_msgs wheelchair_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_publisher src/odom_publisher.cpp)
target_link_libraries(odom_publisher ${catkin_LIBRARIES})
add_dependencies(odom_publisher ${catkin_EXPORTED_TARGETS})

install(TARGETS odom_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
