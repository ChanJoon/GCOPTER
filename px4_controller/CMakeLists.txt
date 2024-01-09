cmake_minimum_required(VERSION 2.8.3)
project(px4_controller)

set(CMAKE_VERBOSE_MAKEFILE "false")

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  sensor_msgs
  visualization_msgs
  quadrotor_msgs
  geometry_msgs
  uav_utils
  mavros
  gcopter
)
add_service_files(FILES
  Command.srv
)
generate_messages(DEPENDENCIES std_msgs)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen REQUIRED) # try to find manually installed eigen (Usually in /usr/local with provided FindEigen3.cmake)
find_package(ompl REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES Ctrl
 CATKIN_DEPENDS roscpp rospy message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  # ${gcopter_INCLUDE_DIRS}
  "../gcopter/include"        # For #include "misc/visualizer.hpp", Or copy 'misc' folder in this package.
  include/${PROJECT_NAME}
  include
)

add_executable(px4_control_node 
	src/px4_control_node.cpp
  src/controller.cpp
  src/hovthrkf.cpp
)
add_dependencies(px4_control_node quadrotor_msgs)
target_link_libraries(px4_control_node
  ${catkin_LIBRARIES}
)


add_executable(simple_planning src/simple_planning.cpp)
target_link_libraries(simple_planning
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(multi_points_planning src/multi_points_planning.cpp)
target_link_libraries(multi_points_planning
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
)
