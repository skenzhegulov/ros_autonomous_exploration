cmake_minimum_required(VERSION 2.8.3)
project(autonomous_exploration)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  tf
  nav_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib_msgs
  actionlib
  geometry_msgs
  nav_msgs
  kobuki_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  FILES
  Explore.action
)

add_service_files(
  FILES
  ExploreTarget.srv  
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  geometry_msgs
  nav_msgs
  kobuki_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs 
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(explore src/explore_client.cpp)
target_link_libraries(explore ${catkin_LIBRARIES})
add_dependencies(explore ${autonomous_exploration_EXPORTED_TARGETS})

add_executable(get_map_server src/get_map_server.cpp)
target_link_libraries(get_map_server ${catkin_LIBRARIES})
add_dependencies(get_map_server ${autonomous_exploration_EXPORTED_TARGETS})
