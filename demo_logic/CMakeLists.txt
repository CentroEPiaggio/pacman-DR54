cmake_minimum_required(VERSION 2.8.3)
project(demo_logic)

add_definitions(-fpermissive -std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  decision_making
  decision_making_parser
  pacman_vision_comm
  moveit_msgs
  moveit_ros_planning_interface
  gp_regression
  kdl_conversions
  kdl_parser
  geometric_shapes
)

add_message_files(
  FILES
  ContactState.msg
)
add_service_files(
  FILES
  GetPathLog.srv
)
generate_messages(
  DEPENDENCIES
  gp_regression
)

catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES demo_logic
  CATKIN_DEPENDS
    roscpp
    decision_making
    decision_making_parser
    pacman_vision_comm
    moveit_msgs
    moveit_ros_planning_interface
    gp_regression
    kdl_conversions
    kdl_parser
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(contact_observer
  src/contact_observer.cpp
)

add_dependencies(contact_observer
  demo_logic_generate_messages_cpp
)

target_link_libraries(contact_observer
  ${catkin_LIBRARIES}
)


add_executable(DR54Logic
  src/DR54Logic.cpp
)

add_dependencies(DR54Logic
  demo_logic_generate_messages_cpp
)

target_link_libraries(DR54Logic
  ${catkin_LIBRARIES}
)

decision_making_parsing(
  src/DR54Logic.cpp
)
