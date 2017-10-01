cmake_minimum_required(VERSION 2.8.3)
project(tud_firefly_application)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy

  geometry_msgs
  mav_msgs
  nav_msgs
  sensor_msgs
  std_msgs

  robot_localization
  rotors_control
  tud_firefly_control
  tud_firefly_state_estimation

  tf
  ar_sys
  cv_bridge
)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gazebo REQUIRED)
find_package(ACADO REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${ACADO_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS 
    include 
  LIBRARIES
  CATKIN_DEPENDS 
    geometry_msgs 
    mav_msgs 
    nav_msgs 
    roscpp 
    rospy
    sensor_msgs 
    rotors_control 
    tud_firefly_control
    robot_localization
    tud_firefly_state_estimation 
    ar_sys
    cv_bridge 
    tf
  DEPENDS Eigen opencv gazebo ACADO
)

add_executable(landing_state_machine_node src/nodes/landing_state_machine/landing_state_machine_node.cpp)
target_link_libraries(landing_state_machine_node ${catkin_LIBRARIES} ${ACADO_SHARED_LIBRARIES})
add_dependencies(landing_state_machine_node ${catkin_LIBRARIES})

add_executable(landing_monitor_node src/nodes/landing_monitor/landing_monitor_node.cpp)
target_link_libraries(landing_monitor_node ${catkin_LIBRARIES})
add_dependencies(landing_monitor_node ${catkin_EXPORTED_TARGETS})

add_executable(ar_sys_single_board_mod_node src/nodes/ar_sys_marker_detection/ar_sys_single_board_mod.cpp src/nodes/ar_sys_marker_detection/utils.cpp)
target_link_libraries(ar_sys_single_board_mod_node ${catkin_LIBRARIES})
add_dependencies(ar_sys_single_board_mod_node ${catkin_EXPORTED_TARGETS})


