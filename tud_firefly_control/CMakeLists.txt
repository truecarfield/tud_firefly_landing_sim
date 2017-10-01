cmake_minimum_required(VERSION 2.8.3)
project(tud_firefly_control)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy

  std_msgs
  geometry_msgs
  mav_msgs
  nav_msgs
  sensor_msgs

  rotors_control
  # acado
  # qp_oases
  
  tf
  tf_conversions
)

find_package(Eigen REQUIRED)
find_package(ACADO REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${ACADO_INCLUDE_DIRS}
)

catkin_package(
INCLUDE_DIRS 
    include 
  LIBRARIES 
    differential_tracker
    generic_pid
    firefly_twist_controller
    firefly_attitude_controller
    firefly_lee_position_controller
    firefly_landing_controller
  CATKIN_DEPENDS
    # qp_oases
    # acado
    geometry_msgs 
    mav_msgs 
    nav_msgs 
    roscpp 
    sensor_msgs 
    rotors_control
    tf
    tf_conversions 
  DEPENDS Eigen Eigen3 ACADO
)

add_library(differential_tracker src/library/differential_tracker.cpp)
target_link_libraries(differential_tracker ${catkin_LIBRARIES})
add_dependencies(differential_tracker ${catkin_EXPORTED_TARGETS})

add_library(generic_pid src/library/generic_pid.cpp)
target_link_libraries(generic_pid ${catkin_LIBRARIES})
add_dependencies(generic_pid ${catkin_EXPORTED_TARGETS})

add_library(firefly_twist_controller src/library/twist_controller.cpp)
target_link_libraries(firefly_twist_controller generic_pid ${catkin_LIBRARIES})
add_dependencies(firefly_twist_controller ${catkin_EXPORTED_TARGETS})

add_library(firefly_attitude_controller src/library/attitude_controller.cpp)
target_link_libraries(firefly_attitude_controller generic_pid ${catkin_LIBRARIES})
add_dependencies(firefly_attitude_controller ${catkin_EXPORTED_TARGETS})

add_library(firefly_lee_position_controller src/library/lee_position_controller_mod.cpp)
target_link_libraries(firefly_lee_position_controller differential_tracker ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
add_dependencies(firefly_lee_position_controller ${catkin_EXPORTED_TARGETS})

add_library(firefly_landing_controller src/library/landing_controller.cpp)
target_link_libraries(firefly_landing_controller differential_tracker generic_pid ${catkin_LIBRARIES} ${ACADO_SHARED_LIBRARIES})
add_dependencies(firefly_landing_controller ${catkin_EXPORTED_TARGETS})

add_executable(firefly_twist_controller_node src/nodes/twist_controller_node.cpp)
target_link_libraries(firefly_twist_controller_node
  firefly_twist_controller generic_pid ${catkin_LIBRARIES})
add_dependencies(firefly_twist_controller_node ${catkin_EXPORTED_TARGETS})

add_executable(firefly_attitude_controller_node src/nodes/attitude_controller_node.cpp)
target_link_libraries(firefly_attitude_controller_node
  firefly_attitude_controller generic_pid ${catkin_LIBRARIES})
add_dependencies(firefly_attitude_controller_node ${catkin_EXPORTED_TARGETS})

add_executable(firefly_landing_controller_node src/nodes/landing_controller_node.cpp)
target_link_libraries(firefly_landing_controller_node firefly_landing_controller differential_tracker generic_pid ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${ACADO_SHARED_LIBRARIES})
add_dependencies(firefly_landing_controller_node ${catkin_EXPORTED_TARGETS})

add_executable(firefly_lee_position_controller_node src/nodes/lee_position_controller_mod_node.cpp)
add_dependencies(firefly_lee_position_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(firefly_lee_position_controller_node firefly_lee_position_controller differential_tracker generic_pid ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(joy_controller_node src/nodes/joy_controller_node.cpp)
add_dependencies(joy_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(joy_controller_node ${catkin_LIBRARIES})

add_executable(tracker_test_node src/nodes/position_W_tracker_node.cpp)
target_link_libraries(tracker_test_node differential_tracker ${catkin_LIBRARIES})
add_dependencies(tracker_test_node ${catkin_EXPORTED_TARGETS})

