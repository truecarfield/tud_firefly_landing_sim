cmake_minimum_required(VERSION 2.8.3)
project(tud_firefly_state_estimation)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  rospy

  geographic_msgs
  geometry_msgs
  mav_msgs
  nav_msgs
  sensor_msgs

  message_generation
  robot_localization
  tud_firefly_control

  tf
)

find_package(Eigen REQUIRED)

link_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_service_files(
  FILES
  SetPose.srv
  SetDatum.srv
)

generate_messages(
  DEPENDENCIES
  geographic_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include 
    ${Eigen_INCLUDE_DIRS}
  LIBRARIES
    mahony_complementary_filter 
    complementary_filter
    firefly_ekf
    firefly_filter_base
    firefly_ros_filter
  CATKIN_DEPENDS
    cmake_modules 
    geometry_msgs 
    mav_msgs 
    nav_msgs 
    roscpp 
    sensor_msgs  
    robot_localization  
    tud_firefly_control
    tf
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(firefly_filter_base src/library/firefly_filter_base.cpp) 
target_link_libraries(firefly_filter_base ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(firefly_filter_base ${catkin_EXPORTED_TARGETS})

add_library(firefly_ekf src/library/firefly_ekf.cpp) 
target_link_libraries(firefly_ekf firefly_filter_base ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(firefly_ekf ${catkin_EXPORTED_TARGETS})

add_library(firefly_ros_filter src/library/firefly_ros_filter.cpp)
target_link_libraries(firefly_ros_filter firefly_ekf ${catkin_LIBRARIES})
add_dependencies(firefly_ros_filter ${catkin_EXPORTED_TARGETS})

add_library(complementary_filter src/library/complementary_filter.cpp)
target_link_libraries(complementary_filter ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(complementary_filter ${catkin_EXPORTED_TARGETS})

add_library(mahony_complementary_filter src/library/mahony_complementary_filter.cpp)
target_link_libraries(mahony_complementary_filter ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(mahony_complementary_filter ${catkin_EXPORTED_TARGETS})

add_executable(complementary_filter_node src/nodes/complementary_filter_node.cpp)
target_link_libraries(complementary_filter_node complementary_filter ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(complementary_filter_node ${catkin_EXPORTED_TARGETS})

add_executable(mahony_complementary_filter_node src/nodes/mahony_complementary_filter_node.cpp)
target_link_libraries(mahony_complementary_filter_node mahony_complementary_filter ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})
add_dependencies(mahony_complementary_filter_node ${catkin_EXPORTED_TARGETS})

add_executable(firefly_state_estimation_node src/nodes/state_estimation_node.cpp)
target_link_libraries(firefly_state_estimation_node firefly_ros_filter ${catkin_LIBRARIES})
add_dependencies(firefly_state_estimation_node ${catkin_EXPORTED_TARGETS})
