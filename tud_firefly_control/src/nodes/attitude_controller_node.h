#ifndef ATTITUDE_CONTROLLER_NODE_H
#define ATTITUDE_CONTROLLER_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include "tud_firefly_control/tud_firefly_common.h"
#include "tud_firefly_control/attitude_controller.h"

namespace rotors_control {

class FireflyAttitudeControllerNode {
public:
  FireflyAttitudeControllerNode();
  ~FireflyAttitudeControllerNode();

  void InitializeParams();
  void Publish();

protected:
  FireflyAttitudeController firefly_attitude_controller_;

  std::string namespace_;

  ros::Time t;
  ros::Duration dt;

  // subscribers and publishers
  ros::Subscriber clock_sub_;
  ros::Subscriber step_sub_;
  ros::Subscriber cmd_sub_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber gt_imu_sub_;
  ros::Subscriber motor_sub_;
  ros::Timer command_timer_;

  ros::Publisher motor_velocity_reference_pub_;

  //  std::deque<ros::Duration> command_waiting_times_;
  //  ros::Timer command_timer_;

  // callbacks
  void clockCallback(const rosgraph_msgs::Clock& msg);
  void motorStatusCallback(const std_msgs::Bool& msg);
  void commandCallback(const std_msgs::Float64MultiArray& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void timedCommandCallback(const ros::TimerEvent& e);
};

}

#endif // ATTITUDE_CONTROLLER_NODE_H
