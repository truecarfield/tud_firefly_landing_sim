/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROTORS_CONTROL_LEE_POSITION_CONTROLLER_MOD_NODE_H
#define ROTORS_CONTROL_LEE_POSITION_CONTROLLER_MOD_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Bool.h>

#include "tud_firefly_control/tud_firefly_common.h"
#include "tud_firefly_control/lee_position_controller_mod.h"

namespace rotors_control {

class LeePositionControllerModNode {
 public:
  LeePositionControllerModNode();
  ~LeePositionControllerModNode();

  void InitializeParams();
  void reset();
  void Publish();

 private:

  LeePositionControllerMod lee_position_controller_mod_;

  std::string namespace_;

  bool twist_mode_;
  bool motor_status_;

  // topics
  std::string clock_topic_;
  std::string motor_topic_;
  std::string cmd_twist_topic_;

  // subscribers
  ros::Subscriber cmd_trajectory_sub_;
  ros::Subscriber cmd_multi_dof_joint_trajectory_sub_;
  ros::Subscriber cmd_pose_sub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber cmd_twist_sub_;
  ros::Subscriber clock_sub_;
  ros::Subscriber motor_sub_;
  ros::Subscriber test_sub_;

  // publishers
  ros::Publisher motor_velocity_reference_pub_;

  // important signals
  mav_msgs::EigenTrajectoryPointDeque commands_;
  std::deque<ros::Duration> command_waiting_times_;
  ros::Timer command_timer_;

  geometry_msgs::Twist cmd_twist_;

  ros::Time t_;
  ros::Duration dt_;

  void ClockCallback(const rosgraph_msgs::Clock& clock);

  void TestCallback(const std_msgs::Bool& msg);

  void MotorCallback(const std_msgs::Bool& msg);

  void TimedCommandCallback(const ros::TimerEvent& e);

  void MultiDofJointTrajectoryCallback(
      const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);

  void CommandPoseCallback(
      const geometry_msgs::PoseStampedConstPtr& pose_msg);

  void CommandTwistCallback(const geometry_msgs::Twist& msg);

  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
};
}

#endif // ROTORS_CONTROL_LEE_POSITION_CONTROLLER_MOD_NODE_H
