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

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "lee_position_controller_mod_node.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control {

LeePositionControllerModNode::LeePositionControllerModNode():
 t_(0),
 dt_(0),
 clock_topic_("clock"),
 motor_topic_("motor_status"),
 cmd_twist_topic_("cmd_vel"),
 twist_mode_(true)
{
  InitializeParams();

  ros::NodeHandle nh;

  test_sub_ = nh.subscribe("firefly/test_activation", 1, &LeePositionControllerModNode::TestCallback, this);

  motor_sub_ = nh.subscribe(motor_topic_, 1, &LeePositionControllerModNode::MotorCallback, this);

  clock_sub_ = nh.subscribe(clock_topic_, 1, &LeePositionControllerModNode::ClockCallback, this);

  cmd_twist_sub_ = nh.subscribe(cmd_twist_topic_, 1, &LeePositionControllerModNode::CommandTwistCallback, this);

  cmd_pose_sub_ = nh.subscribe(
      mav_msgs::default_topics::COMMAND_POSE, 1,
      &LeePositionControllerModNode::CommandPoseCallback, this);

  cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &LeePositionControllerModNode::MultiDofJointTrajectoryCallback, this);

  odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                             &LeePositionControllerModNode::OdometryCallback, this);

  motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

  command_timer_ = nh.createTimer(ros::Duration(0), &LeePositionControllerModNode::TimedCommandCallback, this,
                                  true, false);
}

LeePositionControllerModNode::~LeePositionControllerModNode() { }

void LeePositionControllerModNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  // Read parameters from rosparam.
  GetRosParameter(pnh, "position_gain/x",
                  lee_position_controller_mod_.controller_parameters_.position_gain_.x(),
                  &lee_position_controller_mod_.controller_parameters_.position_gain_.x());
  GetRosParameter(pnh, "position_gain/y",
                  lee_position_controller_mod_.controller_parameters_.position_gain_.y(),
                  &lee_position_controller_mod_.controller_parameters_.position_gain_.y());
  GetRosParameter(pnh, "position_gain/z",
                  lee_position_controller_mod_.controller_parameters_.position_gain_.z(),
                  &lee_position_controller_mod_.controller_parameters_.position_gain_.z());
  GetRosParameter(pnh, "position_integral_gain/x",
                  lee_position_controller_mod_.controller_parameters_.position_integral_gain_.x(),
                  &lee_position_controller_mod_.controller_parameters_.position_integral_gain_.x());
  GetRosParameter(pnh, "position_integral_gain/y",
                  lee_position_controller_mod_.controller_parameters_.position_integral_gain_.y(),
                  &lee_position_controller_mod_.controller_parameters_.position_integral_gain_.y());
  GetRosParameter(pnh, "position_integral_gain/z",
                  lee_position_controller_mod_.controller_parameters_.position_integral_gain_.z(),
                  &lee_position_controller_mod_.controller_parameters_.position_integral_gain_.z());
  GetRosParameter(pnh, "velocity_gain/x",
                  lee_position_controller_mod_.controller_parameters_.velocity_gain_.x(),
                  &lee_position_controller_mod_.controller_parameters_.velocity_gain_.x());
  GetRosParameter(pnh, "velocity_gain/y",
                  lee_position_controller_mod_.controller_parameters_.velocity_gain_.y(),
                  &lee_position_controller_mod_.controller_parameters_.velocity_gain_.y());
  GetRosParameter(pnh, "velocity_gain/z",
                  lee_position_controller_mod_.controller_parameters_.velocity_gain_.z(),
                  &lee_position_controller_mod_.controller_parameters_.velocity_gain_.z());
  GetRosParameter(pnh, "attitude_gain/x",
                  lee_position_controller_mod_.controller_parameters_.attitude_gain_.x(),
                  &lee_position_controller_mod_.controller_parameters_.attitude_gain_.x());
  GetRosParameter(pnh, "attitude_gain/y",
                  lee_position_controller_mod_.controller_parameters_.attitude_gain_.y(),
                  &lee_position_controller_mod_.controller_parameters_.attitude_gain_.y());
  GetRosParameter(pnh, "attitude_gain/z",
                  lee_position_controller_mod_.controller_parameters_.attitude_gain_.z(),
                  &lee_position_controller_mod_.controller_parameters_.attitude_gain_.z());
  GetRosParameter(pnh, "attitude_integral_gain/x",
                  lee_position_controller_mod_.controller_parameters_.attitude_integral_gain_.x(),
                  &lee_position_controller_mod_.controller_parameters_.attitude_integral_gain_.x());
  GetRosParameter(pnh, "attitude_integral_gain/y",
                  lee_position_controller_mod_.controller_parameters_.attitude_integral_gain_.y(),
                  &lee_position_controller_mod_.controller_parameters_.attitude_integral_gain_.y());
  GetRosParameter(pnh, "attitude_integral_gain/z",
                  lee_position_controller_mod_.controller_parameters_.attitude_integral_gain_.z(),
                  &lee_position_controller_mod_.controller_parameters_.attitude_integral_gain_.z());
  GetRosParameter(pnh, "angular_rate_gain/x",
                  lee_position_controller_mod_.controller_parameters_.angular_rate_gain_.x(),
                  &lee_position_controller_mod_.controller_parameters_.angular_rate_gain_.x());
  GetRosParameter(pnh, "angular_rate_gain/y",
                  lee_position_controller_mod_.controller_parameters_.angular_rate_gain_.y(),
                  &lee_position_controller_mod_.controller_parameters_.angular_rate_gain_.y());
  GetRosParameter(pnh, "angular_rate_gain/z",
                  lee_position_controller_mod_.controller_parameters_.angular_rate_gain_.z(),
                  &lee_position_controller_mod_.controller_parameters_.angular_rate_gain_.z());
  GetVehicleParameters(pnh, &lee_position_controller_mod_.vehicle_parameters_);
  lee_position_controller_mod_.InitializeParameters(pnh);
}

void LeePositionControllerModNode::Publish() {
}

void LeePositionControllerModNode::MotorCallback(const std_msgs::Bool &msg)
{
  lee_position_controller_mod_.SetActive(msg.data);
}

void LeePositionControllerModNode::TestCallback(const std_msgs::Bool &msg)
{
  lee_position_controller_mod_.SetTest(msg.data);
}

void LeePositionControllerModNode::CommandTwistCallback(const geometry_msgs::Twist &msg)
{
  cmd_twist_ = msg;
}

void LeePositionControllerModNode::CommandPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
  commands_.push_front(eigen_reference);

  lee_position_controller_mod_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
}

void LeePositionControllerModNode::MultiDofJointTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference); // this is the first element  by ZD

  for (size_t i = 1; i < n_commands; ++i) {
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before = msg->points[i-1];
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference = msg->points[i];

    mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);

    commands_.push_back(eigen_reference); // this is the elements from the second to the end
    command_waiting_times_.push_back(current_reference.time_from_start - reference_before.time_from_start);
  }

  // We can trigger the first command immediately.
  lee_position_controller_mod_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();

  if (n_commands > 1) {
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void LeePositionControllerModNode::TimedCommandCallback(const ros::TimerEvent& e) {

  if(commands_.empty()){
    ROS_WARN("Commands empty, this should not happen here");
    return;
  }

  const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
  lee_position_controller_mod_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
  command_timer_.stop();
  if(!command_waiting_times_.empty()){
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void LeePositionControllerModNode::ClockCallback(const rosgraph_msgs::Clock &clock)
{
  if (dt_.isZero())
  {
    dt_.fromSec(0.001);
  }
  else
  {
    dt_ = clock.clock - t_;
  }
  t_ = clock.clock;

  lee_position_controller_mod_.SetTime(t_);
  lee_position_controller_mod_.SetPeriod(dt_);

  if (twist_mode_)
  {
    lee_position_controller_mod_.SetCmdFromTwist(cmd_twist_);
  }

  Eigen::VectorXd ref_rotor_velocities;
  lee_position_controller_mod_.CalculateRotorVelocities(&ref_rotor_velocities);

  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  actuator_msg->header.stamp = t_;
  motor_velocity_reference_pub_.publish(actuator_msg);
}

void LeePositionControllerModNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  ROS_INFO_ONCE("LeePositionController got first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  lee_position_controller_mod_.SetOdometry(odometry);
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lee_position_controller_mod_node");

  rotors_control::LeePositionControllerModNode lee_position_controller_mod_node;

  ros::spin();

  return 0;
}

