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

#include "landing_controller_node.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control {

LandingControllerNode::LandingControllerNode():
 t_(0),
 dt_(0),
 clock_topic_("clock"),
 motor_topic_("firefly/motor_status"),
 cmd_twist_topic_("firefly/cmd_vel"),
 max_tilt_angle_topic_("firefly/max_tilt_angle"),
 cmd_yaw_topic_("firefly/traj_cmd_yaw"),
 traj_error_topic_("firefly/traj_error"),
 traj_error_point_topic_("firefly/traj_error_point"),
 twist_mode_(false)
{
  InitializeParams();

  ros::NodeHandle nh;

  test_sub_ = nh.subscribe("firefly/test_activation", 1, &LandingControllerNode::TestCallback, this);

  motor_sub_ = nh.subscribe(motor_topic_, 1, &LandingControllerNode::MotorCallback, this);

  clock_sub_ = nh.subscribe(clock_topic_, 1, &LandingControllerNode::ClockCallback, this);

  cmd_twist_sub_ = nh.subscribe(cmd_twist_topic_, 1, &LandingControllerNode::CommandTwistCallback, this);

  cmd_pose_sub_ = nh.subscribe(
      mav_msgs::default_topics::COMMAND_POSE, 1,
      &LandingControllerNode::CommandPoseCallback, this);

  cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &LandingControllerNode::MultiDofJointTrajectoryCallback, this);

  max_tilt_angle_sub_ = nh.subscribe(max_tilt_angle_topic_, 1, &LandingControllerNode::maxTiltAngleCallback, this);

  cmd_yaw_sub_ = nh.subscribe(cmd_yaw_topic_, 1, &LandingControllerNode::cmdYawCallback, this);

  traj_error_sub_ = nh.subscribe(
        traj_error_topic_, 1, &LandingControllerNode::trajErrorCmdAccelYawCallback, this);

  traj_error_point_sub_ = nh.subscribe(
        traj_error_point_topic_, 1, &LandingControllerNode::trajErrorCmdAccelYawPointCallback, this);

  odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                             &LandingControllerNode::OdometryCallback, this);

  motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

  motor_velocity_reference_uncorrect_pub_ = nh.advertise<mav_msgs::Actuators>(
       "firefly/command/motor_speed_sqr_uncorrected", 1);

  command_timer_ = nh.createTimer(ros::Duration(0), &LandingControllerNode::TimedCommandCallback, this,
                                  true, false);
}

LandingControllerNode::~LandingControllerNode() { }

void LandingControllerNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  pnh.param("twist_mode", twist_mode_, false);

  // Initial the controller
  landing_controller_.InitializeParameters(pnh);
}

void LandingControllerNode::Publish() {
}

void LandingControllerNode::MotorCallback(const std_msgs::Bool &msg)
{
  landing_controller_.SetActive(msg.data);

  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  // twist_mode_ = msg.data;
}

void LandingControllerNode::TestCallback(const std_msgs::Bool &msg)
{
  landing_controller_.SetTest(msg.data);
}

void LandingControllerNode::CommandTwistCallback(const geometry_msgs::Twist &msg)
{
  cmd_twist_ = msg;
}

void LandingControllerNode::CommandPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
  commands_.push_front(eigen_reference);

  landing_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
}

void LandingControllerNode::maxTiltAngleCallback(const std_msgs::Float32 &msg)
{
  landing_controller_.SetMaxTiltAngle(msg.data);
}

void LandingControllerNode::cmdYawCallback(const std_msgs::Float32 &msg)
{
  landing_controller_.SetCmdYaw(msg.data);
}

void LandingControllerNode::trajErrorCmdAccelYawPointCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& msg)
{
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  mav_msgs::EigenTrajectoryPoint trajErrorCmdAccelYaw;
  mav_msgs::eigenTrajectoryPointFromMsg(msg, &trajErrorCmdAccelYaw);

  //std::cout<<"trajCmdYaw:"<<trajErrorCmdAccelYaw.getYaw()<<std::endl;
  landing_controller_.SetTrajErrorCmdAccelYawPoint(trajErrorCmdAccelYaw);
}

void LandingControllerNode::trajErrorCmdAccelYawCallback(
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
  landing_controller_.SetTrajErrorCmdAccelYawPoint(commands_.front());
  commands_.pop_front();

  if (n_commands > 1) {
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void LandingControllerNode::MultiDofJointTrajectoryCallback(
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
  landing_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();

  if (n_commands > 1) {
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void LandingControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {

  if(commands_.empty()){
    ROS_WARN("Commands empty, this should not happen here");
    return;
  }

  const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
  landing_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
  command_timer_.stop();
  if (!command_waiting_times_.empty()) {
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void LandingControllerNode::ClockCallback(const rosgraph_msgs::Clock &clock)
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

  landing_controller_.SetTime(t_);
  landing_controller_.SetPeriod(dt_);

  if (twist_mode_)
  {
    landing_controller_.SetCmdFromTwist(cmd_twist_);
  }

  Eigen::VectorXd ref_rotor_velocities;
  landing_controller_.CalculateRotorVelocities(&ref_rotor_velocities);       // BK
  //landing_controller_.PID(&ref_rotor_velocities);                                              // pid
  //landing_controller_.HYB(&ref_rotor_velocities);                                            // HYB
  //landing_controller_.LQT(&ref_rotor_velocities);                                             // LQT
  /* std::cout<<"w0:"<<ref_rotor_velocities(0)
                   <<", w1:"<<ref_rotor_velocities(1)
                   <<", w2:"<<ref_rotor_velocities(2)
                   <<", w3:"<<ref_rotor_velocities(3)
                   <<", w4:"<<ref_rotor_velocities(4)
                   <<", w5:"<<ref_rotor_velocities(5)<<std::endl; // */

  // for test
  landing_controller_.GetRotorVelSqr(rotor_velocities_sqr_);
  mav_msgs::ActuatorsPtr rotor_velocities_uncorrect(new mav_msgs::Actuators);
  rotor_velocities_uncorrect->angular_velocities.clear();
  for (int i = 0; i < 6; i++)
  {
    rotor_velocities_uncorrect->angular_velocities.push_back(rotor_velocities_sqr_(i));
  }
  rotor_velocities_uncorrect->header.stamp = t_;
  motor_velocity_reference_uncorrect_pub_.publish(rotor_velocities_uncorrect); // */

  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  actuator_msg->header.stamp = t_;
  motor_velocity_reference_pub_.publish(actuator_msg);
}

void LandingControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {
  ROS_INFO_ONCE("LandingController got first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  landing_controller_.SetOdometry(odometry);
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "landing_controller_node");

  rotors_control::LandingControllerNode landing_controller_node;

  ros::spin();

  return 0;
}

