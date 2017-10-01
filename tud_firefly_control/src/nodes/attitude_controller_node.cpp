
#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "attitude_controller_node.h"
#include <rotors_control/parameters_ros.h>

namespace rotors_control{

FireflyAttitudeControllerNode::FireflyAttitudeControllerNode():
  t(0),
  dt(0.001)
{
  InitializeParams();

  ros::NodeHandle nh;

  motor_sub_ = nh.subscribe("motor_status",1, &FireflyAttitudeControllerNode::motorStatusCallback, this);
  clock_sub_ = nh.subscribe("clock", 1, &FireflyAttitudeControllerNode::clockCallback, this);
  cmd_sub_= nh.subscribe("cmd", 1, &FireflyAttitudeControllerNode::commandCallback, this);
  odom_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &FireflyAttitudeControllerNode::odometryCallback, this);
  motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>("firefly/command/motor_speed", 1);
  command_timer_ = nh.createTimer(ros::Duration(0), &FireflyAttitudeControllerNode::timedCommandCallback, this, true, false);
}

FireflyAttitudeControllerNode::~FireflyAttitudeControllerNode() { }

void FireflyAttitudeControllerNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  // read controller parameters
  firefly_attitude_controller_.InitializeParams(pnh);
}

void FireflyAttitudeControllerNode::motorStatusCallback(const std_msgs::Bool& motor_status_msg) {
  firefly_attitude_controller_.ControllerActivation(motor_status_msg.data);
}

void FireflyAttitudeControllerNode::clockCallback(const rosgraph_msgs::Clock& clock) {
  if (t.isZero())
  {
  }
  else
  {
    dt = clock.clock - t;
  }
  t = clock.clock;
  firefly_attitude_controller_.SetPeriod(dt);

  Eigen::VectorXd ref_rotor_velocities;
  firefly_attitude_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  actuator_msg->header.stamp = t;
  actuator_msg->header.seq++;

  motor_velocity_reference_pub_.publish(actuator_msg);
}

void FireflyAttitudeControllerNode::timedCommandCallback(const ros::TimerEvent& e) {}

void FireflyAttitudeControllerNode::commandCallback(const std_msgs::Float64MultiArray& msg) {
    firefly_attitude_controller_.SetCmd(msg.data);
}

void FireflyAttitudeControllerNode::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  ROS_INFO_ONCE("FireflyTwistController got first odometry message.");
  EigenOdometry odometry;
  eigenOdometryFromMsg(msg, &odometry);
  firefly_attitude_controller_.SetOdometry(odometry);
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "firefly_attitude_controller_node");

  rotors_control::FireflyAttitudeControllerNode firefly_attitude_controller_node;

  ros::spin();

  return 0;
}
