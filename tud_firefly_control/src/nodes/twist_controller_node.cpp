
#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "twist_controller_node.h"
#include <rotors_control/parameters_ros.h>

namespace rotors_control{

FireflyTwistControllerNode::FireflyTwistControllerNode():
  t_in_sec(0.0),
  dt_in_sec(0.001),
  dt_in_sec_auto(0.001),
  auto_step_size(false) {

InitializeParams();

ros::NodeHandle nh;

motor_sub_ = nh.subscribe("motor_status",1, &FireflyTwistControllerNode::MotorStatusCallback, this);

// command twist
cmd_twist_sub_= nh.subscribe("cmd_vel", 1, &FireflyTwistControllerNode::CommandTwistCallback, this);

// self twist
odom_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1, &FireflyTwistControllerNode::OdometryCallback, this);

gt_imu_sub_ = nh.subscribe(mav_msgs::default_topics::IMU, 1,  &FireflyTwistControllerNode::ImuCallback, this);

clock_sub_ = nh.subscribe("clock", 1, &FireflyTwistControllerNode::ClockCallback, this);

step_sub_ = nh.subscribe("firefly/sim_update_event", 1, &FireflyTwistControllerNode::StepsizeCallback, this);

// output
motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(
   "firefly/command/motor_speed", 1);

// command_timer_ = nh.createTimer(ros::Duration(0), &FireflyTwistControllerNode::TimedCommandCallback, this, true, false);
}

FireflyTwistControllerNode::~FireflyTwistControllerNode() { }

void FireflyTwistControllerNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  // read controller parameters
  firefly_twist_controller_.pidX_.initialize(ros::NodeHandle(pnh, "linear/xy"));
  firefly_twist_controller_.pidY_.initialize(ros::NodeHandle(pnh, "linear/xy"));
  firefly_twist_controller_.pidZ_.initialize(ros::NodeHandle(pnh, "linear/z"));
  firefly_twist_controller_.pidR_.initialize(ros::NodeHandle(pnh, "angular/xy"));
  firefly_twist_controller_.pidP_.initialize(ros::NodeHandle(pnh, "angular/xy"));
  firefly_twist_controller_.pidYaw_.initialize(ros::NodeHandle(pnh, "angular/z"));

  // Read parameters from rosparam.
  GetRosParameter(pnh, "velocity_gain/x",
                  firefly_twist_controller_.controller_parameters_.velocity_gain_.x(),
                  &firefly_twist_controller_.controller_parameters_.velocity_gain_.x());
  GetRosParameter(pnh, "velocity_gain/y",
                  firefly_twist_controller_.controller_parameters_.velocity_gain_.y(),
                  &firefly_twist_controller_.controller_parameters_.velocity_gain_.y());
  GetRosParameter(pnh, "velocity_gain/z",
                  firefly_twist_controller_.controller_parameters_.velocity_gain_.z(),
                  &firefly_twist_controller_.controller_parameters_.velocity_gain_.z());
  GetRosParameter(pnh, "attitude_gain/x",
                  firefly_twist_controller_.controller_parameters_.attitude_gain_.x(),
                  &firefly_twist_controller_.controller_parameters_.attitude_gain_.x());
  GetRosParameter(pnh, "attitude_gain/y",
                  firefly_twist_controller_.controller_parameters_.attitude_gain_.y(),
                  &firefly_twist_controller_.controller_parameters_.attitude_gain_.y());
  GetRosParameter(pnh, "attitude_gain/z",
                  firefly_twist_controller_.controller_parameters_.attitude_gain_.z(),
                  &firefly_twist_controller_.controller_parameters_.attitude_gain_.z());
  GetRosParameter(pnh, "angular_rate_gain/x",
                  firefly_twist_controller_.controller_parameters_.angular_rate_gain_.x(),
                  &firefly_twist_controller_.controller_parameters_.angular_rate_gain_.x());
  GetRosParameter(pnh, "angular_rate_gain/y",
                  firefly_twist_controller_.controller_parameters_.angular_rate_gain_.y(),
                  &firefly_twist_controller_.controller_parameters_.angular_rate_gain_.y());
  GetRosParameter(pnh, "angular_rate_gain/z",
                  firefly_twist_controller_.controller_parameters_.angular_rate_gain_.z(),
                  &firefly_twist_controller_.controller_parameters_.angular_rate_gain_.z());
  GetVehicleParameters(pnh, &firefly_twist_controller_.vehicle_parameters_);

  GetVehicleParameters(pnh, &firefly_twist_controller_.vehicle_parameters_);
  firefly_twist_controller_.InitializeParameters();
}

void FireflyTwistControllerNode::MotorStatusCallback(const std_msgs::Bool& motor_status_msg) {
  firefly_twist_controller_.ControllerActivation(motor_status_msg.data);
}

void FireflyTwistControllerNode::ClockCallback(const rosgraph_msgs::Clock& clock) {
  double t_in_sec_now;
  t_in_sec_now = clock.clock.toSec();
  if (dt_in_sec == 0.0)
  {dt_in_sec_auto = 0.001;} else {dt_in_sec_auto = t_in_sec_now - t_in_sec;}
  t_in_sec = t_in_sec_now;
  firefly_twist_controller_.SetTime(t_in_sec);
}

void FireflyTwistControllerNode::StepsizeCallback(const std_msgs::Float64MultiArray& step_msg) {
  if (auto_step_size)
  {dt_in_sec = dt_in_sec_auto;}
  else
  {dt_in_sec = step_msg.data[1];}
  firefly_twist_controller_.SetPeriod(dt_in_sec);
}

// void FireflyTwistControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {}

void FireflyTwistControllerNode::CommandTwistCallback(const geometry_msgs::Twist& twist) {
    firefly_twist_controller_.SetCmdTwist(twist);
} // command twist



void FireflyTwistControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  ROS_INFO_ONCE("FireflyTwistController got first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  firefly_twist_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
  firefly_twist_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  actuator_msg->header.stamp = odometry_msg->header.stamp;
  actuator_msg->header.seq++;

  motor_velocity_reference_pub_.publish(actuator_msg);
} // self twist

void FireflyTwistControllerNode::ImuCallback(const sensor_msgs::Imu& imu) {
  firefly_twist_controller_.SetAccel(imu);
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "firefly_twist_controller_node");

  rotors_control::FireflyTwistControllerNode firefly_twist_controller_node;

  ros::spin();

  return 0;
}

