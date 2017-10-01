#include "tud_firefly_control/differential_tracker.h"
#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

bool start = false;
double frequency = 1000.0;
double dt = 1./frequency;

void orderCallback(const std_msgs::Bool& msg)
{
  start = msg.data;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "firefly_W_tracker_node");
  ros::NodeHandle nh_;
  ros::Publisher odomPub_;
  ros::Subscriber orderSub_;
  ros::Time::init();

  ros::Time curTime(0);
  ros::Time lastTime = ros::Time::now();

  ros::Rate loop_rate(frequency);

  std_msgs::Float64MultiArray msg;
  msg.data.resize(18);
  int dim = 6;

  orderSub_ = nh_.subscribe("firefly/motor_status", 1, orderCallback);
  odomPub_ = nh_.advertise<std_msgs::Float64MultiArray>("odom_test", 1);
  Eigen::VectorXd odom(dim), odom_f(dim), odom_v(dim), tau(dim);
  std::vector<int> tauConfig(dim);
  for (int i=0;i<dim;i++)
  {
    tauConfig[i] = 1;
    tau(i) = 1.0;
    odom(i) = 100;
  }

  rotors_control::DifferentialTracker W_tracker;
  W_tracker.init(dim, tau, tauConfig);


  while (ros::ok())
  {
    ros::spinOnce();
    if (curTime.isZero())
    {
      dt = 1./frequency;
    }
    else
    {
      dt = (ros::Time::now() - lastTime).toSec();
    }
    lastTime = ros::Time::now();

    if (start)
    {
      ROS_INFO_ONCE("Tracker test start!");
      W_tracker.track(odom, odom_f, odom_v, dt);
      for (int i=0;i<dim;i++)
      {
        msg.data[i] = odom(i);
        msg.data[6+i] = odom_f(i);
        msg.data[12+i] = odom_v(i);
      }
      odomPub_.publish(msg);
    }

    loop_rate.sleep();
  }

  return 0;
}
