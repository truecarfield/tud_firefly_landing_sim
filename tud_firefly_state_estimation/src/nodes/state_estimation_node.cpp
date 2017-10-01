#include "tud_firefly_state_estimation/firefly_ros_filter.h"
#include "tud_firefly_state_estimation/firefly_ros_filter_types.h"

#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ekf_landing_node");

  RobotLocalization::FireflyRosEkf ekf_landing;

  ekf_landing.run();

  // RobotLocalization::RosEkfLanding ekf_landing;

  // ekf_landing.run();

  return 0;
}

