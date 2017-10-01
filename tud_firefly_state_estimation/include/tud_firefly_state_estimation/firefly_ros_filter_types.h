#ifndef ROS_FILTER_TYPES_LANDING_H
#define ROS_FILTER_TYPES_LANDING_H

#include <robot_localization/ros_filter.h>
#include "tud_firefly_state_estimation/firefly_ros_filter.h"

#include <robot_localization/ros_filter_types.h>

#include "tud_firefly_state_estimation/firefly_ekf.h"
#include <robot_localization/ekf.h>
#include <robot_localization/ukf.h>

namespace RobotLocalization
{
  typedef FireflyRosFilter<FireflyEkf> FireflyRosEkf;
}

#endif // ROS_FILTER_TYPES_LANDING_H
