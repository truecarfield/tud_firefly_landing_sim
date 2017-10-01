#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include <ros/ros.h>
#include <ros/subscriber.h>

#include <sensor_msgs/Imu.h>
#include <tud_firefly_control/differential_tracker.h>

namespace RobotLocalization{

class  CompFilter {
public:
  CompFilter();
  ~CompFilter();

  void update(const Eigen::Vector3d& accData, const Eigen::Vector3d& gyrData, const double& dt, Eigen::Vector3d& filteredData);
  void init(const Eigen::Matrix3d& beta,
                    const Eigen::Vector3d& tauGyr,
                    const Eigen::Vector3d& tauAccel,
                    const std::vector<int>& tauGyrConfig,
                    const std::vector<int>& tauAccelConfig);

protected:
  double gravity_;
  Eigen::Matrix3d beta_;
  Eigen::Matrix3d Identity_;

  double gyrRtest;
  bool testbad;

  rotors_control::DifferentialTracker gyrTracker_;
  rotors_control::DifferentialTracker accelTracker_;

  Eigen::VectorXd filteredData_;
  Eigen::Vector3d filteredDataTest_;
};

}

#endif // COMPLEMENTARY_FILTER_H
