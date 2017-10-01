#ifndef DIFFERENTIAL_TRACKER_H
#define DIFFERENTIAL_TRACKER_H

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <std_msgs/Float64MultiArray.h>

namespace rotors_control {

class DifferentialTracker
{
public:
  DifferentialTracker();
  ~DifferentialTracker();

  void init(const ros::NodeHandle& pnh, const std::string& trackerName);
  void init(const int& dim, const  Eigen::VectorXd& tau, const std::vector<int>& tauConfig);
  void report(Eigen::VectorXd& tau, std::vector<int>& config);
  void reportX(const Eigen::VectorXd& x);
  void reset();
  void diff(const Eigen::VectorXd& x, Eigen::VectorXd& dx_f_out, const double &dt);
  void trackAdapt(const Eigen::VectorXd& x, Eigen::VectorXd& dx_f_out, const double& adaptVar, const double &dt);
  void track(const Eigen::VectorXd& x, Eigen::VectorXd& x_f_out, const double &dt);
  void track(const Eigen::VectorXd& x, Eigen::VectorXd& x_f_out, Eigen::VectorXd& dx_f_out, const double &dt);
  void track(const Eigen::VectorXd& x, Eigen::VectorXd& x_f_out, Eigen::VectorXd& dx_f_out, Eigen::VectorXd& ddx_f_out, const double &dt);

  void PT1(const double& x, const int& i, const double& dt);
  void PT2(const double& x, const int& i, const double& dt);
  void pass(const double& x, const int& i, const double& dt);
  void prevent(const double& x, const int& i, const double& dt);
  void adaptPT2(const double& x, const int& i, const double& dt);

  void publish(const Eigen::VectorXd& x);

protected:

  void (DifferentialTracker::*switchMethod[5])(const double& x, const int& i, const double& dt);

  Eigen::VectorXd x_f;
  Eigen::VectorXd dx_f;
  Eigen::VectorXd ddx_f;
  Eigen::VectorXd  tau_;
  double adaptVar_;
  std::vector<int>  tauConfig_;

  int dim_;
  bool pubResult_;
  bool firstValueReceived_;

  std::string trackerName_;
  std_msgs::Float64MultiArray resultMsg_;
  ros::Publisher resultPub_;

  ros::NodeHandle nh_;
  ros::NodeHandle nhLocal_;
};

}

#endif // DIFFERENTIAL_TRACKER_H
