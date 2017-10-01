#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_ODOMETRY_MOD_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_ODOMETRY_MOD_PLUGIN_H

#include <cmath>
#include <deque>
#include <random>
#include <stdio.h>

#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <mav_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/core/core.hpp>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <rotors_gazebo_plugins/common.h>
#include <rotors_gazebo_plugins/sdf_api_wrapper.hpp>
#include <tf/transform_broadcaster.h>

namespace gazebo {
// Default values
static const std::string kDefaultParentFrameId = "world";  // by ZD
static const std::string kDefaultOdometryFrameId = "odom";  // by ZD
static const std::string kDefaultWorldFrameId = "world";   // by ZD
static const std::string kDefaultChildFrameId = "odometry_sensor";
static const std::string kDefaultLinkName = "odometry_sensor_link";

static constexpr int kDefaultMeasurementDelay = 0;
static constexpr int kDefaultMeasurementDivisor = 1;
static constexpr int kDefaultGazeboSequence = 0;
static constexpr int kDefaultOdometrySequence = 0;
static constexpr double kDefaultUnknownDelay = 0.0;
static constexpr double kDefaultCovarianceImageScale = 1.0;

class GazeboOdometryModPlugin : public ModelPlugin {
 public:
  typedef std::normal_distribution<> NormalDistribution;
  typedef std::uniform_real_distribution<> UniformDistribution;
  typedef std::deque<std::pair<int, nav_msgs::Odometry> > OdometryQueue;

  GazeboOdometryModPlugin()
      : ModelPlugin(),
        random_generator_(random_device_()),
        pose_pub_topic_(mav_msgs::default_topics::POSE),
        pose_with_covariance_pub_topic_(mav_msgs::default_topics::POSE_WITH_COVARIANCE),
        position_pub_topic_(mav_msgs::default_topics::POSITION),
        transform_pub_topic_(mav_msgs::default_topics::TRANSFORM),
        odometry_pub_topic_(mav_msgs::default_topics::ODOMETRY),
        odometry_frame_id_(kDefaultOdometryFrameId),  // frame_id of the to be published odometry message by ZD
        world_frame_id_(kDefaultWorldFrameId),  // frame_id of the world in Gazebo by ZD
        parent_frame_id_(kDefaultParentFrameId), // frame id of the tf parent  by ZD
        child_frame_id_(kDefaultChildFrameId),
        link_name_(kDefaultLinkName),
        measurement_delay_(kDefaultMeasurementDelay),
        measurement_divisor_(kDefaultMeasurementDivisor),
        unknown_delay_(kDefaultUnknownDelay),
        gazebo_sequence_(kDefaultGazeboSequence),
        odometry_sequence_(kDefaultOdometrySequence),
        covariance_image_scale_(kDefaultCovarianceImageScale),
        node_handle_(NULL) {}

  ~GazeboOdometryModPlugin();

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:
  OdometryQueue odometry_queue_;

  std::string namespace_;
  std::string pose_pub_topic_;
  std::string pose_with_covariance_pub_topic_;
  std::string position_pub_topic_;
  std::string transform_pub_topic_;
  std::string odometry_pub_topic_;

  std::string odometry_frame_id_;  //  frame_id of the to be published odometry message  by ZD
  std::string world_frame_id_;  //  frame_id of the world in Gazebo by ZD
  std::string parent_frame_id_;  // frame id of the tf parent  by ZD
  std::string child_frame_id_;
  std::string link_name_;

  NormalDistribution position_n_[3];
  NormalDistribution attitude_n_[3];
  NormalDistribution linear_velocity_n_[3];
  NormalDistribution angular_velocity_n_[3];
  UniformDistribution position_u_[3];
  UniformDistribution attitude_u_[3];
  UniformDistribution linear_velocity_u_[3];
  UniformDistribution angular_velocity_u_[3];

  geometry_msgs::PoseWithCovariance::_covariance_type pose_covariance_matrix_;
  geometry_msgs::TwistWithCovariance::_covariance_type twist_covariance_matrix_;

  int measurement_delay_;
  int measurement_divisor_;
  int gazebo_sequence_;
  int odometry_sequence_;
  double unknown_delay_;
  double covariance_image_scale_;
  cv::Mat covariance_image_;

  std::random_device random_device_;
  std::mt19937 random_generator_;

  ros::NodeHandle* node_handle_;
  ros::Publisher pose_pub_;
  ros::Publisher pose_with_covariance_pub_;
  ros::Publisher position_pub_;
  ros::Publisher transform_pub_;
  ros::Publisher odometry_pub_;

  tf::Transform tf_;
  tf::TransformBroadcaster transform_broadcaster_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  physics::EntityPtr parent_link_;

  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread();
};
}

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_ODOMETRY__MOD_PLUGIN_H

