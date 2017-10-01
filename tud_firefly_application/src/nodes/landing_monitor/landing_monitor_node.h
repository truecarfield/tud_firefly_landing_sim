#ifndef LANDING_MONITOR_NODE_H
#define LANDING_MONITOR_NODE_H

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <nav_msgs/Odometry.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/AccelWithCovariance.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>

#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>

#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <vector>

namespace landing_monitor{

class LandingMonitor{
public:
  LandingMonitor();
  ~LandingMonitor();

 void initialize();
 void run();

 void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg, const std::string &odomTopic);

 ros::NodeHandle nh_;
 ros::NodeHandle nhLocal_;
 std::vector<ros::Subscriber> topicSubs_;
 tf2_ros::TransformBroadcaster odomBroadcaster_;
 std::map<std::string, tf::TransformBroadcaster> TransformsBroadcasters_;

 std::FILE *file;

};

}

#endif // LANDING_MONITOR_NODE_H
