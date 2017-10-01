#include "landing_monitor_node.h"

#include <algorithm>
#include <map>
#include <string>
#include <utility>
#include <vector>
#include <limits>

namespace landing_monitor{

LandingMonitor::LandingMonitor():
  nhLocal_("~")
{}

LandingMonitor::~LandingMonitor()
{
  topicSubs_.clear();
}

void LandingMonitor::odometryCallback(const nav_msgs::Odometry::ConstPtr &msg, const std::string &odomTopic)
{
  geometry_msgs::TransformStamped odomTransMsg;
  odomTransMsg.header.stamp = msg->header.stamp;
  odomTransMsg.header.frame_id = msg->header.frame_id;
  odomTransMsg.child_frame_id = odomTopic;

  odomTransMsg.transform.translation.x = msg->pose.pose.position.x;
  odomTransMsg.transform.translation.y = msg->pose.pose.position.y;
  odomTransMsg.transform.translation.z = msg->pose.pose.position.z;
  odomTransMsg.transform.rotation = msg->pose.pose.orientation;

  // odomBroadcaster_.sendTransform(odomTransMsg);
  TransformsBroadcasters_[odomTopic].sendTransform(odomTransMsg);

  //std::cout<<"header.frame_id is: "<<msg->header.frame_id<<std::endl;
 // std::cout<<"topicName is: "<<odomTopic<<std::endl;
}

void LandingMonitor::initialize()
{
  size_t topicInd = 0;
  bool moreParams = false;
  do
  {
    std::stringstream ss;
    ss << "odom" << topicInd++;
    std::string odomTopicName = ss.str();
    moreParams = nhLocal_.hasParam(odomTopicName);

     if (moreParams)
     {
       std::string odomTopic;
       nhLocal_.getParam(odomTopicName, odomTopic);

       int odomQueueSize = 1;
       nhLocal_.param(odomTopicName + "_queue_size", odomQueueSize, 1);

       bool nodelayOdom = false;
       nhLocal_.param(odomTopicName + "_nodelay", nodelayOdom, false);

       tf::TransformBroadcaster TB;
       TransformsBroadcasters_.insert(std::map<std::string, tf::TransformBroadcaster>::value_type(odomTopic, TB));

       topicSubs_.push_back(
          nh_.subscribe<nav_msgs::Odometry>(odomTopic, odomQueueSize,
             boost::bind(&LandingMonitor::odometryCallback, this, _1, odomTopic),
                ros::VoidPtr(), ros::TransportHints().tcpNoDelay(nodelayOdom)));

      // hl = odomTopic+"\t\t\t\t\t\t\t\t\t\t\t\t\t" ; // a odometry has 13 dimensions "t x y z r p y u v w p q r"
     }
  }
  while(moreParams);
}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "landing_monitor");

 landing_monitor::LandingMonitor landing_monitor;

 landing_monitor.initialize();

  // rotors_control::JoyInterface joy_interface_;
  ros::spin();

  return 0;
}
