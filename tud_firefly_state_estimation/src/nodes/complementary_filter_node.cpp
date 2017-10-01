#include "tud_firefly_state_estimation/complementary_filter.h"
#include <tud_firefly_control/tud_firefly_common.h>
#include <geometry_msgs/PointStamped.h>
#include <XmlRpcException.h>
#include <ros/ros.h>
#include <deque>

namespace RobotLocalization {

class CompFilterNode {
public:
   CompFilterNode();
   ~CompFilterNode();

   void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
   void accelCoCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
   void init();

protected:
   Eigen::Vector3d filteredAngle_;
   Eigen::Vector3d accelData_;
   Eigen::Vector3d gyrData_;
   Eigen::Vector3d accelCo_; // for test
   ros::Time lastMessageTime_;
   double dt;
   sensor_msgs::Imu imu_message_;

   ros::NodeHandle nh_;
   ros::NodeHandle nhLocal_;

   ros::Subscriber imuSub_;
   ros::Subscriber accelCoSub_;
   ros::Publisher imuPub_;
   std::string imuPubTopic;
   std::string imuSubTopic;
   std::string accelCoSubTopic;
   CompFilter complementary_filter_;
};

CompFilterNode::CompFilterNode():
   nhLocal_("~"),
   imuPubTopic("firefly/complementary/imu"),
   imuSubTopic("firefly/imu"),
   accelCoSubTopic("imu0_accel_corrected")
 {
   dt = 0.0;
   filteredAngle_.setZero();
   accelData_.setZero();
   gyrData_.setZero();
   accelCo_.setZero();
   init();
 }

CompFilterNode::~CompFilterNode() {}

void CompFilterNode::init()
{
  std::vector<double> betaStdVector;
  Eigen::Matrix3d beta;
  if (nhLocal_.getParam("beta", betaStdVector))
  {
    if(betaStdVector.size() != 3)
    {
      ROS_ERROR_STREAM("Constant beta of the complementary filter must be of size " << 3 << ". Provided config was of "
      "size " << betaStdVector.size() << ". This will lead to serious failure!");
    }
    else
    {
      beta << betaStdVector[0] , 0.0,   0.0,
                      0.0,   betaStdVector[1],  0.0,
                      0.0,   0.0,   betaStdVector[2];
    }
  }

  // Load up the tau for differential filter of the gyroscope (from the launch file/parameter server)
  std::vector<double> tauGyrStdVector;
  Eigen::Vector3d tauGyr;
  tauGyr.setZero();
  if (nhLocal_.getParam("tau_gyr_config", tauGyrStdVector))
  {
    if(tauGyrStdVector.size() != 3)
    {
      ROS_ERROR_STREAM("Tau of the gyroscope differential filter must be of size " << 3 << ". Provided config was of "
      "size " << tauGyrStdVector.size() << ". No time constants will be initialized for gyroscope differential filter");
    }
    else
    {
      tauGyr << tauGyrStdVector[0], tauGyrStdVector[1], tauGyrStdVector[2];
    }
  }

  // Load up the tau config for differential filter of the gyroscope
  std::vector<int> tauGyrConfig;
  if(nhLocal_.getParam("tau_gyr_config", tauGyrConfig))
  {
    if(tauGyrConfig.size() != 3)
    {
      ROS_ERROR_STREAM("Tau configuration of the gyroscope differential filter must be of size " << 3 << ". Provided config was of "
        "size " << tauGyrConfig.size() << ". ");
    }
  }

  // Load up the tau for differential filter of the gyroscope (from the launch file/parameter server)
  std::vector<double> tauAccelStdVector;
  Eigen::Vector3d tauAccel;
  tauAccel.setZero();
  if (nhLocal_.getParam("tau_accel", tauAccelStdVector))
  {
    if(tauAccelStdVector.size() != 3)
    {
      ROS_ERROR_STREAM("Tau of the accelerometer differential filter must be of size " << 3 << ". Provided config was of "
      "size " << tauAccelStdVector.size() << ". No time constants will be initialized for gyroscope differential filter");
    }
    else
    {
      tauAccel << tauAccelStdVector[0], tauAccelStdVector[1], tauAccelStdVector[2];
    }
  }

  // Load up the tau config for differential filter of the gyroscope
  std::vector<int> tauAccelConfig;
  if(nhLocal_.getParam("tau_accel_config", tauAccelConfig))
  {
    if(tauAccelConfig.size() != 3)
    {
      ROS_ERROR_STREAM("Tau configuration of the accelerometer differential filter must be of size " << 3 << ". Provided config was of "
        "size " << tauGyrConfig.size() << ". ");
    }
  }

  complementary_filter_.init(beta, tauGyr, tauAccel, tauGyrConfig, tauAccelConfig);

  nhLocal_.param("imu_pub_topic", imuPubTopic, imuPubTopic);
  nhLocal_.param("imu_sub_topic", imuSubTopic, imuSubTopic);

  imuSub_ = nh_.subscribe<sensor_msgs::Imu>(imuSubTopic, 1, &CompFilterNode::imuCallback, this);
  accelCoSub_  = nh_.subscribe<geometry_msgs::PointStamped>(accelCoSubTopic, 2, &CompFilterNode::accelCoCallback, this);
  imuPub_ = nh_.advertise<sensor_msgs::Imu>(imuPubTopic, 1);

}

void CompFilterNode::accelCoCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  accelCo_ << msg->point.x, msg->point.y, msg->point.z;
}

void CompFilterNode::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  accelData_ << msg->linear_acceleration.x,
                              msg->linear_acceleration.y,
                              msg->linear_acceleration.z + 9.81*cos(filteredAngle_(1))*cos(filteredAngle_(0));
     gyrData_ << msg->angular_velocity.x,
                              msg->angular_velocity.y,
                              msg->angular_velocity.z;
  if (dt == 0.0)
  {
    dt = 0.001;
  }
  else
  {
    dt = (msg->header.stamp - lastMessageTime_).toSec();
  }
  lastMessageTime_ = msg->header.stamp;

  complementary_filter_.update(accelData_, gyrData_, dt, filteredAngle_);

  imu_message_.header = msg->header;
  imu_message_.angular_velocity = msg->angular_velocity;
  imu_message_.angular_velocity_covariance = msg->angular_velocity_covariance;
  imu_message_.linear_acceleration = msg->linear_acceleration;
  imu_message_.angular_velocity_covariance = msg->angular_velocity_covariance;
  Eigen::Quaterniond quaternionFilteredData;
  quaternionFilteredData = rotors_control::toQuaternion(filteredAngle_(0),  filteredAngle_(1), filteredAngle_(2));
  imu_message_.orientation.w = quaternionFilteredData.w();
  imu_message_.orientation.x = quaternionFilteredData.x();
  imu_message_.orientation.y = quaternionFilteredData.y();
  imu_message_.orientation.z = quaternionFilteredData.z();
  imu_message_.orientation_covariance[0] = 1e-8;

  imuPub_.publish(imu_message_);
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "complementary_filter");

  RobotLocalization::CompFilterNode comp_filter_node;

  ros::spin();

  return 0;
}


