#include "tud_firefly_state_estimation/mahony_complementary_filter.h"
#include <tud_firefly_control/tud_firefly_common.h>

#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/MagneticField.h>

#include <XmlRpcException.h>
#include <ros/ros.h>
#include <deque>
#include <boost/thread/mutex.hpp>

namespace RobotLocalization {

class MahonyCompFilterNode {
public:
   MahonyCompFilterNode();
   ~MahonyCompFilterNode();
   void init();

   void clockCallback(const rosgraph_msgs::Clock& msg);
   void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
   void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg);
   void accelCoCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
protected:
   boost::mutex command_mutex_;

   Eigen::Vector3d filteredAngle_;

   // Measurements
   Eigen::VectorXd accelData_;
   Eigen::VectorXd gyrData_;
   Eigen::VectorXd magData_;

   // Low pass prefiltered measurements
   Eigen::VectorXd accel_lp_;
   Eigen::VectorXd gyr_lp_;
   Eigen::VectorXd mag_lp_;

   Eigen::Vector3d accelCo_; // for test
   ros::Time lastMessageTime_;
   double dt;
   sensor_msgs::Imu imu_message_;

   bool useMag_;

   ros::NodeHandle nh_;
   ros::NodeHandle nhLocal_;

   ros::Subscriber clockSub_;
   ros::Subscriber imuSub_;
   ros::Subscriber magSub_;
   ros::Subscriber accelCoSub_;
   ros::Publisher imuPub_;
   ros::Publisher testPub_;

   std::string clockSubTopic;
   std::string magSubTopic;
   std::string imuSubTopic;
   std::string accelCoSubTopic;
   std::string testPubTopic;
   std::string imuPubTopic;

   MahonyCompFilter mahony_complementary_filter_;

   rotors_control::DifferentialTracker accelTracker_;
   rotors_control::DifferentialTracker gyrTracker_;
   rotors_control::DifferentialTracker magnTracker_;
};

MahonyCompFilterNode::MahonyCompFilterNode():
   nhLocal_("~"),
   imuPubTopic("firefly/mahony_complementary/imu"),
   clockSubTopic("clock"),
   magSubTopic("firefly/magnetic_field"),
   imuSubTopic("firefly/imu"),
   accelCoSubTopic("imu0_accel_corrected"),
   useMag_(true)
 {
   dt = 0.001;
   filteredAngle_.resize(3);
   accelData_.resize(3);
   gyrData_.resize(3);
   magData_.resize(3);
   accelCo_.resize(3);
   accel_lp_.resize(3);
   gyr_lp_.resize(3);
   mag_lp_.resize(3);

   filteredAngle_.setZero();
   accelData_.setZero();
   gyrData_.setZero();
   magData_.setZero();
   accelCo_.setZero();
   accel_lp_.setZero();
   gyr_lp_.setZero();
   mag_lp_.setZero();

   init();
 }

MahonyCompFilterNode::~MahonyCompFilterNode() {}

void MahonyCompFilterNode::init()
{
  std::vector<double> kStdVector(4, 0);
  Eigen::Vector4d k;
  if (nhLocal_.getParam("k", kStdVector))
  {
    if(kStdVector.size() != 4)
    {
      ROS_ERROR_STREAM("Constant k of the complementary filter must be of size " << 4 << ". Provided config was of "
      "size " << kStdVector.size() << ". This will lead to serious failure!");
    }
    else
    {
      k << kStdVector[0], kStdVector[1], kStdVector[2],  kStdVector[3];
    }
  }

  // Load up the tau for differential filter of the gyroscope (from the launch file/parameter server)
  std::vector<double> tauGyrStdVector(3, 0);
  Eigen::Vector3d tauGyr;
  tauGyr.setZero();
  if (nhLocal_.getParam("tau_gyr", tauGyrStdVector))
  {
    if(tauGyrStdVector.size() != 3)
    {
      ROS_ERROR_STREAM("Tau of the gyroscope differential filter must be of size " << 3 << ". Provided config was of "
      "size " << tauGyrStdVector.size() << ". No time constants will be initialized for gyroscope differential filter");
    }
    else
    {
      /* std::cout<<"tauGyrStdVector0: "
                       <<tauGyrStdVector[0]<<std::endl
                       <<"tauGyrStdVector1: "
                       <<tauGyrStdVector[1]<<std::endl
                       <<"tauGyrStdVector2: "
                       <<tauGyrStdVector[2]<<std::endl; */
      tauGyr << tauGyrStdVector[0], tauGyrStdVector[1], tauGyrStdVector[2];
    }
  }
  else
  {
    ROS_ERROR_STREAM("Tau of the gyroscope differential filter initialization failed. No time constants will be initialized for gyroscope differential filter");
  }

  // Load up the tau config for differential filter of the gyroscope
  std::vector<int> tauGyrConfig(3, 0);
  if(nhLocal_.getParam("tau_gyr_config", tauGyrConfig))
  {
    if(tauGyrConfig.size() != 3)
    {
      ROS_ERROR_STREAM("Tau configuration of the gyroscope differential filter must be of size " << 3 << ". Provided config was of "
        "size " << tauGyrConfig.size() << ". ");
    }
  }

  // Load up the tau for differential filter of the gyroscope (from the launch file/parameter server)
  std::vector<double> tauAccelStdVector(3, 0);
  Eigen::Vector3d tauAccel;
  tauAccel.setZero();
  if (nhLocal_.getParam("tau_accel", tauAccelStdVector))
  {
    /* std::cout<<"tauAccelStdVector0: "
                     <<tauAccelStdVector[0]<<std::endl
                     <<"tauAccelStdVector1: "
                     <<tauAccelStdVector[1]<<std::endl
                     <<"tauAccelStdVector2: "
                     <<tauAccelStdVector[2]<<std::endl; */
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
  else
  {
    ROS_ERROR_STREAM("Tau of the accelerometer differential filter initialization failed. No time constants will be initialized for gyroscope differential filter");
  }

  // Load up the tau config for differential filter of the accelerometer
  std::vector<int> tauAccelConfig(3, 0);
  if(nhLocal_.getParam("tau_accel_config", tauAccelConfig))
  {
    if(tauAccelConfig.size() != 3)
    {
      ROS_ERROR_STREAM("Tau configuration of the accelerometer differential filter must be of size " << 3 << ". Provided config was of "
        "size " << tauGyrConfig.size() << ". ");
    }
  }

  nhLocal_.param("use_mag", useMag_, useMag_);
   std::vector<double> tauMagStdVector(3, 0);
  std::vector<int> tauMagConfig(3, 0);
  Eigen::Vector3d tauMag;
  tauMag.setZero();
  if (useMag_)
  {
    // Load up the tau for differential filter of the magnetometer (from the launch file/parameter server)
    if (nhLocal_.getParam("tau_mag", tauMagStdVector))
    {
      if(tauMagStdVector.size() != 3)
      {
        ROS_ERROR_STREAM("Tau of the magnetometer differential filter must be of size " << 3 << ". Provided config was of "
        "size " << tauMagStdVector.size() << ". No time constants will be initialized for magnetometer differential filter");
      }
      else
      {
        tauMag << tauMagStdVector[0], tauMagStdVector[1], tauMagStdVector[2];
      }
    }
    else
    {
      ROS_ERROR_STREAM("Tau of the magnetometer differential filter initialization failed. No time constants will be initialized for gyroscope differential filter");
    }

    // Load up the tau config for differential filter of the magnetometer
    if(nhLocal_.getParam("tau_mag_config", tauMagConfig))
    {
      if(tauMagConfig.size() != 3)
      {
        ROS_ERROR_STREAM("Tau configuration of the magnetometer differential filter must be of size " << 3 << ". Provided config was of "
          "size " << tauMagConfig.size() << ". ");
      }
    }
  }
  else
  {
    ROS_INFO_NAMED("mahony_complementary_filter", "Magnetometer data is deactivated for mahony complementary filter.");
    tauMag << 0.0, 0.0, 0.0;
    tauMagConfig.push_back(0.0);
    tauMagConfig.push_back(0.0);
    tauMagConfig.push_back(0.0);
  }

 /*  std::cout<<"tauGyr is "<<std::endl
                   <<tauGyr<<std::endl
                   <<"tauAccel is "<<std::endl
                   <<tauAccel<<std::endl
                   <<"tauMag is "<<std::endl
                   <<tauMag<<std::endl; */
  gyrTracker_.init(tauGyr.size(), tauGyr, tauGyrConfig);
  accelTracker_.init(tauAccel.size(), tauAccel, tauAccelConfig);
  if (useMag_)
  {
    magnTracker_.init(tauMagConfig.size(), tauMag, tauMagConfig);
  }
  mahony_complementary_filter_.initParams(k, tauGyr, tauAccel, tauMag, tauGyrConfig, tauAccelConfig, tauMagConfig);

  nhLocal_.param("clock_sub_topic", clockSubTopic, clockSubTopic);
  nhLocal_.param("mag_sub_topic", magSubTopic, magSubTopic);
  nhLocal_.param("imu_sub_topic", imuSubTopic, imuSubTopic);
  nhLocal_.param("mahony_pub_topic", imuPubTopic, imuPubTopic);

  clockSub_ = nh_.subscribe(clockSubTopic, 1, &MahonyCompFilterNode::clockCallback, this);
  magSub_ = nh_.subscribe<sensor_msgs::MagneticField>(magSubTopic, 1, &MahonyCompFilterNode::magCallback, this);
  imuSub_ = nh_.subscribe<sensor_msgs::Imu>(imuSubTopic, 1, &MahonyCompFilterNode::imuCallback, this);
  accelCoSub_  = nh_.subscribe<geometry_msgs::PointStamped>(accelCoSubTopic, 2, &MahonyCompFilterNode::accelCoCallback, this);

  testPub_ = nh_.advertise<geometry_msgs::Point>("test_g_b", 1);
  imuPub_ = nh_.advertise<sensor_msgs::Imu>(imuPubTopic, 1);
}

void MahonyCompFilterNode::accelCoCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  accelCo_ << msg->point.x, msg->point.y, msg->point.z;
}

void MahonyCompFilterNode::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  boost::mutex::scoped_lock lock(command_mutex_);

  accelData_ << msg->linear_acceleration.x,
                              msg->linear_acceleration.y,
                              msg->linear_acceleration.z;
     gyrData_ << msg->angular_velocity.x,
                              msg->angular_velocity.y,
                              msg->angular_velocity.z;
  imu_message_.header = msg->header;
  imu_message_.angular_velocity = msg->angular_velocity;
  imu_message_.angular_velocity_covariance = msg->angular_velocity_covariance;
  imu_message_.linear_acceleration = msg->linear_acceleration;
  imu_message_.linear_acceleration_covariance = msg->linear_acceleration_covariance;
}

void MahonyCompFilterNode::magCallback(const sensor_msgs::MagneticField::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(command_mutex_);
  magData_ << msg->magnetic_field.x,
                             msg->magnetic_field.y,
                             msg->magnetic_field.z;
}

void MahonyCompFilterNode::clockCallback(const rosgraph_msgs::Clock &msg)
{
  boost::mutex::scoped_lock lock(command_mutex_);
  if (dt == 0.0)
  {
    dt = 0.001;
  }
  else
  {
    dt = (msg.clock - lastMessageTime_).toSec();
  }

  accelTracker_.track(accelData_, accel_lp_, dt);
  gyrTracker_.track(gyrData_, gyr_lp_, dt);
  if (useMag_)
  {
    magnTracker_.track(magData_, mag_lp_, dt);
  }

  //std::cout<<"iter:"<<msg.clock.toSec()<<std::endl
  //                 <<"acclp:"<<accel_lp_<<std::endl
  //                 <<"gyrlp:"<<gyr_lp_<<std::endl
  //                 <<"maglp:"<<mag_lp_<<std::endl;

  lastMessageTime_ = msg.clock;
  Eigen::Quaterniond quaternionFilteredData;
  mahony_complementary_filter_.update(accel_lp_, gyr_lp_, mag_lp_, dt, quaternionFilteredData);
  imu_message_.header.stamp = msg.clock;
  imu_message_.orientation.w = quaternionFilteredData.w();
  imu_message_.orientation.x = quaternionFilteredData.x();
  imu_message_.orientation.y = quaternionFilteredData.y();
  imu_message_.orientation.z = quaternionFilteredData.z();
  imu_message_.orientation_covariance[0] = 1e-8;

  /* Eigen::Vector3d testpoint_;
  mahony_complementary_filter_.sendTest(testpoint_);
  geometry_msgs::Point testmsg;
  testmsg.x = testpoint_(0);
  testmsg.y = testpoint_(1);
  testmsg.z = testpoint_(2);
  testPub_.publish(testmsg); */
  imuPub_.publish(imu_message_);
  // */
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "complementary_filter");

  RobotLocalization::MahonyCompFilterNode comp_filter_node;

  ros::spin();

  return 0;
}


