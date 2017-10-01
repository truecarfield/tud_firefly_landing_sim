/* =================================================================================================
// This source file is modified by Ding Zhang from the original file gazebo_ros_sonar.cpp from
// the package hector_gazebo_plugins, Copyright (c) 2012, Johannes Meyer, TU Darmstadt
*/

#include <tud_firefly_gazebo_plugins/gazebo_sonar_plugin.h>

#include <limits>
#include <gazebo/gazebo_config.h>
#include <boost/bind.hpp>

namespace gazebo {

GazeboSonarPlugin::GazeboSonarPlugin()
  : SensorPlugin(),
    node_handle_(0) {}

GazeboSonarPlugin::~GazeboSonarPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);

  sensor_->SetActive(false);

  if (node_handle_)
  {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

// Load the controller
void GazeboSonarPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
  sensor_ = boost::dynamic_pointer_cast<sensors::RaySensor>(_sensor);
  if (!sensor_)
  {
    gzthrow("GazeboSonarPlugin requires a Ray Sensor as its parent");
    return;
  }

  // Get the world name.
  std::string worldName = sensor_->GetWorldName();
  world_ = physics::get_world(worldName);

  // default params
  namespace_.clear();
  topic_ = "sonar";
  frame_id_ = "/sonar_link";

  // load parameters
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();
  else
    gzerr << "[gazebo_sonar_plugin] Please specify a robotNamespace.\n";

  if (_sdf->HasElement("frameId"))
    frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();
  else
    gzerr << "[gazebo_sonar_plugin] Please specify a frameId.\n";

  if (_sdf->HasElement("topicName"))
    topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();
  else
    gzerr << "[gazebo_sonar_plugin] Please specify a topicName.\n";

  last_time_= world_->GetSimTime();

  range_.header.frame_id = frame_id_;
  range_.radiation_type = sensor_msgs::Range::ULTRASOUND;

  range_.field_of_view = std::min(fabs((sensor_->GetAngleMax() - sensor_->GetAngleMin()).Radian()), fabs((sensor_->GetVerticalAngleMax() - sensor_->GetVerticalAngleMin()).Radian()));
  range_.max_range = sensor_->GetRangeMax();
  range_.min_range = sensor_->GetRangeMin();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  node_handle_ = new ros::NodeHandle(namespace_);
  publisher_ = node_handle_->advertise<sensor_msgs::Range>(topic_, 1);

  // connect Update function
  this->updateConnection_ =
      event::Events::ConnectWorldUpdateBegin(
                                                            boost::bind(&GazeboSonarPlugin::OnUpdate, this, _1));
  // activate RaySensor
  sensor_->SetActive(true);
}

// This gets called by the world update start event.
void GazeboSonarPlugin::OnUpdate(const common::UpdateInfo& _info)
{
  common::Time current_time  = world_->GetSimTime();
  double dt = (current_time - last_time_).Double();
  last_time_ = current_time;
  double t = current_time.Double();

  // activate RaySensor if it is not yet active
  if (!sensor_->IsActive()) sensor_->SetActive(true);

  range_.header.stamp.sec  = current_time.sec;
  range_.header.stamp.nsec = current_time.nsec;

  // find ray with minimal range
  range_.range = std::numeric_limits<sensor_msgs::Range::_range_type>::max();

  int num_ranges = sensor_->GetLaserShape()->GetSampleCount() * sensor_->GetLaserShape()->GetVerticalSampleCount();

  for(int i = 0; i < num_ranges; ++i) {
    double ray = sensor_->GetLaserShape()->GetRange(i);
    if (ray < range_.range) range_.range = ray;
  }

  // add Gaussian noise (and limit to min/max range)
  if (range_.range < range_.max_range) {
    // range_.range = sensor_model_(range_.range, dt);
    if (range_.range < range_.min_range) range_.range = range_.min_range;
    if (range_.range > range_.max_range) range_.range = range_.max_range;
  }
//  std::cout<<"ray update time is "<<range_.header.stamp.toSec()<<std::endl;// by ZD
  publisher_.publish(range_);
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboSonarPlugin)

} // namespace gazebo
