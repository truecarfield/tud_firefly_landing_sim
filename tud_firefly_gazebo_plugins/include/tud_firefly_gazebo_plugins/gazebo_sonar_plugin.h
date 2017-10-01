/* =================================================================================================
// This source file is modified by Ding Zhang from the original file gazebo_ros_sonar.h from
// the package hector_gazebo_plugins, Copyright (c) 2012, Johannes Meyer, TU Darmstadt
*/

#ifndef GAZEBO_SONAR_PLUGIN_H
#define GAZEBO_SONAR_PLUGIN_H

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <gazebo/sensors/RaySensor.hh>

#include <sensor_msgs/Range.h>

// #include <dynamic_reconfigure/server.h>

namespace gazebo
{

class GazeboSonarPlugin : public SensorPlugin {
public:
  GazeboSonarPlugin();
  virtual ~GazeboSonarPlugin();

protected:
  void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& _info);

private:
  /// \brief The parent World
  physics::WorldPtr world_;

  sensors::RaySensorPtr sensor_;

  ros::NodeHandle* node_handle_;
  ros::Publisher publisher_;

  sensor_msgs::Range range_;
  common::Time last_time_;

  std::string namespace_;
  std::string topic_;
  std::string frame_id_;

  // SensorModel sensor_model_;

  // UpdateTimer updateTimer;
  event::ConnectionPtr updateConnection_;

  // boost::shared_ptr<dynamic_reconfigure::Server<SensorModelConfig> > dynamic_reconfigure_server_;
};

} // namespace gazebo

#endif // GAZEBO_SONAR_PLUGIN_H
