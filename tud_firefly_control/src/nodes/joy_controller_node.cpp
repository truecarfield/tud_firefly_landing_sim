#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <rosgraph_msgs/Clock.h>

// Migrated from hector_quadrotor

namespace rotors_control
{

class JoyInterface
{
private:
  ros::NodeHandle node_handle_;
  ros::Subscriber joy_subscriber_;
  ros::Subscriber landed_subscriber_;

  // for test
  ros::Subscriber clock_subcriber_;
  double t_;

  ros::Publisher test_publisher_;
  ros::Publisher velocity_publisher_;
  ros::Publisher attitude_publisher_;
  ros::Publisher motor_publisher_;
  ros::Publisher landing_publisher_;
  geometry_msgs::Twist velocity_;
  std::vector<double> attitude_data_;  // roll. pitch, yaw_rate, Vz_W_
  std_msgs::Float64MultiArray attitude_;

  std_msgs::Bool motor_status_;
  std_msgs::Bool landing_activation_; // Command for landing initialization
  std_msgs::Bool test_activation_;

  struct Axis
  {
    int axis;
    double max;
  };

  struct Button
  {
    int button;
  };

  struct
  {
    Axis x;
    Axis y;
    Axis z;
    Axis yaw;
  } axes_;

  struct
  {
    Button slow, motor, landing, test;
  } buttons_;

  double slow_factor_;

public:
  JoyInterface ()
  {
    ros::NodeHandle params("~");

    t_ = 0.0;

    params.param<int>("x_axis", axes_.x.axis, 2);
    params.param<int>("y_axis", axes_.y.axis, 1);
    params.param<int>("z_axis", axes_.z.axis, 4);
    params.param<int>("yaw_axis", axes_.yaw.axis, 3);

    params.param<double>("yaw_velocity_max", axes_.yaw.max, 90.0 * M_PI / 180.0);
    params.param<int>("slow_button", buttons_.slow.button, 1);
    params.param<int>("motor_status", buttons_.motor.button, 9);
    params.param<int>("landing_activation", buttons_.landing.button, 10);
    params.param<int>("test_activation", buttons_.test.button, 2);
    params.param<double>("slow_factor", slow_factor_, 0.2);

    std::string control_mode_str;
    params.param<std::string>("control_mode", control_mode_str, "twist");
    attitude_.data.resize(4);

    if (control_mode_str == "twist")
    {
      params.param<double>("x_velocity_max", axes_.x.max, 2.0);
      params.param<double>("y_velocity_max", axes_.y.max, 2.0);

      joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("joy", 1, boost::bind(&JoyInterface::joyTwistCallback, this, _1));
      velocity_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("firefly/cmd_vel", 10);
    }
    else if (control_mode_str == "attitude")
    {
      params.param<double>("x_roll_max", axes_.x.max, 0.35);
      params.param<double>("y_pitch_max", axes_.y.max, 0.35);

      joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("joy", 1, boost::bind(&JoyInterface::joyAttitudeCallback, this, _1));
      attitude_publisher_ = node_handle_.advertise<std_msgs::Float64MultiArray>("firefly/cmd", 10);
    } // */
    params.param<double>("z_velocity_max", axes_.z.max, 2.0);
    params.param<double>("yaw_rate_max", axes_.yaw.max, 1.0);

    landed_subscriber_ = node_handle_.subscribe("firefly/landed", 1, &JoyInterface::landedCallback, this);
    //clock_subcriber_ = node_handle_.subscribe("clock", 1, &JoyInterface::clockCallback, this);

    motor_publisher_ = node_handle_.advertise<std_msgs::Bool>("firefly/motor_status",1);
    landing_publisher_ = node_handle_.advertise<std_msgs::Bool>("firefly/landing_activation", 1);
    test_publisher_ = node_handle_.advertise<std_msgs::Bool>("firefly/test_activation", 1);
  }

  ~JoyInterface()
  {
    stop();
  }

  void clockCallback(const rosgraph_msgs::Clock &clock)
  {
    t_ = clock.clock.toSec();
    if (!motor_status_.data) //&& t_ > 0.0 )
    {
      motor_status_.data = true;
      motor_publisher_.publish(motor_status_);
    }

    if (!test_activation_.data)
    {
      test_activation_.data = true;
      test_publisher_.publish(test_activation_);
    }
  }

  void joyTwistCallback(const sensor_msgs::JoyConstPtr &joy)
  {
    velocity_.linear.x = getAxis(joy, axes_.x);
    velocity_.linear.y = getAxis(joy, axes_.y);
    velocity_.linear.z = getAxis(joy, axes_.z);
    velocity_.angular.z = getAxis(joy, axes_.yaw);

    if (getButton(joy, buttons_.slow.button))
    {
      std::cout<<"get slow"<<std::endl;
      velocity_.linear.x *= slow_factor_;
      velocity_.linear.y *= slow_factor_;
      velocity_.linear.z *= slow_factor_;
      velocity_.angular.z *= slow_factor_;
    }
    velocity_publisher_.publish(velocity_);

    if (getButton(joy, buttons_.motor.button))  // moto_status_ by ZD
    {
      // std::cout<<"get motor"<<std::endl;
      bool motor_status;
      if (motor_status_.data)
      {
        motor_status = false;
      }
      else {
        motor_status = true;
      }
      motor_status_.data = motor_status;
      motor_publisher_.publish(motor_status_);
    }

    if (getButton(joy, buttons_.landing.button))
    {
      // std::cout<<"get landing"<<std::endl;
      bool landing_activation;
      if (landing_activation_.data)
      {
        landing_activation = false;
      }
      else {
        landing_activation = true;
      }
      landing_activation_.data = landing_activation;
      landing_publisher_.publish(landing_activation_);
    }

    if (getButton(joy, buttons_.test.button))
    {
      // std::cout<<"get test"<<std::endl;
      bool test_activation;
      if (test_activation_.data)
      {
        test_activation = false;
      }
      else {
        test_activation = true;
      }
      test_activation_.data = test_activation;
      test_publisher_.publish(test_activation_);
    }
  }

  void joyAttitudeCallback(const sensor_msgs::JoyConstPtr &joy)
  {
    attitude_.data[0] = getAxis(joy, axes_.x);
    attitude_.data[1]= getAxis(joy, axes_.y);
    attitude_.data[2] = getAxis(joy, axes_.z);
    attitude_.data[3] = getAxis(joy, axes_.yaw);
    if (getButton(joy, buttons_.slow.button))
    {
      // std::cout<<"get slow"<<std::endl;
      attitude_.data[3] *= slow_factor_;
    }
    attitude_publisher_.publish(attitude_);

    if (getButton(joy, buttons_.motor.button))  // moto_status_ by ZD
    {
      // std::cout<<"get motor"<<std::endl;
      bool motor_status;
      if (motor_status_.data)
      {
        motor_status = false;
        landing_activation_.data = false;
        test_activation_.data = false;
        landing_publisher_.publish(landing_activation_);
        test_publisher_.publish(test_activation_);
      }
      else {
        motor_status = true;
      }
      motor_status_.data = motor_status;
      motor_publisher_.publish(motor_status_);
    }

    if (getButton(joy, buttons_.landing.button))
    {
      // std::cout<<"get landing"<<std::endl;
      bool landing_activation;
      if (landing_activation_.data)
      {
        landing_activation = false;
      }
      else {
        landing_activation = true;
      }
      if (motor_status_.data)
      {
        landing_activation_.data = landing_activation;
        landing_publisher_.publish(landing_activation_);
      }
      if (getButton(joy, buttons_.test.button))
      {
        // std::cout<<"get test"<<std::endl;
        bool test_activation;
        if (test_activation_.data)
        {
          test_activation = false;
        }
        else {
          test_activation = true;
        }
        test_activation_.data = test_activation;
        test_publisher_.publish(test_activation_);
      }
    }
  } // */

  sensor_msgs::Joy::_axes_type::value_type getAxis(const sensor_msgs::JoyConstPtr &joy, Axis axis)
  {
    if (axis.axis == 0)
    {return 0;}
    sensor_msgs::Joy::_axes_type::value_type sign = 1.0;
    if (axis.axis < 0)
    {
      sign = -1.0;
      axis.axis = -axis.axis;
    }
    if ((size_t) axis.axis > joy->axes.size())
    {return 0;}
    return sign * joy->axes[axis.axis - 1] * axis.max;
  }

  sensor_msgs::Joy::_buttons_type::value_type getButton(const sensor_msgs::JoyConstPtr &joy, int button)
  {
    if (button <= 0)
    {return 0;}
    if ((size_t) button > joy->buttons.size())
    {return 0;}
    return joy->buttons[button - 1];
  }

  void stop()
  {
    if(velocity_publisher_.getNumSubscribers() > 0)
    {
      velocity_ = geometry_msgs::Twist();
      velocity_publisher_.publish(velocity_);
    }
    if(attitude_publisher_.getNumSubscribers() > 0)
    {
      attitude_ = std_msgs::Float64MultiArray();
      attitude_publisher_.publish(attitude_);
    } // */
  }

  void landedCallback(const std_msgs::Bool& landed)
  {
    motor_status_.data = false;
    landing_activation_.data = false;
    motor_publisher_.publish(motor_status_);
    landing_publisher_.publish(landing_activation_);
  }
};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rotors_joy_interface");

  rotors_control::JoyInterface joy_interface_;

  ros::spin();

  return 0;
}
