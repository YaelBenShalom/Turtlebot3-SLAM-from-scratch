/// \file   follow_circle_node.cpp
/// \brief  A node  that publishes commands that let the robot drive in a circle
/// of a specified radius at a specified speed
///
/// PARAMETERS:
///     frequency (int): control loop frequency
///     control_flag (bool): specifies when new /control message is read
///     turtle_speed (float): The linear speed of the robot
///     turtle_controlled_speed (float): The linear speed of the robot, when
///         taking into account the control service circle_radius (float): The
///         radius of the circle
///     twist (geometry_msgs::Twist): the robot's twist
/// PUBLISHES:
///     cmd_vel (geometry_msgs/Twist): Publishes to the robot's velocity.
/// SERVICES:
///     Control (control): Causes the robot to travel either clockwise, counter
///         clockwise, or stop. 1 - clockwise, -1 - counter clockwise, 0 - stop.

#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"
#include "ros/ros.h"

#include <geometry_msgs/Twist.h>
#include <nuturtle_robot/Control.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

/// \brief Class FollowCircle
class FollowCircle
{
public:
  FollowCircle()
  {
    ROS_INFO("Initialize the variables");
    // Init Parameters
    load_parameter();

    // Init publishers, and services
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    control_srv =
        nh.advertiseService("/control", &FollowCircle::control_callback, this);
  }

  /// \brief Load the parameters from the parameter server
  /// \returns void
  void load_parameter()
  {
    nh.getParam("turtle_speed",
                turtle_speed);                   // The distance between the wheels
    nh.getParam("circle_radius", circle_radius); // The radius of the wheels
  }

  /// \brief Restarts the location of the odometry, so that the robot thinks
  /// it is at the requested configuration.
  /// \param req - SetPose request.
  /// \param res - SetPose response.
  /// \returns bool
  bool control_callback(nuturtle_robot::Control::Request &req,
                        nuturtle_robot::Control::Response &res)
  {
    ROS_INFO("Setting control");
    control = req.dir;
    res.result = true;

    // Raise the reset flag
    control_flag = true;

    return true;
  }

  /// \brief Main loop for the turtle's motion
  /// \returns void
  void main_loop()
  {
    ROS_INFO("Entering the loop");
    ros::Rate loop_rate(frequency);
    turtle_controlled_speed = turtle_speed;

    while (ros::ok())
    {
      // // ROS_INFO("Looping");

      if (control_flag)
      {
        if (control == 1)
        {
          turtle_controlled_speed = turtle_speed;
        }
        else if (control == -1)
        {
          turtle_controlled_speed = -turtle_speed;
        }
        else if (control == 0)
        {
          turtle_controlled_speed = 0;
        }

        control_flag = false;
      }

      ROS_INFO("turtle_speed = %f\t turtle_controlled_speed = %f\r",
               turtle_speed, turtle_controlled_speed);
      twist.linear.x = turtle_controlled_speed;
      twist.linear.y = 0;
      twist.angular.z = turtle_controlled_speed / circle_radius; // w = v/r

      vel_pub.publish(twist);

      loop_rate.sleep();
      ros::spinOnce();
    }
  }

private:
  int frequency = 100;
  int control;
  bool control_flag = false;
  double turtle_speed, circle_radius;
  double turtle_controlled_speed;

  ros::NodeHandle nh;
  ros::Publisher vel_pub;
  ros::ServiceServer control_srv;

  geometry_msgs::Twist twist;
};

/// \brief Main function
/// \param argc - input int argument
/// \param argv - input array argument
/// \returns int
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "follow_circle_node");
  FollowCircle node;
  node.main_loop();
  ros::spin();
  return 0;
}
