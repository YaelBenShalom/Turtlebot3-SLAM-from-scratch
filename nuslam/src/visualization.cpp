/// \file   visualization.cpp
/// \brief  The simulation visualization
///
/// PARAMETERS:
///     frequency (int): control loop frequency
///     odom_callback_flag (bool): specifies when new odom msg is read
///     slam_callback_flag (bool): specifies when new slam msg is read
///
///     odom_path (nav_msgs::Path): the trajectory of the robot according only
///         to wheel odometry
///     slam_path (nav_msgs::Path): the trajectory of the robot according to
///         the SLAM algorithm
///
/// PUBLISHES:
///     odom_path_pub (nav_msgs/Odometry): publishes Odometry message on the
///         odom_path topic.
///     slam_path_pub (nav_msgs/Odometry): publishes Odometry message on the
///         slam_path topic.
///
/// SUBSCRIBES:
///     odom_sub (nav_msgs/Odometry): Subscribes to the odometry readings.
///     slam_sub (nav_msgs/Odometry): Subscribes to the slam readings.

#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"
#include "ros/ros.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

/// \brief Class Visualization
class Visualization
{
public:
  Visualization()
  {
    ROS_INFO("Initialize the variables");
    // Init publishers, subscribers, and services
    odom_path_pub = nh.advertise<nav_msgs::Path>("/odom_path", 1);
    slam_path_pub = nh.advertise<nav_msgs::Path>("/map_path", 1);

    odom_sub = nh.subscribe("/odom", 1, &Visualization::odom_callback, this);
    slam_sub = nh.subscribe("/map", 1, &Visualization::slam_callback, this);
  }

  /// \brief Subscribes to the robot's odom position data from Odometry msg
  /// \param odom - the trajectory of the robot according only to wheel odometry
  void odom_callback(const nav_msgs::Odometry &odom)
  {
    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    pose.pose = odom.pose.pose;

    odom_path.header.stamp = ros::Time::now();
    odom_path.header.frame_id = "world";
    odom_path.poses.push_back(pose);

    odom_callback_flag = true;
  }

  /// \brief Subscribes to the robot's slam position data from Odometry msg
  /// \param slam - the trajectory of the robot according to the SLAM algorithm
  void slam_callback(const nav_msgs::Odometry &slam)
  {
    geometry_msgs::PoseStamped pose;

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "world";
    pose.pose = slam.pose.pose;

    slam_path.header.stamp = ros::Time::now();
    slam_path.header.frame_id = "world";
    slam_path.poses.push_back(pose);

    slam_callback_flag = true;
  }

  /// \brief Main loop for the turtle's motion
  /// \returns void
  void main_loop()
  {
    ROS_INFO("Entering the loop");
    ros::Rate loop_rate(frequency);
    while (ros::ok())
    {
      // // ROS_INFO("Looping");
      current_time = ros::Time::now();

      // If the odom_callback was called
      if (odom_callback_flag)
      {
        odom_path_pub.publish(odom_path);

        // Remove the odom flag
        odom_callback_flag = false;
      }

      // If the slam_callback was called
      if (slam_callback_flag)
      {
        slam_path_pub.publish(slam_path);

        // Remove the slam flag
        slam_callback_flag = false;
      }

      loop_rate.sleep();
      ros::spinOnce();
    }
  }

private:
  int frequency = 100;
  bool odom_callback_flag = false;
  bool slam_callback_flag = false;

  ros::NodeHandle nh;
  ros::Publisher odom_path_pub;
  ros::Publisher slam_path_pub;
  ros::Subscriber odom_sub;
  ros::Subscriber slam_sub;
  ros::Time current_time;

  // geometry_msgs::PoseStamped pose;
  nav_msgs::Path odom_path, slam_path;
};

/// \brief Main function
/// \param argc - input int argument
/// \param argv - input array argument
/// \returns int
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "visualization");
  Visualization node;
  node.main_loop();
  ros::spin();
  return 0;
}
