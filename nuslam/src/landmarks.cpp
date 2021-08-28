/// \file   landmarks.cpp
/// \brief  A package to detect landmarks and publish their relative locations.
///
/// PARAMETERS:
///     frequency (int): control loop frequency
///     scan_flag (bool): specifies when the robot scanned the environment
///     wheel_base (float): The distance between the wheels
///     wheel_radius (float): The radius of the wheels
///     left_wheel_joint (std::string): The name of the left wheel joint
///     right_wheel_joint (std::string): The name of the right wheel joint
///     world_frame_id (std::string): The name of the world tf frame
///     odom_frame_id (std::string): The name of the odom tf frame
///     body_frame_id (std::string): The name of the body tf frame
///     stddev_linear (float): The standard deviation of the linear twist noise
///     stddev_angular (float): The standard deviation of the angular twist
///     noise slip_min (float): The minimum wheel slip for the slip noise
///     slip_max (float): The maximum wheel slip for the slip noise
///     obstacles_coordinate_x (std::vector<double>): The x coordinate of the
///         obstacles
///     obstacles_coordinate_y (std::vector<double>): The y coordinate of the 
///         obstacles
///     obstacles_radius (float): The radius of the cardboard tubes [m]
///     max_visable_dist (float): The maximum distance beyond which tubes are
///         not visible [m]
///
/// SUBSCRIBES:
///     lidar_data_sub (sensor_msgs/LaserScan): Subscribes to the robot's lidar
///         scanner.

#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"

#include "ros/ros.h"

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <armadillo>
#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <vector>

/// \brief A struct of points
struct Point {
  double x = 0;
  double y = 0;
};

/// \brief A struct of measurments
struct Measurement {
  double r = 0;
  double phi = 0;
};

/// \brief A struct of clusters
struct Cluster {
  std::vector<Measurement> points;
  double x_center = 0;
  double y_center = 0;
  double radius = 0;
};

/// \brief A struct of laser data properties
struct LaserData {
  double range_min = 0;
  double range_max = 0;
  double angle_min = 0;
  double angle_max = 0;
  double angle_increment = 0;
  std::vector<double> ranges;
};

/// \brief Class Landmarks
class Landmarks {
public:
  Landmarks() {
    ROS_INFO("Initialize the variables");
    // Init Parameters
    load_parameter();

    // Init publishers, subscribers, and services
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/visual_sensor",
                                                               1, true);

    lidar_data_sub =
        nh.subscribe("/scan", 1, &Landmarks::lidar_scan_callback, this);
  }

  /// \brief Load the parameters from the parameter server
  /// \returns void
  void load_parameter() {
    nh.getParam("max_range",
                max_range); // The maximum range of the laser scanner
    nh.getParam("min_range",
                min_range); // The minimum range of the laser scanner
    nh.getParam("mean_scanner_noise",
                mean_scanner_noise); // The mean of the scanner noise
    nh.getParam(
        "stddev_scanner_noise",
        stddev_scanner_noise); // The standard deviation of the scanner noise
    nh.getParam("scan_resolution", scan_resolution);   // The scan resolution
    nh.getParam("min_angle", min_angle);               // The angle increment
    nh.getParam("max_angle", max_angle);               // The angle increment
    nh.getParam("num_of_samples", num_of_samples);     // The number of samples
    nh.getParam("angle_resolution", angle_resolution); // The angle resolution
    nh.getParam("noise_level", noise_level);           // The noise level
    nh.getParam("/radius", radius);                    // The landmark radius
  }

  /// \brief Subscribe to /scan topic and publish a sensor_msgs/LaserScan
  /// message with simulated lidar data at 5Hz \param data - constant pointer to
  /// twist \returns void
  void lidar_scan_callback(const sensor_msgs::LaserScan::ConstPtr &data) {
    // ROS_INFO("Subscribing to LaserScan");

    scan.range_min = data->range_min;
    scan.range_max = data->range_max;
    scan.angle_min = data->angle_min;
    scan.angle_max = data->angle_max;
    scan.angle_increment = data->angle_increment;
    scan.ranges.clear();

    for (auto meas : data->ranges) {
      scan.ranges.push_back(meas);
    }

    // Raise the scan flag
    scan_flag = true;
  }

  void publish_markers() {

    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(clusters.size());
    int i = 0;

    for (auto cluster : clusters) {
      marker_array.markers[i].header.frame_id = "base_link";
      marker_array.markers[i].header.stamp = ros::Time::now();
      marker_array.markers[i].ns = "clusters";
      marker_array.markers[i].id = i;
      marker_array.markers[i].type = visualization_msgs::Marker::CYLINDER;
      marker_array.markers[i].action = visualization_msgs::Marker::ADD;

      marker_array.markers[i].pose.position.x = cluster.x_center;
      marker_array.markers[i].pose.position.y = cluster.y_center;
      marker_array.markers[i].pose.position.z = 0.0;

      marker_array.markers[i].pose.orientation.x = 0.0;
      marker_array.markers[i].pose.orientation.y = 0.0;
      marker_array.markers[i].pose.orientation.z = 0.0;
      marker_array.markers[i].pose.orientation.w = 1.0;

      marker_array.markers[i].scale.x = 2.0 * radius;
      marker_array.markers[i].scale.y = 2.0 * radius;
      marker_array.markers[i].scale.z = 0.5;

      marker_array.markers[i].color.r = 1.0f;
      marker_array.markers[i].color.g = 0.0f;
      marker_array.markers[i].color.b = 1.0f;
      marker_array.markers[i].color.a = 1.0f;

      i += 1;
    }
    marker_pub.publish(marker_array);
  }

  /// \brief Main loop for the turtle's motion
  /// \returns void
  void main_loop() {
    ROS_INFO("Entering the loop\n\r");
    ros::Rate loop_rate(frequency);

    while (ros::ok()) {
      // ROS_INFO("Looping");
      ros::spinOnce();
      current_time = ros::Time::now();

      if (scan_flag) {
        clusters.clear();
        // Cluster
        // Fit
        // Publish landmarks

        // Remove the cmd_vel flag
        scan_flag = false;
      }

      loop_rate.sleep();
    }
  }

private:
  int frequency = 10;
  bool scan_flag = false;
  double max_range, min_range, mean_scanner_noise, stddev_scanner_noise,
      scan_resolution, min_angle, max_angle, num_of_samples, angle_resolution,
      noise_level, radius;

  ros::NodeHandle nh;
  ros::Publisher marker_pub;
  ros::Subscriber lidar_data_sub;
  ros::Time current_time;
  visualization_msgs::MarkerArray marker_array;

  std::vector<Cluster> clusters;
  LaserData scan;
};

/// \brief Main function
/// \returns int
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "landmarks");
  Landmarks node;
  node.main_loop();
  ros::spin();
  return 0;
}
