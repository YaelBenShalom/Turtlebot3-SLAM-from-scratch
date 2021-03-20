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
///     stddev_angular (float): The standard deviation of the angular twist noise
///     slip_min (float): The minimum wheel slip for the slip noise
///     slip_max (float): The maximum wheel slip for the slip noise
///     obstacles_coordinate_x (std::vector<double>): The x coordinate of the obstacles
///     obstacles_coordinate_y (std::vector<double>): The y coordinate of the obstacles
///     obstacles_radius (float): The radous of the cardboard tubes [m]
///     max_visable_dist (float): The maximum distance beyond which tubes are not visible [m]
///
/// SUBSCRIBES:
///     lidar_data_sub (sensor_msgs/LaserScan): Subscribes to the robot's lidar scanner.

#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"

#include "ros/ros.h"

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/LaserScan.h>

#include <iostream>
#include <vector> 
#include <string>
#include <cmath>
#include <random>
#include <armadillo>


/// \brief A struct of points
struct Point
{
    double x = 0;
    double y = 0;
};


/// \brief A struct of measurments
struct Measurement
{
    double r = 0;
    double phi = 0;
};


/// \brief A struct of clusters
struct Cluster
{
    std::vector<Measurement> points;
    double x_center = 0;
    double y_center = 0;
    double radius = 0; 
};


/// \brief A struct of laser data properties
struct LaserData
{
    double range_min = 0;
    double range_max = 0;
    double angle_min = 0;
    double angle_max = 0;
    double angle_increment = 0;
    std::vector<double> ranges;
};


/// \brief Class Landmarks
class Landmarks
{
    public:
        Landmarks(){
            ROS_INFO("Initialize the variables");
            // Init Parameters
            load_parameter();

            // Init publishers, subscribers, and services
            lidar_data_sub = nh.subscribe("/scan", 1, &TubeWorld::lidar_scan_callback, this);
        }

        /// \brief Load the parameters from the parameter server
        /// \returns void
        void load_parameter() {
            nh.getParam("wheel_base", wheel_base);                          // The distance between the wheels
            nh.getParam("wheel_radius", wheel_radius);                      // The radius of the wheels
            nh.getParam("left_wheel_joint", left_wheel_joint);              // The name of the left wheel joint
            nh.getParam("right_wheel_joint", right_wheel_joint);            // The name of the right wheel joint
            nh.getParam("world_frame_id", world_frame_id);                  // The name of the world tf frame
            nh.getParam("odom_frame_id", odom_frame_id);                    // The name of the odom tf frame
            nh.getParam("turtle_frame_id", turtle_frame_id);                // The name of the body tf frame
            nh.getParam("stddev_linear", stddev_linear);                    // The standard deviation of the linear twist noise
            nh.getParam("stddev_angular", stddev_angular);                  // The standard deviation of the angular twist noise
            nh.getParam("slip_min", slip_min);                              // The minimum wheel slip for the slip noise
            nh.getParam("slip_max", slip_max);                              // The maximum wheel slip for the slip noise
            nh.getParam("obstacles_coordinate_x", obstacles_coordinate_x);  // The x coordinate of the obstacles
            nh.getParam("obstacles_coordinate_y", obstacles_coordinate_y);  // The y coordinate of the obstacles
            nh.getParam("obstacles_radius", obstacles_radius);              // The radous of the cardboard tubes [m]
            nh.getParam("max_visable_dist", max_visable_dist);              // The maximum distance beyond which tubes are not visible [m]

            nh.getParam("max_range", max_range);                            // The maximum range of the laser scanner
            nh.getParam("min_range", min_range);                            // The minimum range of the laser scanner
            nh.getParam("mean_scanner_noise", mean_scanner_noise);          // The mean of the scanner noise
            nh.getParam("stddev_scanner_noise", stddev_scanner_noise);      // The standard deviation of the scanner noise
            nh.getParam("scan_resolution", scan_resolution);                // The scan resolution
            nh.getParam("min_angle", min_angle);                            // The angle increment
            nh.getParam("max_angle", max_angle);                            // The angle increment
            nh.getParam("num_of_samples", num_of_samples);                  // The number of samples
            nh.getParam("angle_resolution", angle_resolution);              // The angle resolution
            nh.getParam("noise_level", noise_level);                        // The noise level
        }


        /// \brief Subscribe to /scan topic and publish a sensor_msgs/LaserScan message with simulated lidar data at 5Hz
        /// \param data - constant pointer to twist
        /// \returns void
        void lidar_scan_callback(const sensor_msgs::LaserScan &data) {
            // ROS_INFO("Subscribing to LaserScan");

            scan = data->ranges;

            // Raise the scan flag
            scan_flag = true;
        }
                    

        /// \brief Main loop for the turtle's motion
        /// \returns void
        void main_loop() {
            ROS_INFO("Entering the loop\n\r");
            ros::Rate loop_rate(frequency);

            // Publish real markers  
            set_real_marker_array();

            while(ros::ok()) {
                // ROS_INFO("Looping");
                ros::spinOnce();
                current_time = ros::Time::now();


                if (scan_flag) {

                // Remove the cmd_vel flag
                cmd_vel_flag = false;
                }

                loop_rate.sleep();
            }
        }

    private:
        int frequency = 10;
        bool scan_flag = false;
        double wheel_base, wheel_radius, stddev_linear, stddev_angular, slip_min, slip_max, obstacles_radius, max_visable_dist, markers_dist, collision_dist;
        double max_range, min_range, mean_scanner_noise, stddev_scanner_noise, scan_resolution, min_angle, max_angle, num_of_samples, angle_resolution, noise_level;
        std::string left_wheel_joint, right_wheel_joint, world_frame_id, odom_frame_id, turtle_frame_id;
        std::vector<double> obstacles_coordinate_x, obstacles_coordinate_y;
        static std::vector<float> scan;

        ros::NodeHandle nh;
        ros::Publisher joint_states_pub, real_joint_states_pub, marker_pub, real_marker_pub, robot_path_pub, robot_marker_pub;
        ros::Subscriber vel_sub, lidar_data_sub;
        ros::Time current_time;

        visualization_msgs::MarkerArray marker_array, real_marker_array;
        visualization_msgs::Marker robot_marker;
        tf2::Quaternion quat;
        tf2_ros::TransformBroadcaster world_broadcaster;
        geometry_msgs::TransformStamped world_tf;
        geometry_msgs::Quaternion world_quat, real_quat;
        geometry_msgs::PoseStamped real_pose_stamped;
        nav_msgs::Path real_path;

        rigid2d::Config2D pose, real_pose;
        rigid2d::Twist2D twist, twist_noised;
        rigid2d::DiffDrive diff_drive, real_diff_drive;
        rigid2d::WheelVelocity wheel_vel, real_wheel_vel;
        rigid2d::WheelAngle wheel_angle, real_wheel_angle;
};

/// \brief Main function
/// \returns int
int main(int argc, char * argv[])
{
    ros::init(argc, argv, "landmarks");
    Landmarks node;
    node.main_loop();
    ros::spin();
    return 0;
}