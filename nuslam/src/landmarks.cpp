/// \file   landmarks.cpp
/// \brief  A package to detect landmarks and publish their relative locations.
///
/// PARAMETERS:
                    ///     frequency (int): control loop frequency
                    ///     cmd_vel_flag (bool): specifies when new cmd_vel is read
                    ///     collision_flag (bool): specifies when the robot collides with an obstacle
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
                    ///     joint_state (sensor_msgs::JointState): message to publish joint_state readings to /joint_states topic
                    ///     marker_array (visualization_msgs::MarkerArray): the array of the obstacles' configuration
                    ///     real_marker_array (visualization_msgs::MarkerArray): the array of the real obstacles' configuration
                    ///     world_broadcaster (TransformBroadcaster): Broadcast the transform between worls_frame_id and the body_frame_id on /tf using a tf2
                    ///     world_tf (geometry_msgs::TransformStamped): world transform
                    ///     real_pose_stamped (geometry_msgs::PoseStamped): the real position of the robot
                    ///     real_path (nav_msgs::Path): the real path of the robot

                    ///     pose (rigid2d::Config2D): the robot's position (based on the wheel angles)
                    ///     real_pose (rigid2d::Config2D): the robot's real position
                    ///     twist (rigid2d::Twist2D): the robot's twist
                    ///     twist_noised (rigid2d::Twist2D): the robot's twist (plus noise)
                    ///     diff_drive (rigid2d::DiffDrive): an instance of the diff_drive robot
                    ///     real_diff_drive (rigid2d::DiffDrive): an instance of the real diff_drive robot
                    ///     wheel_vel (rigid2d::WheelVelocity): the velocity of the robot's wheels
                    ///     real_wheel_vel (rigid2d::WheelVelocity): the real velocity of the robot's wheels
                    ///     wheel_angle (rigid2d::WheelAngle): the angles of the robot's wheels
                    ///     real_wheel_angle (rigid2d::WheelAngle): the real angles of the robot's wheels
///
/// PUBLISHES:
                    ///     joint_states_pub (sensor_msgs/JointState): publishes JointState message on the joint_states topic.
                    ///     marker_pub (visualization_msgs::MarkerArray): publishes MarkerArray message on the fake_sensor topic.
                    ///     real_marker_pub (visualization_msgs::MarkerArray): publishes MarkerArray message on the real_landmarks topic.
                    ///     robot_path_pub (nav_msgs::Path): publishes JointState message on the real_path topic.
///
/// SUBSCRIBES:
                    ///     vel_sub (geometry_msgs/Twist): Subscribes to the robot's velocity.

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


/// \brief Class TubeWorld
class TubeWorld
{
    public:
        TubeWorld(){
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
        bool cmd_vel_flag = false;
        bool scan_flag = false;
        bool collision_flag = false;
        double wheel_base, wheel_radius, stddev_linear, stddev_angular, slip_min, slip_max, obstacles_radius, max_visable_dist, markers_dist, collision_dist;
        std::string left_wheel_joint, right_wheel_joint, world_frame_id, odom_frame_id, turtle_frame_id;
        std::vector<double> obstacles_coordinate_x, obstacles_coordinate_y;

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
    ros::init(argc, argv, "tube_world");
    TubeWorld node;
    node.main_loop();
    ros::spin();
    return 0;
}