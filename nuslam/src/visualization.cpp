/// \file   visualization_node.cpp
/// \brief  The simulation visualization
///
/// PARAMETERS:
                    ///     frequency (int): control loop frequency
                    ///     cmd_vel_flag (bool): specifies when new cmd_vel is read
                    ///     sensor_data_flag (bool): specifies when new sensor_data is read
                    ///     wheel_base (float): The distance between the wheels
                    ///     wheel_radius (float): The radius of the wheels
                    ///     left_wheel_joint (std::string): The name of the left wheel joint
                    ///     right_wheel_joint (std::string): The name of the right wheel joint
                    ///
                    ///     joint_state (sensor_msgs::JointState): message to publish joint_state readings to /joint_states topic
                    ///
                    ///     twist (rigid2d::Twist2D): the robot's twist
                    ///     diff_drive (rigid2d::DiffDrive): an instance of the diff_drive robot
                    ///     wheel_vel (rigid2d::WheelVelocity): the velocity of the robot's wheels
                    ///     wheel_angle (rigid2d::WheelAngle): the angles of the robot's wheels
/// PUBLISHES:
                    ///     wheel_cmd (nuturtlebot/WheelCommands): publishes WheelCommands message on the wheel_cmd topic.
                    ///                                            Make the turtlebot3 follow the specified twist
                    ///     joint_states (sensor_msgs/JointState): publishes JointState message on the joint_states topic.
                    ///                                            Reflect the current joint angles of each wheel
/// SUBSCRIBES:
                    ///     cmd_vel (geometry_msgs/Twist): Subscribes to the robot's velocity.
                    ///     sensor_data_sub (nuturtlebot/SensorData): Subscribes to the robot's sensor data.
                    ///                                               Provide the angle (in radians) and velocity (in rad/sec) of the wheels, based on encoder data.

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "ros/ros.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <vector> 
#include <string>
#include <cmath>


/// \brief Class Visualization
class Visualization
{
    public:
        Visualization(){
            ROS_INFO("Initialize the variables");
            // Init publishers, subscribers, and services
            odom_path_pub = nh.advertise<nav_msgs::Path>("/odom_path", 1);
            slam_path_pub = nh.advertise<nav_msgs::Path>("/slam_path", 1);

            odom_sub = nh.subscribe("/odom", 1, &Visualization::odom_callback, this);
            slam_sub = nh.subscribe("/slam", 1, &Visualization::slam_callback, this);
         }

        /// \brief Subscribes to the robot's odom position data from Odometry msg
        /// \param odom - the trajectory of the robot according only to wheel odometry
        void odom_callback(const nav_msgs::Odometry &odom) {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "map";
            pose.pose = odom.pose.pose;

            odom_path.header.stamp = ros::Time::now();
            // odom_path.frame_id = "map";
            odom_path.poses.push_back(pose);

            odom_callback_flag = true;
        }

        /// \brief Subscribes to the robot's slam position data from Odometry msg
        /// \param slam - the trajectory of the robot according to the SLAM algorithm
        void slam_callback(const nav_msgs::Odometry &slam) {
            geometry_msgs::PoseStamped pose;

            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "map";
            pose.pose = slam.pose.pose;

            slam_path.header.stamp = ros::Time::now();
            // slam_path.frame_id = "map";
            slam_path.poses.push_back(pose);

            slam_callback_flag = true;
        }


        /// \brief Main loop for the turtle's motion
        /// \returns void
        void main_loop() {
            ROS_INFO("Entering the loop");
            ros::Rate loop_rate(frequency);
            while(ros::ok()) {
                // // ROS_INFO("Looping");
                current_time = ros::Time::now();
                
                // If the odom_callback was called
                if (odom_callback_flag) {
                    odom_path_pub.publish(odom_path);

                    // Remove the odom flag
                    odom_callback_flag = false;
                }

                // If the slam_callback was called
                if (slam_callback_flag) {
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
/// \returns int
int main(int argc, char * argv[])
{
    ros::init(argc, argv, "visualization_node");
    Visualization node;
    node.main_loop();
    ros::spin();
    return 0;
}