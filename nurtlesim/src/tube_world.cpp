/// \file   tube_world.cpp
/// \brief  A kinematic simulation of a differential drive robot using the DiffDrive class.
///
/// PARAMETERS:
///     frequency (int): control loop frequency
///     cmd_vel_flag (bool): specifies when new cmd_vel is read
///     wheel_base (float): The distance between the wheels
///     wheel_radius (float): The radius of the wheels
///     left_wheel_joint (std::string): The name of the left wheel joint
///     right_wheel_joint (std::string): The name of the right wheel joint
///
///     joint_state (sensor_msgs::JointState): message to publish joint_state readings to /joint_states topic
///
///     pose (rigid2d::Config2D): the robot's position (based on the wheel angles)
///     twist (rigid2d::Twist2D): the robot's twist
///     diff_drive (rigid2d::DiffDrive): an instance of the diff_drive robot
///     wheel_vel (rigid2d::WheelVelocity): the velocity of the robot's wheels
///     wheel_angle (rigid2d::WheelAngle): the angles of the robot's wheels
/// PUBLISHES:
///     joint_states (sensor_msgs/JointState): publishes JointState message on the joint_states topic.
///                                            Reflect the current joint angles of each wheel
/// SUBSCRIBES:
///     cmd_vel (geometry_msgs/Twist): Subscribes to the robot's velocity.

#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"

#include "ros/ros.h"

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <vector> 
#include <string>
#include <cmath>


/// \brief Class TubeWorld
class TubeWorld
{
    public:
        TubeWorld(){
            ROS_INFO("Initialize the variables");
            // Init Parameters
            load_parameter();

            // Init publishers, subscribers, and services
            joint_states_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
            vel_sub = nh.subscribe("/cmd_vel", 1, &TubeWorld::cmd_vel_callback, this);
         }

        /// \brief Load the parameters from the parameter server
        /// \returns void
        void load_parameter() {
            nh.getParam("wheel_base", wheel_base);                  // The distance between the wheels
            nh.getParam("wheel_radius", wheel_radius);              // The radius of the wheels
            nh.getParam("stddev_linear", stddev_linear);            // The standard deviation of the linear twist noise
            nh.getParam("stddev_angular", stddev_angular);          // The standard deviation of the angular twist noise
            nh.getParam("left_wheel_joint", left_wheel_joint);      // The name of the left wheel joint
            nh.getParam("right_wheel_joint", right_wheel_joint);    // The name of the right wheel joint
        }
        

        /// \brief Subscribes to the robot's velocity.
        /// \param tw - constant pointer to twist
        /// \returns void
        void cmd_vel_callback(const geometry_msgs::Twist &tw) {
            std::default_random_engine generator;
            std::normal_distribution<double> dist_linear(0, stddev_linear);
            std::normal_distribution<double> dist_angular(0, stddev_angular);

            twist.thetadot = tw.angular.z;
            twist.xdot = tw.linear.x;
            twist.ydot = tw.linear.y;

            twist_noised.thetadot = tw.angular.z + dist_angular(generator);
            twist_noised.xdot = tw.linear.x + dist_linear(generator);
            twist_noised.ydot = tw.linear.y;

            // Raise the cmd_vel flag
            cmd_vel_flag = true;
        }

        /// \brief Main loop for the turtle's motion
        /// \returns void
        void main_loop() {
            ROS_INFO("Entering the loop");
            ros::Rate loop_rate(frequency);
            while(ros::ok()) {
                // ROS_INFO("Looping");
                current_time = ros::Time::now();
                
                // If the cmd_vel_callback was called
                if (cmd_vel_flag) {
                    // ROS_INFO("Entering if statement/n");
                    sensor_msgs::JointState joint_state;
                    
                    wheel_angle = diff_drive.updateOdometryWithTwist(twist_noised);
                    joint_state.header.stamp = current_time;

                    joint_state.name.push_back(right_wheel_joint);
                    joint_state.name.push_back(left_wheel_joint);
                    joint_state.position.push_back(wheel_angle.right_wheel_angle);
                    joint_state.position.push_back(wheel_angle.left_wheel_angle);

                    joint_states_pub.publish(joint_state);

                    // Remove the cmd_vel flag
                    cmd_vel_flag = false;
                }
                loop_rate.sleep();
                ros::spinOnce();
            }
        }

    private:
        int frequency = 100;
        bool cmd_vel_flag = false;
        double wheel_base, wheel_radius, stddev_linear, stddev_angular;
        std::string left_wheel_joint, right_wheel_joint;

        ros::NodeHandle nh;
        ros::Publisher joint_states_pub;
        ros::Subscriber vel_sub;
        ros::Time current_time;

        rigid2d::Config2D pose;
        rigid2d::Twist2D twist, twist_noised;
        rigid2d::DiffDrive diff_drive;
        rigid2d::WheelVelocity wheel_vel;
        rigid2d::WheelAngle wheel_angle;
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