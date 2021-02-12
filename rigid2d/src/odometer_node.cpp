/// \file   odometer_node.cpp
/// \brief  Causes the turtlesim turtle to follow a rectangle path.
///
/// PARAMETERS:
///     frequency (int): control loop frequency
///     reset_flag (bool): specifies when new pose is read
///     wheel_base (float): The distance between the wheels
///     wheel_radius (float): The radius of the wheels
///     odom_frame_id (std::string): The name of the odometry tf frame
///     body_frame_id (std::string): The name of the body tf frame
///     left_wheel_joint (std::string): The name of the left wheel joint
///     right_wheel_joint (std::string): The name of the right wheel joint
///
///     joint_msg (sensor_msgs::JointState): message to publish wheel angle readings to /joint_states topic
///     odom_broadcaster (TransformBroadcaster): Broadcast the transform between odom_frame_id and the body_frame_id on /tf using a tf2
///     odom_tf (geometry_msgs::TransformStamped): odometry transform
///     odom (nav_msgs::Odometry): odometry message
///
///     pose (rigid2d::Config2D): the robot's position (based on the wheel angles)
///     reset_pose (rigid2d::Config2D): the robot's reset position
///     twist (rigid2d::Twist2D): the robot's twist
///     diff_drive (rigid2d::DiffDrive): an instance of the diff_drive robot
///     wheel_vel (rigid2d::WheelVelocity): the velocity of the robot's wheels
/// PUBLISHES:
///     odom (nav_msgs/Odometry): publishes Odometry message on the odom topic.
/// SUBSCRIBES:
///     joint_states (sensor_msgs/JointState): Subscribes to the robot's joint_states.
/// SERVICES:
///     SetPose (set_pose): Restarts the location of the odometry, so that the robot thinks
///                         it is at the requested configuration.

#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"

#include "ros/ros.h"

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rigid2d/SetPose.h>

#include <iostream>
#include <vector> 
#include <string>
#include <cmath>


class Odometer
{
    public:
        Odometer(){
            ROS_INFO("Initialize the variables");
            // Init Parameters
            load_parameter();

            // Init publishers, subscribers, and services
            odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 5);
            joint_states_sub = nh.subscribe("/joint_states", 10, &Odometer::joint_state_callback, this);
            set_pose_srv = nh.advertiseService("/set_pose", &Odometer::set_pose_callback, this);
         }

        /// \brief Load the parameters from the parameter server
        /// \returns void
        void load_parameter() {
            nh.getParam("wheel_base", wheel_base);                  // The distance between the wheels
            nh.getParam("wheel_radius", wheel_radius);              // The radius of the wheels
            nh.getParam("odom_frame_id", odom_frame_id);            // The name of the odometry tf frame
            nh.getParam("body_frame_id", body_frame_id);            // The name of the body tf frame
            nh.getParam("left_wheel_joint", left_wheel_joint);      // The name of the left wheel joint
            nh.getParam("right_wheel_joint", right_wheel_joint);    // The name of the right wheel joint
        }
        
        /// \brief Subscribes to the robot's joint_states.
        /// \param joint_state - constant pointer to joint_states
        /// \returns void
        void joint_state_callback(const sensor_msgs::JointState::ConstPtr &joint_state) {
            right_wheel_angle = joint_state->position.at(0);
            left_wheel_angle = joint_state->position.at(1);

            wheel_vel = diff_drive.updateOdometryWithAngles(right_wheel_angle, left_wheel_angle);
            twist = diff_drive.wheels2Twist(wheel_vel);
        }

        /// \brief Restarts the location of the odometry, so that the robot thinks
        /// it is at the requested configuration.
        /// \param request - SetPose request.
        /// \param response - SetPose response.
        /// \returns bool
        bool set_pose_callback(rigid2d::SetPose::Request &req, rigid2d::SetPose::Response &res) {
            ROS_INFO("Setting pose");

            reset_pose.x = req.x;
            reset_pose.y = req.y;
            reset_pose.theta = req.theta;
            res.result = true;

            // Raise the reset flag
            reset_flag = true;

            return true;
        }

        /// \brief Main loop for the turtle's motion
        /// \returns void
        void main_loop() {
            ROS_INFO("Entering the loop");
            ros::Rate loop_rate(frequency);
            while(ros::ok()) {
                current_time = ros::Time::now();
                
                // If the set_pose_callback was called
                if (reset_flag) {
                    diff_drive.set_config(reset_pose);
                    // diff_drive.WheelVelocity();

                    // Provide the configuration of the robot
                    ROS_INFO("Reset robot position:");
                    ROS_INFO("x = %f\n", diff_drive.get_config().x);
                    ROS_INFO("y = %f\n", diff_drive.get_config().y);
                    ROS_INFO("theta = %f\n", diff_drive.get_config().theta);

                    // Remove the reset flag
                    reset_flag = false;
                }

                pose = diff_drive.get_config();
                odom_tf.header.stamp = current_time;
                odom_tf.header.frame_id = odom_frame_id;
                odom_tf.child_frame_id = body_frame_id;
                odom_tf.transform.translation.x = pose.x;
                odom_tf.transform.translation.y = pose.y;
                odom_tf.transform.translation.z = 0;

                quat.setRPY(0, 0, pose.theta);
                odom_quat = tf2::toMsg(quat);
                odom_tf.transform.rotation = odom_quat;

                odom_broadcaster.sendTransform(odom_tf);

                odom.header.stamp = current_time;
                odom.header.frame_id = odom_frame_id;
                odom.child_frame_id = body_frame_id;

                odom.pose.pose.position.x = pose.x;
                odom.pose.pose.position.y = pose.y;
                odom.pose.pose.position.z = 0.0;
                odom.pose.pose.orientation = odom_quat;

                odom.twist.twist.linear.x = twist.xdot;
                odom.twist.twist.linear.y = twist.ydot;
                odom.twist.twist.angular.z = twist.thetadot;

                odom_pub.publish(odom);
                
                loop_rate.sleep();
                ros::spinOnce();
            }
        }

    private:
        int frequency = 100;
        bool reset_flag = false;
        float wheel_base, wheel_radius, right_wheel_angle, left_wheel_angle;
        std::string odom_frame_id, body_frame_id, left_wheel_joint, right_wheel_joint;
        
        ros::NodeHandle nh;
        ros::Publisher odom_pub;
        ros::Subscriber joint_states_sub;
        ros::ServiceServer set_pose_srv;
        ros::Time current_time;

        tf2::Quaternion quat;
        tf2_ros::TransformBroadcaster odom_broadcaster;
        sensor_msgs::JointState joint_msg;
        geometry_msgs::TransformStamped odom_tf;
        geometry_msgs::Quaternion odom_quat;
        nav_msgs::Odometry odom;

        rigid2d::Config2D pose, reset_pose;
        rigid2d::Twist2D twist;
        rigid2d::DiffDrive diff_drive;
        rigid2d::WheelVelocity wheel_vel;
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "odometer_node");
    
    Odometer node;
    node.main_loop();
    ros::spin();
    return 0;
}