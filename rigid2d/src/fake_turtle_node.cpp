/// \file   fake_turtle_node.cpp
/// \brief  A kinematic simulation of a differential drive robot using the DiffDrive class.
///
/// PUBLISHES:
///     sensor_msgs/JointState (joint_states): publishes JointState message on the joint_states topic.
///                                            Reflect the current joint angles of each wheel
/// SUBSCRIBES:
///     geometry_msgs/Twist (cmd_vel): Subscribes to the robot's velocity.

#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"

#include "ros/ros.h"

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <vector> 
#include <string>
#include <cmath>


class FakeTurtle
{
    public:
        FakeTurtle(){
            ROS_INFO("Initialize the variables");
            // Init Parameters
            load_parameter();

            // Init publishers, subscribers, and services
            joint_states_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 5);
            vel_sub = nh.subscribe("turtle1/cmd_vel", 10, &FakeTurtle::cmd_vel_callback, this);
         }

        /// \brief Load the parameters from the parameter server
        /// \returns void
        void load_parameter() {
            nh.getParam("wheel_base", wheel_base);                  // The distance between the wheels
            nh.getParam("wheel_radius", wheel_radius);              // The radius of the wheels
            nh.getParam("left_wheel_joint", left_wheel_joint);      // The name of the left wheel joint
            nh.getParam("right_wheel_joint", right_wheel_joint);    // The name of the right wheel joint
        }
        

        /// \brief Subscribes to the robot's velocity.
        /// \param tw - constant pointer to twist
        /// \returns void
        void cmd_vel_callback(const geometry_msgs::Twist &tw) {
            twist.thetadot = tw.angular.z / (double)frequency;
            twist.xdot = tw.linear.x / (double)frequency;
            twist.ydot = tw.linear.y / (double)frequency;

            cmd_vel_flag = true;
        }


        /// \brief Main loop for the turtle's motion
        /// \returns void
        void main_loop() {
            ROS_INFO("Entering the loop");
            ros::Rate loop_rate(frequency);
            while(ros::ok()) {
                current_time = ros::Time::now();
                
                // If the cmd_vel_callback was called
                if (cmd_vel_flag) {
                    wheel_angle = diff_drive.get_wheel_angle();
                    joint_state.header.stamp = current_time;

                    joint_state.name.push_back(right_wheel_joint);
                    joint_state.position.push_back(wheel_angle.right_wheel_angle);

                    joint_state.name.push_back(left_wheel_joint);
                    joint_state.position.push_back(wheel_angle.left_wheel_angle);

                    joint_states_pub.publish(joint_state);
                    cmd_vel_flag = false;
                }

                loop_rate.sleep();
                ros::spinOnce();
            }
        }

    private:
        int frequency = 100;
        bool cmd_vel_flag = false;
        float wheel_base, wheel_radius;
        std::string left_wheel_joint, right_wheel_joint;
        
        ros::NodeHandle nh;
        ros::Publisher joint_states_pub;
        ros::Subscriber vel_sub;
        ros::Time current_time;

        sensor_msgs::JointState joint_msg;
        sensor_msgs::JointState joint_state;

        rigid2d::Config2D pose;
        rigid2d::Twist2D twist;
        rigid2d::DiffDrive diff_drive;
        rigid2d::WheelVelocity wheel_vel;
        rigid2d::WheelAngle wheel_angle;

};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "fake_turtle_node");
    
    FakeTurtle node;
    node.main_loop();
    ros::spin();
    return 0;
}