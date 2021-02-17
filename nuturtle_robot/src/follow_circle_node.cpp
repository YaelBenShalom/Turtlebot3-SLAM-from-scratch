/// \file   follow_circle_node.cpp
/// \brief  A node  that publishes commands that let the robot drive in a circle of a specified radius at a specified speed
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

#include <geometry_msgs/Twist.h>
#include <nuturtle_robot/Control.h>

#include <iostream>
#include <vector> 
#include <string>
#include <cmath>


/// \brief Class TurtleInterface
class FollowCircle
{
    public:
        FollowCircle(){
            ROS_INFO("Initialize the variables");
            // Init Parameters
            load_parameter();

            // Init publishers, subscribers, and services
            vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
            control_srv = nh.advertiseService("/control", &FollowCircle::control_callback, this);
         }

        /// \brief Load the parameters from the parameter server
        /// \returns void
        void load_parameter() {
            nh.getParam("turtle_speed", turtle_speed);                // The distance between the wheels
            nh.getParam("circle_radius", circle_radius);              // The radius of the wheels
        }

        /// \brief Restarts the location of the odometry, so that the robot thinks
        /// it is at the requested configuration.
        /// \param req - SetPose request.
        /// \param res - SetPose response.
        /// \returns bool
        bool control_callback(nuturtle_robot::Control::Request &req, nuturtle_robot::Control::Response &res) {
            ROS_INFO("Setting control");
            control = req.dir;
            res.result = true;


            // Raise the reset flag
            control_flag = true;

            return true;
        }

        /// \brief Main loop for the turtle's motion
        /// \returns void
        void main_loop() {
            ROS_INFO("Entering the loop");
            ros::Rate loop_rate(frequency);
            while(ros::ok()) {
                // // ROS_INFO("Looping");

                if (control_flag) {
                    if (control == 1) {
                        turtle_controlled_speed = turtle_speed;
                    }
                    else if (control == -1) {
                        turtle_controlled_speed = -turtle_speed;
                    }
                    else if (control == 0) {
                        turtle_controlled_speed = 0;
                    }

                    control_flag = false;
                }
                
                twist.linear.x = turtle_controlled_speed;
                twist.linear.y = 0;
                twist.angular.z = turtle_controlled_speed / circle_radius;  // w = v/r

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
        double turtle_controlled_speed = turtle_speed;

        ros::NodeHandle nh;
        ros::Publisher vel_pub;
        ros::ServiceServer control_srv;

        geometry_msgs::Twist twist;
};

/// \brief Main function
/// \returns int
int main(int argc, char * argv[])
{
    ros::init(argc, argv, "follow_circle_node");
    FollowCircle node;
    node.main_loop();
    ros::spin();
    return 0;
}