#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "ros/ros.h"

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <nuturtlebot/SensorData.h>
#include <nuturtlebot/WheelCommands.h>

#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <catch_ros/catch.hpp>


bool cmd_flag = false;
bool joint_state_flag = true;

nuturtlebot::WheelCommands wheel_cmd;
rigid2d::WheelVelocity wheel_vel;
rigid2d::WheelAngle wheel_angle;

// /wheel_cmd callback
void wheel_cmd_callback(const nuturtlebot::WheelCommands &cmd)
{
    wheel_cmd = cmd;

    cmd_flag = true;
}

// joint_states callback
void joint_state_callback(const sensor_msgs::JointState &joint_state)
{
    wheel_angle.right_wheel_angle = joint_state.position[0];
    wheel_angle.left_wheel_angle = joint_state.position[1];
    wheel_vel.right_wheel_vel = joint_state.velocity[0];
    wheel_vel.left_wheel_vel = joint_state.velocity[1];

    joint_state_flag = true;
}

TEST_CASE( "Test for turtle_interface ROS API cmd_vel to wheel_cmd" ) {
    using namespace rigid2d;

    ros::NodeHandle nh;
    ros::Subscriber wheel_cmd_sub = nh.subscribe("/wheel_cmd", 1, wheel_cmd_callback);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
	geometry_msgs::Twist twist;

    SECTION( "test pure translation" ) {
        twist.linear.x = 0.1; // TODO!!!!!!
        twist.angular.z = 0;
	    wheel_cmd.right_velocity = 0;
        wheel_cmd.left_velocity = 0;

        cmd_flag = false;

        while(!cmd_flag) {
            ros::spinOnce();
            vel_pub.publish(twist);
        }
    
        REQUIRE( wheel_cmd.right_velocity == 61);
        REQUIRE( wheel_cmd.left_velocity == 61);
    }

    SECTION( "test pure rotation" ) {
        twist.linear.x = 0;
        twist.angular.z = 1;
	    wheel_cmd.right_velocity = 0;
        wheel_cmd.left_velocity = 0;

        cmd_flag = false;

        while(!cmd_flag) {
            ros::spinOnce();
            vel_pub.publish(twist);
        }

        REQUIRE( wheel_cmd.right_velocity == -97);
        REQUIRE( wheel_cmd.left_velocity == 97);
    }
}

TEST_CASE( "Test for turtle_interface ROS API sensors to joint_states" ) {
    using namespace rigid2d;

    ros::NodeHandle nh;
    ros::Subscriber joint_state_sub = nh.subscribe("/joint_state", 1, joint_state_callback);
    ros::Publisher sensor_pub = nh.advertise<nuturtlebot::SensorData>("/sensor_data", 1, true);
	nuturtlebot::SensorData sensor_data;

    SECTION( "test sensors to joint_states" ) {
	    sensor_data.right_encoder = 100;
        sensor_data.left_encoder = 100;
        wheel_angle.right_wheel_angle = 0;
        wheel_angle.left_wheel_angle = 0;
        wheel_vel.right_wheel_vel = 0;
        wheel_vel.left_wheel_vel = 0;

        joint_state_flag = false;

        while(!joint_state_flag) {
            ros::spinOnce();
            sensor_pub.publish(sensor_data);
        }

        // 2*PI*100/4096 = 0.1534
        REQUIRE( rigid2d::almost_equal(wheel_angle.right_wheel_angle, 0.1534, 1e-3));
        REQUIRE( rigid2d::almost_equal(wheel_angle.left_wheel_angle, 0.1534, 1e-3));
        REQUIRE( rigid2d::almost_equal(wheel_vel.right_wheel_vel, 0.1534, 1e-3));
        REQUIRE( rigid2d::almost_equal(wheel_vel.left_wheel_vel, 0.1534, 1e-3));
    }
}