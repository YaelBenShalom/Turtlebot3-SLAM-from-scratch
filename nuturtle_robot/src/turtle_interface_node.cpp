/// \file   turtle_interface_node.cpp
/// \brief  A low-level control and sensor routines in ROS
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
///     joint_state (sensor_msgs::JointState): message to publish joint_state
///         readings to /joint_states topic
///
///     twist (rigid2d::Twist2D): the robot's twist
///     diff_drive (rigid2d::DiffDrive): an instance of the diff_drive robot
///     wheel_vel (rigid2d::WheelVelocity): the velocity of the robot's wheels
///     wheel_angle (rigid2d::WheelAngle): the angles of the robot's wheels
/// PUBLISHES:
///     wheel_cmd (nuturtlebot/WheelCommands): publishes WheelCommands message
///         on the wheel_cmd topic. Make the turtlebot3 follow the specified twist
///     joint_states (sensor_msgs/JointState): publishes JointState message on
///         the joint_states topic. Reflect the current joint angles of each wheel
/// SUBSCRIBES:
///     cmd_vel (geometry_msgs/Twist): Subscribes to the robot's velocity.
///         sensor_data_sub (nuturtlebot/SensorData): Subscribes to the robot's
///         sensor data. Provide the angle (in radians) and velocity (in rad/sec) of
///         the wheels, based on encoder data.

#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"
#include "ros/ros.h"

#include <geometry_msgs/Twist.h>
#include <nuturtlebot/SensorData.h>
#include <nuturtlebot/WheelCommands.h>
#include <sensor_msgs/JointState.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

/// \brief Class TurtleInterface
class TurtleInterface {
public:
  TurtleInterface() {
    ROS_INFO("Initialize the variables");
    // Init Parameters
    load_parameter();

    // Init publishers, subscribers, and services
    wheel_cmd_pub = nh.advertise<nuturtlebot::WheelCommands>("/wheel_cmd", 1);
    joint_states_pub =
        nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
    vel_sub =
        nh.subscribe("/cmd_vel", 1, &TurtleInterface::cmd_vel_callback, this);
    sensor_data_sub = nh.subscribe(
        "/sensor_data", 1, &TurtleInterface::sensor_data_callback, this);
  }

  /// \brief Load the parameters from the parameter server
  /// \returns void
  void load_parameter() {
    nh.getParam("wheel_base", wheel_base); // The distance between the wheels
    nh.getParam("wheel_radius", wheel_radius); // The radius of the wheels
    nh.getParam(
        "max_trans_speed",
        max_trans_speed); // The maximum translational speed of the motor
    nh.getParam("max_rot_speed",
                max_rot_speed); // The maximum rotational speed of the motor
    nh.getParam("max_motor_rot",
                max_motor_rot); // The maximum rotational speed of the motor
    nh.getParam("resolution", resolution); // The resolution of the motor
    nh.getParam("left_wheel_joint",
                left_wheel_joint); // The name of the left wheel joint
    nh.getParam("right_wheel_joint",
                right_wheel_joint); // The name of the right wheel joint
  }

  /// \brief Subscribes to the robot's velocity.
  /// \param tw - constant pointer to twist
  /// \returns void
  void cmd_vel_callback(const geometry_msgs::Twist &tw) {
    if (tw.linear.x > max_trans_speed) {
      twist.xdot = max_trans_speed / frequency;
    } else if (tw.linear.x < -max_trans_speed) {
      twist.xdot = -max_trans_speed / frequency;
    } else {
      twist.xdot = tw.linear.x / frequency;
    }
    twist.ydot = tw.linear.y;
    if (tw.angular.z > max_rot_speed) {
      twist.thetadot = max_rot_speed / frequency;
    } else if (tw.angular.z < -max_rot_speed) {
      twist.thetadot = -max_rot_speed / frequency;
    } else {
      twist.thetadot = tw.angular.z / frequency;
    }

    wheel_vel = diff_drive.twist2Wheels(twist);

    if (wheel_vel.right_wheel_vel > max_motor_rot) {
      wheel_vel.right_wheel_vel = max_motor_rot;
    } else if (wheel_vel.right_wheel_vel < -max_motor_rot) {
      wheel_vel.right_wheel_vel = -max_motor_rot;
    }
    if (wheel_vel.left_wheel_vel > max_motor_rot) {
      wheel_vel.left_wheel_vel = max_motor_rot;
    } else if (wheel_vel.left_wheel_vel < -max_motor_rot) {
      wheel_vel.left_wheel_vel = -max_motor_rot;
    }
    // flag
    cmd_vel_flag = true;
  }

  /// \brief Subscribes to the robot's sensor data.
  /// \param data - constant pointer to sensor data
  /// \returns void
  void sensor_data_callback(const nuturtlebot::SensorData &data) {
    right_angle = rigid2d::normalize_angle(
        (2 * data.right_encoder * rigid2d::PI) / resolution);
    left_angle = rigid2d::normalize_angle(
        (2 * data.left_encoder * rigid2d::PI) / resolution);

    // Raise the sensor_data flag
    sensor_data_flag = true;
  }

  /// \brief Main loop for the turtle's motion
  /// \returns void
  void main_loop() {
    ROS_INFO("Entering the loop");
    ros::Rate loop_rate(frequency);
    while (ros::ok()) {
      current_time = ros::Time::now();

      // If the cmd_vel_callback was called
      if (cmd_vel_flag) {
        power_rot_ratio = 256 / (max_motor_rot);
        wheel_command.right_velocity =
            std::round(wheel_vel.right_wheel_vel * power_rot_ratio);
        wheel_command.left_velocity =
            std::round(wheel_vel.left_wheel_vel * power_rot_ratio);
        wheel_cmd_pub.publish(wheel_command);

        // Remove the cmd_vel flag
        cmd_vel_flag = false;
      }

      // if (sensor_data_flag) {
      // ROS_INFO("Entering if statement/n");
      sensor_msgs::JointState joint_state;

      wheel_angle = diff_drive.updateOdometryWithTwist(twist);
      joint_state.header.stamp = current_time;

      joint_state.name.push_back(right_wheel_joint);
      joint_state.name.push_back(left_wheel_joint);
      joint_state.position.push_back(wheel_angle.right_wheel_angle);
      joint_state.position.push_back(wheel_angle.left_wheel_angle);

      joint_states_pub.publish(joint_state);

      // Remove the sensor_data flag
      // sensor_data_flag = false;
      // }
      loop_rate.sleep();
      ros::spinOnce();
    }
  }

private:
  int frequency = 100;
  bool cmd_vel_flag = false;
  bool sensor_data_flag = false;
  double power_rot_ratio, encoder_ticks, right_angle, left_angle;
  double wheel_base, wheel_radius, max_trans_speed, max_rot_speed,
      max_motor_rot, resolution;
  std::string left_wheel_joint, right_wheel_joint;

  ros::NodeHandle nh;
  ros::Publisher wheel_cmd_pub;
  ros::Publisher joint_states_pub;
  ros::Subscriber vel_sub;
  ros::Subscriber sensor_data_sub;
  ros::Time current_time;

  nuturtlebot::WheelCommands wheel_command;

  rigid2d::Twist2D twist;
  rigid2d::DiffDrive diff_drive;
  rigid2d::WheelVelocity wheel_vel;
  rigid2d::WheelAngle wheel_angle;
};

/// \brief Main function
/// \returns int
int main(int argc, char *argv[]) {
  ros::init(argc, argv, "turtle_interface_node");
  TurtleInterface node;
  node.main_loop();
  ros::spin();
  return 0;
}