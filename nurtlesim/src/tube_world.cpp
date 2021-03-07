/// \file   tube_world.cpp
/// \brief  A kinematic simulation of a differential drive robot using the DiffDrive class.
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
///     real_marker_pub (visualization_msgs::MarkerArray): publishes MarkerArray message on the real_markers topic.
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
            joint_states_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
            real_joint_states_pub = nh.advertise<sensor_msgs::JointState>("/real_joint_states", 1);
            real_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/real_markers", 1, true);
            marker_pub = nh.advertise<visualization_msgs::MarkerArray>("/fake_sensor", 1);
            robot_path_pub = nh.advertise<nav_msgs::Path>("/real_path", 1);
            vel_sub = nh.subscribe("/cmd_vel", 1, &TubeWorld::cmd_vel_callback, this);
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
            nh.getParam("body_frame_id", body_frame_id);                    // The name of the body tf frame
            nh.getParam("stddev_linear", stddev_linear);                    // The standard deviation of the linear twist noise
            nh.getParam("stddev_angular", stddev_angular);                  // The standard deviation of the angular twist noise
            nh.getParam("slip_min", slip_min);                              // The minimum wheel slip for the slip noise
            nh.getParam("slip_max", slip_max);                              // The maximum wheel slip for the slip noise
            nh.getParam("obstacles_coordinate_x", obstacles_coordinate_x);  // The x coordinate of the obstacles
            nh.getParam("obstacles_coordinate_y", obstacles_coordinate_y);  // The y coordinate of the obstacles
            nh.getParam("obstacles_radius", obstacles_radius);              // The radous of the cardboard tubes [m]
            nh.getParam("max_visable_dist", max_visable_dist);              // The maximum distance beyond which tubes are not visible [m]
        }


        /// \brief Subscribes to the robot's velocity.
        /// \param tw - constant pointer to twist
        /// \returns void
        void cmd_vel_callback(const geometry_msgs::Twist &tw) {
            ROS_INFO("Subscribing to Twist");

            // Calculate the twist without noise factor
            twist.thetadot = tw.angular.z;
            twist.xdot = tw.linear.x;
            twist.ydot = tw.linear.y;
            
            // Raise the cmd_vel flag
            cmd_vel_flag = true;
        }


        /// \brief Main loop for the turtle's motion
        /// \returns void
        void main_loop() {
            ROS_INFO("Entering the loop\n\r");
            ros::Rate loop_rate(frequency);
            static std::default_random_engine generator;
            static std::uniform_real_distribution<double> distribution(slip_min, slip_max);

            real_marker_array.markers.resize(obstacles_coordinate_x.size());
            for (unsigned int i=0; i<obstacles_coordinate_x.size(); i++) {
                real_marker_array.markers[i].header.frame_id = world_frame_id;
                real_marker_array.markers[i].header.stamp = ros::Time();
                real_marker_array.markers[i].ns = "real";
                real_marker_array.markers[i].id = i;
                real_marker_array.markers[i].type = visualization_msgs::Marker::CYLINDER;
                real_marker_array.markers[i].action = visualization_msgs::Marker::ADD;
                real_marker_array.markers[i].pose.position.x = obstacles_coordinate_x[i];
                real_marker_array.markers[i].pose.position.y = obstacles_coordinate_y[i];
                real_marker_array.markers[i].pose.position.z = 0.0;
                real_marker_array.markers[i].pose.orientation.x = 0.0;
                real_marker_array.markers[i].pose.orientation.y = 0.0;
                real_marker_array.markers[i].pose.orientation.z = 0.0;
                real_marker_array.markers[i].pose.orientation.w = 1.0;
                real_marker_array.markers[i].scale.x = 2 * obstacles_radius;
                real_marker_array.markers[i].scale.y = 2 * obstacles_radius;
                real_marker_array.markers[i].scale.z = 1.0;
                real_marker_array.markers[i].color.a = 1.0;
                real_marker_array.markers[i].color.r = 0.0;
                real_marker_array.markers[i].color.g = 1.0;
                real_marker_array.markers[i].color.b = 0.0;
                // real_marker_array.markers[i].lifetime = ros::Duration(10);
                }
            real_marker_pub.publish(real_marker_array);

            while(ros::ok()) {
                // ROS_INFO("Looping");
                ros::spinOnce();
                current_time = ros::Time::now();

                world_tf.header.stamp = current_time;
                world_tf.header.frame_id = world_frame_id;
                world_tf.child_frame_id = body_frame_id;
                world_tf.transform.translation.x = pose.x;
                world_tf.transform.translation.y = pose.y;
                world_tf.transform.translation.z = 0;

                quat.setRPY(0, 0, pose.theta);
                world_quat = tf2::toMsg(quat);
                world_tf.transform.rotation = world_quat;

                world_broadcaster.sendTransform(world_tf);


                for (unsigned int i=0; i<obstacles_coordinate_x.size(); i++) {
                    marker_array.markers.resize(obstacles_coordinate_x.size());
                    marker_array.markers[i].header.frame_id = world_frame_id;
                    marker_array.markers[i].header.stamp = ros::Time();
                    marker_array.markers[i].ns = "marker";
                    marker_array.markers[i].id = i;
                    marker_array.markers[i].type = visualization_msgs::Marker::CYLINDER;

                    pose = diff_drive.get_config();
                    real_pose = real_diff_drive.get_config();

                    markers_dist = sqrt(pow(real_marker_array.markers[i].pose.position.x - real_pose.x, 2) + \
                                        pow(real_marker_array.markers[i].pose.position.y - real_pose.y, 2));
                    if (markers_dist > max_visable_dist) {
                        marker_array.markers[i].action = visualization_msgs::Marker::DELETE;
                    }
                    else {
                        marker_array.markers[i].action = visualization_msgs::Marker::ADD;
                    }

                    marker_array.markers[i].pose.position.x = obstacles_coordinate_x[i] + distribution(generator);
                    marker_array.markers[i].pose.position.y = obstacles_coordinate_y[i] + distribution(generator);
                    marker_array.markers[i].pose.position.z = 0.0;
                    marker_array.markers[i].pose.orientation.x = 0.0;
                    marker_array.markers[i].pose.orientation.y = 0.0;
                    marker_array.markers[i].pose.orientation.z = 0.0;
                    marker_array.markers[i].pose.orientation.w = 1.0;
                    marker_array.markers[i].scale.x = 2 * obstacles_radius + distribution(generator);
                    marker_array.markers[i].scale.y = 2 * obstacles_radius + distribution(generator);
                    marker_array.markers[i].scale.z = 1.0;
                    marker_array.markers[i].color.a = 1.0;
                    marker_array.markers[i].color.r = 1.0;
                    marker_array.markers[i].color.g = 0.0;
                    marker_array.markers[i].color.b = 0.0;
                    marker_array.markers[i].lifetime = ros::Duration(10);
                    }
                marker_pub.publish(marker_array);


                real_pose_stamped.header.stamp = current_time;
                real_pose_stamped.pose.position.x = real_pose.x;
                real_pose_stamped.pose.position.y = real_pose.y;
                real_pose_stamped.pose.position.z = 0.0;

                quat.setRPY(0, 0, real_pose.theta);
                real_quat = tf2::toMsg(quat);
                real_pose_stamped.pose.orientation = real_quat;

                real_path.header.stamp = current_time;
                real_path.header.frame_id = world_frame_id;
                real_path.poses.push_back(real_pose_stamped);
                
                robot_path_pub.publish(real_path);


                // If the cmd_vel_callback was called
                if (cmd_vel_flag) {
                    sensor_msgs::JointState joint_state, real_joint_state;

                    // Calculate the twist with noise factor
                    static std::default_random_engine generator;
                    static std::normal_distribution<double> dist_linear(0, stddev_linear);
                    static std::normal_distribution<double> dist_angular(0, stddev_angular);
                    double twist_angular_noised = dist_angular(generator);
                    double twist_linear_noised = dist_linear(generator);

                    twist_noised.thetadot = twist.thetadot + twist_angular_noised;
                    twist_noised.xdot = twist.xdot + twist_linear_noised;
                    twist_noised.ydot = twist.ydot;

                    // ROS_INFO("twist_noised.thetadot = %f\t twist_noised.xdot = %f\n\r", twist_noised.thetadot, twist_noised.xdot);
                    
                    // TODO - collision detection
                    for (unsigned int i=0; i<obstacles_coordinate_x.size(); i++) {

                        collision_dist = sqrt(pow(real_marker_array.markers[i].pose.position.x - real_pose.x, 2) + \
                                              pow(real_marker_array.markers[i].pose.position.y - real_pose.y, 2)) - \
                                              obstacles_radius - wheel_base;

                        if (collision_dist <= 0) {
                            ROS_INFO("Collosion!!\n\r");
                            // wheel_vel = diff_drive.get_wheel_vel();


                            // twist.thetadot = 0;
                            // twist.xdot = 0;
                            // twist.ydot = 0;

                            // twist_noised.thetadot = 0;
                            // twist_noised.xdot = 0;
                            // twist_noised.ydot = 0;

                            wheel_angle = diff_drive.rotatingWheelsWithTwist(twist_noised);
                            real_wheel_angle = diff_drive.rotatingWheelsWithTwist(twist);

                            collision_flag = true;
                            break;
                        }
                    }

                    if (!collision_flag) {
                    wheel_angle = diff_drive.updateOdometryWithTwist(twist_noised);
                    real_wheel_angle = real_diff_drive.updateOdometryWithTwist(twist);
                    }

                    // Calculate the wheel_angle with the wheel angle noise factor
                    wheel_angle.right_wheel_angle += wheel_angle.right_wheel_angle * distribution(generator);
                    wheel_angle.left_wheel_angle += wheel_angle.right_wheel_angle * distribution(generator);

                    real_wheel_angle.right_wheel_angle += real_wheel_angle.right_wheel_angle;
                    real_wheel_angle.left_wheel_angle += real_wheel_angle.right_wheel_angle;

                    joint_state.header.stamp = current_time;
                    joint_state.name.push_back(right_wheel_joint);
                    joint_state.name.push_back(left_wheel_joint);
                    joint_state.position.push_back(wheel_angle.right_wheel_angle);
                    joint_state.position.push_back(wheel_angle.left_wheel_angle);

                    joint_states_pub.publish(joint_state);

                    real_joint_state.header.stamp = current_time;
                    real_joint_state.name.push_back(right_wheel_joint);
                    real_joint_state.name.push_back(left_wheel_joint);
                    real_joint_state.position.push_back(real_wheel_angle.right_wheel_angle);
                    real_joint_state.position.push_back(real_wheel_angle.left_wheel_angle);

                    real_joint_states_pub.publish(real_joint_state);

                    // Remove the cmd_vel flag
                    cmd_vel_flag = false;
                }
                loop_rate.sleep();
            }
        }

    private:
        int frequency = 100;
        bool cmd_vel_flag = false;
        bool collision_flag = false;
        double wheel_base, wheel_radius, stddev_linear, stddev_angular, slip_min, slip_max, obstacles_radius, max_visable_dist, markers_dist, collision_dist;
        std::string left_wheel_joint, right_wheel_joint, world_frame_id, odom_frame_id, body_frame_id;
        std::vector<double> obstacles_coordinate_x, obstacles_coordinate_y;

        ros::NodeHandle nh;
        ros::Publisher joint_states_pub, real_joint_states_pub, marker_pub, real_marker_pub, robot_path_pub;
        ros::Subscriber vel_sub;
        ros::Time current_time;

        visualization_msgs::MarkerArray marker_array, real_marker_array;
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