/// \file   slam.cpp
/// \brief  Implementation of the extended Kalman Filter SLAM
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
///
/// PUBLISHES:
                    ///     odom (nav_msgs/Odometry): publishes Odometry message on the odom topic.
///
/// SUBSCRIBES:
                    ///     joint_states (sensor_msgs/JointState): Subscribes to the robot's joint_states.
///
/// SERVICES:
                    ///     SetPose (set_pose): Restarts the location of the odometry, so that the robot thinks
                    ///                         it is at the requested configuration.


#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"
#include "nuslam/nuslam.hpp"

#include "ros/ros.h"

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rigid2d/SetPose.h>

#include <iostream>
#include <vector>
#include <utility>
#include <string>
#include <cmath>
#include <armadillo>


/// \brief Class KFSlam
class KFSlam
{
    public:
        KFSlam(){
            ROS_INFO("Initialize the variables");
            // Init Parameters
            load_parameter();

            // Init publishers, subscribers, and services
            odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1);
            // slam_pub = nh.advertise<nav_msgs::Odometry>("/slam", 1);
            landmarks_pub = nh.advertise<visualization_msgs::MarkerArray>("/landmarks", 1);
            slam_landmarks_pub = nh.advertise<visualization_msgs::MarkerArray>("/slam_landmarks", 1);

            joint_states_sub = nh.subscribe("/joint_states", 1, &KFSlam::joint_state_callback, this);
            landmarks_sub = nh.subscribe("/fake_sensor", 1, &KFSlam::landmarks_callback, this);

            set_pose_srv = nh.advertiseService("/set_pose", &KFSlam::set_pose_callback, this);
         }

        /// \brief Load the parameters from the parameter server
        /// \returns void
        void load_parameter() {
            nh.getParam("wheel_base", wheel_base);                  // The distance between the wheels
            nh.getParam("wheel_radius", wheel_radius);              // The radius of the wheels
            nh.getParam("world_frame_id", world_frame_id);          // The name of the world tf frame
            nh.getParam("map_frame_id", map_frame_id);              // The name of the map tf frame
            nh.getParam("odom_frame_id", odom_frame_id);            // The name of the odometry tf frame
            nh.getParam("body_frame_id", body_frame_id);            // The name of the body tf frame
            nh.getParam("left_wheel_joint", left_wheel_joint);      // The name of the left wheel joint
            nh.getParam("right_wheel_joint", right_wheel_joint);    // The name of the right wheel joint
            nh.getParam("obstacles_radius", obstacles_radius);      // The radous of the cardboard tubes [m]
        }
        
        /// \brief Subscribes to the robot's joint_states.
        /// \param joint_state - constant pointer to joint_states
        /// \returns void
        void joint_state_callback(const sensor_msgs::JointState::ConstPtr &joint_state) {
            // ROS_INFO("Subscribing to joint state");
            right_angle = joint_state->position.at(0);
            left_angle = joint_state->position.at(1);

            wheel_vel = diff_drive.updateOdometryWithAngles(right_angle, left_angle);
            twist = diff_drive.wheels2Twist(wheel_vel);

            joint_state_flag = true;
        }

        /// \brief Subscribes to the map's landmarks.
        /// \param req - SetPose request.
        /// \param res - SetPose response.
        /// \returns void
        void landmarks_callback(const visualization_msgs::MarkerArray &markers) {
            // ROS_INFO("Subscribing to landmarks");
            for (auto& marker: markers.markers) {
                double x = marker.pose.position.x;
                double y = marker.pose.position.y;
                int id = marker.id;
                measurements.push_back(nuslam::Measurement(x, y, id));
            }

            // Raise the landmarks flag
            landmarks_flag = true;
        }

        /// \brief Restarts the location of the odometry, so that the robot thinks
        /// it is at the requested configuration.
        /// \param req - SetPose request.
        /// \param res - SetPose response.
        /// \returns bool
        bool set_pose_callback(rigid2d::SetPose::Request &req, rigid2d::SetPose::Response &res) {
            // ROS_INFO("Setting pose");
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
            // ROS_INFO("Entering the loop");
            ros::Rate loop_rate(frequency);

            // Initializing Extended Kalman Filter
            nuslam::EKF Kalman_Filter;

            while(ros::ok()) {
                current_time = ros::Time::now();
                
                // If the set_pose_callback was called
                if (reset_flag) {
                    diff_drive.set_config(reset_pose);
                    
                    // Provide the configuration of the robot
                    ROS_INFO("Reset robot position:");
                    ROS_INFO("x = %f\n\r", diff_drive.get_config().x);
                    ROS_INFO("y = %f\n\r", diff_drive.get_config().y);
                    ROS_INFO("theta = %f\n\r", diff_drive.get_config().theta);

                    // Remove the set_pose flag
                    reset_flag = false;
                }

                if (landmarks_flag) {
                    wheel_vel_new = diff_drive.get_wheel_vel();
                    wheel_vel_del.right_wheel_vel = wheel_vel_new.right_wheel_vel - wheel_vel.right_wheel_vel;
                    wheel_vel_del.right_wheel_vel = wheel_vel_new.right_wheel_vel - wheel_vel.right_wheel_vel;

                    twist_del = diff_drive.wheels2Twist(wheel_vel_del);

                    Kalman_Filter.run_ekf(twist_del, measurements);
                    q_t = Kalman_Filter.output_state();
                    m_t = Kalman_Filter.output_map_state();
                    std::cout << "m_t " << (m_t) << "\n\r" << std::endl;

                    new_config.theta = q_t(0, 0);
                    new_config.x = q_t(1, 0);
                    new_config.x = q_t(2, 0);
                    diff_drive.set_config(new_config);

                    // for (unsigned int i=0; i<m_t.n_rows/2; i++) {
                    //     slam_marker_array.markers[2*i].header.frame_id = map_frame_id;
                    //     slam_marker_array.markers[2*i].header.stamp = ros::Time();
                    //     slam_marker_array.markers[2*i].ns = "marker";
                    //     slam_marker_array.markers[2*i].id = i;
                    //     slam_marker_array.markers[2*i].type = visualization_msgs::Marker::CYLINDER;
                    //     slam_marker_array.markers[2*i].action = visualization_msgs::Marker::ADD;
                    //     slam_marker_array.markers[2*i].pose.position.x = m_t(2*i, 0);
                    //     slam_marker_array.markers[2*i].pose.position.y = m_t(2*i + 1, 0);
                    //     slam_marker_array.markers[2*i].pose.position.z = 0.0;
                    //     slam_marker_array.markers[2*i].pose.orientation.x = 0.0;
                    //     slam_marker_array.markers[2*i].pose.orientation.y = 0.0;
                    //     slam_marker_array.markers[2*i].pose.orientation.z = 0.0;
                    //     slam_marker_array.markers[2*i].pose.orientation.w = 1.0;
                    //     slam_marker_array.markers[2*i].scale.x = obstacles_radius;
                    //     slam_marker_array.markers[2*i].scale.y = obstacles_radius;
                    //     slam_marker_array.markers[2*i].scale.z = 1.2;
                    //     slam_marker_array.markers[2*i].color.a = 1.0;
                    //     slam_marker_array.markers[2*i].color.r = 0.0;
                    //     slam_marker_array.markers[2*i].color.g = 0.0;
                    //     slam_marker_array.markers[2*i].color.b = 1.0;
                    //     }
                    // slam_landmarks_pub.publish(slam_marker_array);
                }

                if ((joint_state_flag) || (landmarks_flag) || (reset_flag)) {
                    rigid2d::Transform2D T_mo;
                    rigid2d::Vector2D v_mb, v_ob;
                    double angle_mb, angle_ob;

                    odom_pose = diff_drive.get_config();

                    angle_mb = q_t(0, 0);
                    v_mb.x = q_t(1, 0);
                    v_mb.y = q_t(2, 0);
                    rigid2d::Transform2D T_mb(v_mb, angle_mb);
                
                    angle_ob = odom_pose.theta;
                    v_ob.x = odom_pose.x;
                    v_ob.y = odom_pose.y;
                    rigid2d::Transform2D T_ob(v_ob, angle_ob);
                    
                    T_mo = T_mb * (T_ob).inv();

                    // Transform from "world" to "map" frame
                    world_tf.header.stamp = current_time;
                    world_tf.header.frame_id = world_frame_id;
                    world_tf.child_frame_id = map_frame_id;
                    world_tf.transform.translation.x = 0;
                    world_tf.transform.translation.y = 0;
                    world_tf.transform.translation.z = 0;
                    world_tf.transform.rotation.x = 0.0;
                    world_tf.transform.rotation.y = 0.0;
                    world_tf.transform.rotation.z = 0.0;
                    world_tf.transform.rotation.w = 1.0;

                    world_broadcaster.sendTransform(world_tf);

                    // Transform from "map" to "odom" frame
                    map_tf.header.stamp = current_time;
                    map_tf.header.frame_id = map_frame_id;
                    map_tf.child_frame_id = odom_frame_id;
                    map_tf.transform.translation.x = T_mo.x();
                    map_tf.transform.translation.y = T_mo.y();
                    map_tf.transform.translation.z = 0;
                    quat.setRPY(0, 0, T_mo.theta());
                    map_quat = tf2::toMsg(quat);
                    map_tf.transform.rotation = map_quat;

                    map_broadcaster.sendTransform(map_tf);

                    // Transform from "odom" to "body" frame
                    odom_tf.header.stamp = current_time;
                    odom_tf.header.frame_id = odom_frame_id;
                    odom_tf.child_frame_id = body_frame_id;
                    odom_tf.transform.translation.x = odom_pose.x;
                    odom_tf.transform.translation.y = odom_pose.y;
                    odom_tf.transform.translation.z = 0;
                    quat.setRPY(0, 0, odom_pose.theta);
                    odom_quat = tf2::toMsg(quat);
                    odom_tf.transform.rotation = odom_quat;

                    odom_broadcaster.sendTransform(odom_tf);

                    odom.header.stamp = current_time;
                    odom.header.frame_id = odom_frame_id;
                    odom.child_frame_id = body_frame_id;
                    odom.pose.pose.position.x = odom_pose.x;
                    odom.pose.pose.position.y = odom_pose.y;
                    odom.pose.pose.position.z = 0.0;
                    odom.pose.pose.orientation = odom_quat;
                    odom.twist.twist.linear.x = twist.xdot;
                    odom.twist.twist.linear.y = twist.ydot;
                    odom.twist.twist.angular.z = twist.thetadot;

                    odom_pub.publish(odom);


                    // slam.header.stamp = current_time;
                    // slam.header.frame_id = odom_frame_id;
                    // slam.child_frame_id = body_frame_id;
                    // slam.pose.pose.position.x = odom_pose.x;
                    // slam.pose.pose.position.y = odom_pose.y;
                    // slam.pose.pose.position.z = 0.0;
                    // slam.pose.pose.orientation = odom_quat;
                    // slam.twist.twist.linear.x = twist.xdot;
                    // slam.twist.twist.linear.y = twist.ydot;
                    // slam.twist.twist.angular.z = twist.thetadot;

                    // slam_pub.publish(slam);

                    joint_state_flag = false;
                    landmarks_flag = false;
                    reset_flag = false;
                }

                loop_rate.sleep();
                ros::spinOnce();
            }
        }

    private:
        int frequency = 100;
        bool joint_state_flag = false;
        bool landmarks_flag = false;
        bool reset_flag = false;
        double wheel_base, wheel_radius, right_angle, left_angle, obstacles_radius;
        std::string world_frame_id, map_frame_id, odom_frame_id, body_frame_id, \
                    left_wheel_joint, right_wheel_joint;

        arma::mat m_t, q_t;
        
        ros::NodeHandle nh;
        ros::Publisher odom_pub, landmarks_pub, slam_landmarks_pub;
        ros::Subscriber joint_states_sub, landmarks_sub;
        ros::ServiceServer set_pose_srv;
        ros::Time current_time;

        visualization_msgs::MarkerArray slam_marker_array;
        tf2::Quaternion quat;
        tf2_ros::TransformBroadcaster world_broadcaster, map_broadcaster, odom_broadcaster;
        geometry_msgs::TransformStamped world_tf, map_tf, odom_tf;
        geometry_msgs::Quaternion odom_quat, map_quat;
        nav_msgs::Odometry odom, slam;

        rigid2d::Config2D odom_pose, reset_pose, new_config;
        rigid2d::Twist2D twist, twist_del;
        rigid2d::DiffDrive diff_drive;
        rigid2d::WheelVelocity wheel_vel, wheel_vel_new, wheel_vel_del;

        std::vector<nuslam::Measurement> measurements;
};

/// \brief Main function
/// \returns int
int main(int argc, char * argv[])
{
    ros::init(argc, argv, "slam");
    KFSlam node;
    node.main_loop();
    ros::spin();
    return 0;
}