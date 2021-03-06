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
///     joint_msg (sensor_msgs::JointState): message to publish wheel angle
///         readings to /joint_states topic
///     static_world_broadcaster (StaticTransformBroadcaster): Broadcast the
///         static transform between world_frame_id and the map_frame_id on /tf
///         using a tf2
///     odom_broadcaster (TransformBroadcaster): Broadcast the transform between
///         map_frame_id and the odom_frame_id on /tf using a tf2
///     map_broadcaster (TransformBroadcaster): Broadcast the transform between
///         odom_frame_id and the body_frame_id on /tf using a tf2
///     static_world_tf(geometry_msgs::TransformStamped): static world transform
///     map_tf(geometry_msgs::TransformStamped): map transform
///     odom_tf(geometry_msgs::TransformStamped): odometry transform
///     odom(nav_msgs::Odometry): odometry message
///     slam (nav_msgs::Odometry): slam message
///     slam_marker_array (visualization_msgs::MarkerArray): an array of
///         obstacles publishing to the slam simulator.
///
///     odom_pose (rigid2d::Config2D): the robot's position (based on the wheel
///         angles)
///     reset_pose (rigid2d::Config2D): the robot's reset position
///     twist(rigid2d::Twist2D): the robot's twist
///     twist_del (rigid2d::Twist2D): the delta between the robot's new twist
///         to the robot's last twist
///     diff_drive (rigid2d::DiffDrive): an instance of the diff_drive robot
///     wheel_vel (rigid2d::WheelVelocity): the velocity of the robot's wheels
///     wheel_angle (rigid2d::WheelAngle): the angle of the robot's wheels
///
///     measurements (std::vector<nuslam::Measurement>): a vector with
///     components of type nuslam::Measurement. m_t (arma::mat): the map matrix,
///     as measured in the slam simulator. q_t(arma::mat): the state matrix, as
///     measured in the slam simulator.
///
/// PUBLISHES:
///     odom (nav_msgs/Odometry): publishes Odometry message on the odom topic.
///     landmarks_pub (visualization_msgs/MarkerArray): publishes real
///     landmarks. slam_landmarks_pub (visualization_msgs/MarkerArray):
///     publishes landmark
///         to be subscribed by the slam simulation.
///
/// SUBSCRIBES:
///     landmarks_sub (visualization_msgs/Marker): Subscribes to the MarkerArray
///         that's publish by the simulation.
///
/// SERVICES:
///     SetPose (set_pose): Restarts the location of the odometry, so that the
///         robot thinksit is at the requested configuration.

#include "nuslam/nuslam.hpp"
#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"

#include "ros/ros.h"

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <rigid2d/SetPose.h>

#include <armadillo>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

/// \brief Class KFSlam
class KFSlam
{
public:
  KFSlam()
  {
    ROS_INFO("Initialize the variables");
    // Init Parameters
    load_parameter();

    // Init publishers, subscribers, and services
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1);
    slam_landmarks_pub =
        nh.advertise<visualization_msgs::MarkerArray>("/slam_landmarks", 1);

    joint_states_sub =
        nh.subscribe("/joint_states", 1, &KFSlam::joint_state_callback, this);
    landmarks_sub =
        nh.subscribe("/fake_sensor", 1, &KFSlam::landmarks_callback, this);

    set_pose_srv =
        nh.advertiseService("/set_pose", &KFSlam::set_pose_callback, this);
  }

  /// \brief Load the parameters from the parameter server
  /// \returns void
  void load_parameter()
  {
    nh.getParam("wheel_base", wheel_base);     // The distance between the wheels
    nh.getParam("wheel_radius", wheel_radius); // The radius of the wheels
    nh.getParam("world_frame_id",
                world_frame_id);               // The name of the world tf frame
    nh.getParam("map_frame_id", map_frame_id); // The name of the map tf frame
    nh.getParam("odom_frame_id",
                odom_frame_id); // The name of the odometry tf frame
    nh.getParam("body_frame_id",
                body_frame_id); // The name of the body tf frame
    nh.getParam("left_wheel_joint",
                left_wheel_joint); // The name of the left wheel joint
    nh.getParam("right_wheel_joint",
                right_wheel_joint); // The name of the right wheel joint
    nh.getParam("obstacles_radius",
                obstacles_radius); // The radous of the cardboard tubes [m]
  }

  /// \brief Subscribes to the robot's joint_states.
  /// \param joint_state - constant pointer to joint_states
  /// \returns void
  void
  joint_state_callback(const sensor_msgs::JointState::ConstPtr &joint_state)
  {
    // ROS_INFO("Subscribing to joint state");
    right_angle = joint_state->position.at(0);
    left_angle = joint_state->position.at(1);

    joint_state_flag = true;
  }

  /// \brief Subscribes to the map's landmarks.
  /// \param markers - markers visualization msgs.
  /// \returns void
  void landmarks_callback(const visualization_msgs::MarkerArray &markers)
  {
    // ROS_INFO("Subscribing to landmarks");
    for (auto &marker : markers.markers)
    {
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
  bool set_pose_callback(rigid2d::SetPose::Request &req,
                         rigid2d::SetPose::Response &res)
  {
    // ROS_INFO("Setting pose");
    reset_pose.x = req.x;
    reset_pose.y = req.y;
    reset_pose.theta = req.theta;
    res.result = true;

    // Raise the reset flag
    reset_flag = true;

    return true;
  }

  /// \brief Gets markers from map
  /// \returns void
  void get_marker_from_map()
  {
    slam_marker_array.markers.resize(m_t.size());
    for (unsigned int i = 0; i < m_t.n_rows / 2; i++)
    {
      slam_marker_array.markers.resize(m_t.n_rows / 2);
      slam_marker_array.markers[i].header.frame_id = map_frame_id;
      slam_marker_array.markers[i].header.stamp = ros::Time();
      slam_marker_array.markers[i].ns = "marker";
      slam_marker_array.markers[i].id = i;
      slam_marker_array.markers[i].type = visualization_msgs::Marker::CYLINDER;
      slam_marker_array.markers[i].action = visualization_msgs::Marker::ADD;
      slam_marker_array.markers[i].pose.position.x = m_t(2 * i, 0);
      slam_marker_array.markers[i].pose.position.y = m_t(2 * i + 1, 0);
      slam_marker_array.markers[i].pose.position.z = 0.0;
      slam_marker_array.markers[i].pose.orientation.x = 0.0;
      slam_marker_array.markers[i].pose.orientation.y = 0.0;
      slam_marker_array.markers[i].pose.orientation.z = 0.0;
      slam_marker_array.markers[i].pose.orientation.w = 1.0;
      slam_marker_array.markers[i].scale.x = 3 * obstacles_radius;
      slam_marker_array.markers[i].scale.y = 3 * obstacles_radius;
      slam_marker_array.markers[i].scale.z = 0.5;
      slam_marker_array.markers[i].color.a = 1.0;
      slam_marker_array.markers[i].color.r = 0.0;
      slam_marker_array.markers[i].color.g = 0.0;
      slam_marker_array.markers[i].color.b = 1.0;

      slam_landmarks_pub.publish(slam_marker_array);
    }
  }

  /// \brief Broadcast the static "world" to "map" transform
  /// \returns void
  void world_map_transform()
  {
    // Transform from "world" to "map" frame
    static_world_tf.header.stamp = ros::Time::now();
    static_world_tf.header.frame_id = world_frame_id;
    static_world_tf.child_frame_id = map_frame_id;
    static_world_tf.transform.translation.x = 0;
    static_world_tf.transform.translation.y = 0;
    static_world_tf.transform.translation.z = 0;
    static_world_tf.transform.rotation.x = 0.0;
    static_world_tf.transform.rotation.y = 0.0;
    static_world_tf.transform.rotation.z = 0.0;
    static_world_tf.transform.rotation.w = 1.0;

    static_world_broadcaster.sendTransform(static_world_tf);
  }

  /// \brief Broadcast the "map" to "odom" transform
  /// \returns void
  void map_odom_transform()
  {
    rigid2d::Transform2D T_mo, T_om;
    rigid2d::Vector2D v_mb, v_ob;
    // double angle_mb, angle_ob;

    // angle_mb = q_t(0, 0);
    v_mb.x = q_t(1, 0);
    v_mb.y = q_t(2, 0);

    rigid2d::Transform2D T_mb(v_mb, 0); // angle_mb

    // angle_ob = odom_pose.theta;
    v_ob.x = odom_pose.x;
    v_ob.y = odom_pose.y;
    rigid2d::Transform2D T_ob(v_ob, 0); // angle_ob

    T_mo = T_mb * T_ob.inv();

    // Transform from "map" to "odom" frame
    map_tf.header.stamp = ros::Time::now(); // TODO
    map_tf.header.frame_id = map_frame_id;
    map_tf.child_frame_id = odom_frame_id;
    map_tf.transform.translation.x = 0; // T_mo.x();
    map_tf.transform.translation.y = 0; // T_mo.y();
    map_tf.transform.translation.z = 0;
    quat.setRPY(0, 0, 0); // T_mo.theta()
    map_quat = tf2::toMsg(quat);
    map_tf.transform.rotation = map_quat;

    map_broadcaster.sendTransform(map_tf);
  }

  /// \brief Broadcast the "odom" to "body" transform
  /// \returns void
  void odom_body_transform()
  {
    // Transform from "odom" to "body" frame
    odom_tf.header.stamp = ros::Time::now();
    odom_tf.header.frame_id = odom_frame_id;
    odom_tf.child_frame_id = body_frame_id;
    odom_tf.transform.translation.x = odom_pose.x;
    odom_tf.transform.translation.y = odom_pose.y;
    odom_tf.transform.translation.z = 0;
    quat.setRPY(0, 0, odom_pose.theta);
    odom_quat = tf2::toMsg(quat);
    odom_tf.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_tf);
  }

  /// \brief Broadcast the "world" to "slam" transform
  /// \returns void
  void world_slam_transform()
  {
    // Transform from "world" to "slam" frame
    slam_tf.header.stamp = ros::Time::now();
    slam_tf.header.frame_id = world_frame_id;
    slam_tf.child_frame_id = "slam";
    slam_tf.transform.translation.x = q_t(1, 0);
    slam_tf.transform.translation.y = q_t(2, 0);
    slam_tf.transform.translation.z = 0;
    quat.setRPY(0, 0, q_t(0, 0));
    odom_quat = tf2::toMsg(quat);
    slam_tf.transform.rotation = odom_quat;
    slam_broadcaster.sendTransform(slam_tf);
  }

  /// \brief Publish odometry messages
  /// \returns void
  void publish_odometry()
  {
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame_id;
    odom.child_frame_id = body_frame_id;
    odom.pose.pose.position.x = odom_pose.x;
    odom.pose.pose.position.y = odom_pose.y;
    odom.pose.pose.position.z = 0.0;
    quat.setRPY(0, 0, odom_pose.theta);
    odom_quat = tf2::toMsg(quat);
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = twist.xdot;
    odom.twist.twist.linear.y = twist.ydot;
    odom.twist.twist.angular.z = twist.thetadot;

    odom_pub.publish(odom);
  }

  /// \brief Main loop for the turtle's motion
  /// \returns void
  void main_loop()
  {
    // ROS_INFO("Entering the loop");
    ros::Rate loop_rate(frequency);

    // Initializing Extended Kalman Filter
    nuslam::EKF Kalman_Filter;
    diff_drive = rigid2d::DiffDrive();
    wheel_angle_old = {0, 0};

    while (ros::ok())
    {
      current_time = ros::Time::now();

      if (joint_state_flag)
      {
        // ROS_INFO("wheel_angle = %f, %f\n\r", right_angle, left_angle);

        diff_drive.updateOdometryWithAngles(right_angle, left_angle);
        odom_pose = diff_drive.get_config();
        // ROS_INFO("odom_pose = %f, %f\n\r", odom_pose.x, odom_pose.y);

        joint_state_flag = false;
      }

      // If the set_pose_callback was called
      if (reset_flag)
      {
        diff_drive.set_config(reset_pose);

        // Provide the configuration of the robot
        ROS_INFO("Reset robot position:");
        ROS_INFO("x = %f\n\r", diff_drive.get_config().x);
        ROS_INFO("y = %f\n\r", diff_drive.get_config().y);
        ROS_INFO("theta = %f\n\r", diff_drive.get_config().theta);

        // Remove the set_pose flag
        reset_flag = false;
      }

      if (landmarks_flag)
      {
        wheel_angle_new = diff_drive.get_wheel_angle();
        // ROS_INFO("wheel_angle_old = %f, %f \n\r",
        // wheel_angle_old.right_wheel_angle,
        // wheel_angle_old.right_wheel_angle); ROS_INFO("wheel_angle_new = %f,
        // %f \n\r", wheel_angle_new.right_wheel_angle,
        // wheel_angle_new.right_wheel_angle);
        wheel_vel_del.right_wheel_vel =
            rigid2d::normalize_angle(wheel_angle_new.right_wheel_angle -
                                     wheel_angle_old.right_wheel_angle);
        wheel_vel_del.left_wheel_vel =
            rigid2d::normalize_angle(wheel_angle_new.left_wheel_angle -
                                     wheel_angle_old.left_wheel_angle);

        twist_del = diff_drive.wheels2Twist(wheel_vel_del);
        wheel_angle_old = wheel_angle_new;

        // Running Extended Kalman Filter
        Kalman_Filter.run_ekf(twist_del, measurements);
        q_t = Kalman_Filter.output_state();
        m_t = Kalman_Filter.output_map_state();

        measurements.clear();

        // Publishing map markers
        get_marker_from_map();

        landmarks_flag = false;
      }

      // Setting transformations
      world_map_transform();
      world_slam_transform(); // TODO
      map_odom_transform();
      odom_body_transform();

      // Publish odometry messages
      publish_odometry();

      loop_rate.sleep();
      ros::spinOnce();
    }
  }

private:
  int frequency = 10;
  bool joint_state_flag = false;
  bool landmarks_flag = false;
  bool reset_flag = false;
  double wheel_base, wheel_radius, right_angle, left_angle, obstacles_radius;
  std::string world_frame_id, map_frame_id, odom_frame_id, body_frame_id,
      left_wheel_joint, right_wheel_joint;

  arma::mat m_t;
  arma::mat q_t = arma::mat(3, 1).fill(0.0);

  ros::NodeHandle nh;
  ros::Publisher odom_pub, slam_pub, landmarks_pub, slam_landmarks_pub;
  ros::Subscriber joint_states_sub, landmarks_sub;
  ros::ServiceServer set_pose_srv;
  ros::Time current_time;

  visualization_msgs::MarkerArray slam_marker_array;
  tf2::Quaternion quat;
  tf2_ros::TransformBroadcaster map_broadcaster, odom_broadcaster,
      slam_broadcaster;
  tf2_ros::StaticTransformBroadcaster static_world_broadcaster;
  geometry_msgs::TransformStamped map_tf, odom_tf, slam_tf;
  geometry_msgs::TransformStamped static_world_tf;

  geometry_msgs::Quaternion odom_quat, map_quat;
  nav_msgs::Odometry odom, slam;

  rigid2d::Config2D odom_pose, reset_pose;
  rigid2d::Twist2D twist, twist_del;
  rigid2d::DiffDrive diff_drive;
  rigid2d::WheelVelocity wheel_vel, wheel_vel_del;
  rigid2d::WheelAngle wheel_angle, wheel_angle_new, wheel_angle_old;

  std::vector<nuslam::Measurement> measurements;
};

/// \brief Main function
/// \param argc - input int argument
/// \param argv - input array argument
/// \returns int
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "slam");
  KFSlam node;
  node.main_loop();
  ros::spin();
  return 0;
}
