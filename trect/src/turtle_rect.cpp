/// \file   turtle_rect.cpp
/// \brief  Causes the turtlesim turtle to follow a rectangle path.
///
/// PUBLISHES:
///     cmd_vel (Twist): publishes the turtle's linear and angular velocity as a
///         twist massage.
/// SUBSCRIBES:
///     turtlesim/Pose (Pose): SubsCriber to the turtle's position.
/// SERVICES:
///     turtle_start (TurtleStart): Starts the turtle motion and restart it when
///         called again.

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <trect/TurtleStart.h>
#include <turtlesim/Pose.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/TeleportAbsolute.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

/// \brief Class State
enum class State
{
  STOP,
  TURN,
  FORWARD
};

/// \brief Class TurtleRect
class TurtleRect
{
public:
  TurtleRect()
  {
    ROS_INFO("Initialize the variables");

    state = State::STOP;

    load_parameter();

    vel_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 5);
    pose_sub =
        nh.subscribe("turtle1/pose", 10, &TurtleRect::pose_callback, this);
    clear = nh.serviceClient<std_srvs::Empty>("/clear");
    set_pen = nh.serviceClient<turtlesim::SetPen>("turtle1/set_pen");
    teleport = nh.serviceClient<turtlesim::TeleportAbsolute>(
        "turtle1/teleport_absolute");
    start = nh.advertiseService("/start", &TurtleRect::turtle_start, this);

    ros::service::waitForService("turtle1/set_pen", -1);
    ros::service::waitForService("turtle1/teleport_absolute", -1);
    ros::service::waitForService("/start", -1);
    main_loop();
  }

  /// \brief Load the parameters from the parameter server
  /// \returns void
  void load_parameter()
  {
    // The maximum translational velocity of the robot
    nh.getParam("max_xdot", max_xdot);
    // The maximum rotational velocity of the robot
    nh.getParam("max_wdot", max_wdot);
    // The frequency of the control loop
    nh.getParam("frequency", frequency);

    // Log the parameters as ROS_INFO messages
    ROS_INFO("max_xdot is: %f", max_xdot);
    ROS_INFO("max_wdot is: %f", max_wdot);
    ROS_INFO("frequency is: %d", frequency);
  }

  /// \brief Subscribes to the turtle position
  /// \param msg - constant pointer to turtle Pose
  /// \returns void
  void pose_callback(const turtlesim::PoseConstPtr &msg)
  {
    pose_x = msg->x;
    pose_y = msg->y;
    pose_theta = msg->theta;
  }

  /// \brief Clear the screen, draw a rectangle and starts the turtle motion.
  /// \param req - TurtleStart request.
  /// \param res - TurtleStart response.
  /// \returns bool
  bool turtle_start(trect::TurtleStart::Request &req,
                    trect::TurtleStart::Response &res)
  {
    ROS_INFO("start turtle");
    res.my_msg = "Lets start!";

    state = State::STOP;
    point_num = 1;

    x_0 = req.x;
    y_0 = req.y;
    height = req.height;
    width = req.width;

    teleport_msg.request.x = x_0;
    teleport_msg.request.y = y_0;
    teleport_msg.request.theta = 0;

    teleport.call(teleport_msg);
    clear.call(empty_msg);

    teleport_msg.request.x = x_0 + width;
    teleport_msg.request.y = y_0;
    teleport.call(teleport_msg);

    teleport_msg.request.x = x_0 + width;
    teleport_msg.request.y = y_0 + height;
    teleport.call(teleport_msg);

    teleport_msg.request.x = x_0;
    teleport_msg.request.y = y_0 + height;
    teleport.call(teleport_msg);

    teleport_msg.request.x = x_0;
    teleport_msg.request.y = y_0;
    teleport.call(teleport_msg);

    teleport_msg.request.x = x_0;
    teleport_msg.request.y = y_0;
    teleport.call(teleport_msg);

    state = State::FORWARD;

    return true;
  }

  /// \brief Main loop for the turtle's motion
  /// \returns void
  void main_loop()
  {
    ROS_INFO("Entering the loop");
    ros::Rate loop_rate(frequency);
    while (ros::ok())
    {
      switch (state)
      {
      case State::STOP:
        twist.linear.x = 0;
        twist.angular.z = 0;
        vel_pub.publish(twist);
        break;

      case State::TURN:
        twist.linear.x = 0;
        twist.angular.z = max_wdot / 2;
        vel_pub.publish(twist);
        ROS_INFO("dTheta is: %f", fabs(pose_theta - current_angle));
        if (fabs(pose_theta - current_angle) >= PI / 2)
        {
          state = State::FORWARD;
        }
        break;

      case State::FORWARD:
        twist.linear.x = max_xdot / 2;
        twist.angular.z = 0;
        vel_pub.publish(twist);

        ROS_INFO("X and Y are: %f, %f", pose_x, pose_y);
        if (((point_num == 1) && (pose_x >= x_0 + width)) ||
            ((point_num == 2) && (pose_y >= y_0 + height)) ||
            ((point_num == 3) && (pose_x <= x_0)) ||
            ((point_num == 0) && (pose_y <= y_0)))
        {
          point_num += 1;
          if (point_num == 4)
          {
            point_num = 0;
          }
          current_angle = pose_theta;
          state = State::TURN;
        }
        break;

      default:
        // should never get here
        throw std::logic_error("Invalid State");
      }

      loop_rate.sleep();
      ros::spinOnce();
    }
  }

private:
  double PI = 3.14159265358979323846;
  int frequency, x_0, y_0, width, height, point_num;
  float max_xdot, max_wdot, pose_x, pose_y, pose_theta, current_angle;
  ros::NodeHandle nh;
  ros::Publisher vel_pub;
  ros::Subscriber pose_sub;
  ros::Timer timer;
  ros::ServiceClient clear;
  ros::ServiceClient set_pen;
  ros::ServiceClient teleport;
  ros::ServiceServer start;
  turtlesim::SetPen pen_srv;
  turtlesim::TeleportAbsolute teleport_msg;
  geometry_msgs::Twist twist;
  std_srvs::Empty empty_msg;
  State state;
};

/// \brief Main function
/// \param argc - input int argument
/// \param argv - input array argument
/// \returns int
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "turtle_rect");
  TurtleRect node;
  node.main_loop();
  ros::spin();
  return 0;
}
