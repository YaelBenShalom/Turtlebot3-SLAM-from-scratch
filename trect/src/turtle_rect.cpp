#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
// #include "../include/trect/turtle_rect.hpp"

#include <geometry_msgs/Twist.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include <turtlesim/Pose.h>
#include <std_srvs/Empty.h>
#include <trect/TurtleStart.h>

#include <iostream>
#include <vector> 
#include <string>
#include <cmath>

class TurtleRect
{
    public:
        TurtleRect(){
            // nh{},
            // pub(nh.advertise<sensor_msgs::JointState>("js", 5)),
            // sub(nh.subscribe("topic", 1000, &TurtleRect::callback, this)),
            // timer(nh.createTimer(ros::Duration(0.1), &TurtleRect::main_loop, this))
        
            ROS_INFO("x_0 is: ");
            load_parameter();

            vel_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 5);
            pose_sub = nh.subscribe("turtle1/pose", 10, &TurtleRect::pose_callback, this);
            clear = nh.serviceClient<std_srvs::Empty>("/clear");
            set_pen = nh.serviceClient<turtlesim::SetPen>("turtle1/set_pen");
            teleport = nh.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
            start = nh.advertiseService("/start", &TurtleRect::turtle_start, this);

            ros::service::waitForService("turtle1/set_pen", -1);
            ros::service::waitForService("turtle1/teleport_absolute", -1);
            ros::service::waitForService("/start", -1);
            main_loop();
         }
         
        void load_parameter() {
            // Load the parameters from the parameter server
            nh.getParam("max_xdot", max_xdot);         // The maximum translational velocity of the robot
            nh.getParam("max_wdot", max_wdot);         // The maximum rotational velocity of the robot
            nh.getParam("frequency", frequency);       // The frequency of the control loop
            // nh.getParam("x_0", x_0);                   // The starting x coordinate of the turtle
            // nh.getParam("y_0", y_0);                   // The starting y coordinate of the turtle
            // nh.getParam("width", width);               // The rectangle's width
            // nh.getParam("height", height);             // The rectangle's height

            // Log the parameters as ROS_INFO messages
            ROS_INFO("max_xdot is: %f", max_xdot);
            ROS_INFO("max_wdot is: %f", max_wdot);
            ROS_INFO("frequency is: %d", frequency);
            // ROS_INFO("x_0 is: %d", x_0);
            // ROS_INFO("y_0 is: %d", y_0);
            // ROS_INFO("width is: %d", width);
            // ROS_INFO("height is: %d", height);
        }
        
        void pose_callback(const turtlesim::PoseConstPtr & msg) {
            ROS_INFO("Getting pose from subscriber");
            pose_x = msg->x;
            pose_y = msg->y;
            pose_theta = msg->theta;
        }

        bool turtle_start(trect::TurtleStart::Request &req, trect::TurtleStart::Response &res) {
            ROS_INFO("start turtle");
            res.my_msg = "Lets start!";

            twist.linear.x = 0;
            twist.angular.z = 0;

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

            // set_pen.call();
            return true;
        }

        void main_loop() {
            ROS_INFO("Entering the loop");
            ros::Rate loop_rate(frequency);
            while(ros::ok()) {

                twist.linear.x = 1;
                twist.angular.z = 2;         
                vel_pub.publish(twist);
                loop_rate.sleep();
                ros::spinOnce();
            }
        }

    private:
        int frequency, x_0, y_0, width, height;
        float max_xdot, max_wdot, pose_x, pose_y, pose_theta;
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
        
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "turtle_rect");
    
    TurtleRect node;
        node.main_loop();
    ros::spin();
    return 0;
}