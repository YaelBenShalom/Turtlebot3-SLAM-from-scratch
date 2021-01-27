#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
// #include "../include/trect/turtle_rect.hpp"

#include <iostream>
#include <vector> 
#include <string>
#include <cmath>

class TurtleRect
{
    public:
        TurtleRect():
            nh{},
            pub(nh.advertise<sensor_msgs::JointState>("js", 5)),
            sub(nh.subscribe("topic", 1000, &TurtleRect::callback, this)),
            timer(nh.createTimer(ros::Duration(0.1), &TurtleRect::main_loop, this))
         {
            void load_parameter();
         }
         
        void load_parameter() {
            // Load the parameters from the parameter server
            nh.getParam("max_xdot", max_xdot);         // The maximum translational velocity of the robot
            nh.getParam("max_wdot", max_wdot);         // The maximum rotational velocity of the robot
            nh.getParam("frequency", frequency);       // The frequency of the control loop
            nh.getParam("x_0", x_0);                   // The starting x coordinate of the turtle
            nh.getParam("y_0", y_0);                   // The starting y coordinate of the turtle
            nh.getParam("width", width);               // The rectangle's width
            nh.getParam("height", height);             // The rectangle's height

            // Log the parameters as ROS_INFO messages
            printf("max_xdot is: %f", max_xdot);
            ROS_INFO("max_xdot is: %f", max_xdot);
            ROS_INFO("max_wdot is: %f", max_wdot);
            ROS_INFO("frequency is: %d", frequency);
            ROS_INFO("x_0 is: %d", x_0);
            ROS_INFO("y_0 is: %d", y_0);
            ROS_INFO("width is: %d", width);
            ROS_INFO("height is: %d", height);
        }
        
         void callback(const sensor_msgs::JointState & js) const
         {
         }

         void main_loop(const ros::TimerEvent &) const
         {
            // implement the state machine here
            //  load_parameter();
             pub.publish(sensor_msgs::JointState{});
         }

    private:
        int frequency, x_0, y_0, width, height;
        float max_xdot, max_wdot;
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Subscriber sub;
        ros::Timer timer;
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "turtle_rect");
    TurtleRect node;
    ros::spin();
    return 0;
}