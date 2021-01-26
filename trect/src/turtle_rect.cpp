#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

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
         }

         void callback(const sensor_msgs::JointState & js) const
         {
         }

         void main_loop(const ros::TimerEvent &) const
         {
            // implement the state machine here
             pub.publish(sensor_msgs::JointState{});
         }

    private:
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