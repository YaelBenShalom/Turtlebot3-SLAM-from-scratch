# ifndef TURTLE_RECT_H
# define TURTLE_RECT_H

# include <ros/ros.h>
# include <turtlesim/Pose.h>
# include <std_msgs/Bool.h>
# include <cmath>

namespace turtle_rect
{

class TurtleRect
{

public:

  TurtleRect();
  
  void control();
private:

  //***************** NODE HANDLES ***************//
  ros::NodeHandle nh_;         // public node handle for subscribing, publishing, etc.
  ros::NodeHandle nh_private_; // private node handle for pulling parameter values from the parameter server

  //***************** PUBLISHERS AND SUBSCRIBERS ***************//
  ros::Subscriber pose_subscriber_;
  // will end up getting hooked up to the callback for the Pose message

  ros::Publisher bool_publisher_;
  // will publish the flag to ROS

  //***************** PARAMETERS ***************//
  double threshold_;
  // a parameter we get from the ROS server, in this case the value below which
  // we consider the turtle as not moving.  This is basically a class variable at this point,
  // but it is distinct from the other class variables, so we separate them here.

  //***************** STATE VARIABLES ***************//
  // in this node, we don't have any variables.  Often though, we need to remember
  // things between loops, so we could create variables here to hold those values

  //***************** CALLBACKS ***************//
  void poseCallback(const turtlesim::PoseConstPtr &msg);
  // this function will get called every time ROS "spins"
  // and there is a Pose message in the queue.  More on this
  // later

  //***************** FUNCTIONS ***************//
  // Also, in this node, we don't have any "helper" functions.  These are useful
  // if you need to break up the work in the node into different functions
};

} // namespace moving_sensor

#endif // movingSensor_H