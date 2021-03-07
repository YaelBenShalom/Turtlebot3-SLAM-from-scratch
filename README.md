# ME495 Sensing, Navigation and Machine Learning For Robotics
* Yael Ben Shalom
* Winter 2021


# Package List
This repository consists of several ROS packages:
- [**nuturtle_description**](https://github.com/ME495-Navigation/assignment-YaelBenShalom/tree/master/nuturtle_description) - A package that adapts the tutlebot3_burger, a differential drive robot, for our needs.
- [**rigid2d**](https://github.com/ME495-Navigation/assignment-YaelBenShalom/tree/master/rigid2d) - A package that handls transformations in SE(2).
- [**trect**](https://github.com/ME495-Navigation/assignment-YaelBenShalom/tree/master/trect) - A package that causes the turtlesim turtle to follow a rectanglep path.
- [**nuturtle_robot**](https://github.com/ME495-Navigation/assignment-YaelBenShalom/tree/master/nuturtle_robot) - A package that stores the code that interacts with the turtlebot hardware.
- [**nurtlesim**](https://github.com/ME495-Navigation/assignment-YaelBenShalom/tree/master/nurtlesim) - A package that simulates the robot kinematics and a sensor that detects the relative x, y positions of landmarks and a landmark id.
- [**nuslam**](https://github.com/ME495-Navigation/assignment-YaelBenShalom/tree/master/nuslam) - A package contains the implemention Feature-Based Kalman Filter SLAM.

This repository uses exteral package:
- **nuturtlebot** - Additional package to help us work with the lower-level hardware on the turtlebot. In order to run this project, please download the `turtle.repos` file.