# Sensing, Navigation and Machine Learning For Robotics
* ME-495, Winter 2021


# Package List
This repository consists of several ROS packages:
- [**nuturtle_description**](https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/tree/master/nuturtle_description) - A package that adapts the turtlebot3_burger, a differential drive robot, for our needs.
- [**rigid2d**](https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/tree/master/rigid2d) - A package that handles transformations in SE(2).
- [**trect**](https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/tree/master/trect) - A package that causes the turtlesim turtle to follow a rectangle path.
- [**nuturtle_robot**](https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/tree/master/nuturtle_robot) - A package that stores the code that interacts with the turtlebot hardware.
- [**nurtlesim**](https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/tree/master/nurtlesim) - A package that simulates the robot kinematics and a sensor that detects the relative x, y positions of landmarks and a landmark id.
- [**nuslam**](https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/tree/master/nuslam) - A package contains the implementation Feature-Based Kalman Filter SLAM.

This repository uses external package:
- **nuturtlebot** - Additional package to help us work with the lower-level hardware on the turtlebot. In order to run this project, please download the `turtle.repos` file.

![Implementation of Feature-Based Kalman Filter SLAM](https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/blob/master/nuslam/images/slam2.gif)