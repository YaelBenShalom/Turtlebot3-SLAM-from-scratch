# Sensing, Navigation and Machine Learning For Robotics

Author: Yael Ben Shalom


## Table of Contents

- [Description](#description)
- [Package List](#package-list)
- [Getting Started](#getting-started)
- [Results](#results)

# Project Description

This repository contains my implementation of feature-based EKF SLAM with unsupervised learning. I implemented all the packages from Scratch using ROS in C++.<br><br>
Take a look at my [portfolio post](https://yaelbenshalom.github.io/EKF_SLAM/index.html) for more information about the project.

# Package List

This repository consists of several ROS packages:

- [**nuturtle_description**](https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/tree/master/nuturtle_description) - A package that adapts the turtlebot3_burger, a differential drive robot, for our needs.
- [**rigid2d**](https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/tree/master/rigid2d) - A package that handles transformations in SE(2).
- [**trect**](https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/tree/master/trect) - A package that causes the turtlesim turtle to follow a rectangle path.
- [**nuturtle_robot**](https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/tree/master/nuturtle_robot) - A package that stores the code that interacts with the turtlebot hardware.
- [**nurtlesim**](https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/tree/master/nurtlesim) - A package that simulates the robot kinematics and a sensor that detects the relative x, y positions of landmarks and a landmark id.
- [**nuslam**](https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/tree/master/nuslam) - A package contains the implementation Feature-Based Kalman Filter SLAM.

This repository uses external package:

- **nuturtlebot** - Additional package to help us work with the lower-level hardware on the turtlebot. In order to run this project, please download the `turtle.rosinstall` file.

# Getting Started

Create a workspace and clone the repo:
```
mkdir -p ws/src && cd ws/src
git clone https://github.com/YaelBenShalom/Sensing_Navigation_and_ML.git
```

Download and install the `turtle.rosinstall` file to get the necessary packages:
```
cd ..
vcs import src <PATH_TO_ROSINSTALL_FILE>.rosinstall
```

Then build and source the workspace:
```
cd ..
catkin_make
source devel/setup.bash 
```

# Results

A demo of the EKF-SLAM in action:
<p align="center">
  <img align="center" src="https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/blob/master/nuslam/images/EKF_SLAM_demo.gif">
</p>
