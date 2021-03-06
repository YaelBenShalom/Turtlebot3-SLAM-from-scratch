# Turtlebot3 SLAM from Scratch

Author: Yael Ben Shalom


## Table of Contents

- [Description](#description)
- [Package List](#package-list)
- [Getting Started](#getting-started)

## Project Description

This repository contains my implementation of feature-based EKF SLAM with unsupervised learning. I implemented all the packages from Scratch using ROS in C++.<br><br>
Please visit [my website](https://yaelbenshalom.github.io/EKF_SLAM/index.html) for more information about this project.<br>
For further information about the packages, classes, and methods used in this project, please visit [package documentation](https://yaelbenshalom.github.io/Turtlebot3-SLAM-from-scratch/html/index.html).


## Package List

This repository consists of several ROS packages:

- [**nuturtle_description**](https://github.com/YaelBenShalom/Turtlebot3-SLAM-from-scratch/tree/master/nuturtle_description) - A package that adapts the turtlebot3_burger, a differential drive robot, for our needs.
- [**rigid2d**](https://github.com/YaelBenShalom/Turtlebot3-SLAM-from-scratch/tree/master/rigid2d) - A package that handles transformations in SE(2).
- [**trect**](https://github.com/YaelBenShalom/Turtlebot3-SLAM-from-scratch/tree/master/trect) - A package that causes the turtlesim turtle to follow a rectangle path.
- [**nuturtle_robot**](https://github.com/YaelBenShalom/Turtlebot3-SLAM-from-scratch/tree/master/nuturtle_robot) - A package that stores the code that interacts with the turtlebot hardware.
- [**nurtlesim**](https://github.com/YaelBenShalom/Turtlebot3-SLAM-from-scratch/tree/master/nurtlesim) - A package that simulates the robot kinematics and a sensor that detects the relative x, y positions of landmarks and a landmark id.
- [**nuslam**](https://github.com/YaelBenShalom/Turtlebot3-SLAM-from-scratch/tree/master/nuslam) - A package contains the implementation Feature-Based Kalman Filter SLAM.

This repository uses external package:

- **nuturtlebot** - Additional package to help us work with the lower-level hardware on the turtlebot. In order to run this project, please download the `turtle.rosinstall` file.

## Getting Started

1. Create a workspace and clone the repo:
    ```
    mkdir -p ws/src && cd ws/src
    git clone https://github.com/YaelBenShalom/Turtlebot3-SLAM-from-scratch.git
    ```

2. Download and install the `turtle.rosinstall` file to get the necessary packages:
    ```
    cd ..
    vcs import src <PATH_TO_ROSINSTALL_FILE>.rosinstall
    ```
    Alternativly, clone [this repository](https://github.com/ME495-Navigation/nuturtlebot) into the source space `ws/src`.

3. Build and source the workspace:
    ```
    cd ..
    catkin_make
    source devel/setup.bash 
    ```
