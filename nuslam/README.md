# Package: nuslam

Author: Yael Ben Shalom


## Package Summary

This package contains the implementation for the Feature-Based Kalman Filter SLAM. The package stores the simulation visualization and the SLAM implementation in a controlled environment.<br>
Please visit [my website](https://yaelbenshalom.github.io/EKF_SLAM/index.html) for more information about this package.<br>


## Nodes, Library and launchfiles

This library contains several nodes and launchfiles:

- **visualization node** - The simulation visualization.
- **slam node** - Implementation of the extended Kalman Filter SLAM
- **landmarks node** - A package to detect landmarks and publish their relative locations.

- **nuslam library** - library for implementation of the extended Kalman Filter SLAM algorithm.

- **slam launchfile** - Launch file for running the visualization for the turtlebot.

  - Launch `roslaunch nuslam slam.launch robot:=localhost` to run the robot in the simulator. Slam will use relative x, y positions and landmark ids as sensor information.
  - Launch `roslaunch nuslam slam.launch robot:=fasturtle.local` to run the code on the turtlebot.

- **landmark_detect launchfile** - Launch file for the feature detection node.
  The launchfile takes a simulate argument.

  - If the simulate is true, it should launch your simulator and the landmarks node and display everything in rviz.
  - If the simulate is false, it should launch the landmark detection node on the turtlebot and display everything in rviz.

  * Launch `roslaunch nuturtle_robot landmarks.launch` to launch the feature detection node.

<p align="center">
  <img align="center" src="https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/blob/master/nuslam/images/slam2.gif">
</p>

<p align="center">
  <img align="center" src="https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/blob/master/nuslam/images/simulation.png" width="50%">
</p>
