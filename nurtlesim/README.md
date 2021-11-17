# Package: nurtlesim

Author: Yael Ben Shalom


## Package Summary

This package stores the simulator for the turtlebot and SLAM. The simulator simulates the robot kinematics and a sensor that detects the relative x, y positions of landmarks and a landmark id.<br>
Please visit [my website](https://yaelbenshalom.github.io/EKF_SLAM/index.html) for more information about this package.

## Nodes and launchfiles

This library contains several nodes and launchfiles:

- _tube_world node_ - A kinematic simulation of a differential drive robot using the DiffDrive class. The node also tracks the ground truth location of the robot.

- _tube_world launchfile_ - Launch file for running simulator for the turtlebot.

  - Launch `roslaunch nurtlesim tube_world.launch` to run the simulator.

<p align="center">
  <img align="center" src="https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/blob/master/nurtlesim/videos/Task_G.gif">
</p>
