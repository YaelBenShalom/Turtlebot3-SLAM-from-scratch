# Package: nuturtle_robot

Author: Yael Ben Shalom


## Package Summary

This package stores the code that interacts with the turtlebot hardware.

- Launch `roslaunch nuturtle_robot basic_remote.launch --ros-args` to view any optional arguments and their instructions.
- Launch `roslaunch nuturtle_robot basic_remote.launch robot:=localhost` to run the node on the local machine.

Please visit [my website](https://yaelbenshalom.github.io/EKF_SLAM/index.html) for more information about this package.

## Nodes and launchfiles

This library contains several nodes and launchfiles:

- **turtle_interface node** - A low-level control and sensor routines in ROS.
- **follow_circle node** - A node that publishes commands that let the robot drive in a circle of a specified radius at a specified speed.

  - This node contains a service Control, that causes the robot to travel either clockwise (1), counter clockwise (-1), or stop (0) - `rosservice call /control`.

- **basic_remote launchfile** - Launch file for interaction with the turtlebot hardware.

  - Launch `roslaunch nuturtle_robot basic_remote.launch --ros-args` to view any optional arguments and their instructions.
  - Launch `roslaunch nuturtle_robot basic_remote.launch robot:=localhost` to run the node on the local machine.

- **odom_teleop launchfile** - Launch file for making a turtle move in rviz in a circular path or using turtlebot3_teleop.
  In order to make the real robot move, follow the `Real Turtle Instructions` section before launching.

  - Launch `roslaunch nuturtle_robot odom_teleop.launch` to making a turtle move in a circulat path.

    <p align="center">
      <img align="center" src="(https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/blob/master/nuturtle_robot/videos/Task_Fa.gif">
    </p>

  - Launch `roslaunch nuturtle_robot odom_teleop.launch circle:=False` to making a turtle move using turtlebot3_teleop, and move it using the `w`, `d`, `x`, `a` letters on your keyboard.

    <p align="center">
      <img align="center" src="https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/blob/master/nuturtle_robot/videos/Task_Fb.gif">
    </p>

## Real Turtle Instructions:

To activate the real turtle, please follow the following steps:

1. SSH into the turtlebot - `ssh ubuntu@<turtle-name>.local`.
2. Launch the turtlebot3_bringup launchfile - `roslaunch turtlebot3_bringup turtlebot3_robot.launch`.

## Physical Testing

### Experiment I:

Driving the robot forward and backward in a straight line several times, and then stopping when the turtlebot is in its initial configuration.

<p align="center">
  <img align="center" src="https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/blob/master/nuturtle_robot/videos/Task_F8-3a.gif">
</p>

<p align="center">
  <img align="center" src="https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/blob/master/nuturtle_robot/videos/Task_F8-3b.gif">
</p>

### Experiment II:

Rotating the robot clockwise and counter clockwise several times, stopping when the turtlebot is in its initial configuration.

<p align="center">
  <img align="center" src="https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/blob/master/nuturtle_robot/videos/Task_F8-4a.gif">
</p>

<p align="center">
  <img align="center" src="https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/blob/master/nuturtle_robot/videos/Task_F8-4b.gif">
</p>

We can see that in reality the robot slips on the floor, so we don't get the same results as in the simulation.

### Experiment III:

Driving the robot in a circle, clockwise and counter clockwise several times, stopping when the turtlebot is its initial configuration.

<p align="center">
  <img align="center" src="https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/blob/master/nuturtle_robot/videos/Task_Fa.gif">
</p>

The robot's final configuration was almost equal to the initial configuration, with ~3 cm difference on the y axis.

### Experiment IV:

Try one of the previous experiments again, but try to get either a significantly better or significantly worse result.

I repeated the first experiment, although this one, I didn't reduce the turtlebot's velocity fast enough before the finish point, so the robot crossed the initial position and stopped ~10 cm after it.

<p align="center">
  <img align="center" src="https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/blob/master/nuturtle_robot/videos/Task_F8-6a.gif">
</p>
