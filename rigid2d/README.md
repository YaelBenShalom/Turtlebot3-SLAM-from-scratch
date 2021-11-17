# Package: rigid2d

Author: Yael Ben Shalom


## Package Summary

A library for handling transformations in SE(2).

## Nodes and launchfiles

This library contains several nodes and launchfiles:

- **odometer node** - Publishes Odometry messages for differential drive robot based on wheel joint states.
- **fake_turtle node** - A kinematic simulation of a differential drive robot using the DiffDrive class.

- **fake_turtle_odom launchfile** - Launch file for making a turtle move in Rviz using turtlebot3_teleop. Run `roslaunch rigid2d fake_turtle_odom.launch` to launch the turtle in Rviz, and move it using the `w`, `d`, `x`, `a` letters on your keyboard.

  ![Demonstration](https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/blob/master/rigid2d/videos/Task_E.gif)
