# Package: rigid2d

Author: Yael Ben Shalom


## Package Summary

A 2D Lie Group library used for transformation, vectors and twist operations in 2D for differential drive robots, based on [Modern Robotics](http://hades.mech.northwestern.edu/index.php/Modern_Robotics) by Kevin Lynch.<br>
Please visit [my website](https://yaelbenshalom.github.io/EKF_SLAM/index.html) for more information about this package.

## Nodes and launchfiles

This library contains several nodes and launchfiles:

- **odometer node** - Publishes Odometry messages for differential drive robot based on wheel joint states.
- **fake_turtle node** - A kinematic simulation of a differential drive robot using the DiffDrive class.

- **fake_turtle_odom launchfile** - Launch file for making a turtle move in Rviz using turtlebot3_teleop. Run `roslaunch rigid2d fake_turtle_odom.launch` to launch the turtle in Rviz, and move it using the `w`, `d`, `x`, `a` letters on your keyboard.

<p align="center">
  <img align="center" src="https://github.com/YaelBenShalom/Turtlebot3-SLAM-from-scratch/blob/master/rigid2d/videos/Task_E.gif">
</p>
