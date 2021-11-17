# Package: trect

Author: Yael Ben Shalom


## Package Summary

A package that causes the turtlesim turtle to follow a rectangle path.<br>
Please visit [my website](https://yaelbenshalom.github.io/EKF_SLAM/index.html) for more information about this package.

## Nodes, Library and launchfiles

- **turtle_rect node** - A node that causes the turtlesim turtle to follow a rectangle path.

- **trect launchfile** - Launch file for making a turtle in the turtle simulator move in a rectangular trajectory.
  - Launch `roslaunch trect trect.launch` to make a turtle in the turtle simulator move in a rectangular trajectory.

# Example Usage

To launch the package, launch the trect.launch as instructed in the following example, and call the `/start` service from a new terminal. Choose x, y (the turtle starting point) and width, height (the width and height of the rectangle) as you wish:

```
roslaunch trect trect.launch
rosservice call /start "x: 2.0
y: 3.0
width: 4.0
height: 5.0"
```

<p align="center">
  <img align="center" src="https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/blob/master/trect/videos/Task_C.gif">
</p>
