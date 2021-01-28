# ME495 Sensing, Navigation and Machine Learning For Robotics
* Yael Ben Shalom
* Winter 2021


# Turtle Rect
* A package that causes the turtlesim turtle to follow a rectangle path.


# Example Usage
To launch the package, launch the trect.launch as instructed in the following example, and call the `/start` service from a new terminal. Choose x, y (the turtle starting point) and width, height (the width and height of the rectangle) as you wish:
```
roslaunch trect trect.launch
rosservice call /start "x: 2.0
y: 3.0
width: 4.0
height: 5.0"
```
![Demonstration](https://github.com/ME495-Navigation/assignment-YaelBenShalom/blob/master/trect/videos/Task_C.gif)