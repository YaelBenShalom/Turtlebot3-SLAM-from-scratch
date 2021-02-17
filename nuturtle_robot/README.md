# ME495 Sensing, Navigation and Machine Learning For Robotics
* Yael Ben Shalom
* Winter 2021


# Nuturtle Robot Package
This packaage stores the code that interacts with the turtlebot hardware.
* Launch Run `roslaunch nuturtle_robot basic_remote.launch --ros-args` to view any optional arguments and their instructions.
* Launch `roslaunch nuturtle_robot basic_remote.launch robot:=localhost` to run the node on the local machine.


## Nodes and launchfiles
This library contains several nodes and launchfiles:
- turtle_interface node - A low-level control and sensor routines in ROS.
- follow_circle node - A node  that publishes commands that let the robot drive in a circle of a specified radius at a specified speed.
    * This node contains a service Control, that causes the robot to travel either clockwise (1), counter clockwise (-1), or stop (0) - `rosservice call /control`.

- basic_remote launchfile - Launch file for interaction with the turtlebot hardware.
    * Launch Run `roslaunch nuturtle_robot basic_remote.launch --ros-args` to view any optional arguments and their instructions.
    * Launch `roslaunch nuturtle_robot basic_remote.launch robot:=localhost` to run the node on the local machine.

- odom_teleop launchfile - Launch file for making a turtle move in rviz in a circular path or using turtlebot3_teleop.
    In order to make the real robot move, follow the `Real Turtle Instructions` section before launching.
    * Launch `roslaunch nuturtle_robot odom_teleop.launch` to making a turtle move in a circulat path.

        ![Demonstration](https://github.com/ME495-Navigation/assignment-YaelBenShalom/blob/master/nuturtle_robot/videos/Task_Fa.gif)


    * Launch `roslaunch nuturtle_robot odom_teleop.launch circle:=False` to making a turtle move using turtlebot3_teleop, and move it using the `w`, `d`, `x`, `a` letters on your keyboard.

        ![Demonstration](https://github.com/ME495-Navigation/assignment-YaelBenShalom/blob/master/nuturtle_robot/videos/Task_Fb.gif)


## Real Turtle Instructions:
To activate the real turtle, please follow the following steps:
1. SSH into the turtlebot - `ssh ubuntu@<turtle-name>.local`.
2. Launch the turtlebot3_bringup launchfile - `roslaunch turtlebot3_bringup turtlebot3_robot.launch`.


## Physical Testing

### Expirament I:
Driving the robot forward and backward in a straight line several times, and then stopping when the turtlebot is in its initial configuration.

![Demonstration](https://github.com/ME495-Navigation/assignment-YaelBenShalom/blob/master/nuturtle_robot/videos/Task_F8-3a.gif)

![Demonstration](https://github.com/ME495-Navigation/assignment-YaelBenShalom/blob/master/nuturtle_robot/videos/Task_F8-3b.gif)


### Expirament II:
Rotating the robot clockwise and counter clockwise several times, stopping when the turtlebot is in its initial configuration.

![Demonstration](https://github.com/ME495-Navigation/assignment-YaelBenShalom/blob/master/nuturtle_robot/videos/Task_F8-4a.gif)

![Demonstration](https://github.com/ME495-Navigation/assignment-YaelBenShalom/blob/master/nuturtle_robot/videos/Task_F8-4b.gif)

We can see that in reality the robot slips on the floor, so we don't get the same results as in the simulation.


### Expirament III:
Driving the robot in a circle, clockwise and counter clockwise several times, stopping when the turtlebot is its initial configuration.

![Demonstration](https://github.com/ME495-Navigation/assignment-YaelBenShalom/blob/master/nuturtle_robot/videos/Task_Fa.gif)

The robot's final configuration was almost equal to the inital configuration, with ~3 cm difference on the y axis.


### Expirament IV:
Try one of the previous experiments again, but try to get either a significantly better or significantly worse result.

I repeated the first expirament, although this one, I didn't reduce the turtlebot's velocity fast enough before the finish point, so the robot crossed the initial position and stopped ~10 cm after it.

![Demonstration](https://github.com/ME495-Navigation/assignment-YaelBenShalom/blob/master/nuturtle_robot/videos/Task_F8-6a.gif)



Note - all the real turtlebot's videos are speed by X2.