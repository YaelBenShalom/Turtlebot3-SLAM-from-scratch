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
    * Launch Run `roslaunch nuturtle_robot odom_teleop.launch` to making a turtle move in rviz in a circulat path.

    ![Demonstration](https://github.com/ME495-Navigation/assignment-YaelBenShalom/blob/master/nuturtle_robot/videos/Task_Fa.gif)


    * Launch Run `roslaunch nuturtle_robot odom_teleop.launch circle:=False` to making a turtle move in rviz using turtlebot3_teleop, and move it using the `w`, `d`, `x`, `a` letters on your keyboard.

    ![Demonstration](https://github.com/ME495-Navigation/assignment-YaelBenShalom/blob/master/nuturtle_robot/videos/Task_Fb.gif)

