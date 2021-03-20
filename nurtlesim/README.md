# ME495 Sensing, Navigation and Machine Learning For Robotics
* Yael Ben Shalom
* Winter 2021


# Nurtlesim Package
* This package stores the simulator for the turtlebot and SLAM.<br>
The simulator simulates the robot kinematics and a sensor that detects the relative x, y positions of landmarks and a landmark id.


## Nodes and launchfiles
This library contains several nodes and launchfiles:
- *tube_world node* - A kinematic simulation of a differential drive robot using the DiffDrive class. The node also tracks the ground truth location of the robot.

- *tube_world launchfile* - Launch file for running simulator for the turtlebot.
    * Launch `roslaunch nurtlesim tube_world.launch` to run the simulator.



    ![Simulation](https://github.com/ME495-Navigation/assignment-YaelBenShalom/blob/master/nurtlesim/videos/Task_G.gif)