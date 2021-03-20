# ME495 Sensing, Navigation and Machine Learning For Robotics
* Yael Ben Shalom
* Winter 2021


# Nuslam Package
* This package contains the implementation for the Feature-Based Kalman Filter SLAM. The package stores the simulation visualization and the SLAM implementation in a controlled environment.<br>


    ![Simulation](https://github.com/ME495-Navigation/assignment-YaelBenShalom/blob/master/nuslam/images/slam2.gif)


## Nodes, Library and launchfiles
This library contains several nodes and launchfiles:
- visualization node - The simulation visualization.
- slam node - Implementation of the extended Kalman Filter SLAM
- landmarks node - A package to detect landmarks and publish their relative locations.


- nuslam library - library for implementation of the extended Kalman Filter SLAM algorithm.


- slam launchfile - Launch file for running the visualization for the turtlebot.
    * Launch `roslaunch nuslam slam.launch robot:=localhost` to run the robot in the simulator. Slam will use relative x, y positions and landmark ids as sensor information.
    * Launch `roslaunch nuslam slam.launch robot:=fasturtle.local` to run the code on the turtlebot.



    ![Simulation](https://github.com/ME495-Navigation/assignment-YaelBenShalom/blob/master/nuslam/images/simulation.png)