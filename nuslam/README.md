# ME495 Sensing, Navigation and Machine Learning For Robotics
* Yael Ben Shalom
* Winter 2021


# Nuslam Package
* This package contains the simulation visualization and the SLAM implemention in a controlled environment.<br>


## Nodes and launchfiles
This library contains several nodes and launchfiles:
- visualization node - The simulation visualization.

- slam launchfile - Launch file for running the visualization for the turtlebot.
    * Launch `roslaunch nuslam slam.launch robot:=localhost` to run the robot in the simulator. Slam will use relative x, y positions and landmark ids as sensor information.
    * Launch `roslaunch nuslam slam.launch robot:=fasturtle.local` to run the code on the turtlebot.