# Sensing, Navigation and Machine Learning For Robotics

- ME-495, Winter 2021

# Rigid 2D transformation Library

A library for handling transformations in SE(2).

## Nodes and launchfiles

This library contains several nodes and launchfiles:

- **odometer node** - Publishes Odometry messages for differential drive robot based on wheel joint states.
- **fake_turtle node** - A kinematic simulation of a differential drive robot using the DiffDrive class.

- **fake_turtle_odom launchfile** - Launch file for making a turtle move in Rviz using turtlebot3_teleop. Run `roslaunch rigid2d fake_turtle_odom.launch` to launch the turtle in Rviz, and move it using the `w`, `d`, `x`, `a` letters on your keyboard.

  ![Demonstration](https://github.com/YaelBenShalom/Sensing_Navigation_and_ML/blob/master/rigid2d/videos/Task_E.gif)

# Conceptual Questions

1. What is the difference between a class and a struct in C++?

   The difference between a class and struct in C++ is the default accessibility of member variables and methods. In a struct they are public, and in a class they are private.

2. Why is Vector2D a struct and Transform2D Class (refer to at least 2 specific C++ core guidelines in your answer)?

   In a struct, the members can vary independently, and they are public. For example, the variables x and y in the Vector2D are independent and public, so we can define and refer to them from outside the struct, and we can change them independently of each other. On the other hand, the members in Transform2D are private, so we can't refer or change them from outside the class. in order to refer them, I defined x, y, and theta function to output the variables values.

3. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?

   Some constructors in Transform2D are explicit To avoid unintended conversions. In our case, it converts to Transform2D explicitly.

4. We need to be able to normalize Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):

   - Propose three different designs for implementing the ~normalize~ functionality
   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
   - Which of the methods would you implement and why?

   One implementation of the normalize is to implement the function inside the Vector2D struct (Vector2D::normalize()). Second method is to define another struct for normalized vectors, and define the function normalize as part of this struct (NormalVector2D::normalize()). Third method is to define the function outside of any class or struct.
   I would implement the function normalize as part of the Vector2D struct (Vector2D::normalize()), because it is only used on Vector2D type variables, and as part of a struct it is a public function and can be used outside of the struct.

5. Why is Transform2D::inv() declared const while Transform2D::operator\*=() is not (Refer to C++ core guidelines in your answer)?

   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer.

   By default, it is better to make member functions const, unless it changes the object’s observable state. This gives a more precise statement of design intent, better readability, more errors caught by the compiler, and sometimes more optimization opportunities. In the Transform2D::inv() function, we do not change object’s observable state and we return a new internal variable, therefore it is const. On the other hand, in the Transform2D::operator\*=() function, we do change the object’s observable state and return the new state, and therefore it is non-const.
