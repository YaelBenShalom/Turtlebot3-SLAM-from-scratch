# ME495 Sensing, Navigation and Machine Learning For Robotics
* Yael Ben Shalom
* Winter 2021


# Rigid 2D transformation Library
A library for handling transformations in SE(2).
* Run `g++ src/rigid2d.cpp -I include/rigid2d -o rigid2d` to compile rigid2d.cpp (include rigid2d.hpp).
* Run `g++ -Wall -Wextra -g -std=c++17 -o rigid2d src/main.cpp src/rigid2d.cpp -I include/rigid2d` to compile main.cpp (include rigid2d.cpp, rigid2d.hpp), and then run `./rigid2d` to run it.
* Run `g++ tests/tests.cpp -o test src/rigid2d.cpp -I include/rigid2d` to compile test.cpp (include rigid2d.cpp, rigid2d.hpp, catch.hpp), and then run `./test` to run it.


# Conceptual Questions
1. What is the difference between a class and a struct in C++?

   The difference between a class and struct in C++ is the default accessibility of member variables and methods. In a struct they are public, and in a class they are private.

2. Why is Vector2D a struct and Transform2D Class (refer to at least 2 specific C++ core guidelines in your answer)?

   In a struct, the members can vary independently, and they are public. For example, the variables x and y in the Vector2D are independent and public, so we can define and refer to them from outshde the struct. On the other hand, the members in Transform2D are private, so we can change them only when specificly refering to the class.
   Another reason is that in struct we can only define members variables - x and y, and in class we can also define logic operations and functions.

3. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?

   Some constructors in Transform2D are explicit To avoid unintended conversions. In our case, it converts to Transform2D explicitly.

4. We need to be able to normalize Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
   - Which of the methods would you implement and why?

   One implementation of the normalize is to implement the function inside the Vectore2D struct (Vector2D::normalize()). Second method is to define another struct for normalized vectors, and define the function normalize as part of this struct (NormalVector2D::normalize()). Third method is to define the function outside of any class or struct.
   I would implement the function normalize as part of the Vector2D struct (Vector2D::normalize()), because it is only used on Vector2D type variables, and as part of a struct it is a public function and can be used putside of the struct.

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not (Refer to C++ core guidelines in your answer)?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer.

   By default, it is better to make member functions const, unless it changes the object’s observable state. This gives a more precise statement of design intent, better readability, more errors caught by the compiler, and sometimes more optimization opportunities. In the Transform2D::inv() function, we do not change object’s observable state and we return a new internal variable, therefore it is const. On the other hand, in the Transform2D::operator*=() function, we do change the object’s observable state and return the new state, and therefore it is non-const.