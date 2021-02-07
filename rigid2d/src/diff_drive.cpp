#include "../include/rigid2d/diff_drive.hpp"
#include "../include/rigid2d/rigid2d.hpp"

#include <cmath>
#include <iostream>


using namespace rigid2d;

/********** Config2D struct member functions **********/

        /// Constructor for zero configuration
        Config2D::Config2D() {
            x = 0;
            y = 0;
            theta = 0;
        }

        /// Constructor for non-zero configuration
        Config2D::Config2D(double x_, double y_, double theta_) {
            x = x_;
            y = y_;
            theta = theta_;
        }


/********** WheelVelocity struct member functions **********/

        /// Constructor for zero configuration
        WheelVelocity::WheelVelocity() {
            right_wheel_vel = 0;
            left_wheel_vel = 0;
        }

        /// Constructor for non-zero configuration
        WheelVelocity::WheelVelocity(double right_wheel_vel_, double left_wheel_vel_) {
            right_wheel_vel = right_wheel_vel_;
            left_wheel_vel = left_wheel_vel_;
        }



/********** DiffDrive class member functions **********/

        /// Defines the robot configuration as (0,0,0)
        DiffDrive::DiffDrive() {
            Config2D config();
        }

        /// Defines the robot configuration
        DiffDrive::DiffDrive(const Config2D &configuration) {
            config = configuration;
        }

        /// Convert a desired twist to the equivalent wheel velocities
        /// required to achieve that twist
        WheelVelocity DiffDrive::twist2wheels(const Twist2D &twist) {
            WheelVelocity vel;
            vel.right_wheel_vel = (-wheel_base * twist.thetadot + twist.xdot)
                                   /wheel_radius;
            vel.left_wheel_vel = (wheel_base * twist.thetadot + twist.xdot)
                                   /wheel_radius;
            return vel;
        }

        /// Get the current configuration of the robot
        // Config2D DiffDrive::config() {
        //     return config;
        // }

        // /// Update the configuration of the robot, given updated wheel
        // /// angles (assuming constant wheel velocity in-between updates)
        // Config2D DiffDrive::update_config(wheel_angle) {

        // }