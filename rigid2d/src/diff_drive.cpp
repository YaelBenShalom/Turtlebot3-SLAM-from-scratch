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
        WheelVelocity DiffDrive::twist2Wheels(const Twist2D &twist) {
            WheelVelocity vel;
            vel.right_wheel_vel = (-wheel_base * twist.thetadot + twist.xdot)/
                                    wheel_radius;
            vel.left_wheel_vel = (wheel_base * twist.thetadot + twist.xdot)/
                                  wheel_radius;
            return vel;
        }

        /// Convert a desired wheel velocities to the equivalent twist
        /// required to achieve those wheel velocities
        Twist2D DiffDrive::wheels2Twist(WheelVelocity vel) {
            Twist2D twist;
            twist.thetadot = wheel_radius * (vel.right_wheel_vel - vel.left_wheel_vel) / wheel_base;
            twist.xdot = wheel_radius * (vel.right_wheel_vel + vel.left_wheel_vel) / 2;
            twist.ydot = 0;

            return twist;
        }

        /// Convert a wheels angles to the equivalent wheel velocities
        WheelVelocity DiffDrive::wheelAngle2WheelVel(double right_angle, double left_angle) {
            WheelVelocity vel;
            int del_t = 1;

            vel.right_wheel_vel = normalize_angle(right_angle - right_wheel_angle)/del_t;
            vel.left_wheel_vel = normalize_angle(left_angle - left_wheel_angle)/del_t;

            right_wheel_angle = normalize_angle(right_angle);
            left_wheel_angle = normalize_angle(left_angle);

            return vel;
        }

        /// Updates the odometry
        WheelVelocity DiffDrive::updateOdometry(double right_angle, double left_angle) {
            WheelVelocity vel;
            Twist2D twist;

            vel = DiffDrive::wheelAngle2WheelVel(right_angle, left_angle);
            twist = DiffDrive::wheels2Twist(vel);

            // TODO - Missing position update!!
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