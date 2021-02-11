#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"

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


/********** WheelAngle struct member functions **********/

        /// Constructor for zero angle
        WheelAngle::WheelAngle() {
            right_wheel_angle = 0;
            left_wheel_angle = 0;
        }

        /// Constructor for non-zero angle
        WheelAngle::WheelAngle(double right_wheel_angle_, double left_wheel_angle_) {
            right_wheel_angle = right_wheel_angle_;
            left_wheel_angle = left_wheel_angle_;
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

        /// Returns the configuration of the robot
        Config2D DiffDrive::get_config() {
	        return config;
        }

        /// ReResetturns the configuration of the robot
        void DiffDrive::set_config(const Config2D &pos) {
            config = pos;
        }

        /// Returns the wheel angle of the robot
        WheelAngle DiffDrive::get_wheel_angle() {
            return wheel_angle;
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
        Twist2D DiffDrive::wheels2Twist(const WheelVelocity &vel) {
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

            vel.right_wheel_vel = normalize_angle(right_angle - wheel_angle.right_wheel_angle)/del_t;
            vel.left_wheel_vel = normalize_angle(left_angle - wheel_angle.left_wheel_angle)/del_t;

            wheel_angle.right_wheel_angle = normalize_angle(right_angle);
            wheel_angle.left_wheel_angle = normalize_angle(left_angle);

            return vel;
        }

        /// Updates the odometry
        WheelVelocity DiffDrive::updateOdometry(double right_angle, double left_angle) {
            Transform2D T_b, T_bbp;
            WheelVelocity vel;
            Vector2D v;
            double angle;
            Twist2D twist;

            vel = DiffDrive::wheelAngle2WheelVel(right_angle, left_angle);
            twist = DiffDrive::wheels2Twist(vel);

            v.x = config.x;
            v.y = config.y;
            angle = config.theta;
            T_b = Transform2D(v, angle);
            T_bbp = T_b.integrateTwist(twist);

            config.x = T_bbp.x();
            config.y = T_bbp.y();
            config.theta = normalize_angle(T_bbp.theta());

            return vel;
        }