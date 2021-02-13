#ifndef DIFFDRIVE_INCLUDE_GUARD_HPP
#define DIFFDRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for the kinematics of a differential drive robot with a given wheel base and wheel radius.

#include "rigid2d/rigid2d.hpp"

#include <iosfwd>
#include <cmath>


namespace rigid2d
{

    struct Config2D
    {
        double x;
        double y;
        double theta;

        /// \brief constructor for zero configuration
        Config2D();

        /// \brief constructor for non-zero configuration
        /// \param x - the x configuration of the robot
        /// \param y - the y configuration of the robot
        /// \param theta - the theta configuration of the robot
        Config2D(double x_, double y_, double theta_);
    };

    struct WheelVelocity
    {
        double right_wheel_vel;
        double left_wheel_vel;

        /// \brief constructor for zero wheel velocity
        WheelVelocity();

        /// \brief constructor for non-zero wheel velocity
        /// \param right_wheel_vel - the velocity of the right wheel
        /// \param left_wheel_vel - the velocity of the left wheel
        WheelVelocity(double right_wheel_vel_, double left_wheel_vel_);
    };

    struct WheelAngle
    {
        double right_wheel_angle;
        double left_wheel_angle;

        /// \brief constructor for zero wheel angle
        WheelAngle();

        /// \brief constructor for non-zero wheel angle
        /// \param right_wheel_angle - the angle of the right wheel
        /// \param left_wheel_angle - the angle of the left wheel
        WheelAngle(double right_wheel_angle_, double left_wheel_angle_);
    };

    /// \brief The class models the kinematics of a differential drive robot with a given wheel base and wheel radius. The class:
    /// 1. Track the configuration  of a differential-drive robot
    /// 2. Convert a desired twist to the equivalent wheel velocities required to achieve that twist
    /// 3. Update the configuration of the robot, given updated wheel angles (assuming constant wheel velocity in-between updates)
    class DiffDrive
    {
    private:
        double wheel_base, wheel_radius;
        Config2D config;
        WheelVelocity wheel_vel;
        WheelAngle wheel_angle;

    public:

        /// \brief defines the robot configuration as (0,0,0)
        DiffDrive();

        /// \brief defines the robot configuration
        /// \param config - the current configuration of the robot
        /// \param wheel_base_ - the distance between the wheels
        /// \param wheel_radius_ - the radius of the wheels
        DiffDrive(const Config2D &configuration, double wheel_base_, double wheel_radius_);

        /// \brief returns the configuration of the robot
        /// \return the configuration of the robot
        Config2D get_config();

        /// \brief reset the configuration of the robot
        /// \return void
        void set_config(const Config2D &pos);

        /// \brief returns the wheel angle of the robot
        /// \return the wheel angle of the robot
        WheelAngle get_wheel_angle();
        
        /// \brief convert a desired twist to the equivalent wheel velocities
        /// required to achieve that twist
        /// \param twist - the robot's twist
        /// \return the wheel velocities of the robot
        WheelVelocity twist2Wheels(const Twist2D &twist);

        /// \brief convert a desired wheel velocities to the equivalent twist
        /// required to achieve those wheel velocities
        /// \param twist - the robot's twist
        /// \return the twist of the robot
        Twist2D wheels2Twist(const WheelVelocity &vel);

        /// \brief convert a wheels angles to the equivalent wheel velocities
        /// \param right_wheel_angle - the right wheel angle (phi1)
        /// \param left_wheel_angle - the left wheel angle (phi2)
        /// \return the wheel velocities of the robot
        WheelVelocity wheelAngle2WheelVel(double right_wheel_angle_, double left_wheel_angle_);

        /// \brief convert wheel velocities to the equivalent wheel angles
        /// \param vel - the wheel velocities of the robot
        /// \return the wheel angles of the robot
        WheelAngle wheelVel2WheelAngle(const WheelVelocity &vel);

        /// \brief updates the odometry with wheel angles
        /// \param right_wheel_angle - the right wheel angle (phi1)
        /// \param left_wheel_angle - the left wheel angle (phi2)
        /// \return the wheel velocities of the robot
        WheelVelocity updateOdometryWithAngles(double right_wheel_angle_, double left_wheel_angle_);

        /// \brief updates the odometry with twist input
        /// \param tw - the twist of the robot
        /// \return the wheel angles of the robot
        WheelAngle updateOdometryWithTwist(const Twist2D &tw);
    };


}

#endif