/// \file diff_drive.hpp
/// \brief Library for the kinematics of a differential drive robot with a given
///     wheel base and wheel radius.

#ifndef DIFFDRIVE_INCLUDE_GUARD_HPP
#define DIFFDRIVE_INCLUDE_GUARD_HPP

#include "rigid2d/rigid2d.hpp"

#include <cmath>
#include <iosfwd>

namespace rigid2d {
/// \brief The configuration of the robot
struct Config2D {
  /// \param x - the x configuration of the robot
  double x;
  /// \param y - the y configuration of the robot
  double y;
  /// \param theta - the theta configuration of the robot
  double theta;

  /// \brief constructor for zero configuration
  Config2D();

  /// \brief constructor for non-zero configuration
  /// \param x_ - the x configuration of the robot
  /// \param y_ - the y configuration of the robot
  /// \param theta_ - the theta configuration of the robot
  Config2D(double x_, double y_, double theta_);
};

/// \brief The velocity of the robot's wheels
struct WheelVelocity {
  /// \param right_wheel_vel - the velocity of the right wheel
  double right_wheel_vel;
  /// \param left_wheel_vel - the velocity of the left wheel
  double left_wheel_vel;

  /// \brief constructor for zero wheel velocity
  WheelVelocity();

  /// \brief constructor for non-zero wheel velocity
  /// \param right_wheel_vel_ - the velocity of the right wheel
  /// \param left_wheel_vel_ - the velocity of the left wheel
  WheelVelocity(double right_wheel_vel_, double left_wheel_vel_);
};

/// \brief The angle of the robot's wheels
struct WheelAngle {
  /// \param right_wheel_angle - the angle of the right wheel
  double right_wheel_angle;
  /// \param left_wheel_angle - the angle of the left wheel
  double left_wheel_angle;

  /// \brief constructor for zero wheel angle
  WheelAngle();

  /// \brief constructor for non-zero wheel angle
  /// \param right_wheel_angle_ - the angle of the right wheel
  /// \param left_wheel_angle_ - the angle of the left wheel
  WheelAngle(double right_wheel_angle_, double left_wheel_angle_);
};

/// \brief The class models the kinematics of a differential drive robot with a
/// given wheel base and wheel radius. The class:
/// 1. Track the configuration  of a differential-drive robot
/// 2. Convert a desired twist to the equivalent wheel velocities required to
/// achieve that twist
/// 3. Update the configuration of the robot, given updated wheel angles
/// (assuming constant wheel velocity in-between updates)
class DiffDrive {
private:
  /// \param wheel_base - the distance between the wheels
  double wheel_base;
  /// \param wheel_radius - the radius of the wheels
  double wheel_radius;

  /// \param config - the current configuration of the robot
  Config2D config;
  /// \param wheel_vel - the wheel velocities of the robot
  WheelVelocity wheel_vel;
  /// \param wheel_angle - the wheel angle of the robot
  WheelAngle wheel_angle;

public:
  /// \brief defines the robot configuration as (0,0,0)
  DiffDrive();

  /// \brief defines the robot configuration as (0,0,0)
  /// \param wheel_base_ - the distance between the wheels
  /// \param wheel_radius_ - the radius of the wheels
  DiffDrive(double wheel_base_, double wheel_radius_);

  /// \brief defines the robot configuration
  /// \param config_ - the current configuration of the robot
  /// \param wheel_base_ - the distance between the wheels
  /// \param wheel_radius_ - the radius of the wheels
  DiffDrive(const Config2D &config_, double wheel_base_, double wheel_radius_);

  /// \brief returns the configuration of the robot
  /// \return the configuration of the robot
  Config2D get_config();

  /// \brief reset the configuration of the robot
  /// \param config_ - the current configuration of the robot
  /// \return void
  void set_config(const Config2D &config_);

  /// \brief returns the transform of the robot
  /// \return the transform of the robot
  Transform2D get_transform();

  /// \brief returns the wheel angle of the robot
  /// \return the wheel angle of the robot
  WheelAngle get_wheel_angle();

  /// \brief returns the wheel velocity of the robot
  /// \return the wheel velocity of the robot
  WheelVelocity get_wheel_vel();

  /// \brief convert a desired twist to the equivalent wheel velocities
  /// required to achieve that twist
  /// \param twist - the robot's twist
  /// \return the wheel velocities of the robot
  WheelVelocity twist2Wheels(const Twist2D &twist);

  /// \brief convert a desired wheel velocities to the equivalent twist
  /// required to achieve those wheel velocities
  /// \param vel - the robot's wheel velocity
  /// \return the twist of the robot
  Twist2D wheels2Twist(const WheelVelocity &vel);

  /// \brief convert a wheels angles to the equivalent wheel velocities
  /// \param right_wheel_angle_ - the right wheel angle (phi1)
  /// \param left_wheel_angle_ - the left wheel angle (phi2)
  /// \return the wheel velocities of the robot
  WheelVelocity wheelAngle2WheelVel(double right_wheel_angle_,
                                    double left_wheel_angle_);

  /// \brief convert wheel velocities to the equivalent wheel angles
  /// \param vel - the wheel velocities of the robot
  /// \return the wheel angles of the robot
  WheelAngle wheelVel2WheelAngle(const WheelVelocity &vel);

  /// \brief updates the odometry with wheel angles
  /// \param right_wheel_angle_ - the right wheel angle (phi1)
  /// \param left_wheel_angle_ - the left wheel angle (phi2)
  /// \return the wheel velocities of the robot
  WheelVelocity updateOdometryWithAngles(double right_wheel_angle_,
                                         double left_wheel_angle_);

  /// \brief updates the odometry with twist input
  /// \param tw - the twist of the robot
  /// \return the wheel angles of the robot
  WheelAngle updateOdometryWithTwist(const Twist2D &tw);

  /// \brief rotating the wheels with twist input (without updating the
  /// odometry)
  /// \param tw - the twist of the robot \return the wheel angles of
  /// the robot
  WheelAngle rotatingWheelsWithTwist(const Twist2D &tw);
};
} // namespace rigid2d

#endif
