#ifndef EKF_INCLUDE_GUARD_HPP
#define EKF_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for Extended Kalman Filter Slam implementation.

#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

#include <iosfwd>
#include <cmath>
#include <vector>
#include <utility>
// #include <eigen3/Eigen/Dense>
#include <armadillo>

namespace nuslam
{
    // /// \brief The configuration of the robot
    // struct Config2D
    // {
    //     /// \param x - the x configuration of the robot
    //     double x;
    //     /// \param y - the y configuration of the robot
    //     double y;
    //     /// \param theta - the theta configuration of the robot
    //     double theta;

    //     /// \brief constructor for zero configuration
    //     Config2D();

    //     /// \brief constructor for non-zero configuration
    //     /// \param x_ - the x configuration of the robot
    //     /// \param y_ - the y configuration of the robot
    //     /// \param theta_ - the theta configuration of the robot
    //     Config2D(double x_, double y_, double theta_);
    // };

    // /// \brief The velocity of the robot's wheels
    // struct WheelVelocity
    // {
    //     /// \param right_wheel_vel - the velocity of the right wheel
    //     double right_wheel_vel;
    //     /// \param left_wheel_vel - the velocity of the left wheel
    //     double left_wheel_vel;

    //     /// \brief constructor for zero wheel velocity
    //     WheelVelocity();

    //     /// \brief constructor for non-zero wheel velocity
    //     /// \param right_wheel_vel_ - the velocity of the right wheel
    //     /// \param left_wheel_vel_ - the velocity of the left wheel
    //     WheelVelocity(double right_wheel_vel_, double left_wheel_vel_);
    // };


    /// \brief The Covariance Matrix
    struct CovarianceMatrix
    {
        // /// \param robot_state_covariance - the covariance of the robot_state
        // std::vector<double>  robot_state_covariance;
        // /// \param map_state_covariance - the covariance of the map_state
        // std::vector<double>  map_state_covariance;
        // /// \param covariance_matrix - the full covariance matrix
        // Eigen::MatrixXd covariance_matrix;

        // /// \brief constructor for initializing the covariance matrix
        // /// Initialzes the covariance of the robot_state to zero and the
        // /// covariance of the map_state to infinity matrix of zeros
        // CovarianceMatrix();

        // /// \brief constructor for initializing the covariance matrix
        // /// Initialzes the covariance of the robot_state to zero and the
        // /// covariance of the map_state to infinity matrix with input values
        // /// \param map_state_covariance - the current state of the map
        // CovarianceMatrix(const std::vector<Point> & map_state_covariance);

        // /// \brief constructor for initializing the covariance matrix
        // /// Initialzes the covariance of the robot_state to the robot_state covariance
        // /// and the covariance of the map_state to infinity matrix with input values
        // /// \param map_state_covariance - the current state of the map
        // CovarianceMatrix(const std::vector<Point> & map_state_covariance, \
        //                  const std::vector<double> & robot_state_covariance);
    };


    /// \brief The class models the kinematics of a differential drive robot with a given wheel base and wheel radius. The class:
    /// 1. Track the configuration  of a differential-drive robot
    /// 2. Convert a desired twist to the equivalent wheel velocities required to achieve that twist
    /// 3. Update the configuration of the robot, given updated wheel angles (assuming constant wheel velocity in-between updates)
    class EKF
    {
    private:
    //                 // /// \param wheel_base - the distance between the wheels
    //                 // double wheel_base;
    //                 // /// \param wheel_radius - the radius of the wheels
    //                 // double wheel_radius;

    //     /// \param robot_state - the current state of the robot
    //     rigid2d::Config2D robot_state;
    //     /// \param map_state - the current state of the map
    //     std::vector<Point> map_state;
    //                 // /// \param wheel_angle - the wheel angle of the robot
    //                 // WheelAngle wheel_angle;

    public:
        // /// \brief initialize the guess of the robot's state and covariance matrix.
        // /// Start with a guess for the robot state (0, 0, 0) and zero covariance matrix.
        // EKF();

        // /// \brief initialize the guess of the robot's state and covariance matrix.
        // /// Start with a guess for the robot state (0, 0, 0) and zero covariance matrix.
        // /// \param robot_state_ - the current state of the robot
        // /// \param map_state_ - the current state of the map
        // /// \param landmark_num - the number of known landmarks
        // /// \param landmark_th - the threshold for updating landmark
        // EKF(const rigid2d::Config2D &robot_state_, const std::vector<Point> & map_state_, \
        //     int landmark_num, double landmark_th);

        // /// \brief gets the satate transition ξ_t (q_t, m_t) and  the derivative of model of the robot’s movement
        // /// and the map’s movement with respect to the state ξ.
        // /// \param twist - the twist of the robot
        // void get_state_transition(const rigid2d::Twist2D &twist);


    //     /// \brief  update the estimate using the model and propagate the uncertainty using the linearized
    //     /// state transition model
    //     /// \param twist - the twist of the robot
    //     void predict(const Twist2D &twist);


        // /// \brief returns the configuration of the robot
        // /// \return the configuration of the robot
        // Config2D get_config();

        // /// \brief reset the configuration of the robot
        // /// \param config_ - the current configuration of the robot
        // /// \return void
        // void set_config(const Config2D &config_);

        // /// \brief returns the wheel angle of the robot
        // /// \return the wheel angle of the robot
        // WheelAngle get_wheel_angle();

        // /// \brief returns the wheel velocity of the robot
        // /// \return the wheel velocity of the robot
        // WheelVelocity get_wheel_vel();

        // /// \brief convert a desired twist to the equivalent wheel velocities
        // /// required to achieve that twist
        // /// \param twist - the robot's twist
        // /// \return the wheel velocities of the robot
        // WheelVelocity twist2Wheels(const Twist2D &twist);

        // /// \brief convert a desired wheel velocities to the equivalent twist
        // /// required to achieve those wheel velocities
        // /// \param vel - the robot's wheel velocity
        // /// \return the twist of the robot
        // Twist2D wheels2Twist(const WheelVelocity &vel);

        // /// \brief convert a wheels angles to the equivalent wheel velocities
        // /// \param right_wheel_angle_ - the right wheel angle (phi1)
        // /// \param left_wheel_angle_ - the left wheel angle (phi2)
        // /// \return the wheel velocities of the robot
        // WheelVelocity wheelAngle2WheelVel(double right_wheel_angle_, double left_wheel_angle_);

        // /// \brief convert wheel velocities to the equivalent wheel angles
        // /// \param vel - the wheel velocities of the robot
        // /// \return the wheel angles of the robot
        // WheelAngle wheelVel2WheelAngle(const WheelVelocity &vel);

        // /// \brief updates the odometry with wheel angles
        // /// \param right_wheel_angle_ - the right wheel angle (phi1)
        // /// \param left_wheel_angle_ - the left wheel angle (phi2)
        // /// \return the wheel velocities of the robot
        // WheelVelocity updateOdometryWithAngles(double right_wheel_angle_, double left_wheel_angle_);

        // /// \brief updates the odometry with twist input
        // /// \param tw - the twist of the robot
        // /// \return the wheel angles of the robot
        // WheelAngle updateOdometryWithTwist(const Twist2D &tw);

        // /// \brief rotating the wheels with twist input (without updating the odometry)
        // /// \param tw - the twist of the robot
        // /// \return the wheel angles of the robot
        // WheelAngle rotatingWheelsWithTwist(const Twist2D &tw);
    };
}

#endif