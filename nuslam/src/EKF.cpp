#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"
#include "nuslam/EKF.hpp"

#include <cmath>
#include <iostream>
#include <sstream>
#include <armadillo>
#include <vector>
#include <utility>

// /********** Config2D struct member functions **********/

// /// Constructor for zero configuration
// rigid2d::Config2D::Config2D() {
//     x = 0.0;
//     y = 0.0;
//     theta = 0.0;
// }

// /// Constructor for non-zero configuration
// rigid2d::Config2D::Config2D(double x_, double y_, double theta_) {
//     x = x_;
//     y = y_;
//     theta = theta_;
// }


// /********** WheelVelocity struct member functions **********/

// /// Constructor for zero configuration
// rigid2d::WheelVelocity::WheelVelocity() {
//     right_wheel_vel = 0.0;
//     left_wheel_vel = 0.0;
// }

// /// Constructor for non-zero configuration
// rigid2d::WheelVelocity::WheelVelocity(double right_wheel_vel_, double left_wheel_vel_) {
//     right_wheel_vel = right_wheel_vel_;
//     left_wheel_vel = left_wheel_vel_;
// }


// /********** CovarianceMatrix struct member functions **********/

// /// Constructor for initializing the covariance matrix
// /// Initialzes the covariance of the robot_state to zero and the
// /// covariance of the map_state to infinity matrix of zeros
// CovarianceMatrix() {
//     // std::vector<double> robot_state_covariance{0.0, 0.0, 0.0};
//     robot_state_covariance{0.0, 0.0, 0.0};
//     covariance_matrix = (Eigen::Map<VectorXd>(robot_state_cov.data(), robot_state_cov.size())).asDiagonal();
// }

// /// Constructor for initializing the covariance matrix
// /// Initialzes the covariance of the robot_state to zero and the
// /// covariance of the map_state to infinity matrix with input values
// CovarianceMatrix(const std::vector<Point> & map_state_covariance) {
//     // std::vector<double> robot_state_covariance{0.0, 0.0, 0.0};
//     robot_state_covariance{0.0, 0.0, 0.0};
//     covariance_matrix = (Eigen::Map<VectorXd>(robot_state_cov.data(), robot_state_cov.size())).asDiagonal();

// }

// /// Constructor for initializing the covariance matrix
// /// Initialzes the covariance of the robot_state to the robot_state covariance
// /// and the covariance of the map_state to infinity matrix with input values
// CovarianceMatrix(const std::vector<Point> & map_state_covariance, \
//                     const std::vector<double> & robot_state_covariance);


/********** EKF class member functions **********/






// /// Gets the satate transition ξ_t (q_t, m_t) and  the derivative of model of the robot’s movement
// /// and the map’s movement with respect to the state ξ.
// void get_state_transition(const Twist2D &twist) {
//     // Angle Wrap Robot Theta
//     robot_state.theta = rigid2d::normalize_angle(robot_state.theta);
//     // First, update the estimate using the forward model
//     Eigen::VectorXd noise_vect = getMultivarNoise(proc_noise.Q);

//     Pose2D belief;

//     if (rigid2d::almost_equal(twist.w_z, 0.0))
//     // If dtheta = 0
//     {
//         belief = Pose2D(robot_state.x + (twist.v_x * cos(robot_state.theta)) + noise_vect(1),\
//                         robot_state.y + (twist.v_x * sin(robot_state.theta)) + noise_vect(2),\
//                         robot_state.theta + noise_vect(0));
//     } else {
//     // If dtheta != 0
//         belief = Pose2D(robot_state.x + ((-twist.v_x / twist.w_z) * sin(robot_state.theta) + (twist.v_x / twist.w_z) * sin(robot_state.theta + twist.w_z)) + noise_vect(1),\
//                         robot_state.y + ((twist.v_x / twist.w_z) * cos(robot_state.theta) + (-twist.v_x / twist.w_z) * cos(robot_state.theta + twist.w_z)) + noise_vect(2),\
//                         robot_state.theta + twist.w_z + noise_vect(0));
//     }
//     // Angle Wrap Robot Theta
//     belief.theta = rigid2d::normalize_angle(belief.theta);

//     // Next, we propagate the uncertainty using the linearized state transition model
//     // (3+2n)*(3+2n)
//     Eigen::MatrixXd g = Eigen::MatrixXd::Zero(3 + (2 * map_state.size()), 3 + (2 * map_state.size()));
//     if (rigid2d::almost_equal(twist.w_z, 0.0))
//     // If dtheta = 0
//     {
//         // Now replace non-zero entries
//         // using theta,x,y
//         g(1, 0) = -twist.v_x * sin(robot_state.theta);
//         g(2, 0) = twist.v_x * cos(robot_state.theta);
//     } else {
//     // If dtheta != 0
//         // Now replace non-zero entries
//         // using theta,x,y
//         g(1, 0) = (-twist.v_x / twist.w_z) * cos(robot_state.theta) + (twist.v_x / twist.w_z) * cos(robot_state.theta + twist.w_z);
//         g(2, 0) = (-twist.v_x / twist.w_z) * sin(robot_state.theta) + (twist.v_x / twist.w_z) * sin(robot_state.theta + twist.w_z);
//     }
// }




// /// Defines the robot configuration as (0,0,0)
// rigid2d::DiffDrive::DiffDrive() {
//     Config2D config();
// 	wheel_base = 0.16;
//     wheel_radius = 0.033;
// }

// /// Defines the robot configuration as (0,0,0)
// rigid2d::DiffDrive::DiffDrive(double wheel_base_, double wheel_radius_) {
//     rigid2d::Config2D config();
// 	wheel_base = wheel_base_;
//     wheel_radius = wheel_radius_; 
// }

// /// Defines the robot configuration
// rigid2d::DiffDrive::DiffDrive(const Config2D &config_, double wheel_base_, double wheel_radius_) {
//     config = config_;
// 	wheel_base = wheel_base_;
//     wheel_radius = wheel_radius_;            
// }

// /// Returns the configuration of the robot
// rigid2d::Config2D rigid2d::DiffDrive::get_config() {
//     return config;
// }

// /// ReResetturns the configuration of the robot
// void rigid2d::DiffDrive::set_config(const rigid2d::Config2D &config_) {
//     config = config_;
// }

// /// Returns the wheel angle of the robot
// rigid2d::WheelAngle rigid2d::DiffDrive::get_wheel_angle() {
//     return wheel_angle;
// }

// /// Returns the wheel velocity of the robot
// rigid2d::WheelVelocity rigid2d::DiffDrive::get_wheel_vel() {
//     return wheel_vel;
// }

// /// Convert a desired twist to the equivalent wheel velocities
// /// required to achieve that twist
// rigid2d::WheelVelocity rigid2d::DiffDrive::twist2Wheels(const rigid2d::Twist2D &twist) {
//     WheelVelocity vel;
//     // Calculating velocity
//     vel.right_wheel_vel = (-(wheel_base / 2.0) * twist.thetadot + twist.xdot) / wheel_radius;
//     vel.left_wheel_vel = ((wheel_base / 2.0) * twist.thetadot + twist.xdot) / wheel_radius;
//     // std::cout << "twist2Wheels:" << "wheel_base = " << wheel_base << "\t wheel_radius = " << wheel_radius << "\n\r" << std::endl;
//     // std::cout << "twist2Wheels:" << "twist.xdot = " << twist.xdot << "\t twist.thetadot = " << twist.thetadot << "\n\r" << std::endl;
//     // std::cout << "twist2Wheels:" << "vel.right = " << vel.right_wheel_vel << "\t vel.left = " << vel.left_wheel_vel << "\n\r" << std::endl;

//     return vel;
// }

// /// Convert a desired wheel velocities to the equivalent twist
// /// required to achieve those wheel velocities
// rigid2d::Twist2D rigid2d::DiffDrive::wheels2Twist(const rigid2d::WheelVelocity &vel) {
//     rigid2d::Twist2D twist;

//     // Calculating twist
//     twist.thetadot = wheel_radius * (vel.right_wheel_vel - vel.left_wheel_vel) / wheel_base;
//     twist.xdot = wheel_radius * (vel.right_wheel_vel + vel.left_wheel_vel) / 2.0;
//     twist.ydot = 0.0;

//     return twist;
// }

// /// Convert a wheels angles to the equivalent wheel velocities
// rigid2d::WheelVelocity rigid2d::DiffDrive::wheelAngle2WheelVel(double right_angle, double left_angle) {
//     WheelVelocity vel;
//     int del_t = 1; // All velocity calculation are for one time unit

//     // Calculating wheel velocity from wheel angles (for one time unit)
//     vel.right_wheel_vel = (normalize_angle(right_angle - wheel_angle.right_wheel_angle))/del_t;
//     vel.left_wheel_vel = (normalize_angle(left_angle - wheel_angle.left_wheel_angle))/del_t;
//     // std::cout << "wheelAngle2WheelVel:" << "vel.right_wheel_vel = " << vel.right_wheel_vel << "\t vel.left_wheel_vel = " << vel.left_wheel_vel << "\n\r" << std::endl;

//     // Update wheel angles
//     wheel_angle.right_wheel_angle = normalize_angle(right_angle);
//     wheel_angle.left_wheel_angle = normalize_angle(left_angle);
//     // std::cout << "wheelAngle2WheelVel:" << "wheel_angle.right_wheel_angle = " << wheel_angle.right_wheel_angle << "\t wheel_angle.left_wheel_angle = " << wheel_angle.left_wheel_angle << "\n\r" << std::endl;

//     return vel;
// }

// /// Convert wheel velocities to the equivalent wheel angles
// rigid2d::WheelAngle rigid2d::DiffDrive::wheelVel2WheelAngle(const rigid2d::WheelVelocity &vel) {
//     int del_t = 1; // All velocity calculation are for one time unit

//     // Calculating wheel angles from wheel velocity (for one time unit)
//     // std::cout << "wheelVel2WheelAngle:" << "wheel_angle.right_wheel_angle = " << wheel_angle.right_wheel_angle << "\t wheel_angle.left_wheel_angle = " << wheel_angle.left_wheel_angle << "\n\r" << std::endl;
//     // std::cout << "wheelVel2WheelAngle:" << "vel.right_wheel_vel = " << vel.right_wheel_vel << "\t vel.left_wheel_vel = " << vel.left_wheel_vel << "\n\r" << std::endl;

//     wheel_angle.right_wheel_angle = normalize_angle(wheel_angle.right_wheel_angle + vel.right_wheel_vel*del_t);
//     wheel_angle.left_wheel_angle = normalize_angle(wheel_angle.left_wheel_angle + vel.left_wheel_vel*del_t);
//     // std::cout << "wheelVel2WheelAngle:" << "wheel_angle.right_wheel_angle = " << wheel_angle.right_wheel_angle << "\t wheel_angle.left_wheel_angle = " << wheel_angle.left_wheel_angle << "\n\r" << std::endl;

//     return wheel_angle;
// }

// /// Updates the odometry with wheel angles
// rigid2d::WheelVelocity rigid2d::DiffDrive::updateOdometryWithAngles(double right_angle, double left_angle) {
//     rigid2d::Transform2D T_b, T_bbp, T_wbp;
//     rigid2d::Twist2D twist;
//     rigid2d::Vector2D v;
//     double angle;

//     // std::cout << "ANGLE IN:" << "right_angle = " << right_angle << "\t left_angle = " << left_angle << "\n\r" << std::endl;
//     // std::cout << "ANGLE IN:" << "wheel_angle.right = " << wheel_angle.right_wheel_angle << "\t wheel_angle.left = " << wheel_angle.left_wheel_angle << "\n\r" << std::endl;

//     // Calculating wheel velocity from wheel angles (for one time unit)
//     wheel_vel = rigid2d::DiffDrive::wheelAngle2WheelVel(right_angle, left_angle);
//     // std::cout << "ANGLE IN:" << "wheel_vel.right = " << wheel_vel.right_wheel_vel << "\t wheel_vel.left = " << wheel_vel.left_wheel_vel << "\n\r" << std::endl;

//     // Calculating the robot twist
//     twist = rigid2d::DiffDrive::wheels2Twist(wheel_vel);
//     // std::cout << "ANGLE IN:" << "twist.xdot = " << twist.xdot << "\t twist.ydot = " << twist.ydot << "\n\r" << std::endl;

//     // Computing the transformation matrix Tb
//     v.x = config.x;
//     v.y = config.y;
//     angle = config.theta;
//     T_b = rigid2d::Transform2D(v, angle);
//     // std::cout << "ANGLE IN:" << "v.x = " << v.x << "\t v.y = " << v.y << "\n\r" << std::endl;

//     // Integrating the twist to get Tbb'
//     // Tbb' = exp(Vb)
//     T_bbp = T_b.integrateTwist(twist);
//     T_wbp = T_b * T_bbp;

//     // Update the configuratiT_bbpon
//     config.x = T_wbp.x();
//     config.y = T_wbp.y();
//     config.theta = rigid2d::normalize_angle(T_wbp.theta());  
//     // std::cout << "ANGLE IN:" << "config.x = " << config.x << "\t config.y = " << config.y << "\t config.theta = " << config.theta << "\n\r" << std::endl;

//     return wheel_vel;
// }

// /// Updates the odometry with twist input
// rigid2d::WheelAngle rigid2d::DiffDrive::updateOdometryWithTwist(const rigid2d::Twist2D &tw) {
//     rigid2d::Transform2D T_b, T_bbp, T_wbp;
//     rigid2d::Vector2D v;
//     double angle;

//     // std::cout << "TWIST IN:" << "twist.xdot = " << tw.xdot << "\t twist.ydot = " << tw.ydot << "\n\r" << std::endl;

//     // Computing the transformation matrix Tb
//     v.x = config.x;
//     v.y = config.y;
//     angle = config.theta;
//     T_b = rigid2d::Transform2D(v, angle);
//     // std::cout << "TWIST IN:" << "v.x = " << v.x << "\t v.y = " << v.y << "\n\r" << std::endl;

//     // Integrating the twist to get Tbb'
//     // Tbb' = exp(Vb)
//     T_bbp = T_b.integrateTwist(tw);
//     T_wbp = T_b * T_bbp;

//     // Update the configuration
//     config.x = T_wbp.x();
//     config.y = T_wbp.y();
//     config.theta = rigid2d::normalize_angle(T_wbp.theta()); 
//     // std::cout << "TWIST IN:" << "config.x = " << config.x << "\t config.y = " << config.y << "\n\r" << std::endl;

//     // Calculating wheel velocity from the robot twist
//     wheel_vel = rigid2d::DiffDrive::twist2Wheels(tw);
//     // std::cout << "TWIST IN:" << "wheel_vel.right = " << wheel_vel.right_wheel_vel << "\t wheel_vel.left = " << wheel_vel.left_wheel_vel << "\n\r" << std::endl;

//     // Calculating wheel angles from wheel velocity (for one time unit)
//     wheel_angle = rigid2d::DiffDrive::wheelVel2WheelAngle(wheel_vel);
//     // std::cout << "TWIST IN:" << "wheel_angle.right = " << wheel_angle.right_wheel_angle << "\t wheel_angle.left = " << wheel_angle.left_wheel_angle << "\n\r" << std::endl;

//     return wheel_angle;
// }

// /// Rotating the wheels with twist input (without updating the odometry)
// rigid2d::WheelAngle rigid2d::DiffDrive::rotatingWheelsWithTwist(const rigid2d::Twist2D &tw) {
//     // std::cout << "TWIST IN:" << "twist.xdot = " << tw.xdot << "\t twist.ydot = " << tw.ydot << "\n\r" << std::endl;

//     // Calculating wheel velocity from the robot twist
//     wheel_vel = rigid2d::DiffDrive::twist2Wheels(tw);
//     // std::cout << "TWIST IN:" << "wheel_vel.right = " << wheel_vel.right_wheel_vel << "\t wheel_vel.left = " << wheel_vel.left_wheel_vel << "\n\r" << std::endl;

//     // Calculating wheel angles from wheel velocity (for one time unit)
//     wheel_angle = rigid2d::DiffDrive::wheelVel2WheelAngle(wheel_vel);
//     // std::cout << "TWIST IN:" << "wheel_angle.right = " << wheel_angle.right_wheel_angle << "\t wheel_angle.left = " << wheel_angle.left_wheel_angle << "\n\r" << std::endl;

//     return wheel_angle;
// }