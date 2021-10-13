#ifndef NUSLAM_INCLUDE_GUARD_HPP
#define NUSLAM_INCLUDE_GUARD_HPP
/// \file
/// \brief Library for Extended Kalman Filter Slam implementation.

#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"

#include <armadillo>
#include <cmath>
#include <iosfwd>
#include <map>
#include <utility>
#include <vector>

namespace nuslam {
using rigid2d::Config2D;
using rigid2d::Twist2D;

/// \brief A Mesurement vector Vector
struct Measurement {
  /// \param r - the distance to landmark
  double r;
  /// \param phi - the relative bearing of landmark
  double phi;
  /// \param id - the measured landmark id
  int id;

  /// \brief create zero-measurement
  Measurement();

  /// \brief create a measurement with r,phi inputs
  /// \param x_ - x input of the measurement
  /// \param y_ - y input of the measurement
  /// \param id_ - the measured landmark id
  explicit Measurement(double x_, double y_, int id_);

  /// \brief create a measurement vector
  /// \return the measurement vector
  arma::mat compute_z();
};

/// \brief initialize the guess of the robot's state and covariance matrix.
/// Start with a guess for the robot state (0, 0, 0) and zero covariance matrix.
class EKF {
  private:
    arma::mat q_t = arma::mat(3, 1);
    arma::mat m_t;

    // arma::mat xi = arma::mat(3, 1);
    // arma::mat cov = arma::mat(3, 3);

    arma::mat Q_mat = arma::mat(3, 3);
    arma::mat R_mat = arma::mat(2, 2);

    arma::mat xi_predict = arma::mat(3, 1);
    arma::mat cov_predict = arma::mat(3, 3);

    std::map<int, int> id2landmark;

  public:
    /// \brief initialize the combined state vector.
    /// Start with a guess for the robot state (0, 0, 0) and zero e map state.
    EKF();

    /// \brief updates the landmarks matrix and landmark covariance.
    /// \param meas - the measured landmark.
    void add_new_measurement(const Measurement &meas);

    /// \brief check if the measured landmark exists in the landmark dictionary.
    /// \param landmark_id - the id of the measured landmark.
    /// \return bool - is the measured exists?
    bool check_landmarks(const int landmark_id);

    /// \brief updates the landmarks matrix and landmark covariance.
    /// \param meas - the measured landmark.
    void update_landmark(const Measurement &meas);

    /// \brief gets the state q_t.
    /// \param twist - the twist of the robot.
    /// \return the updated state q_t
    arma::mat get_new_state(const Twist2D &twist);

    /// \brief gets derivative of g with respect to the state ξ.
    /// \param twist - the twist of the robot.
    arma::mat get_transition(const Twist2D &twist);

    /// \brief predicts the next step - finds the estimated state and covariance.
    /// \param twist - the twist of the robot.
    void predict(const Twist2D &twist);

    /// \brief compute the measurement h for range and bearing to landmark.
    /// \param index - the index of the measured landmark in the landmark matrix.
    /// \return the h matrix.
    arma::mat get_h(int index);

    /// \brief compute the derivative of h with respect to the state.
    /// \param index - the index of the measured landmark in the landmark matrix.
    /// \return the H matrix.
    arma::mat get_H(int index);

    /// \brief updates the next step.
    /// \param meas - the measured landmark.
    void update(std::vector<Measurement> meas);

    /// \brief add a new zero row and column to a matrix (used when adding new
    /// landmark). \param mat - the current matrix. \return the updated matrix.
    arma::mat update_matrix_size(arma::mat mat);

    /// \brief run the Extended Kalman Filter algorithm.
    /// \param twist - the twist of the robot.
    /// \param meas -  the current measurement.
    void run_ekf(const Twist2D &twist, const std::vector<Measurement> &meas);

    /// \brief output the robot state
    /// \return the robot state
    arma::mat output_state();

    /// \brief output the map state
    /// \return the map state
    arma::mat output_map_state();
  };
} // namespace nuslam

#endif