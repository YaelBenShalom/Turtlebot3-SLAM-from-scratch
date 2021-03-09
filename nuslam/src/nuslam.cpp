#include "rigid2d/diff_drive.hpp"
#include "rigid2d/rigid2d.hpp"
#include "nuslam/nuslam.hpp"

#include <cmath>
#include <iostream>
#include <sstream>
#include <armadillo>
#include <vector>
#include <utility>

namespace nuslam
{
    /********** Measurement struct member functions **********/

    /// Create zero-measurement
    Measurement::Measurement() {
        r = 0.0;
        phi = 0.0;
    }

    /// Create a measurement with r,phi inputs
    Measurement::Measurement(double x_, double y_, int id_) {
        r = std::sqrt(pow(x_, 2) + pow(y_, 2));
        phi = rigid2d::normalize_angle(atan2(y_, x_));
        id = id_;
    }

    /// Create a measurement vector
    arma::mat Measurement::compute_z() {
        arma::mat z = arma::mat(2, 1);
        z(0,0) = r;
        z(1,0) = phi;

        return z;
    }


    /********** EKF class member functions **********/

    /// Initialize the combined state vector.
    /// Start with a guess for the robot state (0, 0, 0) and zero e map state.
    EKF::EKF() {
        q_t.fill(0.0);
        cov.fill(0.0);

        Q_mat.fill(0.0);
        Q_mat(0, 0) = 1e-6;
        Q_mat(1, 1) = 1e-6;
        Q_mat(2, 2) = 1e-6;

        R_mat.fill(0.0);
        R_mat(0, 0) = 1e-6;
        R_mat(1, 1) = 1e-6;

    }

    /// Check if the measured landmark exists in the landmark dictionary.
    bool EKF::check_landmarks(const int landmark_id) {
        if (id2landmark.count(landmark_id) == 1) {
            return false;
        }
        return true;
    }

    /// Update the landmarks matrix and landmark covariance.
    void EKF::add_new_measurment(const Measurement &meas) {
        if (check_landmarks(meas.id)) {
            update_landmark(meas);
        }
    }

    /// Update the landmarks matrix and landmark covariance.
    void EKF::update_landmark(const Measurement &meas) {

        // Update id2landmark dictionary with new mearusement
        id2landmark.insert(std::make_pair(meas.id, m_t.n_rows));
    
        // Update covariance matrix
        cov = update_matrix_size(cov);
        cov(cov.n_rows - 1, cov.n_cols - 1) = 1000.0;  // Infinity
        cov(cov.n_rows - 2, cov.n_cols - 2) = 1000.0;  // Infinity

        // Update process noise for the robot motion model Q (expanding to fill the whole state)
        Q_mat = update_matrix_size(Q_mat);

        // Update map state matrix
        double m_x = q_t(1, 0) + meas.r * cos(meas.phi + q_t(0, 0));
        double m_y = q_t(2, 0) + meas.r * sin(meas.phi + q_t(0, 0));
        
        arma::mat m_addition = arma::mat(2, 1);
        m_addition(0,0) = m_x;
        m_addition(1,0) = m_y;

        m_t = std::move(arma::join_cols(m_t, m_addition));
    }


    /// Get the state transition ξ_t (q_t, m_t) and the map’s movement with respect to the state ξ.
    arma::mat EKF::get_new_state(const Twist2D &twist) {
        arma::mat T_wbp = arma::mat(q_t.n_rows, 1);
        arma::mat w_t = (arma::mat(3, 1)).fill(0.0);

        // If the rotational velocity is zero (twist.thetadot = 0)
        if (rigid2d::almost_equal(twist.thetadot, 0.0, 1.0e-3)) {
            T_wbp(0, 0) = 0.0;
            T_wbp(1, 0) = twist.xdot * cos(q_t(0, 0));
            T_wbp(2, 0) = twist.xdot * sin(q_t(0, 0));
        }

        // If the rotational velocity is not zero (twist.thetadot != 0)
        else {
            double dx_dtheta = twist.xdot/twist.thetadot;
            T_wbp(0, 0) =  twist.thetadot;
            T_wbp(1, 0) = -dx_dtheta * sin(q_t(0, 0)) + dx_dtheta * sin(q_t(0, 0) + twist.thetadot); 
            T_wbp(2, 0) =  dx_dtheta * cos(q_t(0, 0)) - dx_dtheta * cos(q_t(0, 0) + twist.thetadot); 
        }

        // Return the current state
        arma::mat q_t_new = q_t + T_wbp + w_t;
        q_t_new(0, 0) = rigid2d::normalize_angle(q_t_new(0, 0));
        return (std::move(arma::join_cols(q_t_new, m_t)));
    }
    
    /// Get derivative of g with respect to the state ξ.
    arma::mat EKF::get_transition(const Twist2D &twist) {
        arma::mat g = (arma::mat(3 + m_t.n_rows, 3 + m_t.n_rows)).fill(0.0);
        arma::mat A_t = (arma::mat(3 + m_t.n_rows, 3 + m_t.n_rows)).fill(0.0);
        arma::mat I = eye(size(g));

        // If the rotational velocity is zero (twist.thetadot = 0)
        if (rigid2d::almost_equal(twist.thetadot, 0.0, 1.0e-3)) {
            g(0, 0) =  0.0;
            g(1, 0) = -twist.xdot * sin(q_t(0, 0));
            g(2, 0) =  twist.xdot * cos(q_t(0, 0));
        }

        // If the rotational velocity is not zero (twist.thetadot != 0)
        else {
            double dx_dtheta = twist.xdot/twist.thetadot;
            g(0, 0) =  0.0;
            g(1, 0) = -dx_dtheta * cos(q_t(0, 0)) + dx_dtheta * cos(q_t(0, 0) + twist.thetadot); 
            g(2, 0) = -dx_dtheta * sin(q_t(0, 0)) + dx_dtheta * sin(q_t(0, 0) + twist.thetadot); 
        }

        // Return the current state
        return (I + g);
    }

    /// Predict the next step - finds the estimated state and coveriance.
    void EKF::predict(const Twist2D &twist) {
        xi_predict = get_new_state(twist);
        arma::mat A_t = get_transition(twist);
        cov_predict = A_t * cov * A_t.t() + Q_mat;
    }

    /// Compute the measurement h for range and bearing to landmark.
    arma::mat EKF::get_h(int index) {
        arma::mat h = arma::mat(2,1);
        h(0, 0) = std::sqrt(pow(m_t(index, 0) - xi_predict(1, 0), 2) + pow(m_t(index + 1, 0) - xi_predict(2, 0), 2));
        h(1, 0) = rigid2d::normalize_angle(atan2(m_t(index + 1, 0) - xi_predict(2, 0), m_t(index, 0) - xi_predict(1, 0)) - xi_predict(0, 0));

        return h;
    }

    /// Compute the derivative of h with respect to the state.
    arma::mat EKF::get_H(int index) {
        double del_x = m_t(index, 0) - xi_predict(1, 0);
        double del_y = m_t(index + 1, 0) - xi_predict(2, 0);
        double d = pow(del_x, 2) + pow(del_y, 2);

        arma::mat H = arma::mat(2, cov.n_rows).fill(0.0);
        H(0, 1) = -del_x / sqrt(d);
        H(0, 2) = -del_y / sqrt(d);
        H(1, 0) = -1;
        H(1, 1) =  del_y / d;
        H(1, 2) = -del_x / d;

        H(0, index + 3) = del_x / sqrt(d);
        H(0, index + 4) = del_y / sqrt(d);
        H(1, index + 3) =  -del_y / d;
        H(1, index + 4) = del_x / d;

        return H;
    }

    /// Update the next step.
    void EKF::update(std::vector<Measurement> meas) {
        std::cout << "Updating \n\r" << std::endl;

        for (auto& m: meas) {
            int m_row = id2landmark.find(m.id)->second;

            // Compute the theoretical measurement, given the current state estimate
            arma::mat z_theory = get_h(m_row);

            // Compute the Kalman gain from the linearized measurement model
            arma::mat H_i = get_H(m_row);
            arma::mat H_i_t = H_i.t();

            // std::cout << "H_i " << (H_i) << "\n\r" << std::endl;
            // std::cout << "cov_predict " << (cov_predict) << "\n\r" << std::endl;
            // std::cout << "H_i.t() " << (H_i_t) << "\n\r" << std::endl;
            // std::cout << "R_mat " << (R_mat) << "\n\r" << std::endl;
            // std::cout << "H_i * cov_predict * H_i.t() + R_mat " << (H_i * cov_predict * H_i_t + R_mat) << "\n\r" << std::endl;
            // std::cout << "inv(H_i * cov_predict * H_i.t() + R_mat) " << size(inv(H_i * cov_predict * H_i_t + R_mat)) << "\n\r" << std::endl;

            arma::mat int_mat = inv(H_i * cov_predict * H_i_t + R_mat);

            // std::cout << "cov_predict " << (size(cov_predict)) << "\n\r" << std::endl;
            // std::cout << "H_i.t() " << (size(H_i_t)) << "\n\r" << std::endl;
            // std::cout << "int_mat " << (size(int_mat)) << "\n\r" << std::endl;
            // std::cout << "cov_predict * H_i.t() * int_mat " << cov_predict * H_i_t * int_mat << "\n\r" << std::endl;

            arma::mat K = cov_predict * H_i_t * int_mat;

            // Compute the posterior state update
            arma::mat z = m.compute_z();

            // std::cout << "xi_predict " << (size(xi_predict)) << "\n\r" << std::endl;
            // std::cout << "K " << (size(K)) << std::endl;
            // std::cout << "z " << (size(z)) << "\n\r" << std::endl;
            // std::cout << "z_theory " << (size(z_theory)) << "\n\r" << std::endl;

            xi_predict = xi_predict + K * (z - z_theory);

            // Compute the posterior covariance
            arma::mat I = eye(size(cov_predict));

            // std::cout << "I " << ((I)) << "\n\r" << std::endl;
            // std::cout << "K " << ((K)) << "\n\r" << std::endl;
            // std::cout << "H_i " << ((H_i)) << "\n\r" << std::endl;
            // std::cout << "cov_predict " << ((cov_predict)) << "\n\r" << std::endl;
            // std::cout << "I - K * H_i " << (I - K * H_i) << "\n\r" << std::endl;

            cov_predict = (I - K * H_i) * cov_predict;
        }
    }

    /// Add a new zero row and column to a matrix (used when adding new landmark).
    arma::mat EKF::update_matrix_size(arma::mat mat) {
        arma::mat mat_addition_col = (arma::mat(mat.n_rows, 2)).fill(0.0);
        arma::mat mat_addition_row = (arma::mat(2, mat.n_cols + 2)).fill(0.0);

        arma::mat mat_update = std::move(arma::join_rows(mat, mat_addition_col));
        mat_update = std::move(arma::join_cols(mat_update, mat_addition_row));

        return mat_update;
    }

    /// Run the Extended Kalman Filter algorithm.
    void EKF::run_ekf(const Twist2D &twist, const std::vector<Measurement> &meas) {
        // For every measurement
        for (auto& m: meas) {
            add_new_measurment(m);
        }
        
        // Predict and update
        predict(twist);
        update(meas);

        // Update the combined state vector and the covariance
        xi = xi_predict;
        cov = cov_predict;

        // Update robot state and map state
        q_t = xi.submat(0, 0, 2, 0);
        m_t = xi.submat(3, 0, xi.n_rows - 1, 0);
    }

    /// Output the robot state
    arma::mat EKF::output_state() {
        q_t(0, 0) = rigid2d::normalize_angle(q_t(0, 0));
        return q_t;
    }

    /// Output the map state
    arma::mat EKF::output_map_state() {
        return m_t;
    }
}