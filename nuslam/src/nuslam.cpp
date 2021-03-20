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
        z(1,0) = rigid2d::normalize_angle(phi);

        return z;
    }


    /********** EKF class member functions **********/

    /// Initialize the combined state vector.
    /// Start with a guess for the robot state (0, 0, 0) and zero e map state.
    EKF::EKF() {
        q_t.fill(0.0);
        // cov.fill(0.0);

        xi_predict.fill(0.0);
        cov_predict.fill(0.0);

        Q_mat.fill(0.0);
        Q_mat(0, 0) = 1e-3;
        Q_mat(1, 1) = 1e-3;
        Q_mat(2, 2) = 1e-3;

        R_mat.fill(0.0);
        R_mat(0, 0) = 1e-2;
        R_mat(1, 1) = 1e-2;
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
        id2landmark.insert(std::make_pair(meas.id, xi_predict.n_rows - 3));
    
        // Update covariance matrix
        cov_predict = update_matrix_size(cov_predict);
        cov_predict(cov_predict.n_rows - 1, cov_predict.n_cols - 1) = 10000.0;  // Infinity
        cov_predict(cov_predict.n_rows - 2, cov_predict.n_cols - 2) = 10000.0;  // Infinity

        // Update process noise for the robot motion model Q (expanding to fill the whole state)
        Q_mat = update_matrix_size(Q_mat);

        // Update map state matrix
        double m_x = xi_predict(1, 0) + meas.r * cos(rigid2d::normalize_angle(meas.phi + xi_predict(0, 0)));
        double m_y = xi_predict(2, 0) + meas.r * sin(rigid2d::normalize_angle(meas.phi + xi_predict(0, 0)));
        
        arma::mat m_addition = arma::mat(2, 1);
        m_addition(0,0) = m_x;
        m_addition(1,0) = m_y;
        
        xi_predict = std::move(arma::join_cols(xi_predict, m_addition));
        // m_t = std::move(arma::join_cols(m_t, m_addition));
    }

    /// Get the state transition ξ_t (q_t, m_t) and the map’s movement with respect to the state ξ.
    arma::mat EKF::get_new_state(const Twist2D &twist) {
        arma::mat T_wbp = arma::mat(3, 1);
        arma::mat w_t = (arma::mat(3, 1)).fill(0.0);

        // If the rotational velocity is zero (twist.thetadot = 0)
        if (rigid2d::almost_equal(twist.thetadot, 0.0, 1.0e-6)) {
            T_wbp(0, 0) = 0.0;
            T_wbp(1, 0) = twist.xdot * cos(xi_predict(0, 0));
            T_wbp(2, 0) = twist.xdot * sin(xi_predict(0, 0));
        }

        // If the rotational velocity is not zero (twist.thetadot != 0)
        else {
            double dx_dtheta = twist.xdot/twist.thetadot;
            T_wbp(0, 0) =  twist.thetadot;
            T_wbp(1, 0) = -dx_dtheta * sin(xi_predict(0, 0)) + dx_dtheta * sin(xi_predict(0, 0) + twist.thetadot); 
            T_wbp(2, 0) =  dx_dtheta * cos(xi_predict(0, 0)) - dx_dtheta * cos(xi_predict(0, 0) + twist.thetadot); 
        }

        // Return the current state
        xi_predict(0, 0) = rigid2d::normalize_angle(xi_predict(0, 0) + T_wbp(0, 0) + w_t(0, 0));
        xi_predict(1, 0) = xi_predict(1, 0) + T_wbp(1, 0) + w_t(1, 0);
        xi_predict(2, 0) = xi_predict(2, 0) + T_wbp(2, 0) + w_t(2, 0);

        // arma::mat q_t_new = q_t + T_wbp + w_t;
        // q_t_new(0, 0) = rigid2d::normalize_angle(q_t_new(0, 0));
        return xi_predict;
    }
    
    /// Get derivative of g with respect to the state ξ.
    arma::mat EKF::get_transition(const Twist2D &twist) {
        arma::mat g = (arma::mat(xi_predict.n_rows, xi_predict.n_rows)).fill(0.0);
        arma::mat A_t = (arma::mat(xi_predict.n_rows, xi_predict.n_rows)).fill(0.0);
        arma::mat I = eye(size(g));

        // If the rotational velocity is zero (twist.thetadot = 0)
        if (rigid2d::almost_equal(twist.thetadot, 0.0, 1.0e-6)) {
            g(0, 0) =  0.0;
            g(1, 0) = -twist.xdot * sin(xi_predict(0, 0));
            g(2, 0) =  twist.xdot * cos(xi_predict(0, 0));
        }

        // If the rotational velocity is not zero (twist.thetadot != 0)
        else {
            double dx_dtheta = twist.xdot/twist.thetadot;
            g(0, 0) =  0.0;
            g(1, 0) = -dx_dtheta * cos(xi_predict(0, 0)) + dx_dtheta * cos(xi_predict(0, 0) + twist.thetadot); 
            g(2, 0) = -dx_dtheta * sin(xi_predict(0, 0)) + dx_dtheta * sin(xi_predict(0, 0) + twist.thetadot); 
        }

        // Return the current state
        return (I + g);
    }

    /// Predict the next step - finds the estimated state and coveriance.
    void EKF::predict(const Twist2D &twist) {
        xi_predict = get_new_state(twist);
        arma::mat A_t = get_transition(twist);
        cov_predict = A_t * cov_predict * A_t.t() + Q_mat;
    }

    /// Compute the measurement h for range and bearing to landmark.
    arma::mat EKF::get_h(int index) {
        arma::mat h = arma::mat(2,1);
        h(0, 0) = std::sqrt(pow(xi_predict(index + 3, 0) - xi_predict(1, 0), 2) + pow(xi_predict(index + 4, 0) - xi_predict(2, 0), 2));
        h(1, 0) = rigid2d::normalize_angle(atan2(xi_predict(index + 4, 0) - xi_predict(2, 0), xi_predict(index + 3, 0) - xi_predict(1, 0)) - xi_predict(0, 0));

        return h;
    }

    /// Compute the derivative of h with respect to the state.
    arma::mat EKF::get_H(int index) {
        int i = index + 3;
        double del_x = xi_predict(i, 0) - xi_predict(1, 0);
        double del_y = xi_predict(i + 1, 0) - xi_predict(2, 0);
        double d = pow(del_x, 2) + pow(del_y, 2);

        arma::mat H = arma::mat(2, cov_predict.n_rows).fill(0.0);
        H(0, 1) = -del_x / sqrt(d);
        H(0, 2) = -del_y / sqrt(d);
        H(1, 0) = -1;
        H(1, 1) =  del_y / d;
        H(1, 2) = -del_x / d;

        H(0, i) = del_x / sqrt(d);
        H(0, i + 1) = del_y / sqrt(d);
        H(1, i) =  -del_y / d;
        H(1, i + 1) = del_x / d;

        return H;
    }

    /// Update the next step.
    void EKF::update(std::vector<Measurement> meas) {
        // std::cout << "Updating \n\r" << std::endl;

        arma::mat I = eye(size(cov_predict));
        arma::mat z = arma::mat(2, 1);
        arma::mat z_del = arma::mat(2,1);

        for (auto& m: meas) {
            int m_row = id2landmark.find(m.id)->second;

            // Compute the theoretical measurement, given the current state estimate
            auto z_theory = get_h(m_row);

            // Compute the Kalman gain from the linearized measurement model
            auto H_i = get_H(m_row);
            arma::mat H_i_t = H_i.t();
            arma::mat inv_mat = arma::inv(H_i * cov_predict * H_i_t + R_mat);
            arma::mat K = cov_predict * H_i_t * inv_mat;

            // Compute the posterior state update
            // arma::mat z = m.compute_z();
            z(0, 0) = m.r;
            z(1, 0) = rigid2d::normalize_angle(m.phi);
            // z_del = z - z_theory;
            z_del = z - z_theory;
            z_del(1, 0) = rigid2d::normalize_angle(z_del(1, 0));

            // std::cout << "\rz" << z  << std::endl;
            // std::cout << "\rz_del" << z_del(0, 0) << ", " << z_del(1, 0)<< std::endl;
            xi_predict = xi_predict + K * z_del;
            xi_predict(0, 0) = rigid2d::normalize_angle(xi_predict(0, 0));
            // Compute the posterior covariance
            cov_predict = (I - K * H_i) * cov_predict;
                        
            // std::cout << "\rK" << K << std::endl;
            // std::cout << "\rxi_predict" << xi_predict << std::endl;

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
        // xi = xi_predict;
        // cov = cov_predict;

        // Update robot state and map state
        q_t = xi_predict.submat(0, 0, 2, 0);
        q_t(0, 0) = rigid2d::normalize_angle(q_t(0, 0));
        m_t = xi_predict.submat(3, 0, xi_predict.n_rows - 1, 0);
    }

    /// Output the robot state
    arma::mat EKF::output_state() {
        q_t(0, 0) = rigid2d::normalize_angle(q_t(0, 0));
        return q_t;
    }

    /// Output the map state
    arma::mat EKF::output_map_state() {
        // for (int i=0; i<m_t.n_rows; i+=2) {
        //     // std::cout << "\rmap :" << m_t(i,0) << ", " << m_t(i+1,0) << std::endl;
        // }
        return m_t;
    }
}