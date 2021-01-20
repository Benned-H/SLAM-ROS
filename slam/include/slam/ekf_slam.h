/*
 * Abstract class implementing EKF SLAM as defined on pg. 59 of Probabilistic Robotics, 3rd Edition
 * Authors: Abrar Rahman and Benned Hedegaard
 */

#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Dense>

class EKFSLAM {

    public:
    
        EKFSLAM();
        virtual ~EKFSLAM();
        
        // Add functions for updating _u and _z using whatever message types you'd like
        
        virtual Eigen::VectorXd g() const = 0; // Returns mu_bar = g( u, prev_mu )
        virtual Eigen::VectorXd h() const = 0; // Returns z = h( x )
        
        virtual nav_msgs::Odometry estimated_odometry() const = 0; // Returns the current estimated odometry
        
        void step(const double& dt); // Steps the EKF belief forward

    private:
    
        Eigen::VectorXd _mu; // Mean of Gaussian state representation
        Eigen::MatrixXd _sigma; // Covariance of Gaussian state representation
        Eigen::MatrixXd _G; // Jacobian of g
        Eigen::MatrixXd _R; // 
        Eigen::MatrixXd _H; // 
        Eigen::MatrixXd _Q; // 
};

std::ostream& operator<<(std::ostream& out, const EKFSLAM& other);
