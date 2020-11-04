#pragma once

#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "geomery_msgs/Twist.h"

class EKFSLAM{
    public:
        EKFSLAM();
        virtual ~EKFSLAM();

        void handle_input(const geometry_msgs::Twist::ConstPtr& msg);
        void handle_observations(/* const perception::Observations::ConstPtr& msg */);
        void step();

        Eigen::VectorXd mean;
        Eigen::MatrixXd covariance;
        geometry_msgs::Twist input;
        /*perception::Observations observations;*/
        std::vector<int> observed_landmarks;
};

std::ostream& operator<<(std::ostream& out, const EKFSLAM& other);
