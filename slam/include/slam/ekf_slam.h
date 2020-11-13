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
        void handle_odometry(const nav_msgs::Odometry::ConstPtr& msg);
        void handle_observations(/* const perception::Observations::ConstPtr& msg */);
        void step(const double& dt);

        nav_msgs::Odometry estimated_odometry() const;

    private:
        geometry_msgs::Twist _u;
        perception::Observations _z;
        std::vector<int> _observed_landmarks;
        Eigen::VectorXd _mu;
        Eigen::MatrixXd _sigma;
        Eigen::VectorXd _alpha;
        Eigen::MatrixXd _q;
};

std::ostream& operator<<(std::ostream& out, const EKFSLAM& other);
