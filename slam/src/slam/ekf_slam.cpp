#include "ekf_slam.h"
#include <cmath>

geometry_msgs::Quaternion
yaw_to_quaternion(const double& yaw){
    geometry_msgs::Quaternion quaternion;
    quaternion.w = cos(yaw/2.0);
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(yaw/2.0);
    return quaternion;
}

double wrap_angle(double angle){
    if(angle > 1*M_PI){
        angle += -2*M_PI;
    }
    else if(angle < -1*M_PI){
        angle += 2*M_PI;
    }
    return angle;
}

bool is_zero(const double w){
    const double eps = 1e-9;
    return (std::abs(w) <= eps);
}

EKFSLAM::EKFSLAM(){

}

EKFSLAM::~EKFSLAM(){}

void EKFSLAM::handle_input(const geometry_msgs::Twist::ConstPtr& msg){
    _u = *msg;

    // handle
    return;
}

void EKFSLAM::handle_observations(/* const perception::Observations::ConstPtr& msg*/){
    _z = *msg;

    // handle
    return;
}

void EKFSLAM::handle_odometry(const nav_msgs::Odometry::ConstPtr& msg){
    _u = msg -> twist.twist;

    // handle
    return;
}

void step(const double& dt){
    // accounting for zero angular velocity
    if(is_zero(_u.angular.z)) {_u.angular.z = 1e-6;}

    N = _observed_landmarks.size();
    Eigen::MatrixXd F(3, 3*N+3);

    // F_x
    F << Eigen::MatrixXd::Identity(3,3),
         Eigen::MatrixXd::Zero(3, 3*N);
    Eigen::Vector3d A;
    double a1 = -1*_u.linear.x/_u.angular.z*sin(_mu(2)) + _u.linear.x/_u.angular.z*sin(_mu(2) + _u.angular.z*dt);
    double a2 = _u.linear.x/_u.angular.z*cos(_mu(2)) - _u.linear.x/_u.angular.z*cos(_mu(2) + _u.angular.z*dt);
    double a3 = _u.angular.z * dt;
    A << a1, 
         a2, 
         a3;

    Eigen::Matrix3d B;
    double b1 = -1*a2;
    double b2 = a1;
    B << 0, 0, b1,
         0, 0, b2,
         0, 0, 0;

    // motion model step

    Eigen::VectorXd _mu_projected = Eigen::VectorXd::Zero(3*N + 3);
    Eigen::MatrixXd _sigma_projected = Eigen::MatrixXd::Zero(3*N + 3, 3*N + 3);

    _mu_projected = _mu + F.transpose() * A;

    Eigen::MatrixXd G = Eigen::MatrixXd::Identity(3*N+3, 3*N+3) + F.transpose() * B * F;
    // construct V, M, R here 
    _sigma_projected = G*_sigma*G.transpose() + F.transpose()*R*F;


    // measurement model step
    perception::Observation _z_hat;
    for(unsigned int i = 0; i < _z.observations.size(); i++){
        // j = c_t^i
        /*
         *  if j never seen before
         *      \mu_{j,x}, \mu_{j,y}, \mu_{j,s}
         */

        /*
         *  \delta = [\delta_x, \delta_y]'
         *  q = \delta.T @ \delta
         *  compute _z_hat
         *  Compute innovation and update projections
         */
    }

    return;
}

std::ostream& operator<<(std::ostream& out, const EKFSLAM& other){
    // printing
}

nav_msgs::Odometry
EKFSLAM::estimated_odometry() const{
    nav_msgs::Odometry out;
    out.pose.pose.position.x = _mu(0);
    out.pose.pose.position.y = _mu(1);
    out.pose.pose.position.z = 0.0;
    out.pose.pose.orientation = yaw_to_quaternion(_mu(2));
    out.twist.twist.linear.x = _u.linear.x;
    out.twist.twist.angular.z = _u.angular.z;
    return out;
}
