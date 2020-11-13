#include <iostream>
#include "ros/ros.h"
#include "ekf_slam.h"

int main(int argc, char* argv[]){
    EKFSLAM ekf_slam;

	ros::init(argc, argv, "ekf_slam_node");
	ros::NodeHandle node_handle;
	
	ros::Subscriber command_subscriber = node_handle.subscribe("motion_commands", 1, &EKFSLAM::handle_command, &ekf_slam);
    ros::Subscriber observations_subscriber = node_handle.subscribe("perception/observations", 1, &EKFSLAM::handle_observations, &ekf_slam);
    ros::Subscriber handle_odometry = node_handle.subscribe("simulator/odom", 1, &EKFSLAM::handle_odometry, &ekf_slam);

	ros::Publisher estimated_odometry_publisher = node_handle.advertise<nav_msgs::Odometry>("ekf_slam/estimated_odom", 1, true);
		
	sleep(1);

    double frequency = 25.0;
	ros::Rate timer(frequency);
	while (ros::ok()){
		ros::spinOnce();
        ekf_slam.step(1.0/frequency );
		estimated_odometry_publisher.publish(ekf_slam.estimated_odometry());
		timer.sleep();
	}
	return 0;
}
