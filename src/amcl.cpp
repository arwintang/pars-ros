#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <string>
#include <sstream>
#include <vector>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>


geometry_msgs::PoseWithCovarianceStamped amcl_pose;
void amclMessageReceived(const geometry_msgs::PoseWithCovarianceStamped &msg) {
    amcl_pose = msg;
}

int main(int argc, char **argv) {
  	ros::init(argc, argv, "go_three_second");
  	ros::NodeHandle nh;
  	ros::Subscriber amcl_listener = nh.subscribe("amcl_pose", 1, &amclMessageReceived);

	tf::TransformListener listener;
  	ros::Rate rate(10.0);
	
	tf::StampedTransform first_mapstatic_map_transform;
	
	try {
		listener.waitForTransform("/map_static", "/map", ros::Time(0), ros::Duration(10.0));
		listener.lookupTransform("/map_static", "/map", ros::Time(0), first_mapstatic_map_transform);
	} catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(10.0).sleep();
	}
	std::cerr << "map_static to map transform: " << first_mapstatic_map_transform.getOrigin().x() << " " << 
				first_mapstatic_map_transform.getOrigin().y() << std::endl;
	
	usleep(100000);
	tf::StampedTransform mapstatic_map_transform;
	try {
		listener.waitForTransform("/map_static", "/map", ros::Time(0), ros::Duration(10.0));
		listener.lookupTransform("/map_static", "/map", ros::Time(0), mapstatic_map_transform);
	} catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(10.0).sleep();
	}
	std::cerr << "map_static to map transform: " << mapstatic_map_transform.getOrigin().x() << " " << 
				mapstatic_map_transform.getOrigin().y() << std::endl;

  	float x = atof(argv[1]) + mapstatic_map_transform.getOrigin().x();
  	float y = atof(argv[2]) + mapstatic_map_transform.getOrigin().y();
  	float theta = atof(argv[3]);
  	
  	std::string header = "rosrun stdr_robot robot_handler replace /robot0 ";
	std::stringstream ss;
	ss.str("");
	ss << header << x << " " << y << " " << theta; //append the x y and angle information
	system(ss.str().c_str()); // sets initial position
	
	usleep(100000);
	ros::spinOnce();
	
	tf::StampedTransform map_robot_transform;
	try {
		listener.waitForTransform("/map", "/robot0", ros::Time(0), ros::Duration(10.0));
		listener.lookupTransform("/map", "/robot0", ros::Time(0), map_robot_transform);
	} catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(10.0).sleep();
	}
	geometry_msgs::Pose tf_pose;
	tf_pose.position.x = map_robot_transform.getOrigin().x();				
	tf_pose.position.y = map_robot_transform.getOrigin().y();				  		
	
  	// Write the answer to pipe
  	int fd;
  	const char * myfifo = "/tmp/myfifo";
 
  	fd = open(myfifo, O_WRONLY);
  	ROS_INFO_STREAM("The pipe has been created!");
  	std::cerr << "TF pose: " << tf_pose.position.x << "  " << tf_pose.position.y << std::endl;
	std::cerr << "AMCL pose (write into PIPE): " << amcl_pose.pose.pose.position.x << "  " << amcl_pose.pose.pose.position.y << std::endl;
  	
  	write(fd, (char *) &amcl_pose.pose.pose.position.x, sizeof(double));
  	write(fd, (char *) &amcl_pose.pose.pose.position.y, sizeof(double));
  	write(fd, (char *) &amcl_pose.pose.pose.orientation.w, sizeof(double));
  	write(fd, (char *) &amcl_pose.pose.pose.orientation.z, sizeof(double));

  	for (int i=0; i<36; i++)
    	write(fd, (char *) &amcl_pose.pose.covariance[i], sizeof(double));

  	close(fd);
  	 
	return 0;  	
}
