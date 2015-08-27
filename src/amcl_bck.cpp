#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <math.h>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <signal.h>

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

double distance_traveled = 0;
double angle_spinned = 99999999;
double EPISILON_DISTANCE = 0.005;
double EPISILON_ANGLE = 0.03;
double travel_distance;

geometry_msgs::PoseWithCovarianceStamped amcl_pose, amcl_initial;

void speedMessageReceived(const geometry_msgs::Twist &msg) {}
void amclMessageReceived(const geometry_msgs::PoseWithCovarianceStamped &msg) {
    amcl_pose = msg;
    //std::cerr << "AMCL pose: " << amcl_pose.pose.pose.position.x << "  " << amcl_pose.pose.pose.position.y << std::endl;
}

void mySigintHandler(int sig) {
	ros::shutdown();
}

int main(int argc, char **argv) {
  	ros::init(argc, argv, "go_three_second");
  	ros::NodeHandle nh;
  
  	ros::Subscriber speed = nh.subscribe("robot0/cmd_vel", 1, &speedMessageReceived) ;
  	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("robot0/cmd_vel", 1);
  	ros::Subscriber amcl_listener = nh.subscribe("amcl_pose", 1, &amclMessageReceived);

  	geometry_msgs::Twist msg;

  	double v = 0.01;
  	double s = atof(argv[1]);
  	travel_distance = s;
  	double delta_omega = 0.29;
  	double spin_angle = atof(argv[2]);
  
  	std::cout<<"travel_distance:"<< s << ", spin_angle:" << spin_angle << std::endl;
  	std::cerr<<"travel_distance:"<< s << ", spin_angle:" << spin_angle << std::endl;
  
	if (s == 0 && spin_angle == 0) {
		ROS_DEBUG_STREAM("Input is incorrect, it's not moving forward or spinning");
	}
  	if (s != 0 && spin_angle != 0) {
  		ROS_DEBUG_STREAM("Moving forward and spinning at the same time, currently not supported");
  	}
    
	int total_steps = s==0 ?  abs(spin_angle/delta_omega) : s/v;

	double BASE_LINEAR_SPEED = v, BASE_ANGULAR_SPEED = delta_omega;
  	int count = 0;

  	bool movement_forward = s==0 ? false : true;
  	bool movement_spin = !movement_forward;

	tf::TransformListener listener;
  	double CLOCK_SPEED = 0.1;
  	ros::Rate rate(1/CLOCK_SPEED);
 
	geometry_msgs::Pose tf_init_pose;
	geometry_msgs::Pose tf_curr_pose;

	if (movement_spin) {
		while (ros::ok() && nh.ok() && movement_spin && count <= int(std::abs(spin_angle)/BASE_ANGULAR_SPEED/CLOCK_SPEED)) {        
			if (spin_angle < 0) {
				msg.angular.z = -1 * BASE_ANGULAR_SPEED;
			} else {
		        msg.angular.z = BASE_ANGULAR_SPEED;
			}
		    pub.publish(msg);
			ROS_INFO_STREAM("The robot is now spinning!");
			std::cout << "angle_spinned is " << angle_spinned << " angle needs to spin is "<< spin_angle << std::endl;
			count++;
			ros::spinOnce();
			rate.sleep();			
		}		
	} else if (movement_forward) {
		int move_count = 0;
	    while (ros::ok() && nh.ok() && std::abs(distance_traveled - s) > EPISILON_DISTANCE) {
		    tf::StampedTransform map_odom_transform;
		    tf::StampedTransform odom_base_transform;
		    tf::StampedTransform base_laser_transform;
			try {
				listener.waitForTransform("/map", "/odom", ros::Time(0), ros::Duration(10.0));
				listener.lookupTransform("/map", "/odom", ros::Time(0), map_odom_transform);
				listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(10.0));
				listener.lookupTransform("/odom", "/base_link", ros::Time(0), odom_base_transform);
				listener.waitForTransform("/base_link", "/robot0_laser_0", ros::Time(0), ros::Duration(10.0));
				listener.lookupTransform("/base_link", "/robot0_laser_0", ros::Time(0), base_laser_transform);
			} catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}
	        if (move_count == 0) {
				//tf_init_pose.position.x = map_odom_transform.getOrigin().x() + odom_base_transform.getOrigin().x() + base_laser_transform.getOrigin().x();
				//tf_init_pose.position.y = map_odom_transform.getOrigin().y() + odom_base_transform.getOrigin().y() + base_laser_transform.getOrigin().y();	
				//tf_init_pose.orientation.z = map_odom_transform.getRotation().z() + odom_base_transform.getRotation().z() + base_laser_transform.getRotation().z();
				//tf_init_pose.orientation.w = map_odom_transform.getRotation().w() + odom_base_transform.getRotation().w() + base_laser_transform.getRotation().w();
				tf_init_pose.position.x = -5 + odom_base_transform.getOrigin().x();
				tf_init_pose.position.y = -5 + odom_base_transform.getOrigin().y();	
				tf_init_pose.orientation.z = 0 + odom_base_transform.getRotation().z();
				tf_init_pose.orientation.w = 0 + odom_base_transform.getRotation().w();
	        }
	        
			//tf_curr_pose.position.x = map_odom_transform.getOrigin().x() + odom_base_transform.getOrigin().x() + base_laser_transform.getOrigin().x();				
			//tf_curr_pose.position.y = map_odom_transform.getOrigin().y() + odom_base_transform.getOrigin().y() + base_laser_transform.getOrigin().y();				
			//tf_curr_pose.orientation.z = map_odom_transform.getRotation().z() +  odom_base_transform.getRotation().z() + base_laser_transform.getRotation().z();			
			//tf_curr_pose.orientation.w = map_odom_transform.getRotation().w() + odom_base_transform.getRotation().w() + base_laser_transform.getRotation().w();
			tf_curr_pose.position.x = -5 + odom_base_transform.getOrigin().x() + base_laser_transform.getOrigin().x();				
			tf_curr_pose.position.y = -5 + odom_base_transform.getOrigin().y() + base_laser_transform.getOrigin().y();				
			tf_curr_pose.orientation.z = 0 +  odom_base_transform.getRotation().z();			
			tf_curr_pose.orientation.w = 0 + odom_base_transform.getRotation().w();
			
			distance_traveled = sqrt(pow(tf_curr_pose.position.x - tf_init_pose.position.x, 2) + pow(tf_curr_pose.position.y - tf_init_pose.position.y, 2));
            
			//std::cerr<< "map_odom_transform: " << map_odom_transform.getOrigin().x() << "  " << map_odom_transform.getOrigin().y() << std::endl;
			//std::cerr<< "base_laser_transform: " << base_laser_transform.getOrigin().x() << "  " << base_laser_transform.getOrigin().y() << std::endl;
			
			msg.linear.x = BASE_LINEAR_SPEED;
			msg.linear.y = BASE_LINEAR_SPEED;
	        pub.publish(msg);
			std::cerr<< "initial pos: " << tf_init_pose.position.x << "  " << tf_init_pose.position.y << std::endl;
			std::cerr<< "current pos: " << tf_curr_pose.position.x << "  " << tf_curr_pose.position.y << std::endl;
			std::cerr<< "distance left to go is " << std::abs(s - distance_traveled) << std::endl;			
			move_count++;
	        ros::spinOnce();
	        rate.sleep();
	    }
		//std::cerr<< "initial pos: " << tf_init_pose.position.x << "  " << tf_init_pose.position.y << std::endl;
		//std::cerr<< "current pos: " << tf_curr_pose.position.x << "  " << tf_curr_pose.position.y << std::endl;
		//std::cerr<< "distance left to go is " << std::abs(s - distance_traveled) << std::endl;
	}

	// make the robot stop  
  	for (int i = 0; i < 2; i++) {
		msg.linear.x = 0;
  		msg.linear.y = 0;
		msg.linear.z = 0;
		msg.angular.x = 0;
		msg.angular.y = 0;
		msg.angular.z = 0;
		pub.publish(msg);
  	}
    ROS_INFO_STREAM("The robot finished moving forward/ spinning!");
  
    // Guard, make sure the robot stops.
	rate.sleep();
	msg.linear.x = 0;
	msg.linear.y = 0;
	msg.linear.z = 0;
	msg.angular.x = 0;
	msg.angular.y = 0;
	msg.angular.z = 0;
	pub.publish(msg); 
	
  	// Write the answer to pipe
  	int fd;
  	const char * myfifo = "/tmp/myfifo";
 
  	/* write "Hi" to the FIFO */
  	fd = open(myfifo, O_WRONLY);
  	ROS_INFO_STREAM("The pipe has been created!");
	std::cerr<< "write in to PIPE: " << amcl_pose.pose.pose.position.x << "  " << amcl_pose.pose.pose.position.y << std::endl;
   
  	write(fd, (char *) &amcl_pose.pose.pose.position.x, sizeof(double));
  	write(fd, (char *) &amcl_pose.pose.pose.position.y, sizeof(double));
  	write(fd, (char *) &amcl_pose.pose.pose.orientation.w, sizeof(double));
  	write(fd, (char *) &amcl_pose.pose.pose.orientation.z, sizeof(double));

  	for (int i=0; i<36; i++)
    	write(fd, (char *) &amcl_pose.pose.covariance[i], sizeof(double));

  	close(fd);
  	
  	//ros::shutdown();
  	//exit(EXIT_SUCCESS); 
	return 0;
  	/* remove the FIFO */
  	//unlink(myfifo);
  	
}
