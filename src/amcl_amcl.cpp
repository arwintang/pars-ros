#include <iostream>
#include <fstream>
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "/home/feng/pars/amcl.h"

int amcl_msg_count = 0;
double distance_traveled = 99999999;
double angle_spinned = 99999999;
double EPISILON_DISTANCE = 0.05;
double EPISILON_ANGLE = 0.03;
double travel_distance;

geometry_msgs::PoseWithCovarianceStamped amcl_pose, amcl_initial;

// file for debugging
std::ofstream file;

void speedMessageReceived(const geometry_msgs::Twist &msg){}

void amclMessageReceived(const geometry_msgs::PoseWithCovarianceStamped &msg) {
    amcl_pose = msg;
    if (amcl_msg_count == 0) {
      amcl_initial = msg;
      amcl_msg_count++;
    } else {      
      double my_temp1 = atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) * 2;
      double my_temp2 = atan2(amcl_initial.pose.pose.orientation.z, amcl_initial.pose.pose.orientation.w) * 2;
      angle_spinned = abs(my_temp1 - my_temp2);
      
      distance_traveled = sqrt( pow((msg.pose.pose.position.x - amcl_initial.pose.pose.position.x),2) +
        pow((msg.pose.pose.position.y - amcl_initial.pose.pose.position.y),2) );       
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "go_three_second");
  ros::NodeHandle nh;
  
  ros::Subscriber speed = nh.subscribe("robot0/cmd_vel", 1, &speedMessageReceived) ;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("robot0/cmd_vel", 1);
  ros::Subscriber amcl_listener = nh.subscribe("amcl_pose", 1, &amclMessageReceived);

  geometry_msgs::Twist msg;

  // Set parameters
  double v = 0.1;
  double s = atof(argv[1]);
  
  travel_distance = s;
  double delta_omega = 0.29;
  double spin_angle = atof(argv[2]);
  
  	std::cout<<"s/travel_distance:"<< s << ", spin_angle:" << spin_angle << std::endl;
  	std::cerr<<"s/travel_distance:"<< s << ", spin_angle:" << spin_angle << std::endl;
  
  if (s == 0 && spin_angle == 0)
    ROS_DEBUG_STREAM("Input is incorrect, it's not moving forward or spinning");
  if (s != 0 && spin_angle != 0)
    ROS_DEBUG_STREAM("Moving forward and spinning at the same time, currently not supported");
  int total_steps = s==0 ?  abs(spin_angle/delta_omega) : s/v;

  //std::cout<<spin_angle<<"  "<<delta_omega<<"  "<<abs(spin_angle/delta_omega)<<std::endl;
  //std::cout<<"v is "<<v<<"s/v = "<<s/v<< "   abs(spin_angle/delta_omega) = "<< std::abs(spin_angle/delta_omega)<<std::endl;
  //std::cout<<"Total steps is "<<total_steps<<std::endl;
  //std::cout<<StartTimeStep<<"  "<<delta_t<<"  "<<v<<"  "<<s<<"  "<<delta_omega<<std::endl;
  double BASE_LINEAR_SPEED = v, BASE_ANGULAR_SPEED = delta_omega;
  int count = 0;

  file.open("speed_file.txt"); // for debug purpose

  bool movement_forward = s==0 ? false : true;
  bool movement_spin = ! movement_forward;

  double CLOCK_SPEED = 0.1;
  ros::Rate rate(1/CLOCK_SPEED);

  if (movement_spin)
    while(ros::ok() && count<int(std::abs(spin_angle)/BASE_ANGULAR_SPEED/CLOCK_SPEED) + 1)
    {
        if (spin_angle<0)
          msg.angular.z = -1 * BASE_ANGULAR_SPEED;
        else
          msg.angular.z = BASE_ANGULAR_SPEED;
        pub.publish(msg);
        ROS_INFO_STREAM("The robot is now spinning!");
        std::cout<<"angle_spinned is "<<angle_spinned<<"    angle needs to spin is "<<spin_angle<<std::endl;
        //system("rostopic ecsho...");
        count++;
        ros::spinOnce();
        rate.sleep();
    }
  else if (movement_forward)
    while(ros::ok() && std::abs(distance_traveled - s)>EPISILON_DISTANCE )
    {
        msg.linear.x = BASE_LINEAR_SPEED;
        pub.publish(msg);
        ROS_INFO_STREAM("The robot is now moving forward!");
        std::cout<<"distance to goal abs(distance_traveled - s) is "<< std::abs(distance_traveled - s) << std::endl;
        //system("rostopic ecsho...");
        ros::spinOnce();
        rate.sleep();
    }

  file.close(); //for debug purpose

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
  char * myfifo = "/tmp/myfifo";

  /* create the FIFO (named pipe) */
  //mkfifo(myfifo, 0666);

  /* write "Hi" to the FIFO */
  fd = open(myfifo, O_WRONLY);
  ROS_INFO_STREAM("The pipe has been created!");
  //std::cout<<"The pipe has been created"<<std::endl;
  //ROS_INFO_STREAM("The value written is "<<amcl_pose.pose.pose.position.x);
  write(fd, (char *) &amcl_pose.pose.pose.position.x, sizeof(double));
  write(fd, (char *) &amcl_pose.pose.pose.position.y, sizeof(double));

  write(fd, (char *) &amcl_pose.pose.pose.orientation.w, sizeof(double));
  write(fd, (char *) &amcl_pose.pose.pose.orientation.z, sizeof(double));

  for (int i=0; i<36; i++)
    write(fd, (char *) &amcl_pose.pose.covariance[i], sizeof(double));

  close(fd);
  
  /* remove the FIFO */
  //unlink(myfifo);
}
