#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "laser_tf_publisher");
	ros::NodeHandle node;
	
	tf::TransformBroadcaster br;
	tf::TransformListener lr;
	
	ros::Rate rate(10.0);
	while(node.ok()){

		/********************** for stdr *************************/
		tf::StampedTransform transform;
		try{
			lr.waitForTransform("/map_static", "/robot0", ros::Time(0), ros::Duration(10.0));
			lr.lookupTransform("/map_static", "/robot0", ros::Time(0), transform);
		}
		catch(tf::TransformException &ex){
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}

		br.sendTransform(tf::StampedTransform(
			tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(-5,-5,0)),
			ros::Time::now(),"/map", "/odom"));

		double x = transform.getOrigin().x();
		double y = transform.getOrigin().y();
		double z = transform.getRotation().z();
		double w = transform.getRotation().w();
				
		br.sendTransform(tf::StampedTransform(
			tf::Transform(tf::Quaternion(0,0,z,w), tf::Vector3(x,y,0)),
			ros::Time::now(),"/odom", "/base_link"));
		
		br.sendTransform(tf::StampedTransform(
			tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)),
			ros::Time::now(),"/base_link", "/robot0_laser_0"));
		
		/***************************** end stdr ********************************/

		rate.sleep();
	}
}
