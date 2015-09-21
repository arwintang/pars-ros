#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <tf/transform_listener.h>

int main(int argc, char** argv){

	ros::init(argc, argv, "laser_tf_publisher");
	ros::NodeHandle n;

	ros::Rate r(10);
	
	tf::TransformBroadcaster broadcaster;
	tf::TransformListener listener;
	tf::StampedTransform transform;
	
	while(n.ok()){
		try{
			listener.waitForTransform("/map_static", "/robot0", ros::Time(0), ros::Duration(10.0));
			listener.lookupTransform("/map_static", "/robot0", ros::Time(0), transform);
		}
		catch(tf::TransformException &ex){
			ROS_ERROR("%s", ex.what());
		}
		broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(-5,-5,0)),
				ros::Time::now(),"map", "odom"));

		double x = transform.getOrigin().x();
		double y = transform.getOrigin().y();
		double z = transform.getRotation().z();
		double w = transform.getRotation().w();
		broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0,0,z,w), tf::Vector3(x,y,0)),
				ros::Time::now(),"odom", "base_link"));

		broadcaster.sendTransform(
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)),
				ros::Time::now(),"base_link", "robot0_laser_0"));		
		
		r.sleep();
	}
}
