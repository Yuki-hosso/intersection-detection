#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>

#include <math.h>

using namespace std;

std_msgs::Float32 x;

void odom_x_callback(std_msgs::Float32 msg){
	x.data = msg.data;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "sample_pub_odom");

	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("/odom_x", 1, odom_x_callback);

	ros::Publisher pub = n.advertise<nav_msgs::Odometry>("/test_odom", 1);


	nav_msgs::Odometry odom;
	// odom.header.frame_id = "/world";
	odom.header.frame_id = "/map";
	odom.child_frame_id = "/matching_base_link";

	float theta = 0.0 / 180.0 * M_PI;

	odom.pose.pose.position.x = 0.0;
	odom.pose.pose.position.y = 0.0;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
	
	ros::Rate loop_rate(10);

	while(ros::ok()){
		odom.header.stamp = ros::Time(0);
		
		// odom.pose.pose.position.x = x.data;
		cout<<"odom.header.frame_id : "<<odom.header.frame_id<<endl;
		cout<<"odom.child_frame_id : "<<odom.child_frame_id<<endl;
		cout<<"odom.pose.pose.position.x : "<<odom.pose.pose.position.x<<endl; 
		cout<<"odom.pose.pose.position.y : "<<odom.pose.pose.position.y<<endl; 
		cout<<"odom.pose.pose.position.z : "<<odom.pose.pose.position.z<<endl;
		cout<<"odom.pose.pose.orientation : "<<endl
			<<odom.pose.pose.orientation<<endl;

		odom.pose.pose.position.x += 0.6;
		// odom.pose.pose.position.x += 0.01;
		// odom.pose.pose.position.y += 0.1;
		// theta += 0.1;
		odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
		
		pub.publish(odom);
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
