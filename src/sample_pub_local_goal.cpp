#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include <math.h>

using namespace std;

const float V_max = 1.35;
const float V_min = 0.30;

const float L_min = 3.0;
const float L_max = 5.5;

nav_msgs::Odometry odom;


void odom_callback(nav_msgs::Odometry msg){
	odom = msg;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "sample_pub_waypoint");

	ros::NodeHandle n;

	ros::Publisher pub1 = n.advertise<geometry_msgs::PoseStamped>("/local_goal", 1);
	ros::Publisher pub2 = n.advertise<std_msgs::Float32>("/control/velo", 1);
	ros::Publisher pub3 = n.advertise<std_msgs::Float32>("/control/r", 1);

	ros::Subscriber sub = n.subscribe("/test_odom", 1, odom_callback);
	
	float theta = 0.0;

	geometry_msgs::PoseStamped goal;
	
	float x_init = 1.5;
	float y_init = -28.0;
	float theta_init = (180.0 + 2.0) / 180.0 * M_PI;
	goal.header.frame_id = "/map";
	goal.pose.position.x = 1.5 + -1 * odom.pose.pose.position.x + x_init;
	goal.pose.position.y = 0.0 + -1 * odom.pose.pose.position.y + y_init;
	goal.pose.position.z = 0.0;
	
	goal.pose.orientation = tf::createQuaternionMsgFromYaw(theta  + theta_init);
	
	std_msgs::Float32 velo, r;

	velo.data = 0.6;
	r.data = (L_max - L_min) * (velo.data - V_min)/(V_max - V_min) + L_min;
	
	ros::Rate loop_rate(10);

	while(ros::ok()){
		goal.header.stamp = ros::Time(0);
		goal.pose.position.x = -3.0 + -1 * odom.pose.pose.position.x + x_init;
		goal.pose.position.y = 0.0 + -1 * odom.pose.pose.position.y + y_init;

		pub1.publish(goal);
		pub2.publish(velo);
		pub3.publish(r);
		
		cout<<"odom : "<<odom<<endl;

		cout<<"goal.pose.position.x : "<<goal.pose.position.x<<endl;
		cout<<"goal.pose.position.y : "<<goal.pose.position.y<<endl;
		cout<<"goal.pose.position.z : "<<goal.pose.position.z<<endl;
		cout<<"goal.pose.orientation : "<<endl<<goal.pose.orientation<<endl;

		cout<<"velo.data : "<<velo.data<<endl;
		cout<<"r.data : "<<r.data<<endl;
		
		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}
