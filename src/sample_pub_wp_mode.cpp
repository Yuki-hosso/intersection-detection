#include "ros/ros.h"
#include <std_msgs/Int32.h>

using namespace std;

int main(int argc, char** argv){
	ros::init(argc, argv, "sample_pub_wp_mode");

	ros::NodeHandle n;
	
	ros::Publisher pub = n.advertise<std_msgs::Int32>("/wpmode", 1);

	std_msgs::Int32 wp_mode;
	wp_mode.data = 1;

	ros::Rate loop_rate(10);

	while(ros::ok()){
		cout<<"wp_mode.data : "<<wp_mode.data<<endl;

		pub.publish(wp_mode);

		loop_rate.sleep();
	}
	return 0;
}
