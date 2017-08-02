#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

using namespace std;

sensor_msgs::PointCloud eth_220, eth_221, eth_222;

ros::Publisher vis_pub_220, vis_pub_221, vis_pub_222;


void eth_220_callback(const sensor_msgs::PointCloud msg){
	eth_220 = msg;
	// cout<<"sub : "<<eth_220.points.size()<<endl;
	eth_220.header.frame_id = "/3dlaser";
	eth_220.header.stamp = ros::Time::now();
	vis_pub_220.publish(eth_220);
}
void eth_221_callback(const sensor_msgs::PointCloud msg){
	eth_221 = msg;
	// cout<<"sub : "<<eth_221.points.size()<<endl;
	eth_221.header.frame_id = "/3dlaser";
	eth_221.header.stamp = ros::Time::now();
	vis_pub_221.publish(eth_221);
}
void eth_222_callback(const sensor_msgs::PointCloud msg){
	eth_222 = msg;
	// cout<<"sub : "<<eth_222.points.size()<<endl;
	eth_222.header.frame_id = "/3dlaser";
	eth_222.header.stamp = ros::Time::now();
	vis_pub_222.publish(eth_222);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "vis_tom_laser");
	ros::NodeHandle n;

	ros::Subscriber sub_220 = n.subscribe("/eth_220/point_cloud_3", 10, eth_220_callback);
	ros::Subscriber sub_221 = n.subscribe("/eth_221/point_cloud_3", 10, eth_221_callback);
	ros::Subscriber sub_222 = n.subscribe("/eth_222/point_cloud_3", 10, eth_222_callback);

	vis_pub_220 = n.advertise<sensor_msgs::PointCloud>("/eth_220/point_cloud_3/vis", 1);
	vis_pub_221 = n.advertise<sensor_msgs::PointCloud>("/eth_221/point_cloud_3/vis", 1);
	vis_pub_222 = n.advertise<sensor_msgs::PointCloud>("/eth_222/point_cloud_3/vis", 1);
	
	ros::spin();
	return 0;
}
