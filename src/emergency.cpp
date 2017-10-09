/*
 * 
 * emergency用プログラム 
 * 
 * 
 *
*/

#include <stdio.h>
#include <iostream>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>

#include <omp.h>


using namespace Eigen;
using namespace std;	

typedef pcl::PointXYZINormal PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

bool centroid_flag = false;
bool emergency_mode = false;

ros::Publisher emerg_pub;

bool Dif(PointA centroid){
	double diff = sqrt( pow(centroid.x,2) + pow(centroid.y,2) );

	if(diff<1.5){
		return true;
	}else{
		return false;
	}
}

int global_count = 0;
void Emergency_mode(CloudAPtr centroid){
	size_t centroid_size = centroid->points.size();
	int count = 0;
	for(size_t i = 0;i<centroid_size;i++){
		if(Dif(centroid->points[i])){
			count++;
		}
	}
	if(count==0){
		global_count++;
	}else{
		global_count = 0;
	}
	if(global_count==10){
		emergency_mode = false;
		std_msgs::Bool emerg;
		emerg.data = false;
		emerg_pub.publish(emerg);
		global_count = 0;
		cout<<"Emergency release"<<endl;
	}
}

void Emergency(CloudAPtr centroid){
	size_t centroid_size = centroid->points.size();
	for(size_t i = 0;i<centroid_size;i++){
		if(Dif(centroid->points[i])){
			std_msgs::Bool emerg;
			emerg.data = true;
			emerg_pub.publish(emerg);
			cout<<"Emergency stop"<<endl;
			emergency_mode = true;
		}
	}
}

CloudAPtr cluster_centroid (new CloudA);
void centroid_callback(const sensor_msgs::PointCloud2::Ptr &msg){
	pcl::fromROSMsg(*msg,*cluster_centroid);
	centroid_flag = true;
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "emergency");
  	ros::NodeHandle n;
    ros::NodeHandle nh;
	
    // Create a ROS subscriber for the input point cloud
    // ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, velodyne_cb);
    ros::Subscriber sub = nh.subscribe ("/cluster/centroid", 1, centroid_callback);
    // Create a ROS publisher for the output point cloud
    emerg_pub = nh.advertise<std_msgs::Bool> ("/emergency", 1);

	// CloudAPtr conv_cloud (new CloudA);
	ros::Rate loop_rate(20); // 20
	// ros::Rate loop_rate(30); // 20
	while (ros::ok()){
		if(centroid_flag){
			centroid_flag = false;
			if(emergency_mode){
				cout<<"emergency_mode"<<endl;
				Emergency_mode(cluster_centroid);
			}else{
				cout<<"check_mode"<<endl;
				Emergency(cluster_centroid);
			}
		}
        ros::spinOnce();
        loop_rate.sleep();
	}
	return (0);
}

