
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/SVD>
//PCL

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/passthrough.h>

using namespace std;
using namespace Eigen;

typedef pcl::PointXYZINormal PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

bool static_flag = false;
bool cluster_flag = false;

int divide = 360;
const double PI = 3.141592;

ros::Publisher local_pub;

inline void pubPointCloud2 (ros::Publisher& pub, 
                            const CloudA& cloud,
                            const char* frame_id,
                            ros::Time& time)
{
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id ="velodyne";
    output.header.stamp = time;
    pub.publish (output);
}

inline void Copy_point(PointA& input,PointA& output)
{
	output.x = input.x;
	output.y = input.y;
	output.z = input.z;
	output.normal_x = input.normal_x;
	output.normal_y = input.normal_y;
	output.normal_z = input.normal_z;
	output.intensity = input.intensity;
	output.curvature = input.curvature;
}

void Remove_cluster(CloudAPtr& cluster,CloudAPtr& st_cloud)
{
	size_t cluster_size = cluster->points.size();
	size_t cloud_size = st_cloud->points.size();
	cout<<"cloud_size"<<cloud_size<<endl;
	cout<<"cluster_size"<<cluster_size<<endl;
	CloudAPtr tmp_cloud(new CloudA);
	bool matching = false;
	cout<<"start!"<<endl;
	for(size_t i = 0;i<cloud_size;i++){
		for(size_t j =0;j<cluster_size;j++){
			// if(st_cloud->points[i].x == cluster->points[j].x && st_cloud->points[i].y == cluster->points[j].y#<{(| && st_cloud->points[i].z == cluster->points[j].z|)}>#){
			if(fabs(st_cloud->points[i].x - cluster->points[j].x) < 0.05 && fabs(st_cloud->points[i].y - cluster->points[j].y) <0.05/* && fabs(st_cloud->points[i].z - cluster->points[j].z) < 0.05*/){
				matching = true;
				// cout<<"aaaaaaaaaaaaaaaaaaa"<<endl;
				break;
			}
		}
		if(matching){
			matching = false;
		}else{
			tmp_cloud->push_back(st_cloud->points[i]);
		}
	}

	cloud_size = tmp_cloud->points.size();
	st_cloud->points.resize(cloud_size);
	for(size_t i = 0;i<cloud_size;i++){
		Copy_point(tmp_cloud->points[i],st_cloud->points[i]);
	}
	cout<<"output_size"<<cloud_size<<endl;
	cout<<"end!"<<endl;
}

//convert to CloudA
void Convert_cloud(CloudAPtr& input,CloudA& output)
{
	size_t cloud_size = input->points.size();
	for(size_t i = 0;i<cloud_size;i++){
		output.push_back(input->points[i]);
	}
}

//callback
CloudAPtr static_cloud(new CloudA);
void Static_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	pcl::fromROSMsg(*msg,*static_cloud);
	static_flag = true;
}

//callback
CloudAPtr cluster_cloud(new CloudA);
void Cluster_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	pcl::fromROSMsg(*msg,*cluster_cloud);
	cluster_flag = true;
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "remove_cluster");
    ros::NodeHandle nh;
    ros::NodeHandle n;

    // Create a ROS subscriber for the input point cloud
    // ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, velodyne_cb);
    ros::Subscriber cluster_sub = nh.subscribe ("/cluster/select", 1, Cluster_callback);
    ros::Subscriber static_sub = nh.subscribe ("/static_cloud", 1, Static_callback);
    // Create a ROS publisher for the output point cloud
    local_pub = nh.advertise<sensor_msgs::PointCloud2> ("/local_cloud", 1);
    // main handle
    ros::Rate loop_rate(40);
    while (ros::ok()){
		if(cluster_flag&&static_flag){
			Remove_cluster(cluster_cloud,static_cloud);

			CloudA ros_cloud;
			Convert_cloud(static_cloud,ros_cloud);

			ros::Time time = ros::Time::now();
			pubPointCloud2(local_pub,ros_cloud,"/velodyne",time);
			cluster_flag = false;
			static_flag = false;
		}
        ros::spinOnce();
        loop_rate.sleep();
    }
}
