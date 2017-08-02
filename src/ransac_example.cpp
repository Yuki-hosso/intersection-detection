//ransac for 
//
//

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

bool point_flag = false;
bool centroid_flag = false;

ros::Publisher line_pub;


void Sampling(CloudA &data,PointA &data_a,PointA &data_b)
{
	//ランダムにデータを抽出
}

void Line_fitting(PointA &data_a,PointA &data_b,double &a,double &b)
{
	a = ( data_b.y - data_a.y )/(data_b.x - data_a.y);
	b = data_a.y - a*data_a.x;
}

double Error_function(CloudA &input_data,double &a,double &b)
{
	size_t cloud_size = input_data.points.size();
	double total_error = 0.0;
	for(size_t i=0;i<cloud_size;i++){
		total_error += fabs(input_data.points[i].y - ( a * input_data.points[i].x + b ));
	}

	return total_error/(double)cloud_size;
}

double Error_function_points(PointA &input_point,double &a,double &b)
{
	double error = 0.0;
	error = fabs(input_point.y - ( a * input_point.x + b ));

	return error;
}

void Ransac(CloudA &input_data)
{
	int k = 100;
	// double t = 2.0;
	double t = 0.7;
	int iteration = 0;
	bool line_flag = false;

	PointA data_a ,data_b;
	double a=0,b=0;
	double d_a=0,d_b=0;;
	double min_error = 100.0;

	//estimate line function
	while(iteration<k){
		Sampling(input_data,data_a,data_b);//random_sampling
		Line_fitting(data_a,data_b,a,b);//desition param a,b --y=ax+b--

		double error = Error_function(input_data,a,b);
		
		if(error<t){
			if(error<min_error){
				min_error = error;
				d_a = a;
				d_b = b;
			}
			line_flag = true;
		}

		iteration++;
	}
	//visualize line 
	if(line_flag){
		CloudA visu_line;
		size_t line_size = input_data.points.size();
		for(size_t i = 0;i<line_size;i++){
			double error = Error_function_points(input_data.points[i],d_a,d_b);
	
			if(error<t){
				visu_line.push_back(input_data.points[i]);
			}
		}
	}
}

// call back
CloudA point_data;
void cluster_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	pcl::fromROSMsg(*msg,point_data);
	point_flag = true;
}

CloudA centroid_data;
void centroid_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	pcl::fromROSMsg(*msg,centroid_data);
	centroid_flag = true;
}

//main function
int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "ransac");
    ros::NodeHandle nh;
    ros::NodeHandle n;

    // Create a ROS subscriber for the input point cloud
    // ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, velodyne_cb);
    ros::Subscriber pt_sub = nh.subscribe ("/cluster/curvature/pt", 1, cluster_callback);
    ros::Subscriber centroid_sub = nh.subscribe ("/cluster/curvature/centroid", 1, centroid_callback);
    // Create a ROS publisher for the output point cloud
    line_pub = nh.advertise<sensor_msgs::PointCloud2> ("/hsd/ransac", 1);
    // main handle
    ros::Rate loop_rate(20);
    while (ros::ok()){
		if(point_flag&&centroid_flag){
			Ransac(point_data);
			point_flag = false;
			centroid_flag = false;
		}

        ros::spinOnce();
        loop_rate.sleep();
    }
}
