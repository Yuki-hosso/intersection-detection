//detect_peak_visu →  intersection_detection
//
//

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/SVD>
//PCL
//ransac
#include <pcl/ModelCoefficients.h>  
#include <pcl/sample_consensus/method_types.h>  
#include <pcl/sample_consensus/model_types.h>  
#include <pcl/segmentation/sac_segmentation.h>  

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
//
#include<std_msgs/Int32MultiArray.h>

using namespace std;
using namespace Eigen;

typedef pcl::PointXYZINormal PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

typedef pcl::PointXYZI PointX;
typedef pcl::PointCloud<PointX> CloudX;
typedef pcl::PointCloud<PointX>::Ptr CloudXPtr;

bool callback_flag = false;
bool screen_flag = false;

int loop_count = 10;
// int loop_count = 20;
int degree_count = 720;

int save_number = loop_count*degree_count;
int save_count = 0;


int save_peak[10*720];
// int save_peak[20*720];

ros::Publisher gauss_pub;
ros::Publisher curv_pub;


// void EM_algorithm(int* data)
// {
// 	for(int i=0;i<degree_count;i++){
// 		cout<<i<<","<<data[i]<<endl;
// 	}
// }

void Intersection_detection(std_msgs::Int32MultiArray &input)
{
	for(int i =0;i<degree_count;i++){
		save_peak[save_count*degree_count+i] = 0;
	}
	if(input.layout.data_offset!=100){
		for(int i=0;i<input.layout.data_offset;i++){
			save_peak[save_count*degree_count + input.data[i]] = 1;
			// cout<<"here!"<<endl;
		}
	}

	int integrate_peak[720];
	//init
	for(int i=0;i<degree_count;i++){
		integrate_peak[i] = 0;
	}
	for(int i=0;i<degree_count;i++){
		for(int j=0;j<loop_count;j++){
			integrate_peak[i] += save_peak[j*degree_count + i];
		}
	}

	if(screen_flag==false){
		if(save_count==loop_count-1){
			screen_flag = true;
		}
	}
	if(screen_flag){
		for(int i=0;i<degree_count;i++){
			cout<<i<<","<<integrate_peak[i]<<endl;
		}
	}

	save_count++;
	save_count = save_count%loop_count;
}

//callback
std_msgs::Int32MultiArray peak_in;
void Peak_deg(const std_msgs::Int32MultiArray::Ptr &msg)
{
	peak_in.layout.data_offset = msg->layout.data_offset;
	if(peak_in.layout.data_offset!=100){
		for(int i=0;i<peak_in.layout.data_offset;i++){
			peak_in.data.push_back(msg->data[i]);
		}
	}
	callback_flag = true;
}
int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "intersection_detection");
    ros::NodeHandle nh;
    ros::NodeHandle n;

    // Create a ROS subscriber for the input point cloud
    // ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, velodyne_cb);
    ros::Subscriber sub = nh.subscribe ("/peak/deg", 1, Peak_deg);
    // Create a ROS publisher for the output point cloud
    gauss_pub = nh.advertise<sensor_msgs::PointCloud2> ("/normal_cloud", 1);
    curv_pub = nh.advertise<sensor_msgs::PointCloud2> ("/curvature_cloud", 1);

	//init
	for(int i=0;i<loop_count*degree_count;i++){
		save_peak[i] = 0;
	}
	//
    // main handle
    ros::Rate loop_rate(20);
    while (ros::ok()){
		if(callback_flag){
			Intersection_detection(peak_in);
			callback_flag = false;
		}
        ros::spinOnce();
        loop_rate.sleep();
    }
}
