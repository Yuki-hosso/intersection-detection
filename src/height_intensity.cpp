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

using namespace std;
using namespace Eigen;

typedef pcl::PointXYZINormal PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

typedef pcl::PointXYZI PointX;
typedef pcl::PointCloud<PointX> CloudX;
typedef pcl::PointCloud<PointX>::Ptr CloudXPtr;

bool velodyne_flag = false;
double height_cut = 0.5;

ros::Publisher inten_pub;
// ros::Publisher curv_pub;

//  output 
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
//
inline double Diff(PointA velodyne)
{
	double diff = sqrt( pow(velodyne.x,2) + pow(velodyne.y,2) );

	return diff;
}

// void Create_intensity_map(PointA &velodyne,PointA &intensity)
// {
// 	// if(velodyne.intensity/10.0>3.1){//long grass
// 	// if(velodyne.intensity/10.0>2.1){//short grass
// 	if(velodyne.intensity/10.0>1.1&&Diff(velodyne)<=8.0){//tsukuba rainy grass
// 		intensity.x = velodyne.x;
// 		intensity.y = velodyne.y;
// 		intensity.z = velodyne.intensity/10.0 - 1.3;
// 		intensity.intensity = velodyne.z;
// 	}else{
// 		intensity.x = 0.0;
// 		intensity.y = 0.0;
// 		intensity.z = 0.0;
// 		intensity.intensity = 0.0;
// 	}
// }
void Create_intensity_map_2(PointA &velodyne,CloudA &intensity)
{
	// if(velodyne.intensity/10.0>3.1){//long grass
	// if(velodyne.intensity/10.0>2.1){//short grass
	if(velodyne.intensity/10.0>1.1&&Diff(velodyne)<=8.0&&Diff(velodyne)>=1.0){//tsukuba rainy grass
		// velodyne.z = velodyne.intensity/10.0 - 1.3;
		velodyne.z = 0.0;
		intensity.points.push_back(velodyne);
	}
}

void Intensity_threshold(CloudAPtr &velo_cloud)
{
	size_t cloud_size = velo_cloud->points.size();
	// for(size_t i = 0;i<cloud_size;i++){
	// 	double diff = Diff(velo_cloud->points[i]);
	// 	if(diff<10.0){
	// 		cout<<diff<<","<<velo_cloud->points[i].intensity<<endl;
	// 	}
	// }
	CloudA intensity_cloud;
	// intensity_cloud.points.resize(cloud_size);
	// for(size_t i=0;i<cloud_size;i++){
	// 	Create_intensity_map(velo_cloud->points[i],intensity_cloud.points[i]);
	// }
	for(size_t i=0;i<cloud_size;i++){
		Create_intensity_map_2(velo_cloud->points[i],intensity_cloud);
	}

	//output
	cout<<"pub cloud size:"<<intensity_cloud.points.size()<<endl;
    ros::Time time = ros::Time::now();
    pubPointCloud2 (inten_pub, intensity_cloud, "/velodyne", time);
}

//callback
CloudAPtr tmp_cloud (new CloudA);
void point_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	pcl::fromROSMsg(*msg,*tmp_cloud);
	velodyne_flag = true;
}
int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "height_intensity");
    ros::NodeHandle nh;
    ros::NodeHandle n;

    // Create a ROS subscriber for the input point cloud
    // ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, velodyne_cb);
	ros::Subscriber sub = n.subscribe("/velodyne_points",1,point_callback);
    // Create a ROS publisher for the output point cloud
    // gauss_pub = nh.advertise<sensor_msgs::PointCloud2> ("/normal_cloud", 1);
    // gauss_pub = nh.advertise<sensor_msgs::PointCloud2> ("/static_cloud2", 1);
    inten_pub = nh.advertise<sensor_msgs::PointCloud2> ("/intensity/height", 1);
    // curv_pub = nh.advertise<sensor_msgs::PointCloud2> ("/curvature_cloud2", 1);
    // curv_pub = nh.advertise<sensor_msgs::PointCloud2> ("/curvature_cloud", 1);
    // main handle
    ros::Rate loop_rate(20);
    while (ros::ok()){
		if(velodyne_flag){
			velodyne_flag = false;
			Intensity_threshold(tmp_cloud);
		}
        ros::spinOnce();
        loop_rate.sleep();
    }
}
