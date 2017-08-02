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

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "otamesi");
    ros::NodeHandle nh;
    ros::NodeHandle n;

    // main handle
	size_t cloud_size = 20;

	CloudA tmp_cloud;
	tmp_cloud.points.resize(cloud_size);

	cout<<tmp_cloud.points[0].x<<endl;
	cout<<tmp_cloud.points[10].x<<endl;

	return 0;
}
