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
// #include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
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

ros::Publisher gauss_pub;

//  output {{{
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
//}}}

inline void Copy_cloud(PointA& point_in,
					   PointA& point_out)
{
	// points_with_normal
	point_out.x = point_in.x;
	point_out.y = point_in.y;
	point_out.z = point_in.z;
	point_out.normal_x = point_in.normal_x;
	point_out.normal_y = point_in.normal_y;
	point_out.normal_z = point_in.normal_z;
	point_out.curvature = point_in.curvature;
	point_out.intensity = point_in.intensity;
	
	// gauss_shere
	// point_out.x = point_in.normal_x;
	// point_out.y = point_in.normal_y;
	// point_out.z = point_in.normal_z;
	// point_out.normal_x = point_in.normal_x;
	// point_out.normal_y = point_in.normal_y;
	// point_out.normal_z = point_in.normal_z;
	// point_out.curvature = point_in.curvature;
	// point_out.intensity = point_in.intensity;
}
inline void Visual(CloudAPtr cloud)
{
	size_t cloud_size = cloud->points.size();
	size_t count = 0;
	double para = 0.98;
	for(size_t i=0;i<cloud_size;i++){
        if (fabs(cloud->points[i].normal_z) >  para){
				count++;
		}
	}

	CloudAPtr output_cloud(new CloudA);
	output_cloud->points.resize(count);
	int output = 0;
	for(size_t i=0;i<cloud_size;i++){
        if (fabs(cloud->points[i].normal_z) >  para){
				Copy_cloud(cloud->points[i],output_cloud->points[output]);
				// cout<<cloud->points[i].x<<endl;
				// cout<<output_cloud->points[output].normal_z<<endl;
				// cout<<"aa"<<endl;
				output++;
		}
	}
    // output
    ros::Time time = ros::Time::now();
    pubPointCloud2 (gauss_pub, *output_cloud, "/gauss_cloud", time);
}
// Downsample {{{
inline void Downsample (CloudXPtr input_cloud, 
                        CloudXPtr downsampled,
                        float size)
{
    pcl::VoxelGrid<PointX> sor;
    sor.setInputCloud (input_cloud);
    sor.setLeafSize (size, size, size);
    sor.filter (*downsampled);
}//}}}

//  NormalEstimation {{{
inline void NormalEstimation   (CloudXPtr input_cloud,
                                pcl::PointCloud<pcl::Normal>:: Ptr output_normal,
                                float size)
{
    pcl::NormalEstimation<PointX, pcl::Normal> ne;
    ne.setInputCloud (input_cloud);
    pcl::search::KdTree<PointX>::Ptr tree(new pcl::search::KdTree<PointX>());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (size);
    ne.compute (*output_normal);
} // }}}

//callback
CloudXPtr tmp_cloud (new CloudX);
void velodyne_cb(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	pcl::fromROSMsg(*msg,*tmp_cloud);
	velodyne_flag = true;
}
int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "hsd_gauss");
    ros::NodeHandle nh;
    ros::NodeHandle n;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, velodyne_cb);
    // Create a ROS publisher for the output point cloud
    gauss_pub = nh.advertise<sensor_msgs::PointCloud2> ("/gauss_cloud", 1);
    // main handle
    ros::Rate loop_rate(40);
    while (ros::ok()){
        if (velodyne_flag){
			velodyne_flag = false;
			//downsampling
			CloudXPtr down_cloud(new CloudX);
			float down_size = 0.2;
			Downsample(tmp_cloud,down_cloud,down_size);

   			// NormalEstimation
   			pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
   			float normal_radius_size = 0.3;
   			NormalEstimation (down_cloud, cloud_normals, normal_radius_size);
   			CloudAPtr cloud_with_normals (new CloudA);
   			pcl::concatenateFields (*down_cloud, *cloud_normals, *cloud_with_normals);

			// visualize_points
			Visual(cloud_with_normals);
        } 
        ros::spinOnce();
        loop_rate.sleep();
    }
}
