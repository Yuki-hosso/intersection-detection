/*
 * velodyneの点群をGyro odometryに付加させ
 * 数秒間貯める
 * 
 * 
 * 
 *
*/

#include <stdio.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <omp.h>


using namespace Eigen;
using namespace std;	

typedef pcl::PointXYZINormal PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

bool odom_callback = false;
bool points_callback = false;
double d_x=0.0, d_y=0.0, d_z=0.0;
double angle_x_=0.0, angle_y_=0.0, angle_z_=0.0;
int loop = 15;
// int SAVE_SIZE = 10000;
int SAVE_SIZE = 15000;

ros::Publisher shape_pub;

//publish pointcloud
inline void pubPointCloud2(ros::Publisher& pub, 
                            const CloudA& cloud,
                            const char* frame_id,
                            ros::Time& time)
{
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id ="map";
    output.header.stamp = time;
    pub.publish (output);
}

inline void Copy_point(PointA& org,PointA& save)
{
	save.x = org.x;
	save.y = org.y;
	save.z = org.z;
	save.normal_x = org.normal_x;
	save.normal_y = org.normal_y;
	save.normal_z = org.normal_z;
	save.intensity = org.intensity;
	save.curvature = org.curvature;
}

void cnv(CloudAPtr org, CloudAPtr rsl, 
		 double dx, double dy, double dz, double angle_x, double angle_y, double angle_z)
{
	// cout << "normal conv" << endl;
	pcl::PointNormal p;

	Vector3d point_in, point_out, axis_x, axis_y ,axis_z;
	Quaterniond quat_X, quat_Y, quat_Z;
	axis_x << 1.0, 0.0, 0.0;
	axis_y << 0.0, 1.0, 0.0;
	axis_z << 0.0, 0.0, 1.0;

	// quat_X = AngleAxisd(angle_x/180.0*M_PI, axis_x); // x軸を中心に回転するquat作成
	// quat_Y = AngleAxisd(angle_y/180.0*M_PI, axis_y); // y軸
	// quat_Z = AngleAxisd(angle_z/180.0*M_PI, axis_z); // z軸
	quat_X = AngleAxisd(angle_x, axis_x); // x軸を中心に回転するquat作成
	quat_Y = AngleAxisd(angle_y, axis_y); // y軸
	quat_Z = AngleAxisd(angle_z, axis_z); // z軸
	Quaterniond total_quat;
	total_quat = quat_Z * quat_Y * quat_X;
	
	size_t SIZE = org->points.size();
	rsl->points.resize(SIZE);

	for(size_t i=0; i<SIZE; i++){	
		point_in << org->points[i].x, org->points[i].y, org->points[i].z;
        
		//回転
		point_out = total_quat * point_in;
        
		//並進
		rsl->points[i].x = point_out(0) + dx;
		rsl->points[i].y = point_out(1) + dy;
		rsl->points[i].z = point_out(2) + dz;
		
		rsl->points[i].curvature = org->points[i].curvature;
	}
	// cout<<"moved!!\n";
}

//callback
CloudAPtr tmp_cloud (new CloudA);
void static_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	pcl::fromROSMsg(*msg,*tmp_cloud);
	points_callback = true;
}

void OdomCallback(const nav_msgs::Odometry input){
	d_x = input.pose.pose.position.x;
	d_y = input.pose.pose.position.y;
	d_z = 0.0;
	angle_z_ = input.pose.pose.orientation.z;

	odom_callback = true;
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "save_velodyne_points");
  	ros::NodeHandle n;
    ros::NodeHandle nh;
	
	size_t max_size = 0;

    // Create a ROS subscriber for the input point cloud
    // ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, velodyne_cb);
    ros::Subscriber sub = nh.subscribe ("/local_cloud", 1, static_callback);
	ros::Subscriber sub_lcl = n.subscribe("/lcl2",1,OdomCallback);
    // Create a ROS publisher for the output point cloud
    shape_pub = nh.advertise<sensor_msgs::PointCloud2> ("/save_cloud", 1);

	// CloudAPtr conv_cloud (new CloudA);
	CloudAPtr conv_cloud (new CloudA);
	CloudAPtr save_cloud (new CloudA);
	save_cloud->resize(SAVE_SIZE * loop);
	int save_count = 0;
	ros::Rate loop_rate(20); // 20
	// ros::Rate loop_rate(30); // 20
	while (ros::ok()){
		if(odom_callback && points_callback){
			// cout<<"in!!!!"<<endl;
			odom_callback = false;
			points_callback = false;
			cnv(tmp_cloud, conv_cloud, d_x, d_y, d_z, angle_x_, angle_y_, angle_z_);
			CloudA pub_cloud;
			size_t cloud_size = conv_cloud->points.size();
			if(max_size<cloud_size){
				cout<<cloud_size<<endl;
				max_size = cloud_size;
			}
			// for(size_t i=0;i<cloud_size;i++){
			// 	pub_cloud.push_back(conv_cloud->points[i]);
			// }
			/////////////////////save_cloud/////////////////////////
			for(size_t i=0;i<cloud_size;i++){
				Copy_point(conv_cloud->points[i],save_cloud->points[SAVE_SIZE*save_count+i]);
			}
			PointA surplus;
			surplus.x = 0;
			surplus.y = 0;
			surplus.z = 0;
			#pragma omp parallel for
			for(size_t i = SAVE_SIZE*save_count+cloud_size;i<SAVE_SIZE*(save_count+1);i++){
				Copy_point(surplus,save_cloud->points[i]);
			}
			///////////////////save_cloud/////////////////////////
			for(size_t i=0;i<SAVE_SIZE*loop;i++){
				pub_cloud.push_back(save_cloud->points[i]);
			}
			save_count++;
			save_count = save_count%loop;
			ros::Time time = ros::Time::now();
			pubPointCloud2(shape_pub,pub_cloud,"/aaaa",time);
		}
        ros::spinOnce();
        loop_rate.sleep();
	}
	return (0);
}

