//入ってきたdataに対してpeakを計算する
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
//vizualize
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

using namespace std;
using namespace Eigen;

typedef pcl::PointXYZINormal PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

bool callback_flag = false;
int devide = 720;
double threshold = 25.0;
// double threshold = 27.0;
// double threshold = 30.0;

ros::Publisher peak_pub;
ros::Publisher marker_pub;

//publish pointcloud
inline void pubPointCloud2(ros::Publisher& pub, 
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

inline double calc_distance(PointA& input)
{
	double dist = sqrt(input.x*input.x + input.y*input.y);
	// cout<<"distance:"<<dist<<endl;//for debug

	return dist;
}

void Calc_peak(CloudAPtr peak)
{
	size_t cloud_size = peak->points.size()-devide/18;
	size_t certification = 40;
	int count = 0;
	bool publish_flag = false;
	visualization_msgs::Marker line_list;
	line_list.header.frame_id = "/velodyne";
	line_list.header.stamp = ros::Time::now();
	line_list.ns = "points_and_lines";
 	line_list.action = visualization_msgs::Marker::ADD;
	line_list.pose.orientation.w = 1.0;
	line_list.id = 0;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.scale.x = 0.1;
	line_list.color.r = 1.0;
 	line_list.color.a = 1.0;

	for(size_t i=0;i<cloud_size;i++){
		if(peak->points[i].intensity >=threshold){
			count=0;
			size_t tmp_j = i;
			for(size_t j = tmp_j;j<tmp_j+certification;j++){
				// visualization_msgs::Marker line_list;
				// line_list.header.frame_id = "/velodyne";
				// line_list.header.stamp = ros::Time::now();
				// line_list.ns = "points_and_lines";
			 	// line_list.action = visualization_msgs::Marker::ADD;
				// line_list.pose.orientation.w = 1.0;
				// line_list.id = 0;
				// line_list.type = visualization_msgs::Marker::LINE_LIST;
				// line_list.scale.x = 0.1;
				// line_list.color.r = 1.0;
			 	// line_list.color.a = 1.0;
				if(peak->points[j].intensity >=threshold){
					count++;
					if(count==20){
						// cout<<"peak!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
						double tmp_i = i;
						double fif_mean = 0;
						while(tmp_i<devide){
							//五点平均が閾値に近い部分を一つのピークとする
							for(size_t k=tmp_i;k<tmp_i+5;k++){
								fif_mean += peak->points[k].intensity;
							}
							fif_mean = fif_mean/5.0;
							// cout<<"五点平均"<<fif_mean<<endl;
							if(fif_mean>=(threshold-3.0)){
								fif_mean = 0.0;
								// cout<<"start:"<<i<<endl;
								// cout<<"end:"<<tmp_i<<endl;
								if(tmp_i - i>=30){
									cout<<"its a load!!!!:"<<(double)i+(tmp_i-i)/2.0<<endl;
									// visualization_msgs::Marker line_list;
									// line_list.header.frame_id = "/velodyne";
									// line_list.header.stamp = ros::Time::now();
									// line_list.ns = "points_and_lines";
								 	// line_list.action = visualization_msgs::Marker::ADD;
									// line_list.pose.orientation.w = 1.0;
									// line_list.id = 0;
									// line_list.type = visualization_msgs::Marker::LINE_LIST;
									// line_list.scale.x = 0.1;
									// line_list.color.r = 1.0;
								 	// line_list.color.a = 1.0;
									geometry_msgs::Point p;
									p.x = 0.0;
									p.y = 0.0;
									p.z = 0.0;
									line_list.points.push_back(p);
									p.x  = peak->points[(int)(i + (tmp_i - i)/2.0)].x;
									p.y  = peak->points[(int)(i + (tmp_i - i)/2.0)].y;
									p.z  = peak->points[(int)(i + (tmp_i - i)/2.0)].z;
									line_list.points.push_back(p);
									// marker_pub.publish(line_list);
									publish_flag = true;
								}
								i = tmp_i+5;
								j = tmp_i+5;
								// break;
							}
							tmp_i += 1;
							fif_mean = 0.0;

							if(tmp_i==devide){
								cout<<"start:"<<i<<endl;
								cout<<"end:"<<tmp_i<<endl;
								if(tmp_i - i>=30){
									cout<<"its a load!!!!:"<<(double)i+(tmp_i-i)/2.0<<endl;
									// visualization_msgs::Marker line_list;
									// line_list.header.frame_id = "/velodyne";
									// line_list.header.stamp = ros::Time::now();
									// line_list.ns = "points_and_lines";
								 	// line_list.action = visualization_msgs::Marker::ADD;
									// line_list.pose.orientation.w = 1.0;
									// line_list.id = 0;
									// line_list.type = visualization_msgs::Marker::LINE_LIST;
									// line_list.scale.x = 0.1;
									// line_list.color.r = 1.0;
								 	// line_list.color.a = 1.0;
									geometry_msgs::Point p;
									p.x = 0.0;
									p.y = 0.0;
									p.z = 0.0;
									line_list.points.push_back(p);
									p.x  = peak->points[(int)(i + (tmp_i - i)/2.0)].x;
									p.y  = peak->points[(int)(i + (tmp_i - i)/2.0)].y;
									p.z  = peak->points[(int)(i + (tmp_i - i)/2.0)].z;
									line_list.points.push_back(p);
									// marker_pub.publish(line_list);
									publish_flag = true;
								}
								i= tmp_i;
								j= tmp_i;
								// break;
							}
						}
					}
				}
			}
		}
	}
	if(publish_flag){
		marker_pub.publish(line_list);
	}
}

void Detect_peak(CloudAPtr shape_cloud)
{
	size_t cloud_size = shape_cloud->points.size();
	size_t tmp_size = cloud_size + devide/18;

	double distance = 0.0;
	CloudAPtr extend_shape(new CloudA);
	//peak detectする前にデータの繰り返しを行いデータの不連続面に対してもpeak detectを行う
	//更にデータに対して最大値を儲ける
	for(size_t i = 0;i<tmp_size;i++){
		extend_shape->push_back(shape_cloud->points[i%cloud_size]);
		distance = calc_distance(extend_shape->points[i]);
		if(distance > threshold){
			extend_shape->points[i].intensity = threshold;
		}else{
			extend_shape->points[i].intensity = distance;
		}
	}
	//Peak を検出する
	Calc_peak(extend_shape);

}


//callback
CloudAPtr tmp_cloud (new CloudA);
void Shape_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	pcl::fromROSMsg(*msg,*tmp_cloud);
	callback_flag = true;
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "detect_peak");
    ros::NodeHandle nh;
    ros::NodeHandle n;

    // Create a ROS subscriber for the input point cloud
    // ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, velodyne_cb);
    ros::Subscriber sub = nh.subscribe ("/detect_shape", 1, Shape_callback);
    // Create a ROS publisher for the output point cloud
    peak_pub = nh.advertise<sensor_msgs::PointCloud2> ("/detect_peak", 1);
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    // main handle
    ros::Rate loop_rate(20);
    while (ros::ok()){
		if(callback_flag){
			Detect_peak(tmp_cloud);
			callback_flag = false;
		}
        ros::spinOnce();
        loop_rate.sleep();
    }
}
