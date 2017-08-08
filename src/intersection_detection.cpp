//detect_peak_visu →  intersection_detection
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
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/passthrough.h>
//
#include<std_msgs/Int32MultiArray.h>
#include<geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

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
bool odom_callback = false;

const int loop_count = 10;
// int loop_count = 20;
int degree_count = 720;

int save_number = loop_count*degree_count;
int save_count = 0;


// int save_peak[10*720];
int save_peak[loop_count*4];
// int save_peak[20*720];

// void EM_algorithm(int* data)
// {
// 	for(int i=0;i<degree_count;i++){
// 		cout<<i<<","<<data[i]<<endl;
// 	}
// }

// void Intersection_detection(std_msgs::Int32MultiArray &input)
// {
// 	for(int i =0;i<degree_count;i++){
// 		save_peak[save_count*degree_count+i] = 0;
// 	}
// 	if(input.layout.data_offset!=100){
// 		for(int i=0;i<input.layout.data_offset;i++){
// 			save_peak[save_count*degree_count + input.data[i]] = 1;
// 			// cout<<"here!"<<endl;
// 		}
// 	}
//
// 	int integrate_peak[720];
// 	//init
// 	for(int i=0;i<degree_count;i++){
// 		integrate_peak[i] = 0;
// 	}
// 	for(int i=0;i<degree_count;i++){
// 		for(int j=0;j<loop_count;j++){
// 			integrate_peak[i] += save_peak[j*degree_count + i];
// 		}
// 	}
//
// 	if(screen_flag==false){
// 		if(save_count==loop_count-1){
// 			screen_flag = true;
// 		}
// 	}
// 	if(screen_flag){
// 		for(int i=0;i<degree_count;i++){
// 			cout<<i<<","<<integrate_peak[i]<<endl;
// 		}
// 	}
//
// 	save_count++;
// 	save_count = save_count%loop_count;
// }

inline void INIT()
{
	for(int i=0;i<loop_count*4;i++){
		save_peak[i] = 0;
	}
}


bool one_flag = false;
bool two_flag = false;
bool three_flag = false;
bool four_flag = false;
bool insc_flag = false;
void Intersection_detection(std_msgs::Int32MultiArray &peak,geometry_msgs::Pose &odom)
{
	for(int i=0;i<peak.layout.data_offset;i++){
		if(peak.data[i]<30||peak.data[i]>690){
			one_flag = true;
		}else if(peak.data[i]>150&&peak.data[i]<210){
			two_flag = true;
		}else if(peak.data[i]>330&&peak.data[i]<390){
			three_flag = true;
		}else if(peak.data[i]>510&&peak.data[i]<570){
			four_flag = true;
		}
	}
	if(one_flag){
		save_peak[(save_count%loop_count)*4] = 1;
		one_flag = false;
		insc_flag = true;
	}else{
		save_peak[(save_count%loop_count)*4] = 0;
	}
	if(two_flag){
		save_peak[(save_count%loop_count)*4+1] = 1;
		two_flag = false;
		insc_flag = true;
	}else{
		save_peak[(save_count%loop_count)*4+1] = 0;
	}
	if(three_flag){
		save_peak[(save_count%loop_count)*4+2] = 1;
		three_flag = false;
		insc_flag = true;
	}else{
		save_peak[(save_count%loop_count)*4+2] = 0;
	}
	if(four_flag){
		save_peak[(save_count%loop_count)*4+3] = 1;
		four_flag = false;
		insc_flag = true;
	}else{
		save_peak[(save_count%loop_count)*4+3] = 0;
	}

	////////////shape-intersection//////////////////
	if(insc_flag){
		int tmp[4] = {0,0,0,0};
		if(save_count>10){
			for(int i=0;i<loop_count*4;i++){
				if(i%4==0){
					if(save_peak[i]==1){
						tmp[0]+=1;
					}
				}
				if(i%4==1){
					if(save_peak[i]==1){
						tmp[1]+=1;
					}
				}
				if(i%4==2){
					if(save_peak[i]==1){
						tmp[2]+=1;
					}
				}
				if(i%4==3){
					if(save_peak[i]==1){
						tmp[3]+=1;
					}
				}
			}
		}
		if(tmp[1]>5||tmp[3]>5){
			cout<<"intersection!!!!!!!!!!!!!!!!!!!!"<<endl;
		cout<<tmp[0]<<endl;
		cout<<tmp[1]<<endl;
		cout<<tmp[2]<<endl;
		cout<<tmp[3]<<endl;
			INIT();
		}
		if(tmp[0]>5||tmp[2]>5){
			cout<<"load!"<<endl;
		cout<<tmp[0]<<endl;
		cout<<tmp[1]<<endl;
		cout<<tmp[2]<<endl;
		cout<<tmp[3]<<endl;
		}
		insc_flag = false;
	}else{
			cout<<"nooooooooooooooooooooo"<<endl;
		save_peak[(save_count%loop_count)*4] = 0;
		save_peak[(save_count%loop_count)*4+1] = 0;
		save_peak[(save_count%loop_count)*4+2] = 0;
		save_peak[(save_count%loop_count)*4+3] = 0;
			INIT();
	}
	save_count++;
}

//callback
std_msgs::Int32MultiArray peak_in;
void Peak_deg(const std_msgs::Int32MultiArray::Ptr &msg)
{
	peak_in.data.clear();
	peak_in.layout.data_offset = msg->layout.data_offset;
	if(peak_in.layout.data_offset!=100){
		for(int i=0;i<peak_in.layout.data_offset;i++){
			peak_in.data.push_back(msg->data[i]);
		}
		callback_flag = true;
		// cout<<"peak_callback"<<endl;
	}
}

geometry_msgs::Pose odom;
void OdomCallback(const nav_msgs::OdometryConstPtr& input){
	odom.position.x = input->pose.pose.position.x;
	odom.position.y = input->pose.pose.position.y;
	odom.position.z = input->pose.pose.position.z;
	odom.orientation.z = input->pose.pose.orientation.z;
	// cout<<"odom_callback"<<endl;

	odom_callback = true;
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "intersection_detection");
    ros::NodeHandle nh;
    ros::NodeHandle n;

    // Create a ROS subscriber for the input point cloud
    // ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, velodyne_cb);
    // ros::Subscriber sub = nh.subscribe ("/peak/deg", 1, Peak_deg);
    ros::Subscriber sub = nh.subscribe ("/peak/deg2", 1, Peak_deg);
	ros::Subscriber sub_lcl = n.subscribe("/lcl2",1,OdomCallback);
    // Create a ROS publisher for the output point cloud

	//init
	// for(int i=0;i<loop_count*4;i++){
	// 	save_peak[i] = 0;
	// }
	INIT();
	cout<<"init"<<endl;
	//
    // main handle
    ros::Rate loop_rate(20);
    while (ros::ok()){
		if(callback_flag && odom_callback){
			callback_flag = false;
			odom_callback = false;
			Intersection_detection(peak_in,odom);
		}
        ros::spinOnce();
        loop_rate.sleep();
    }
}
