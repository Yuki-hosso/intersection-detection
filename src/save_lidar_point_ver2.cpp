#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include<sensor_msgs/PointCloud2.h>
#include <boost/thread.hpp>

#define PI 3.141592
#define UNIT_SIZE 1080

using namespace std;

typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> CloudI;

CloudI unit;
boost::mutex lrf_mutex;
bool callback_flag2=false;

void ScanCallback(const sensor_msgs::PointCloud2& msg)
{
	// boost::mutex::scoped_lock(lrf_mutex);
  	pcl::fromROSMsg(msg, unit);
	callback_flag2=true;	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "save_lidar_point");
	ros::NodeHandle n;
	
	ros::Publisher saved_pt_pub=n.advertise<sensor_msgs::PointCloud2>("/lidar3d/point_cloud", 1);
	// ros::Subscriber scan_sub= n.subscribe("/lidarpoint_to_ver2", 1, ScanCallback);
	// ros::Subscriber scan_sub= n.subscribe("/lidarpoint/unit_points", 1, ScanCallback);
	ros::Subscriber scan_sub= n.subscribe("/lidarpoint_lrf1", 1, ScanCallback);
	ros::Subscriber scan2_sub= n.subscribe("/lidarpoint_lrf2", 1, ScanCallback);
	ros::Subscriber scan3_sub= n.subscribe("/lidarpoint_lrf3", 1, ScanCallback);

  	CloudI saved_pt;//to save the number of 1cycle points
//	double hz= 40.0;
	size_t default_size = UNIT_SIZE * 280;
  	saved_pt.points.resize(default_size);
  	double target_num = default_size;
  	
	ros::Rate rate(40);
  	while (ros::ok()){
		//main process
		if (callback_flag2){
			//shift saved points
        	int end_id=saved_pt.points.size();
        	int start_id=end_id-UNIT_SIZE;
        	while(start_id>0){
        	  for (int i=start_id; i<end_id; ++i)
        	    saved_pt.points[i]=saved_pt.points[i-UNIT_SIZE];
        	  	end_id-=UNIT_SIZE;
        	  	start_id-=UNIT_SIZE;
        	}
			//get the current points
        	for (int i=0; i<UNIT_SIZE; ++i)
        	  saved_pt.points[i]=unit.points[i];

        	if (saved_pt.points.size()==target_num){
        	  //publish msg//
        	  sensor_msgs::PointCloud2 saved_ros;
	    	  toROSMsg(saved_pt, saved_ros);
	    	  saved_ros.header.frame_id="/centerlaser";
	    	  saved_pt_pub.publish(saved_ros);
        	  //release memory//
        	  // CloudI sw_pt;
        	  // sw_pt.swap(saved_pt);
        	}
       
        	//reset flag//
        	callback_flag2=false;
		}
		rate.sleep();
		ros::spinOnce();
	}
	
	return 0;
}
