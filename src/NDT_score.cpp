#include <ros/ros.h>
#include <tf/tf.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <geometry_msgs/Quaternion.h>
#include <sys/time.h>

//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

//msgs
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>

#include "ray_casting.h"
using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data (new pcl::PointCloud<pcl::PointXYZ>);
sensor_msgs::PointCloud2 cloud2;

bool score_flag = false;
bool realtime_flag = false;
bool gauss_pose_flag = false;
nav_msgs::GridCells obstacle_cell;
nav_msgs::OccupancyGrid obstacle_map;
nav_msgs::OccupancyGrid local_map;
nav_msgs::OccupancyGrid realtime_map;
nav_msgs::Odometry gauss_pose;
std_msgs::Float32 score_msg;

ros::Publisher score_pub;

void PointXYZ_to_PointCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr *input, sensor_msgs::PointCloud2 *output){
	pcl::PCLPointCloud2 pcl_cloud2;
	pcl::toPCLPointCloud2(**input, pcl_cloud2);
	pcl_conversions::fromPCL(pcl_cloud2, *output);
}

void PointCloud2_to_PointXYZ(sensor_msgs::PointCloud2 *input, pcl::PointCloud<pcl::PointXYZ>::Ptr *output){
	pcl::PCLPointCloud2 pcl_cloud2;
	pcl_conversions::toPCL(*input, pcl_cloud2);
	pcl::fromPCLPointCloud2(pcl_cloud2, **output);
}

float rotation_x(float x, float y, float theta){
	float x_r;
	x_r = cos(theta) * x - sin(theta) * y;

	return x_r;
}

float rotation_y(float x, float y, float theta){
	float y_r;
	y_r = sin(theta) * x + cos(theta) * y;

	return y_r;
}

void create_obstacle_map(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::OccupancyGrid *map){
	geometry_msgs::Point obstacle_point;
	vector<int>	count((long(800/0.1) * long(800/0.1)), 0);
	map->data.resize(int(800 / 0.1) * int(800 / 0.1));
	map->info.width = int(800 / 0.1);
	map->info.height = int(800 / 0.1);
	map->info.resolution = 0.1;
	map->info.origin.position.x = (0.0 - 800) / 2.0;
	map->info.origin.position.y = (0.0 - 800) / 2.0;

	for(unsigned int i = 0; i != map->data.size(); i++){
		map->data[i] = 0;
	}
	for(size_t i = 0; i < cloud->points.size(); i++){
		int x = int((cloud->points[i].x - map->info.origin.position.x) / 0.1);
		int y = int((cloud->points[i].y - map->info.origin.position.y) / 0.1);

		if((0 <= x && x < 800/0.1) && (0 <= y && y < 800/0.1)){
			long num = x + y * map->info.width;
			count[num] += 1; 
		}
	}

	for(unsigned int i = 0;i<count.size(); i++){
		if(count[i] > 10){
			map->data[i] = 100;
			obstacle_point.x = (i % map->info.width) * 0.1 + map->info.origin.position.x;
			obstacle_point.y = int(i / map->info.width) * 0.1 + map->info.origin.position.y;
			obstacle_point.z = 0;
			
			obstacle_cell.cells.push_back(obstacle_point);
		}
	}

}

void create_local_map(float x, float y, float theta, nav_msgs::GridCells obs_grid, nav_msgs::OccupancyGrid* loc_map){
	RayCasting rc;
	pcl::PointCloud<pcl::PointXYZ>::Ptr static_points (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr static_points_rc (new pcl::PointCloud<pcl::PointXYZ>);
	nav_msgs::OccupancyGrid out;
	nav_msgs::OccupancyGrid out_rc;
	vector<int8_t>::iterator mit;
	
	out = *loc_map;
	out_rc = *loc_map;
	
	for(mit = out.data.begin(); mit != out.data.end(); mit++){
		*mit = 0;
	}
	for(size_t i = 0; i < obs_grid.cells.size(); i++){
		int xi = int((rotation_x(obs_grid.cells[i].x-x, obs_grid.cells[i].y-y, -1*(theta-M_PI/2)) - out.info.origin.position.x) / 0.1);
		int yi = int((rotation_y(obs_grid.cells[i].x-x, obs_grid.cells[i].y-y, -1*(theta-M_PI/2)) - out.info.origin.position.y) / 0.1);

		if((0 <= xi && xi < 20.0/0.1) && (0 <= yi && yi < 20.0/0.1)){
			int num = xi + yi * out.info.width;
			out.data[num] = 100;
		}
		
	}
	
	for(unsigned int i=0; i<out.data.size(); i++){
		if(out.data[i] != 0){
			pcl::PointXYZ temp_point;
			temp_point.x = (int)(i%out.info.width)*out.info.resolution + out.info.origin.position.x;
			temp_point.y = (int)(i/out.info.width)*out.info.resolution + out.info.origin.position.y;
			temp_point.z = 0.0;
			static_points->points.push_back(temp_point);
		}
		out_rc.data[i] = 0;
	}
	rc.setResolution(0.01*M_PI/180.0);
	rc.Cast(*static_points);
	rc.getResult(*static_points_rc);
	
	for(unsigned int j=0; j<static_points_rc->points.size(); j++){
		int x = (int)( (static_points_rc->points[j].x - out_rc.info.origin.position.x)/0.1 );
		int y = (int)( (static_points_rc->points[j].y - out_rc.info.origin.position.y)/0.1 );
		if( (0<=x && x<20.0/0.1) && (0<=y && y<20.0/0.1) ){
			int num = x+y*(int)(out.info.width);
			out_rc.data[num] = 100;
		}
	}

	*loc_map = out_rc;
}

void create_realtime_rc_map(nav_msgs::OccupancyGrid map){
	RayCasting rc;
	pcl::PointCloud<pcl::PointXYZ>::Ptr real_points (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr real_points_rc (new pcl::PointCloud<pcl::PointXYZ>);
	
	for(unsigned int i=0; i<map.data.size(); i++){
		if(map.data[i] != 0){
			pcl::PointXYZ temp_point;
			temp_point.x = (int)(i%map.info.width)*map.info.resolution + map.info.origin.position.x;
			temp_point.y = (int)(i/map.info.width)*map.info.resolution + map.info.origin.position.y;
			temp_point.z = 0.0;
			real_points->points.push_back(temp_point);
		}
		realtime_map.data[i] = 0;
	}
	rc.setResolution(0.5*M_PI/180.0);
	rc.Cast(*real_points);
	rc.getResult(*real_points_rc);

	for(unsigned int j=0; j<real_points_rc->points.size(); j++){
		int x = (int)( (real_points_rc->points[j].x - map.info.origin.position.x)/0.1 );
		int y = (int)( (real_points_rc->points[j].y - map.info.origin.position.y)/0.1 );
		if( (0<=x && x<20.0/0.1) && (0<=y && y<20.0/0.1) ){
			int num = x+y*(int)(map.info.width);
			realtime_map.data[num] = 100;
		}
	}
	
}

void file_write(float score)
{
//	printf("map size : %d\n", (int)(local_map.data.size()) );
	FILE *file;
	file = fopen("/home/amsl/Dgauss_score_data.csv", "a");
	fprintf(file, "%f, %f, %f\n", gauss_pose.pose.pose.position.x, gauss_pose.pose.pose.position.y, score);
	fclose(file);
}

float calcScore(void){
	float score =0.0;	
	int match_cell = 0;
	int obs_cell = 0;
	int total_cell = (int)(local_map.data.size());
	
	create_local_map(gauss_pose.pose.pose.position.x, gauss_pose.pose.pose.position.y, gauss_pose.pose.pose.orientation.z,
					 obstacle_cell, &local_map);
	for(int j=0; j<total_cell; j++){
		if(realtime_map.data[j] == 100){
			if(local_map.data[j] == 100){
				match_cell++;
			}
			obs_cell++;
		}
	}
	score = ( (float)match_cell / (float)obs_cell );
//	printf("match : %d, total : %d, Score : %f\n",match_cell, total_cell, score);
//	file_write(score);
	
	return score;
}

void ndtCallback(nav_msgs::Odometry msg){
	score_flag = true;
	if(realtime_flag && gauss_pose_flag){
		score_msg.data = calcScore();
		printf("NDT score : %f\n", score_msg.data);
		score_pub.publish(score_msg);
	}
}

void realtimeMapCallback(nav_msgs::OccupancyGrid msg){
//	realtime_map.header = msg.header;
//	create_realtime_rc_map(msg);
	realtime_map = msg;
	realtime_flag = true;
}

void ndtOdomCallback(nav_msgs::Odometry msg){
	gauss_pose.header.stamp = msg.header.stamp;
	gauss_pose.pose = msg.pose;
	gauss_pose_flag = true;
}

void Init(void){
	obstacle_cell.header.frame_id = "/map";
	obstacle_cell.cell_width = 0.1;
	obstacle_cell.cell_height = 0.1;
	obstacle_map.header.frame_id = "/map";
	realtime_map.header.frame_id = "velodyne";
	realtime_map.data.resize(int(20.0 / 0.1) * int(20.0 / 0.1));
	realtime_map.info.resolution = 0.1;
	realtime_map.info.width = int(20.0 / 0.1);
	realtime_map.info.height = int(20.0 / 0.1);
	realtime_map.info.origin.position.x = (0.0-20.0)/2.0;
	realtime_map.info.origin.position.y = (0.0-20.0)/2.0;
	local_map.header.frame_id = "/velodyne";
	local_map.data.resize(int(20.0 / 0.1) * int(20.0 / 0.1));
//	printf("map size : %d\n", (int)(local_map.data.size()) );
	local_map.info.resolution = 0.1;
	local_map.info.width = int(20.0 / 0.1);
	local_map.info.height = int(20.0 / 0.1);
	local_map.info.origin.position.x = (0.0-20.0)/2.0;
	local_map.info.origin.position.y = (0.0-20.0)/2.0;
	gauss_pose.header.frame_id = "/map";
}

int main(int argc, char** argv){
	ros::init(argc, argv, "NDT_score");
	ros::NodeHandle n;
	
	ros::Subscriber flag_sub = n.subscribe("/lcl_ndt", 1, ndtCallback);
	ros::Subscriber real_sub = n.subscribe("/local_map_real", 1, realtimeMapCallback);
	ros::Subscriber gauss_sub = n.subscribe("/lcl4", 1, ndtOdomCallback);
	
//	ros::Publisher real_pub = n.advertise<nav_msgs::OccupancyGrid>("/real_map_debug", 1);
//	ros::Publisher stat_pub = n.advertise<nav_msgs::OccupancyGrid>("/static_map_debug", 1);
//	ros::Publisher cell_pub = n.advertise<nav_msgs::GridCells>("/static_cell_debug", 1);
//	ros::Publisher pose_pub = n.advertise<nav_msgs::Odometry>("/gauss_pose_debug", 1);
	score_pub = n.advertise<std_msgs::Float32>("/NDTScore", 1);
	
	struct timeval start_time, finish_time;
	
	printf("Set up now...\n");
	
	string filename = "/home/amsl/obs_map_curvature_after_crcl.pcd";
	if(pcl::io::loadPCDFile (filename, *cloud_data) == -1){
		PCL_ERROR("Couldn't read file '%s'", filename.c_str());
		return -1;
	}
	
	Init();
	
	create_obstacle_map(cloud_data, &obstacle_map);
	obstacle_cell.header.stamp = ros::Time::now();
//	cell_pub.publish(obstacle_cell);
	
	printf("Set up Ok!\n");
	ros::Rate loop_rate(10);
	while(ros::ok()){
//		printf("score_flag: %d, realtime_flag: %d, gauss_pose_flag: %d\n", score_flag, realtime_flag, gauss_pose_flag);
//		if(score_flag && realtime_flag && gauss_pose_flag){
		if(realtime_flag && gauss_pose_flag){
			gettimeofday(&start_time, NULL);
			score_msg.data = calcScore();
			printf("NDT score : %f\n", score_msg.data);
			gettimeofday(&finish_time, NULL);
//			printf("	Duration of prcessing : %f\n", (finish_time.tv_sec - start_time.tv_sec) + (finish_time.tv_usec - start_time.tv_usec)*1e-6);
//			pose_pub.publish(gauss_pose);
			score_flag = false;
			realtime_flag = false;
			gauss_pose_flag = false;
			score_pub.publish(score_msg);
//			real_pub.publish(realtime_map);
//			stat_pub.publish(local_map);
		}
// Debug 
//create_local_map(ekf_pose.pose.pose.position.x, ekf_pose.pose.pose.position.y, ekf_pose.pose.pose.orientation.z, obstacle_cell, &local_map);
//local_map.header.stamp = ros::Time::now();
//real_pub.publish(realtime_map);
//stat_pub.publish(local_map);
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
