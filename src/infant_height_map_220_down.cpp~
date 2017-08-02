#include<ros/ros.h>
#include<math.h>
#include<iostream>
#include<vector>
#include<boost/thread.hpp>
#include<sensor_msgs/PointCloud.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/point_cloud_conversion.h>
#include<nav_msgs/Odometry.h>
#include<knm_tiny_msgs/Velocity.h>
#include<pcl_ros/point_cloud.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ? (x) : (y))
#define cell_size 0.1
#define full_clouds true
#define grid_dimentions 60
#define height_threshold 0.15
#define max_height 2.0
#define down 2


#define LIDAR_POINTS 1081
#define SAVE_NUM 80

using namespace std;
/////////////////////////////////////////////////
///////////// pc callback ///////////////////////
/////////////////////////////////////////////////
boost::mutex pc_mutex;
pcl::PointCloud<pcl::PointXYZ> pcl_in;
pcl::PointCloud<pcl::PointXYZ> pcl_down;
pcl::PointCloud<pcl::PointXYZ> pcl_out;
sensor_msgs::PointCloud pc_callback;
sensor_msgs::PointCloud2 ros_out_obstacle;
sensor_msgs::PointCloud2 ros_out_empty;
ros::Publisher pub_obstacle;
ros::Publisher pub_empty;
int t;
int pc_count = 0;

////////////////////////////////////////////////
//////////////// min_max ///////////////////////
///////////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZ> pcl_obstacle;
pcl::PointCloud<pcl::PointXYZ> pcl_empty;
int obstacle_count=0;
int empty_count=0;
bool save = false;

//////////////////////////////////////////////////
///////////// constract full clouds //////////////
//////////////////////////////////////////////////
void constructFullClouds()
{
        float min[grid_dimentions][grid_dimentions];
        float max[grid_dimentions][grid_dimentions];
        bool init[grid_dimentions][grid_dimentions];
        memset(&init,0,grid_dimentions*grid_dimentions);
        size_t npoints = pcl_in.points.size();

        //cout<<"build height map"<<endl;
        for(unsigned i=0;i<npoints;i++)
        {
                int x = ((grid_dimentions/2)+pcl_in.points[i].x/cell_size);
                int y = ((grid_dimentions/2)+pcl_in.points[i].y/cell_size);

                if( x >= 0 && x < grid_dimentions && y >= 0 && y < grid_dimentions){
                        if(!init[x][y]){
                                min[x][y]  = pcl_in.points[i].z;
                                max[x][y]  = pcl_in.points[i].z;
                                init[x][y] = true;
                        }
                        else{
                                min[x][y] = MIN(min[x][y],pcl_in.points[i].z);
                                max[x][y] = MAX(max[x][y],pcl_in.points[i].z);
                        }
                }
        }

        //cout<<"display poitns where map has height-difference > threshold"<<endl;
        for(unsigned i=0;i<npoints;i++)
        {
                //cout<<"i:"<<i<<endl;
                int x = ((grid_dimentions/2)+pcl_in.points[i].x/cell_size);
                int y = ((grid_dimentions/2)+pcl_in.points[i].y/cell_size);
                //int r = sqrt(pc_in_real.points[i].x*pc_in_real.points[i].x + pc_in_real.points[i].y*pc_in_real.points[i].y);

                if( x >= 0 && x < grid_dimentions && y >= 0 && y < grid_dimentions && init[x][y])
                {
                        if(( max[x][y]-min[x][y] > height_threshold) && ( max[x][y]-min[x][y] < 3.0 ))
                        {
                                pcl::PointXYZ p;
                                p.x=pcl_in.points[i].x;
                                p.y=pcl_in.points[i].y;
                                p.z=pcl_in.points[i].z;
                                pcl_obstacle.push_back(p);
                                obstacle_count++;
                        }
                        else{ 
                                pcl::PointXYZ q;
                                q.x=pcl_in.points[i].x;
                                q.y=pcl_in.points[i].y;
                                q.z=pcl_in.points[i].z;
                                pcl_empty.push_back(q);
                                empty_count++;
                        }
                }
        }
}
void pcCallback(const sensor_msgs::PointCloudConstPtr& msg)
{
        //////////////////////////////////////
        //////////// transform ///////////////
        //////////////////////////////////////
        boost::mutex::scoped_lock(pc_mutex);
        pc_callback = *msg;
        t = pc_count*LIDAR_POINTS;
       // cout<<pc_callback.points.size()<<endl;


        for(int i=0;i<pc_callback.points.size();i++){
                if(i%down==0)
                pcl_in.points[i/down+t/down].x = pc_callback.points[i/down].x;
                pcl_in.points[i/down+t/down].y = pc_callback.points[i/down].y;
                pcl_in.points[i/down+t/down].z = pc_callback.points[i/down].z;
        }

        if(save){
                constructFullClouds();
                pcl::toROSMsg(pcl_obstacle,ros_out_obstacle);
                pcl::toROSMsg(pcl_empty,ros_out_empty);
                ros_out_obstacle.header.frame_id="3dlaser";
                ros_out_empty.header.frame_id="3dlaser";
                pub_obstacle.publish(ros_out_obstacle);
                pub_empty.publish(ros_out_empty);
                if(pc_count==40){
                        cout<<"obstacle:"<<pcl_obstacle.points.size()<<endl;
                        cout<<"empty   :"<<pcl_empty.points.size()<<endl;
                }
                pcl_obstacle.points.clear();
                pcl_empty.points.clear();
        }


        ++pc_count;
        if(pc_count == SAVE_NUM){
                save=true;
                pc_count=0;
        }
}

int main(int argc,char** argv)
{
        ros::init(argc,argv,"infant_height_map_220_down");
        ros::NodeHandle n;

        ros::Subscriber sub = n.subscribe("eth_220/point_cloud_3",10,pcCallback);

        pub_obstacle   = n.advertise<sensor_msgs::PointCloud2>("point_cloud2/obstacle_220",10);
        pub_empty      = n.advertise<sensor_msgs::PointCloud2>("point_cloud2/empty_220",10);

        pcl_in.header.frame_id="3dlaser";
        pcl_in.points.resize(SAVE_NUM*LIDAR_POINTS/down);
        
        pcl_down.header.frame_id="3dlaser";
        pcl_down.points.resize(SAVE_NUM*LIDAR_POINTS/down);

        ros::Rate rate(40);

        while(ros::ok())
        {
                ros::spinOnce();
                rate.sleep();
        }
        return 0;
}
