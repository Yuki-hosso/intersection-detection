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

ros::Publisher gauss_pub;
ros::Publisher esc_pub;

//  output {{{
inline void pubPointCloud2 (ros::Publisher& pub, 
                            const pcl::PointCloud<pcl::PointNormal>& cloud,
                            const char* frame_id,
                            ros::Time& time)
{
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id ="camera_depth_frame";
    output.header.stamp = time;
    pub.publish (output);
}
//}}}

// Downsample {{{
inline void Downsample (pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, 
                        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled,
                        float size)
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (input_cloud);
    sor.setLeafSize (size, size, size);
    sor.filter (*downsampled);
}//}}}

//  NormalEstimation {{{
inline void NormalEstimation   (pcl::PointCloud<pcl::PointXYZ>:: Ptr input_cloud,
                                pcl::PointCloud<pcl::Normal>:: Ptr output_normal,
                                float size)
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (input_cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (size);
    ne.compute (*output_normal);
} // }}}
/*
// Gauss Convert {{{
void gauss_point_convert (pcl::PointCloud<pcl::PointNormal>::Ptr point,
                          pcl::PointCloud<pcl::PointNormal>::Ptr gauss)
{
    gauss.x = point.normal_x;
    gauss.y = point.normal_y;
    gauss.z = point.normal_z;
    gauss.normal_x = point.x;
    gauss.normal_y = point.y;
    gauss.normal_z = point.z;
}
// }}}*/

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*cloud_msg, *cloud);
    
    // Downsample
    float downsampled_size = 0.05;
    Downsample (cloud, cloud_downsampled, downsampled_size);

    // Passthrough filter
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_downsampled);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-1.0, 0.4);
    pass.filter(*cloud_passthrough);

    
    // NormalEstimation
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    float normal_radius_size = 0.1;
    NormalEstimation (cloud_passthrough, cloud_normals, normal_radius_size);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud_passthrough, *cloud_normals, *cloud_with_normals);

    // Gauss Sphere {{{
    pcl::PointCloud<pcl::PointNormal>::Ptr gauss_cloud (new pcl::PointCloud<pcl::PointNormal>);
    size_t cloud_size = cloud_normals->points.size();
    pcl::PointNormal gauss;
    
    //gauss_cloud->points.resize(cloud_size);
    int point_count = 0;
    
    pcl::PointCloud<pcl::PointNormal>::Ptr slanting_gauss (new pcl::PointCloud<pcl::PointNormal>);
    slanting_gauss->points.resize(cloud_size);

    for (size_t i=0; i<cloud_size; i++)
    {
        if (fabs(cloud_with_normals->points[i].normal_z) >  0.92)
        {
            gauss.x = cloud_with_normals->points[i].normal_x;
            gauss.y = cloud_with_normals->points[i].normal_y;
            if (cloud_with_normals->points[i].normal_z < 0.0)
            {
                gauss.z = - cloud_with_normals->points[i].normal_z; 
            } else {
                gauss.z =  cloud_with_normals->points[i].normal_z;
            }
            
            if (gauss.z==0){
                std::cout<<"Cannot exclude zero"<<std::endl;
            }
            gauss.normal_x = cloud_with_normals->points[i].x;  
            gauss.normal_y = cloud_with_normals->points[i].y;  
            gauss.normal_z = cloud_with_normals->points[i].z;
            
            if (cloud_with_normals->points[i].normal_z < -0.6/*  && 
                cloud_with_normals->points[i].normal_z < 0.96 &&
                cloud_with_normals->points[i].normal_x > 0*/)
            {
                slanting_gauss->points[i] = gauss;
            }

            point_count++;
            //gauss_cloud->points[i] = gauss;
            gauss_cloud->push_back(gauss);
        }
    }

    std::cout << "gauss_cloud message: " << cloud_size << std::endl;
//}}}

   // Horizontal Cloud {{{ 
    pcl::PointCloud<pcl::PointNormal>::Ptr horizontal_cloud (new pcl::PointCloud<pcl::PointNormal>);
    cloud_size = gauss_cloud->points.size();
    horizontal_cloud->points.resize (cloud_size);
    
    
    pcl::PointCloud<pcl::PointNormal>::Ptr slanting_cloud (new pcl::PointCloud<pcl::PointNormal>);
    slanting_cloud->points.resize(cloud_size);
    
    
    
    for (size_t i=0; i<cloud_size; i++) 
    {
        //gauss_point_convert (gauss_cloud->points[i], gauss);
        
        gauss.x = gauss_cloud->points[i].normal_x;
        gauss.y = gauss_cloud->points[i].normal_y;
        gauss.z = gauss_cloud->points[i].normal_z;  
        
        gauss.normal_x = gauss_cloud->points[i].x;  
        gauss.normal_y = gauss_cloud->points[i].y;  
        gauss.normal_z = gauss_cloud->points[i].z;
        
        point_count++;
        horizontal_cloud->points[i] = gauss;
         
        gauss.x = slanting_gauss->points[i].normal_x;
        gauss.y = slanting_gauss->points[i].normal_y;
        gauss.z = slanting_gauss->points[i].normal_z;  
        
        gauss.normal_x = slanting_gauss->points[i].x;  
        gauss.normal_y = slanting_gauss->points[i].y;  
        gauss.normal_z = slanting_gauss->points[i].z;

        slanting_cloud->points[i] = gauss;
    }
    std::cout << "Horizontal message : " << cloud_size << std::endl;
    std::cout << std::endl;
   // }}}
     
    // output
    ros::Time time = ros::Time::now();
    pubPointCloud2 (gauss_pub, *gauss_cloud, "/gauss_cloud", time);
    pubPointCloud2 (esc_pub, *horizontal_cloud, "/horizontal_cloud", time);
    pcl::PointCloud<pcl::PointNormal> sw;
    sw.swap(*gauss_cloud);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "xtion_pcl");
    ros::NodeHandle nh;
    ros::NodeHandle n;


    // Create a ROS subscriber for the input point cloud
//    ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);
    ros::Subscriber sub = nh.subscribe ("/point2", 1, cloud_cb);
    // Create a ROS publisher for the output point cloud
    gauss_pub = nh.advertise<sensor_msgs::PointCloud2> ("/gauss_cloud", 1);
    esc_pub = n.advertise<sensor_msgs::PointCloud2> ("/horizontal_esc", 1);
    // Spin
    ros::spin();
}
