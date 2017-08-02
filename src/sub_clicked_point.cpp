#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

bool callback_flag = false;
// geometry_msgs::Pose click_pt;
geometry_msgs::PoseStamped click_pt;

// void ClickedPointCallback(const geometry_msgs::Pose::Ptr& msg)
void ClickedPointCallback(const geometry_msgs::PoseStamped::Ptr& msg)
{
	click_pt = *msg;	
	callback_flag = true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "circle_cloud");
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud>("/circle_cloud", 100);
	ros::Subscriber sub = n.subscribe("/move_base_simple/goal", 1, ClickedPointCallback);

	sensor_msgs::PointCloud circles;

	float r=0;
	cout<<"r= ";
	cin >> r; 
	
	ros::Rate loop_rate(5);
	while (ros::ok()) {
		if (callback_flag){
			float theta=0;

			for(int i=0;i<1000;i++){
				geometry_msgs::Point32 circle;
				theta=((2*M_PI)/1000)*i;		

				circle.x=r*cos(theta) + click_pt.pose.position.x;
				circle.y=r*sin(theta) + click_pt.pose.position.y;
				circle.z=-1.35;
				circles.points.push_back(circle);	
			}		


			circles.header.frame_id="/velodyne";
			circles.header.stamp=ros::Time::now();
			pub.publish(circles);

			circles.points.clear();
			callback_flag = false;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}

