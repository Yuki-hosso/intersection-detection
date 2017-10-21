//detect_peak_visu →  intersection_detection
//走行した軌跡を利用して、交差点認識を行う
//2017/10/12 ikuta finish
//tsukuba用のプログラムは別途intersection_detection_trajectory_rwrc17.cppに作成
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
#include<std_msgs/Int16MultiArray.h>
#include<std_msgs/Int32MultiArray.h>
#include<std_msgs/Float32MultiArray.h>
#include<geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include "node_graph/node_edge_manager.h"

using namespace std;
using namespace Eigen;

typedef pcl::PointXYZINormal PointA;
typedef pcl::PointCloud<PointA> CloudA;
typedef pcl::PointCloud<PointA>::Ptr CloudAPtr;

typedef pcl::PointXYZI PointX;
typedef pcl::PointCloud<PointX> CloudX;
typedef pcl::PointCloud<PointX>::Ptr CloudXPtr;

ros::Publisher flag_pub;

bool callback_flag = false;
bool screen_flag = false;
bool odom_callback = false;
bool update_node_flag = true;
bool observ_flag = false; 

const int loop_count = 10;
// int loop_count = 20;
int degree_count = 720;

int save_number = loop_count*degree_count;
int save_count = 0;

double node_x = -0.995471;
double node_y = -14.6942;
double node_len = sqrt(pow(node_x,2)+pow(node_y,2));
int allow_error = 0;

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
	cout<<"INIT"<<endl;
	for(int i=0;i<loop_count*4;i++){
		save_peak[i] = 0;
	}
}

inline bool intersection_traj_flag(geometry_msgs::Pose &odom,geometry_msgs::Pose &intersec_odom)
{
	// double difference = abs( sqrt( pow(odom.position.x,2)+pow(odom.position.y,2) ) - sqrt( pow(intersec_odom.position.x,2)+pow(intersec_odom.position.y,2) )  );
	double difference = abs( sqrt( pow(odom.position.x-intersec_odom.position.x,2)+pow(odom.position.y-intersec_odom.position.y,2) )  );

	// if(difference>1.0){
	// if(difference>3.0){
	if(difference>5.0){
		return true;
	}else{
		return false;
	}
}
inline bool intersection_flag(geometry_msgs::Pose &odom,geometry_msgs::Pose &intersec_odom)
{
	// double difference = abs( sqrt( pow(odom.position.x,2)+pow(odom.position.y,2) ) - sqrt( pow(intersec_odom.position.x,2)+pow(intersec_odom.position.y,2) )  );
	double difference = abs( sqrt( pow(odom.position.x-intersec_odom.position.x,2)+pow(odom.position.y-intersec_odom.position.y,2) )  );

	// if(difference>1.0){
	if(difference>7.0){
		return true;
	}else{
		return false;
	}
}
inline bool intersection_flag_node(geometry_msgs::Pose &odom,geometry_msgs::Pose &intersec_odom)
{
	double distance = sqrt( pow(odom.position.x-intersec_odom.position.x,2)+pow(odom.position.y-intersec_odom.position.y,2) );

	// if( sqrt( intersec_odom.position.x==0.0 &&  intersec_odom.position.y==0.0 )){
	// 	return true;
	// }
	cout<<distance<<":"<<node_len<<endl;
	// if(distance>node_len*0.8){
	if(distance>node_len*0.85){
		cout<<"true"<<endl;
		return true;
	}else{
		cout<<"false"<<endl;
		return false;
		// return true;
	}
}

int cnt_degree[360];
int loop_tra = 0;
bool unliner_flag = false;
double trajectory_estimate(geometry_msgs::Pose &odom){
	int deg = abs((int)((odom.orientation.z+M_PI)/M_PI*180.0));
	if(loop_tra==0){unliner_flag = false;}
	if(deg>360){
		deg -= 360;
		unliner_flag = true;
	}
	cout<<"current_degree"<<deg<<endl;
	cnt_degree[deg]+=1;
	loop_tra++;
	double trajectory = 0.0;
	if(loop_tra>10){
		// trajectory = 0.0;
		for(int i=0;i<360;i++){
			if(cnt_degree[i]!=0){
				if(unliner_flag){
					if(i>=0&&i<=20){
						trajectory += (double)cnt_degree[i]*(i+360);
					}else{
						trajectory += (double)cnt_degree[i]*i;
					}
				}else{
					trajectory += (double)cnt_degree[i]*i;
				}
			}
		}
		trajectory = trajectory/loop_tra;
		if(trajectory>360.0){
			trajectory -= 360.0;
		}
		// cout<<"trajectory:"<<trajectory<<endl;
	}
	cout<<"trajectory:"<<trajectory<<endl;
	return trajectory;
}
double trajectory_estimate_unliner(geometry_msgs::Pose &odom,double o_tra){
	unliner_flag = false;
	if(loop_tra>0){
		if(o_tra<20.0||o_tra>340.0){
			unliner_flag = true;
		}
	}
	int deg = abs((int)((odom.orientation.z+M_PI)/M_PI*180.0));
	cout<<"current_degree"<<deg<<endl;
	cnt_degree[deg]+=1;
	loop_tra++;
	double trajectory = 0.0;
	if(loop_tra>10){
		// trajectory = 0.0;
		for(int i=0;i<360;i++){
			if(cnt_degree[i]!=0){
				if(unliner_flag){
					if(o_tra<20.0){
						if(340<=i&&i<=360){
							trajectory += (double)cnt_degree[i]*(i-360);
						}else{
							trajectory += (double)cnt_degree[i]*i;
						}
					}
					if(o_tra>340.0){
						if(0<=i&&i<=20){
							trajectory += (double)cnt_degree[i]*(i+360);
						}else{
							trajectory += (double)cnt_degree[i]*i;
						}
					}
				}else{
					trajectory += (double)cnt_degree[i]*i;
				}
			}
		}
		trajectory = trajectory/loop_tra;
		if(trajectory>360.0){
			trajectory -= 360.0;
		}
		if(trajectory<0.0){
			trajectory += 360.0;
		}
		// cout<<"trajectory:"<<trajectory<<endl;
	}
	cout<<"trajectory:"<<trajectory<<endl;
	return trajectory;
}

void Init_deg()
{
	cout<<"Init_degree"<<endl;
	for(int i=0;i<360;i++){
		cnt_degree[i]=0;
	}
}


void Intersec_init()
{
	INIT();
	loop_tra=0;
	Init_deg();
}

void Peak_global(std_msgs::Int32MultiArray &peak,geometry_msgs::Pose &odom,std_msgs::Int32MultiArray &peak_global,double trajectory)
{
	peak_global.data.clear();
	int deg = (int)((odom.orientation.z+M_PI)/M_PI*180.0);
	int deg2 = (int)trajectory;
	int deg_diff = abs(deg - deg2);
	if(deg_diff>200){
		if(deg>deg2){
			deg_diff = deg2+360 - deg;
		}else{
			deg_diff = deg+360 - deg2;
		}
	}

	int tmp_peak = 0;
	if(deg>deg2){
		for(unsigned int i=0;i<peak.layout.data_offset;i++){
			tmp_peak =(int) peak.data[i]/2 - deg_diff;
			// if(tmp_peak<0){
			// 	tmp_peak += 360;
			// }
			while(tmp_peak<0){
				tmp_peak += 360;
			}
			peak_global.layout.data_offset = peak.layout.data_offset;
			peak_global.data.push_back(tmp_peak);
			// cout<<"global_peak"<<tmp_peak<<endl;
		}
	}else{
		for(unsigned int i=0;i<peak.layout.data_offset;i++){
			tmp_peak =(int) peak.data[i]/2 + deg_diff;
			// if(tmp_peak<0){
			// 	tmp_peak += 360;
			// }
			while(tmp_peak>360){
				tmp_peak -= 360;
			}
			peak_global.layout.data_offset = peak.layout.data_offset;
			peak_global.data.push_back(tmp_peak);
			// cout<<"global_peak"<<tmp_peak<<endl;
		}
	}
	cout<<"original"<<peak<<endl;
	cout<<"global"<<peak_global<<endl;

}

bool one_flag = false;
bool two_flag = false;
bool three_flag = false;
bool four_flag = false;
bool insc_flag = false;
int two_degree = 0;
int four_degree = 0;
int observ_mode = 0;
void Intersection_detection(std_msgs::Int32MultiArray &peak,geometry_msgs::Pose &odom,geometry_msgs::Pose &intersec_odom,double trajectory)
{
	int diff_tra = 0;
	for(unsigned int i=0;i<peak.layout.data_offset;i++){
		// diff_tra = peak.data[i] - (int)trajectory;
		diff_tra = peak.data[i];
		if(diff_tra<0){
			diff_tra += 360;
		}
		cout<<"differrent"<<diff_tra<<endl;
		if(diff_tra<(20-allow_error)||diff_tra>(340+allow_error)){
			one_flag = true;
		// }else if(diff_tra>70&&diff_tra<110){
		}else if(diff_tra>(65-allow_error)&&diff_tra<(115+allow_error)){
			two_flag = true;
			two_degree = peak.data[i];
		}else if(diff_tra>(160-allow_error)&&diff_tra<(200+allow_error)){
			three_flag = true;
		// }else if(diff_tra>250&&diff_tra<290){
		}else if(diff_tra>(245-allow_error)&&diff_tra<(295+allow_error)){
			four_flag = true;
			four_degree = peak.data[i];
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
		if(tmp[1]>3||tmp[3]>3){
			observ_flag = true;
			if(tmp[1]>3&&tmp[3]>3){
				observ_mode = 1;
			}else if(tmp[1]>3){
				observ_mode = 2;
			}else{
				observ_mode = 3;
			}
			cout<<"observ mode"<<observ_mode<<endl;
			// cout<<"intersection!!!!!!!!!!!!!!!!!!!!"<<endl;
			// intersec_odom.position.x = odom.position.x;
			// intersec_odom.position.y = odom.position.y;
			// intersec_odom.position.z = odom.position.z;
			// // INIT();
			// std_msgs::Bool intersec;
			// intersec.data = true;
			// update_node_flag = false;
			// flag_pub.publish(intersec);
			// // cout<<tmp[0]<<endl;
			// // cout<<tmp[1]<<endl;
			// // cout<<tmp[2]<<endl;
			// // cout<<tmp[3]<<endl;
			// // INIT();
			// Intersec_init();
		}
		if(tmp[0]>5||tmp[2]>5){
			cout<<"road!"<<endl;
			// cout<<tmp[0]<<endl;
			// cout<<tmp[1]<<endl;
			// cout<<tmp[2]<<endl;
			// cout<<tmp[3]<<endl;
		}
		insc_flag = false;
	}else{
			cout<<"nooooooooooooooooooooo"<<endl;
			save_peak[(save_count%loop_count)*4] = 0;
			save_peak[(save_count%loop_count)*4+1] = 0;
			save_peak[(save_count%loop_count)*4+2] = 0;
			save_peak[(save_count%loop_count)*4+3] = 0;
			// INIT();
	}
	save_count++;
}

int non_update = 0;
int observ_update = 0;
void Observ_function(std_msgs::Int32MultiArray &peak,geometry_msgs::Pose odom,geometry_msgs::Pose &intersec_odom,double trajectory)
{
	int diff_tra = 0;
	// int threshold_degree = 4;
	int threshold_degree = 3;
	bool intersection_detection = false;
	switch(observ_mode){
		case 1://＋字交差点またはT字交差点
			{
				int diff_tra_two = two_degree - (int)trajectory;
				int diff_tra_four = four_degree - (int)trajectory;
				int ideal_two = 90;
				int ideal_four = 270;
				int update_count_two = 0;
				int update_count_four = 0;
				if(diff_tra_two<0){
					diff_tra_two += 360;
				}
				if(diff_tra_four<0){
					diff_tra_four += 360;
				}

				for(unsigned int i=0;i<peak.layout.data_offset;i++){
					diff_tra = peak.data[i] - (int)trajectory;
					if(diff_tra<0){
						diff_tra += 360;
					}
					// cout<<"differrent"<<diff_tra<<endl;
					///////////////two//////////////////////
					if(abs(diff_tra_two - diff_tra)<(10+allow_error)){
						int diff_old = abs(diff_tra_two - ideal_two);
						int diff_new = abs(diff_tra - ideal_two);
						cout<<"old diff:"<<diff_old<<endl;
						cout<<"new diff:"<<diff_new<<endl;
						if(diff_new <threshold_degree ||diff_old <threshold_degree){
							intersection_detection = true;
							cout<<"JUST INTERSECTION"<<endl;
							// cout<<"old diff:"<<diff_old<<endl;
							// cout<<"new diff:"<<diff_new<<endl;
						}else if(diff_new<=diff_old){
							update_count_two++;
							two_degree = peak.data[i];
						}else{
							if(diff_new-diff_old>2){
								cout<<"PAST AWAY"<<endl;
								cout<<"old diff:"<<diff_old<<endl;
								cout<<"new diff:"<<diff_new<<endl;
								intersection_detection = true;
							}else{
								update_count_two++;
							}
						}
					}else if(abs(diff_tra_two - diff_tra)<(20+allow_error)){
						update_count_two++;
					}
					if(abs(diff_tra_four - diff_tra)<(10+allow_error)){
						int diff_old = abs(diff_tra_four - ideal_four);
						int diff_new = abs(diff_tra - ideal_four);
						cout<<"old diff:"<<diff_old<<endl;
						cout<<"new diff:"<<diff_new<<endl;
						if(diff_new <threshold_degree ||diff_old <threshold_degree){
							intersection_detection = true;
							cout<<"JUST INTERSECTION"<<endl;
							// cout<<"old diff:"<<diff_old<<endl;
							// cout<<"new diff:"<<diff_new<<endl;
						}else if(diff_new<=diff_old){
							update_count_four++;
							four_degree = peak.data[i];
						}else{
							if(diff_new-diff_old>2){
								cout<<"PAST AWAY"<<endl;
								cout<<"old diff:"<<diff_old<<endl;
								cout<<"new diff:"<<diff_new<<endl;
								intersection_detection = true;
							}else{
								update_count_four++;
							}
						}
					}else if(abs(diff_tra_four - diff_tra)<(20+allow_error)){
						update_count_four++;
					}
				}
				if(update_count_two==0||update_count_four==0){
					non_update++;
				}else{
					observ_update++;
				}
				break;
			}
		case 2:
			{
				int diff_tra_old = two_degree - (int)trajectory;
				int ideal_two = 90;
				int update_count = 0;
				if(diff_tra_old<0){
					diff_tra_old += 360;
				}

				for(unsigned int i=0;i<peak.layout.data_offset;i++){
					diff_tra = peak.data[i] - (int)trajectory;
					if(diff_tra<0){
						diff_tra += 360;
					}
					// cout<<"differrent"<<diff_tra<<endl;
					if(abs(diff_tra_old - diff_tra)<(10+allow_error)){
						int diff_old = abs(diff_tra_old - ideal_two);
						int diff_new = abs(diff_tra - ideal_two);
						cout<<"two_degree:"<<two_degree<<endl;
						cout<<"diff_tra_old:"<<diff_tra_old<<endl;
						cout<<"diff_tra:"<<diff_tra<<endl;
						// cout<<"old diff:"<<diff_old<<endl;
						// cout<<"new diff:"<<diff_new<<endl;
						if(diff_new <threshold_degree ||diff_old <threshold_degree){
							intersection_detection = true;
							cout<<"JUST INTERSECTION"<<endl;
							// cout<<"old diff:"<<diff_old<<endl;
							// cout<<"new diff:"<<diff_new<<endl;
						}else if(diff_new<=diff_old){
							update_count++;
							// cout<<"old diff:"<<diff_old<<endl;
							// cout<<"new diff:"<<diff_new<<endl;
							two_degree = peak.data[i];
						}else{
							if(diff_new-diff_old>2){
								cout<<"PAST AWAY"<<endl;
								// cout<<"old diff:"<<diff_old<<endl;
								// cout<<"new diff:"<<diff_new<<endl;
								intersection_detection = true;
							}else{
								update_count++;
							}
						}
					}else if(abs(diff_tra_old - diff_tra)<(20+allow_error)){
						update_count++;
					}
				}
				if(update_count==0){
					non_update++;
				}else{
					observ_update++;
				}
				break;
			}
		case 3:
			{
				int diff_tra_old = four_degree - (int)trajectory;
				int ideal_four = 270;
				int update_count = 0;
				if(diff_tra_old<0){
					diff_tra_old += 360;
				}

				for(unsigned int i=0;i<peak.layout.data_offset;i++){
					diff_tra = peak.data[i] - (int)trajectory;
					if(diff_tra<0){
						diff_tra += 360;
					}
					// cout<<"differrent"<<diff_tra<<endl;
					if(abs(diff_tra_old - diff_tra)<(10+allow_error)){
						int diff_old = abs(diff_tra_old - ideal_four);
						int diff_new = abs(diff_tra - ideal_four);
						cout<<"old diff:"<<diff_old<<endl;
						cout<<"new diff:"<<diff_new<<endl;
						if(diff_new <threshold_degree ||diff_old <threshold_degree){
							intersection_detection = true;
							cout<<"JUST INTERSECTION"<<endl;
							// cout<<"old diff:"<<diff_old<<endl;
							// cout<<"new diff:"<<diff_new<<endl;
						}else if(diff_new<=diff_old){
							update_count++;
							// cout<<"old diff:"<<diff_old<<endl;
							// cout<<"new diff:"<<diff_new<<endl;
							four_degree = peak.data[i];
						}else{
							if(diff_new-diff_old>2){
								cout<<"PAST AWAY"<<endl;
								cout<<"old diff:"<<diff_old<<endl;
								cout<<"new diff:"<<diff_new<<endl;
								intersection_detection = true;
							}else{
								update_count++;
							}
						}
					}else if(abs(diff_tra_old - diff_tra)<(20+allow_error)){
						update_count++;
					}
				}
				if(update_count==0){
					non_update++;
				}else{
					observ_update++;
				}
				break;
			}
		default:
			{
				break;
			}
	}

	if(non_update>10){
		cout<<"NON OBSERV"<<endl;
		// intersection_detection = true;
		non_update = 0;
		observ_update = 0;
		two_degree = 0;
		four_degree = 0;
		observ_flag = false;
		Intersec_init();
	}

	if(observ_update>15){
		cout<<"TOO OBSERV"<<endl;
		intersection_detection = true;
	}

	if(intersection_detection){
		cout<<"intersection!!!!!!!!!!!!!!!!!!!!"<<endl;
		intersec_odom.position.x = odom.position.x;
		intersec_odom.position.y = odom.position.y;
		intersec_odom.position.z = odom.position.z;
		std_msgs::Bool intersec;
		intersec.data = true;
		update_node_flag = false;
		flag_pub.publish(intersec);

		observ_flag = false;
		intersection_detection = false;
		non_update = 0;
		observ_update = 0;
		two_degree = 0;
		four_degree = 0;
		Intersec_init();

		cout<<"end intersec_init"<<endl;
	}
}

//callback
std_msgs::Int32MultiArray peak_in;
void Peak_deg(const std_msgs::Int32MultiArray::Ptr &msg)
{
	peak_in.data.clear();
	peak_in.layout.data_offset = msg->layout.data_offset;
	if(peak_in.layout.data_offset!=100){
		for(unsigned int i=0;i<peak_in.layout.data_offset;i++){
			peak_in.data.push_back(msg->data[i]);
		}
		callback_flag = true;
		// cout<<"peak_callback"<<endl;
	}
}

Node node[3];
NodeEdgeManager* nem;
void divCallback(const std_msgs::Int16MultiArray::ConstPtr& msg){
	node[0] = nem->nodeGetter(msg->data[0]);
	node[1] = nem->nodeGetter(msg->data[1]);
	double ratio =  0.01 * msg->data[3];
	node[2].x = node[0].x + ratio * (node[1].x - node[0].x);
	node[2].y = node[0].y + ratio * (node[1].y - node[0].y);
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

geometry_msgs::Pose intersec_odom;
void Next_node(const std_msgs::Float32MultiArray msg)
{
	node_len = sqrt(pow(node_x-msg.data[0],2)+pow(node_y-msg.data[1],2) );
	node_x = msg.data[0];
	node_y = msg.data[1];
	if(node_len<15.0){
		allow_error = 5;
	}else{
		allow_error = 0;
	}
	if(msg.data[3]==1){//o-button
		intersec_odom.position.x = odom.position.x;
		intersec_odom.position.y = odom.position.y;
		intersec_odom.position.z = odom.position.z;
		Intersec_init();
	}else if(msg.data[3]==2){//x-button
		node_len = 1.0;
		allow_error = 10;
	}else if(msg.data[3]==3){//replan
		cout<<"replaning mode"<<endl;
		node_len = sqrt(pow(node_x - node[2].x,2)+pow(node_y - node[2].y,2) );
	}
	update_node_flag = true;
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "intersection_detection_trajectory");
    ros::NodeHandle nh;
    ros::NodeHandle n;

    // Create a ROS subscriber for the input point cloud
    // ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, velodyne_cb);
    ros::Subscriber sub = nh.subscribe ("/peak/deg", 1, Peak_deg);
    ros::Subscriber sub_xy = nh.subscribe ("/next_xy", 1, Next_node);
    // ros::Subscriber sub = nh.subscribe ("/peak/deg2", 1, Peak_deg);
	ros::Subscriber sub_div = n.subscribe<std_msgs::Int16MultiArray>("/edge/certain", 1, divCallback);
	ros::Subscriber sub_lcl = n.subscribe("/lcl",1,OdomCallback);
    // Create a ROS publisher for the output point cloud
    flag_pub = nh.advertise<std_msgs::Bool> ("/intersection_flag", 1);


	//init_param
	cout<<"init_params"<<endl;
	string filename;
	int begin_node;
	int end_node;
	double div;
	n.getParam("/node_edge", filename);
	n.getParam("/init/node/begin", begin_node);
	n.getParam("/init/node/end", end_node);
	n.getParam("/init/node/div", div);
	nem = new NodeEdgeManager(filename);

	cout<<"begin_node"<<begin_node<<endl;
	cout<<"end_node"<<end_node<<endl;
	//end init_param

	int edge_num = nem->edgeGetter(begin_node,end_node);
	node_len = sqrt(nem->distGetter(edge_num)) * div;
	Node node = nem->nodeGetter(end_node);
	node_x = node.x;
	node_y = node.y;

	//init
	// for(int i=0;i<loop_count*4;i++){
	// 	save_peak[i] = 0;
	// }
	INIT();
	Init_deg();
	cout<<"init"<<endl;
	//
	// geometry_msgs::Pose intersec_odom;
	std_msgs::Int32MultiArray peak_global;
	double trajectory = 0.0;
    // main handle
    ros::Rate loop_rate(20);
    while (ros::ok()){
		if(callback_flag && odom_callback){
			cout<<"---------------------------------------------------"<<endl;
			callback_flag = false;
			odom_callback = false;
			if(intersection_traj_flag(odom,intersec_odom)){
				// trajectory = trajectory_estimate(odom);
				trajectory = trajectory_estimate_unliner(odom,trajectory);
			}
			Peak_global(peak_in,odom,peak_global,trajectory);
			if(intersection_flag(odom,intersec_odom)){  //////change here
			// if(intersection_flag_node(odom,intersec_odom)&&update_node_flag){  /////change here
				// Intersection_detection(peak_in,odom,intersec_odom);
				if(observ_flag){
					cout<<"observ_function"<<endl;
					Observ_function(peak_global,odom,intersec_odom,trajectory);
				}else{
					cout<<"detection_function"<<endl;
					Intersection_detection(peak_global,odom,intersec_odom,trajectory);
				}
			}
		}
        ros::spinOnce();
        loop_rate.sleep();
    }
}
