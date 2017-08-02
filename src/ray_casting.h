#ifndef RAY_CASTING_CLASS
#define RAY_CASTING_CLASS
#include <pcl/point_types.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;

struct DistIndex
{
	double dist;
	int idx;
	PointT point;
};

class RayCasting
{
private:
	int d_num_;
	double res_;
	std::vector<DistIndex> dist_list_;
	// CloudT dst_;

public:
	void setResolution(const double res);
	void Cast(const CloudT& p);
	void getResult(CloudT& dst);
	void initDist();
	bool IsOriginOfCoordinate(const PointT& p)
	{
		if (p.x == 0 && p.y == 0){
			return true;
		}
		return false;
	}
	int calcIndex(const PointT& p)
	{
		double rad = atan2(p.y,p.x);
		if (rad*180.0/M_PI<0.000000){
			rad += M_PI*2.0;
		}
		return rad/res_;
	}
	double calcDist(const PointT& p)
	{
		return p.x*p.x + p.y*p.y;
	}
	bool IsFirstPoint(int idx)
	{
		if (idx == -1){
			return true;
		}
		return false;
	}
};

void RayCasting::initDist()
{
	for (int i=0; i<d_num_; i++){
		dist_list_[i].dist = -1;
		dist_list_[i].idx  = -1;
	}
}

void RayCasting::setResolution(const double res)
{
	res_ = res;
	d_num_ = (int)(2.0*M_PI/res);
	dist_list_.resize(d_num_);
	initDist();
	// dst_.points.resize((int)(2.0*M_PI/res));
}

void RayCasting::Cast(const CloudT& src)
{
	int num = src.points.size();
	for (int i=0; i<num; i++){
		if (IsOriginOfCoordinate(src.points[i])){
			continue;
		}
		double dist = calcDist(src.points[i]);
		int idx     = calcIndex(src.points[i]);
		if (IsFirstPoint(dist_list_[idx].idx)){
			dist_list_[idx].idx  = idx;
			dist_list_[idx].dist = dist;
			dist_list_[idx].point = src.points[i];
		}
		else if (dist < dist_list_[idx].dist){
			dist_list_[idx].idx  = idx;
			dist_list_[idx].dist = dist;
			dist_list_[idx].point = src.points[i];
		}
	}
}

void RayCasting::getResult(CloudT& dst)
{
	dst.points.resize(d_num_);
	for (int i=0; i<d_num_; i++){
		if (dist_list_[i].idx == -1){
			dst.points[i].x = 1000;
			dst.points[i].y = 1000;
			dst.points[i].z = 1000;
		}
		else {
			dst.points[i] = dist_list_[i].point;
		}
	}
}
#endif
