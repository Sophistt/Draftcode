#pragma once
#define  DELTA 3
#include "DyObstacle.h"
#include "structhead.h"
#include "GrowCluster.h"
#include <math.h>
#define  DIFFX 100                   //种类最大值
#define  DIFFY 100                   //列表中最大障碍物数
struct NDiff
{
	double d;
	bool visited;
};
class DynamicObFuse
{
private:
	double l_variancex;
	double l_variancey;
	double r_variancex;
	double r_variancey;
	double l_meanx;
	double l_meany;
	double r_meanx;
	double r_meany;

	double m_rk;
	double m_rb;
	double m_lk;
	double m_lb;

public:
	double vehicle_dir;
	CvPoint2D64f vehicle_gps;
	I_Map * current_map;
	v_Map* current_map_v_x;
	v_Map* current_map_v_y;

	DynamicObFuse(void);
	vector<DyObstacle> dylist;
	vector<DyObstacle>usefullist;
	vector<DyObstacle>inroadlist;

	GrowCluster cluster;
	int dy_obstalce_num;
	void SetVehicle(double current_dir,CvPoint2D64f current_gps, I_Map *map);
	CvPoint2D64f Map2Gps(CvPoint2D64f vehicle_gps,double vehicle_dir,CvPoint2D64f point);
	CvPoint2D64f APiontConverD(CvPoint2D64f v,CvPoint2D64f a,double direction);
	double GetDistance(double lat1, double lng1, double lat2, double lng2);
	double GetAngle(CvPoint2D64f apoint, CvPoint2D64f bpoint);
	double MatchDyobstacle(DyObstacle mapob,DyObstacle listob);
	double rad(double d) { return d * 3.1415926535897932 / 180.0; };
	void UpDatelist(I_Map *map,ObjectWT *ibobject,int ibnum);
	void UpDateUselist();
	bool IbeoExpansion2(CvPoint2D64f box_points[4],CvPoint2D64f point,int length,int width);
	void SearchRLine(CvPoint2D64f (&line)[512],int &l_num);
	void SearchLLine(CvPoint2D64f (&line)[512],int &l_num);
	void CountLine(CvPoint2D64f *line,int l_num,double &k,double &b);//计算拟合方程式
	void ObInRoad();
	~DynamicObFuse(void);
};
