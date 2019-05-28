#pragma once
#include "UrbanRoadState.h"
#include "GetGPSData.h"
#include "LaneDriver.h"
#include "vehicle.h"
class CApproachExit :
	public CVehicle
{
public:
	CApproachExit();

	CvPoint2D64f WayPoint1[5];
	void Approach_intersection();
	void Search_intersection_startpoint(I_Map *map, int t,CvPoint2D64f &qqq,double &kkk);
	bool Search_edge(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down,int &edge_x,int &edge_y);
	//int avoid_edge(CvPoint2D64f s_gps, double s_gpsdir,CvPoint2D64f WayPoint[]);
	CvPoint2D64f correct_point(CvPoint2D64f s_gps, double s_gpsdir);
	void solvepoints(CvPoint2D64f P1,CvPoint2D64f P2,double a2,CvPoint2D64f &P3,CvPoint2D64f &P4,CvPoint2D64f &P5);
	void solvepoints1(CvPoint2D64f P1,CvPoint2D64f P2,double a2,CvPoint2D64f &P3,CvPoint2D64f &P4,CvPoint2D64f &P5);
	void solvepoints2(CvPoint2D64f P1,CvPoint2D64f P2,CvPoint2D64f &P3,CvPoint2D64f &P4,CvPoint2D64f &P5);
	CvPoint2D64f startpoint,currentpoint;
	double as,ac;//起始点的方向角，当前点的航向角
	int intersection_direction;//路口转向
	I_Map *temp_map;
	CLaneDriver lane_appro;

public:
	~CApproachExit(void);
};
