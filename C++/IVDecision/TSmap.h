#pragma once
#include "Vlist.h"
#include "Vehicle.h"

struct Vehicle2Point_Time//无人车从当前位置开到某点所需的最大时间和最小时间
{
	double minTime;
	double maxTime;
};


class TSmap:public CVehicle
{
public:
	TSmap(void);
	void Modifymap(I_Map* ditu,NewMap* xinditu);
	int collision_analysis(DyObstacle newobstacle,CvPoint2D64f p);
	bool determine_solution(CvPoint2D64f collision_position,CvPoint2D64f ob_position,double travel_direction);
	//void CreateTimeTable();
	Vehicle2Point_Time time[512][512];
	void Sequence(int x1,int y1,int x2,int y2,int x3,int y3,int x4,int y4);
	int max_x,max_y,min_x,min_y;

	Vlist dyob_list;
	CvPoint2D64f collision_position1,collision_position2;

	//int collision_ID1,collision_ID2;
	
public:
	~TSmap(void);
};
