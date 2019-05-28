// Global_planning.h : Í·ÎÄ¼þ
//
#include "Astar.h"
#pragma once
#include "afxcmn.h"
#include "afxdtctl.h"


class Global_planning
{

public:
	
    CLoadRNDF RNDF;
	CAstar astar;
	void solvewaypointnum();
	MAPPOINT* solvepoint(double x,double y);
	void point2point();
	void planning(int m1,int n1,int p1,int m2,int n2,int p2);
	struct MAPPOINT *start_point,*end_point;
	struct MAPPOINT *path;

protected:

	int waypointnum;

};