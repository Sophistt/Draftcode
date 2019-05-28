/*************************************************************
车道保持任务类

2010-3-3 ZhengFei
**************************************************************/
#pragma once

#include "IVDecision.h"
#include "GetGPSData.h"
#include "SocketTransfer.h"
#include <math.h>
#include "VisionCK.h"
#include "IVLocalPath.h"
class CIVKeepLane 
{
public:
	CIVKeepLane(void);
public:
	~CIVKeepLane(void);
	CIVLocalPath path;
	I_Map *m;
	void WaitMap(queue<I_Map> &Map,I_Map &map);

	CvPoint2D64f KeepWay(Map_Point Apoint1,I_Map *WayMap,double aa,double bb,double hx);
	Map_Point SetAPoint(int y,I_Map *map);

	int SetXPoint(int y,int n,I_Map *map);
	Map_Point ReApoint(Map_Point Ap,I_Map *WayMap);
	double rad(double d) { return d * PI / 180.0; };
	double SetADir(Map_Point A1,Map_Point A2,double rDir);
	Map_Point CountYuan(Map_Point a,double r);
	bool CIVKeepLane::GetLane(I_Map *map,Map_Point (&Ap)[200]);
	//2011
	bool SearchLane(I_Map *map,CvPoint2D64f *MidPoint);
	CvPoint2D64f MaptoGps(CvPoint2D64f Apoint1,double aa,double bb,double hx);
	int SearchMidLane(I_Map *map,CvPoint2D64f MidPoint[]);
	bool SearchMidLane1(I_Map *map,CvPoint2D64f &StartPoint1,int count1);
	void Min2Method(double &xyTopX, double &xyTopY, double  X[], double Y[], int nCount) ; 
	int SearchFromMidLane(I_Map *map,CvPoint2D64f *MidPoint,int &num);
	void NiheLine(double &xyTopX1,double &xyTopY1,CvPoint2D64f *his_MidGpsPoint);
	bool JudgeProperLine(double b,double k,CvPoint2D64f *new_MidGpsPoint);

	CvPoint2D64f TempPoint_Vehicle[501];
	
	CvPoint2D64f TempPoint[500];

	int SearchOneMidLane(I_Map *map,CvPoint2D64f *MidPoint);
	int SearchOnlyOneMidLane(I_Map *map,CvPoint2D64f *MidPoint);

	//2011
};
