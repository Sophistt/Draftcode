/*************************************************************
局部地图规划类

2010-3-3 ZhengFei
**************************************************************/
#pragma once

#include "IVDecision.h"
#include "GetGPSData.h"
#include "SocketTransfer.h"
#include <math.h>
#include <malloc.h>


class CIVLocalPath 
{
public:
	CIVLocalPath(void);
public:
	~CIVLocalPath(void);

private:

	double rx[200],ry[200];

	//double rx,ry;//保存差值结果
	int fac(int n);
	CvSeq* seq_road;
	CvMemStorage* storage_road;
public:
	
	void Get10Sample(I_Map *map);
	void Set10Point(double px[],double py[]);
	void spline( double x[],double y[],int n);
	void chase(double a[],double b[],double c[],double d[],double s[],int n );
	void Get200Point(CvPoint2D64f (&MPoint)[200]);

	void Straight(CvPoint2D64f p1,CvPoint2D64f p2,CvPoint2D64f (&GPoint)[200]);//起始点确定一条直线
	void Straight(double x[],double y[],int n,CvPoint2D64f (&MPoint)[200]);//n个点确定的一条路
	void Cross(double x[],double y[],int n,CvPoint2D64f (&MPoint)[200]);
	void Bezier(CvPoint2D64f p[],int n,CvPoint2D64f (&BPoint)[150]);
	void Bezier(CvPoint2D64f p[],int n,CvPoint2D64f (&BPoint)[200]);
	void Bezier(CvPoint2D64f p[],int n,CvPoint2D64f (&MPoint)[512]);
	int GetMidPoint(I_Map *map,CvPoint2D64f (&MPoint)[300]);
	void ExtendMap(I_Map &map);
	bool ExtendPoint(I_Map &p,int x,int y,int n);
	void Bezier(CvPoint2D64f p[],int n,CvPoint2D64f *MPoint,int m);
	int threeSpline(CvPoint2D64f point[],int n,CvPoint2D64f WayPoint[]);
	void GetControlPoint(CvPoint2D64f oriPoint[], CvPoint2D64f ctlPoint[]);
	void SplineBezier(CvPoint2D64f APoint[],int n,CvPoint2D64f (&WPoint)[200]);
	void BSpline(CvPoint2D64f p[],int n,CvPoint2D64f (&WPoint)[200]);
};
