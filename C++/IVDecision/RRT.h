#pragma once
#include "cxtypes.h"
#include "resource.h"
#include <cmath>
#include <vector>
#include <iostream>
#include <time.h>
#include <stdlib.h>
#define ROUND(a) int(a+0.5)//四舍五入
using namespace std;
#ifndef PI
#define PI 3.1415926535897932385
#endif

const double minDis=15;
const double minPath=/*8*/10;



class RRT
{
public:
	CvPoint2D64f begin;
	CvPoint2D64f end;
	double theta;//起始点航向
	double beita;//目标点航向
	vector<CvPoint2D64f>rPath;//创建一个名为rPath的pos类型的容器,用于存储RRT算法生成的树节点.
	vector<CvPoint2D64f>rPath4;
	vector<CvPoint2D64f>rPath5;//用于存储处理过后的路径点
	vector<CvPoint2D64f>data;//存储障碍物位置信息
	//vector<CvPoint2D64f>rPath7;//用于存储的是B样条的控制点
	vector<CvPoint2D64f>rPath8;
public:
	RRT(void);
	~RRT(void);

	double getDis(CvPoint2D64f a,CvPoint2D64f b);//计算a与b的距离
	CvPoint2D64f getNextPos(CvPoint2D64f a,CvPoint2D64f b);//由 a计算下一个点
	void find_rrt();//产生RRT路径
	void prune();//剪枝处理
	void B_Spline();//B样条
	void Bezier();
	CvPoint2D64f Heading1(CvPoint2D64f a,double b);
	CvPoint2D64f Heading2(CvPoint2D64f a,double b);
	bool check_free(CvPoint2D64f a,CvPoint2D64f b,CvPoint2D64f c);
	void Clear();
};
