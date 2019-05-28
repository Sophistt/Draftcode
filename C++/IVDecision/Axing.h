//Axing.h
//A-star算法头文件

#pragma once
#include <math.h>
#include <iostream>
#include "cxtypes.h"
#include <vector>
#include <stdlib.h>
#include "structhead.h"
using namespace std;

#ifndef PI
#define PI 3.1415926535897932385
#endif

#define kuan 512  //栅格图的宽
#define chang 512  //栅格图的长
#define stepsize 5
#define XX 1.0*stepsize //水平或垂直的cost
#define YY 1.414*stepsize//对角线的cost

struct NodeAxing 
{
	int x,y;//节点的坐标
	double g;//表示从起始点到当前点的代价
	double h;//表示当前点到目标点的代价估计
	double f;//表示该点的总代价估计

	NodeAxing *par;//表示指向父节点
	NodeAxing *next;//表示指向下一个节点

	bool closeexist;//表示判断是否存在于close表中，是一个标志位
};
class Axing
{
public:
	Axing(void);
	~Axing(void);
	void Axingpath(int sx, int sy, int dx,int dy,double theta1,double theta2);
	struct NodeAxing* SearchMin(struct NodeAxing* t);//找出open表中f最小的node
	void Addnearnode(struct NodeAxing* t);//对周围的点进行判断
	void Addopen(struct NodeAxing* t);//
	void Addclose(struct NodeAxing* t);
	void deleteopen(struct NodeAxing* t);
	void Bezier();
	void quzheng();

	struct NodeAxing* existopen(struct NodeAxing* t);
	bool existclose(struct NodeAxing* t);
	void ShowPath(struct NodeAxing* t);
	I_Map* ditu;
	vector<CvPoint2D64f>path1;//初始A*数据
	vector<CvPoint2D64f>path2;//Beizer数据
	vector<CvPoint2D64f>path3;//取整后数据

private:
	struct NodeAxing* open;
	struct NodeAxing* close;
	int Dx,Dy;
};

