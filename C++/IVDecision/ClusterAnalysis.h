#pragma once
#include "stdafx.h"
#include <iostream>
#include <cmath>
#include "DataPoint.h"
#include "structhead.h"
using namespace std;

struct MPoint
{
	int x;
	int y;
};
class ClusterAnalysis
{
private:
	vector <DataPoint> dadaSets;       //数据集合
	unsigned int dimNum;             //维度
	double radius;                   //半径
	unsigned int dataNum;            //数据数量
	unsigned int minPTs;             //领域最小数据个数
	unsigned int clusterNum;

	double GetDistance(DataPoint &dp1,DataPoint dp2);            //距离函数
	void SetArrivalPoints(DataPoint& dp);                        //设置数据点的领域点列表
	void KeyPointCluster(unsigned long dpID, unsigned long clusterId); //对数据点领域内的点执行聚类操作
public:
	ClusterAnalysis(void);         //默认构造函数
	void Init(I_Map *map,double radius, int minPTs);   //初始化操作
	bool DoDBSCANRecursive();                              //DBSCAN递归算法
	void WriteToFile();                      //将聚类结果写入文件
	
public:
	~ClusterAnalysis(void);
};
