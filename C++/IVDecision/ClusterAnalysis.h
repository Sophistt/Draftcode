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
	vector <DataPoint> dadaSets;       //���ݼ���
	unsigned int dimNum;             //ά��
	double radius;                   //�뾶
	unsigned int dataNum;            //��������
	unsigned int minPTs;             //������С���ݸ���
	unsigned int clusterNum;

	double GetDistance(DataPoint &dp1,DataPoint dp2);            //���뺯��
	void SetArrivalPoints(DataPoint& dp);                        //�������ݵ��������б�
	void KeyPointCluster(unsigned long dpID, unsigned long clusterId); //�����ݵ������ڵĵ�ִ�о������
public:
	ClusterAnalysis(void);         //Ĭ�Ϲ��캯��
	void Init(I_Map *map,double radius, int minPTs);   //��ʼ������
	bool DoDBSCANRecursive();                              //DBSCAN�ݹ��㷨
	void WriteToFile();                      //��������д���ļ�
	
public:
	~ClusterAnalysis(void);
};
