#pragma once
#include "cxtypes.h"
#include "resource.h"
#include <cmath>
#include <vector>
#include <iostream>
#include <time.h>
#include <stdlib.h>
#define ROUND(a) int(a+0.5)//��������
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
	double theta;//��ʼ�㺽��
	double beita;//Ŀ��㺽��
	vector<CvPoint2D64f>rPath;//����һ����ΪrPath��pos���͵�����,���ڴ洢RRT�㷨���ɵ����ڵ�.
	vector<CvPoint2D64f>rPath4;
	vector<CvPoint2D64f>rPath5;//���ڴ洢��������·����
	vector<CvPoint2D64f>data;//�洢�ϰ���λ����Ϣ
	//vector<CvPoint2D64f>rPath7;//���ڴ洢����B�����Ŀ��Ƶ�
	vector<CvPoint2D64f>rPath8;
public:
	RRT(void);
	~RRT(void);

	double getDis(CvPoint2D64f a,CvPoint2D64f b);//����a��b�ľ���
	CvPoint2D64f getNextPos(CvPoint2D64f a,CvPoint2D64f b);//�� a������һ����
	void find_rrt();//����RRT·��
	void prune();//��֦����
	void B_Spline();//B����
	void Bezier();
	CvPoint2D64f Heading1(CvPoint2D64f a,double b);
	CvPoint2D64f Heading2(CvPoint2D64f a,double b);
	bool check_free(CvPoint2D64f a,CvPoint2D64f b,CvPoint2D64f c);
	void Clear();
};
