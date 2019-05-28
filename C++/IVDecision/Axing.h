//Axing.h
//A-star�㷨ͷ�ļ�

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

#define kuan 512  //դ��ͼ�Ŀ�
#define chang 512  //դ��ͼ�ĳ�
#define stepsize 5
#define XX 1.0*stepsize //ˮƽ��ֱ��cost
#define YY 1.414*stepsize//�Խ��ߵ�cost

struct NodeAxing 
{
	int x,y;//�ڵ������
	double g;//��ʾ����ʼ�㵽��ǰ��Ĵ���
	double h;//��ʾ��ǰ�㵽Ŀ���Ĵ��۹���
	double f;//��ʾ�õ���ܴ��۹���

	NodeAxing *par;//��ʾָ�򸸽ڵ�
	NodeAxing *next;//��ʾָ����һ���ڵ�

	bool closeexist;//��ʾ�ж��Ƿ������close���У���һ����־λ
};
class Axing
{
public:
	Axing(void);
	~Axing(void);
	void Axingpath(int sx, int sy, int dx,int dy,double theta1,double theta2);
	struct NodeAxing* SearchMin(struct NodeAxing* t);//�ҳ�open����f��С��node
	void Addnearnode(struct NodeAxing* t);//����Χ�ĵ�����ж�
	void Addopen(struct NodeAxing* t);//
	void Addclose(struct NodeAxing* t);
	void deleteopen(struct NodeAxing* t);
	void Bezier();
	void quzheng();

	struct NodeAxing* existopen(struct NodeAxing* t);
	bool existclose(struct NodeAxing* t);
	void ShowPath(struct NodeAxing* t);
	I_Map* ditu;
	vector<CvPoint2D64f>path1;//��ʼA*����
	vector<CvPoint2D64f>path2;//Beizer����
	vector<CvPoint2D64f>path3;//ȡ��������

private:
	struct NodeAxing* open;
	struct NodeAxing* close;
	int Dx,Dy;
};

