#pragma once


#ifndef STRUCT_HEAD_H
#define STRUCT_HEAD_H
#include "stdafx.h"
#include "afxmt.h"
#include <winsock2.h>

using namespace std;
enum WhichDB { SQLServer,Access2000,Access2007 };

struct D_Point
{
	double D_X,D_Y;
	//double Dir;
};
struct I_Point
{
	int I_X,I_Y;
};
struct MidLane_Res
{
	bool Mid_result;
	int num;
};
struct net_set
{
	CString net_address;
	CString net_channel;
};
/************************************************************************/
/* �����µĵ�ͼ��ʽ                                                                     */
/************************************************************************/
struct NewPoint//�µĵ�ͼդ��㶨��
{
	int contant;//��I_map����Ķ�����ȫһ��
	int ID;//��ǰ�ϰ������ϰ����б��еı�ţ����û���ϰ�������Ϊ-1
	int collision;//Ԥ���ڸô��Ƿ�ᷢ����ײ��1Ϊ����ײ��0Ϊ����,2�����˵���ײ��
	int ID1,ID2;//Ԥ�����ײ�����ĸ���ŵ��ϰ��-1��ʾ��û���
	double v;
	double dir;
	double danger;
};
struct NewMap
{
	NewPoint newmap[512][512];
};
struct ObjectWT 
{ 
	unsigned short Object_Id;//ID
	unsigned short Object_Age;//Age

	short ObjectBox_CenterX;//ObjectBox���ĵ�X
	short ObjectBox_CenterY;//ObjectBox���ĵ�Y
	unsigned short ObjectBox_SizeX;//ObjectBox��СX
	unsigned short ObjectBox_SizeY;//ObjectBox��СY

	float ObjectBox_orientation;//����

	float Relative_VelocityX;//����ٶ�X
	float Relative_VelocityY;//����ٶ�Y


	unsigned short Classification;//���� 0����δ���࣬1����δ֪С���壬2����δ֪�����壬3�������ˣ�4�������г���5����γ���6��������7������
	unsigned short Classification_Age;//�����ɨ�����
	unsigned short Classification_Certainty;//���������

	unsigned short iContour_Points;//����������
	short iContour_X[300];
	short iContour_Y[300];
	short danger;
};
struct I_Map
{
	int MapPoint[512][512];
};
struct v_Map
{
	float MapPoint[512][512];
};
struct Tentacle//1������
{
	CvPoint2D64f OnePoint[400];
	double Angle;
};
struct GTentacle
{
	Tentacle OneTentacle[81];
	int SpeedGroup;
};
struct S_Result
{
	int TentacleID;
	double Distance;
	int brake;
	int way_switch;
	int contrl;// 1��2��3��4 = ͣ�������٣�ԭ�٣�����
	int speed;

};
struct Map_Point                          
{
	int x,y;                         
};
struct CDriveState
{
	CString init_upstate;
	int init_road_point;
	double init_direction;
	double init_speed;
};
struct Point3DInt
{
    int m;
    int n;
	int p;
};

//struct T_Shuzu
//{
//	double aa[41];
//	double bb[41];
//};

//CEvent eventWriteD;
#endif