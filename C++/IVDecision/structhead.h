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
/* 定义新的地图格式                                                                     */
/************************************************************************/
struct NewPoint//新的地图栅格点定义
{
	int contant;//跟I_map里面的定义完全一致
	int ID;//当前障碍物在障碍物列表中的编号，如果没有障碍物则设为-1
	int collision;//预测在该处是否会发生碰撞，1为会碰撞，0为不会,2是行人的碰撞点
	int ID1,ID2;//预测的碰撞来自哪个编号的障碍物，-1表示还没求出
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

	short ObjectBox_CenterX;//ObjectBox中心点X
	short ObjectBox_CenterY;//ObjectBox中心点Y
	unsigned short ObjectBox_SizeX;//ObjectBox大小X
	unsigned short ObjectBox_SizeY;//ObjectBox大小Y

	float ObjectBox_orientation;//航向

	float Relative_VelocityX;//相对速度X
	float Relative_VelocityY;//相对速度Y


	unsigned short Classification;//分类 0代表未分类，1代表未知小物体，2代表未知大物体，3代表行人，4代表自行车，5代表轿车，6代表卡车，7保留字
	unsigned short Classification_Age;//分类后扫描次数
	unsigned short Classification_Certainty;//分类可能性

	unsigned short iContour_Points;//轮廓点数量
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
struct Tentacle//1条触须
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
	int contrl;// 1、2、3、4 = 停车，减速，原速，加速
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