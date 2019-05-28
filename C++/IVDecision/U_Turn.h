/*
*Copyright(c) 2012,中科院合肥物质研究院智能车辆中心
*All rights reserved.
*
*当前版本：1.0
*作者：黄如林
*完成日期：
*
*原作者：陈佳佳
*完成日期：2011
*/

#ifndef  UTURN_H  //防止U_Turn被重复使用
#define UTURN_H


#pragma once
#include "cxtypes.h "
const int seq=4;                              //初始序号
class U_Turn
{
public:
	U_Turn(void);
	~U_Turn(void);
    CvPoint2D64f path[200];
    CvPoint2D64f MaptoGPS(CvPoint2D64f v,double dir,CvPoint2D64f a);
    void SetVehicle(CvPoint2D64f pos, double dir, I_Map map, LEAD *leadpoint, CvPoint2D64f waypoint[]); 
    CvPoint2D64f GetMidPoint();
	CvPoint2D64f *GetPath();
	CvPoint2D64f Bezier(CvPoint2D64f p[], int n, CvPoint2D64f (&MidPoint)[200]);

private:
	CvPoint2D64f m_pos;
    double m_dir;
	I_Map m_map;
	LEAD *m_leadpoint;
	
    CvPoint2D64f m_waypoint[4];
};
#endif;