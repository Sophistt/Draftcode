/*
*Copyright(c) 2012,�п�Ժ�Ϸ������о�Ժ���ܳ�������
*All rights reserved.
*
*��ǰ�汾��1.0
*���ߣ�������
*������ڣ�
*
*ԭ���ߣ��¼Ѽ�
*������ڣ�2011
*/

#ifndef  UTURN_H  //��ֹU_Turn���ظ�ʹ��
#define UTURN_H


#pragma once
#include "cxtypes.h "
const int seq=4;                              //��ʼ���
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