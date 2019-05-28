// IVDecision.h : PROJECT_NAME 应用程序的主头文件
//

#pragma once
#include <queue>
#include "StructHead.h"
#include <afxtempl.h>
#ifndef __AFXWIN_H__
	#error "在包含此文件之前包含“stdafx.h”以生成 PCH 文件"
#endif

#include "resource.h"		// 主符号


// CIVDecisionApp:
// 有关此类的实现，请参阅 IVDecision.cpp
//
struct LEAD
{
	int id;
	double lng;
	double lat;
	double height;
	int param1;
	float param2;
	int param3;
};

typedef struct CvPoint5D64f_type
{
    double x;
    double y;
    double z;
	int laneseq;
	int lanenum_;
	double x_left;
	double y_left;
	double x_right;
	double y_right;
	bool b_left;
	bool b_right;
	int spdiniset;
	int laneexist;
};

class CIVDecisionApp : public CWinApp
{
public:
	////2011
	bool mapreceived;
	net_set GlobalSendNetSet;
	net_set GlobalRecNetSet;
	I_Map * PercepMap;
	int lane_count;
	int cuo_zhen;
	bool tj_flag;
	bool zuo_huandao;
	bool you_hundao;
	bool stop_lane; 
	////
	
	CIVDecisionApp();
	queue<CvPoint2D64f> APoint;//目标
	queue<double>RT_Speed;//实时速度
	queue<double>RT_Angle;//实时方向盘转角
	queue<double>RT_Direction;//实时方向
	queue<CvPoint2D64f>RT_Station;//实时位置
	queue<CvPoint3D64f>A_PositionQ;
	queue<I_Map>RT_Map;
	queue<I_Map>Way_Map;
	queue<S_Result>RT_Result;
	queue<int>Traffic_Sign;
	queue<LEAD>Lead_pt;
	queue<LEAD>mission_pt;
	CvSeq *ADAS_points;
	CvSeq *gpsmap_road;

	CvSeq *gpsmap_road_ApaPath1;
	CvSeq *gpsmap_road_ApaPath2;
	CvSeq *gpsmap_road_ApaPath3;

	queue<double>LOG_r;//记录半径
	queue<double>LOG_Direction;//记录方向
	queue<CvPoint2D64f>LOG_Position;//记录位置
	//queue<IVMCONTROL>LOG_ctrl;//记录控制量
	DWORD dwProcThreadId;
	queue<SYSTEMTIME>TIME_proc;//系统时间
	queue<SYSTEMTIME>TIME_send;
	//公用数据
	I_Map road_Map;
	CCriticalSection critical_map;//gps临界区
	CCriticalSection critical_section;//gps临界区
	CCriticalSection critical_planningroad;//临界区
	CCriticalSection critical_pathplanroad;
	CCriticalSection critical_light;
	CCriticalSection critical_winsockread;
	CCriticalSection critical_winsockread2;
	CCriticalSection critical_xinmap;//锁定xingmap
	CCriticalSection critical_dyob;//锁定xingmap

	CCriticalSection critical_lukoumap;

	CCriticalSection critical_haikanglock;


	CCriticalSection critical_adasdata;
	CCriticalSection critical_adasisupdate;
	CvPoint2D64f  GPS_Point;//公用gps点，x-纬度，y-经度
	double GPS_Speed;//公用gps速度
	double 	GPS_NorthSpeed;
	double 	GPS_EastSpeed;
	double GPS_Direction;//公用gps方向

	double GPS_YawRate;
	double GPS_LonAcc;
	int sick_num;
	int state1;

	double cardl;
	double cardr;
	double cspeed;

	CString m_GpsFile;
	bool rec_flag;

	HANDLE m_MapEvent;//2012.6
	HANDLE m_Ibeo1Event;
	HANDLE m_Ibeo2Event;
	HANDLE m_hEvent;
	
	CvSeq* plan_road;
	I_Map* lane_Map;
	double v_desire;
	double v_aim_length;
	//CvPoint2D64f MidGpsPoint[200];
	CString topDecison;
	CString secondDecison;
	int left_right;
	bool sendflag1;
	int onroad;
	double vehicle_speed;
	int speed_status;
	bool stop;
// 重写
	public:
	virtual BOOL InitInstance();
	bool u_t;
	bool bshigongluduanlock;
	bool bguihualukou;

	bool road_ut;
	bool road_goal;
	bool update_flag;
	bool range_flag;
	DWORD perceptimelast;
	DWORD percepIbeoMaptimelast;
	DWORD IbeoMaptimeStamp;
	DWORD proctimelast;
	DWORD proctimeStamp;
	bool inter_small;
	bool inter_UTURN;

	bool Get_Sign;
	bool raozhang;
	bool detpeople;
	double drive_state;
	double drive_obstacle;
	double drive_obstacle2;
	double drive_curvature;
	double drive_people;
	double ob_hxerror;
	// 实现
	int on_road;
	bool stop_map;
	double stopline_dis;
	int light_res;
	double limit_speed;
	 bool limitflag;
	 bool brake_flag;
	 bool goal_flag;
	 bool goal_leftright;

	 bool continue_brake;
	 bool continue_braketmp;

	 bool continue_spdred;
	 bool continue_spdredtmp;
	 double spdredacc;
	 double spdredacctmp;

	 bool obb_right;
	 bool obb_left;

	 bool bzheng;

	 int lanechangenum;

	 bool bGpsMapLoad;

	 double drive_state_update;
	 double drive_curvature_update;
	 double drive_obstacle_update;
	 double drive_obstacle2_update;
	 double limit_speed_update;

	 bool leftturnreq;
	 bool rightturnreq;

	 bool leftturnreq_proc;
	 bool rightturnreq_proc;

	 bool lanekeepreq;
	 double lanekeeptime;
	 bool lanekeepreqhspdapp;

	 bool bdirectrunreq;

	 bool blanechanging;

	 int laneturndir;//1 left; 2 right.

	 int lukoudirnum;
	 int lukoulanechangereq;//1=left,2=right;

	 int ob_width_111;

	 bool braohui;

	 bool bchulukoufound;

	 bool bbanmaxiannoraozhang;

	 bool bjianruiuturn;

	 bool bbiandaoban;

	 bool bthirduturn;

	 bool fangxiangpanbuhuizheng;

	 bool bqishiluduan;

	 int bspecialuturnluduan;

	 int bspecialnoleftturnluduan;

	 bool bleftturnban;
	 bool brightturnban;

	 bool bnobiandaoraozhang;
	 
	 int laneseq_set;
	 int lanenum_set;
	 int speediniset;
	 int laneexist;

	 bool bleftsidelane;
	 bool brightsidelane;

	 int m_task2ctrl;
	 double road_speed2ctrl;


	 bool bGpsGanrao;
	 bool bLKAsuidao;

	 int bApaActive;
	 bool bcheweistrtsearch;
	 int PXApaActive;
	 bool bcheweistrtsearchPX;

	 int bSuidao1Active;
	 int bSuidao2Active;
	 
	 CvPoint2D64f rndfbezier_left[200];
	 CvPoint2D64f rndfbezier_right[200];

	 int uturndaoche;

	 bool bisaiwaiburoad;

	 bool bnearstop;
	 bool bspecialraozhangcanshu;

	 int kuaisulufulu;

	 int qidongstatus;

	 int yinhangdianstatus;

	 int rouchestatus;

	 double realsteerangle;

	 int systemtimer10ms;
	 int systemtimer10ms_his;

	bool AimFlag_ApaPath1;
	bool AimFlag_ApaPath2;
	bool AimFlag_ApaPath3;

	int bjiedaochaoche;

	DECLARE_MESSAGE_MAP()
};

extern CIVDecisionApp theApp;