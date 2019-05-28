/*************************************************************
运动规划和基本执行动作类

运动协调
转向控制
刹车控制
油门控制
换挡控制
速度/距离反馈

2010-3-3 ZhengFei
**************************************************************/
#pragma once
#include "IVDecision.h"
#include "IVKeepLane.h"
#include "GetGPSData.h"
#include "SocketTransfer.h"
#include "LoadRNDF.h"
#include "IVLocalPath.h"
#include <math.h>
const double r[81] = { -5.6,-5.8617,-6.12222,-6.55952,-6.8875,-7.44595,-7.87143,-8.60938,-9.18333,-10.2037,-11.02,-11.3843,-11.9783,-12.9953,-13.775,-14.6543,-15.6534,-16.7988,-17.6603,-18.3667,-20.5597,-22.2177,-23.75,-25.5093,-27.55,-31.3068,-35.3205,-38.2639,-41.7424,-43.0469,-45.9167,-51.0185,-55.1,-62.6136,-76.5278,-105.962,-137.75,-196.786,-459.167,-1377.5,100000,1377.5,459.167,196.786,137.75,105.962,76.5278,62.6136,55.1,51.0185,45.9167,43.0469,41.7424,38.2639,35.3205,31.3068,27.55,25.5093,23.75,22.2177,20.5597,18.3667,17.6603,16.7988,15.6534,14.6543,13.775,12.9953,11.9783,11.3843,11.02,10.2037,9.18333,8.60938,7.87143,7.44595,6.8875,6.55952,6.12222,5.8617,5.51};
const int code[81] = { -1e+006,-940000,-900000,-840000,-800000,-740000,-700000,-640000,-600000,-540000,-500000,-484000,-460000,-424000,-400000,-376000,-352000,-328000,-312000,-300000,-268000,-248000,-232000,-216000,-200000,-176000,-156000,-144000,-132000,-128000,-120000,-108000,-100000,-88000,-72000,-52000,-40000,-28000,-12000,-4000,0,4000,12000,28000,40000,52000,72000,88000,100000,108000,120000,128000,132000,144000,156000,176000,200000,216000,232000,248000,268000,300000,312000,328000,352000,376000,400000,424000,460000,484000,500000,540000,600000,640000,700000,740000,800000,840000,900000,940000,1e+006 };
class CIVExecution
{

public:
	CIVExecution(void);
	~CIVExecution(void);

private:
	CvPoint2D64f a_point;
	CvPoint2D64f rPosition;
	Map_Point A_Point;
	double D(CvPoint2D64f p1, Map_Point p2);
	double D(CvPoint2D64f p1, CvPoint2D64f p2);
	GTentacle *tentacles;
	double rDirection;
	S_Result rResult;
	bool onoff_flag;
	bool on_stopline;
	I_Map *LineMap;

public:
	HANDLE  m_hThreadData;
	HANDLE  m_hThreadSend; 
	DWORD dwDataThreadId;
	DWORD dwSendThreadId;

	DWORD SearchAtenna_GPS();
	DWORD SearchAtenna_GPS_u( );
	DWORD SearchAtenna_SMap();
	DWORD SearchAtenna_VMap();

	virtual void OnStart();
	virtual void OnEnd();
	static DWORD theSearch(LPVOID lpParam);

	GTentacle *GetTentacles(int num);
	int GetAID(GTentacle *A_Tentacles,Map_Point A_Position);
	int GetAID(GTentacle *A_Tentacles,CvPoint2D64f A_Position);
	S_Result Stop();
	int GetWay(GTentacle *A_Tentacles,Map_Point *A_Position);
	int GetWay_V(GTentacle *A_Tentacles,I_Map *map);
	bool GoEnable(I_Map *p,int x,int y);
	void SetTrafficSign( );

	void StartSend();
	void EndSend();

	DWORD SendThread();
	static DWORD theSendThread(LPVOID lpParam);
	void StartGPS();
	CGetGPSData GpsData;
	CLoadRNDF RNDF;//GPS坐标
	CLoadRNDF RNDFXY;//XY坐标
	//double FirstPoint(CvPoint2D64f a,CvPoint2D64f b);
	//CvPoint2D64f SecondPoint(CvPoint2D64f a,CvPoint2D64f b);
	int GetRID(double R);
 //   CvPoint2D64f MapPointConv(Map_Point a,CvPoint2D64f b);
	//CvPoint2D64f CountJiaoDian(CvPoint2D64f a,CvPoint2D64f b);
	void UTurn(double lane_angle );
	Map_Point Crossing(int n,int num, Map_Point Point[3],GTentacle *A_Tentacles);//n: 1,2,3 = 左，中，右
	int Crossing(int n,I_Map *RT_Map,GTentacle *A_Tentacles);//n: 1,2,3 = 左，中，右

	bool keeplane_flag;
	bool on_uturn;
	bool on_cross;
	bool on_speedup;
	CIVKeepLane KeepLane;
	CIVLocalPath LocalPath;
	void Convert200(CvPoint2D64f v,CvPoint2D64f *a,double direction,Map_Point (&Ap)[200]);
	void Convert200(CvPoint2D64f v,CvPoint2D64f *a,double direction,CvPoint2D64f (&Ap)[200]);
	void StopLine();
};

