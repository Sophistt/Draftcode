#pragma once

#include "GetGPSData.h"
#include "pid_reg3.h"
#include "StdAfx.h"
#include "CANBus.h"
#include "vehicle.h"

#define WM_INTERVAL_EXE WM_USER+9998
#define LAMP_OFF 0
#define LEFT_LAMP_ON 1
#define RIGHT_LAMP_ON 2
#define HORN_ON 1
#define HORN_OFF 0
#define EPB_NOMAL 0
#define EPB_ON 1
#define EPB_OFF 2

typedef struct //控制结构
{
	float steerAngle;	//[-780,779.9]°
	UINT brake;			//[0,100]%
	float driveTorque;	//[-3000,3000]Nm
	UINT hornOn;		//0表示不响,1表示响
	UINT light;			//0表示不亮，1表示左转向灯亮，2表示右转向灯亮
	UINT epb;			//电子手刹，0表示常态，1表示拉紧，2表示松开
}IVMCONTROL;

class CControl : public CCANBus, public CVehicle
{
public:
	CControl(void);
	~CControl(void);

public://方法

	struct Uudpdatarcv
		{
			float realSpeed;
			float realSteerAngle;
			float torque;
			float brkposact;
			float brkpresact;
			bool VehEmergencyStop;
			unsigned short int offset_l;
			unsigned short int offset_r;
			unsigned short int ApaRetInfo;
			/*
			double Left_A_C3;
			double Left_A_C2;
			double Left_A_C0;
			double Left_A_C1;
			unsigned short int Left_A_Quality;
			unsigned short int Left_A_LineType;
			unsigned short int Left_A_ViewRange;
			double Left_B_C3;
			double Left_B_C2;
			double Left_B_C0;
			double Left_B_C1;
			unsigned short int Left_B_Quality;
			unsigned short int Left_B_LineType;
			unsigned short int Left_B_ViewRange;
			double Right_A_C3;
			double Right_A_C2;
			double Right_A_C0;
			double Right_A_C1;
			unsigned short int Right_A_Quality;
			unsigned short int Right_A_LineType;
			unsigned short int Right_A_ViewRange;
			double Right_B_C3;
			double Right_B_C2;
			double Right_B_C0;
			double Right_B_C1;
			unsigned short int Right_B_Quality;
			unsigned short int Right_B_LineType;
			unsigned short int Right_B_ViewRange;
			unsigned short int ObstacleTotalNum;
			unsigned short int Obstacle_ID[30];
			unsigned short int Obstacle_Type[30];
			unsigned short int Obstacle_Quality[30];
			unsigned short int Obstacle_PosX[30];
			short int Obstacle_PosY[30];
			unsigned short int Obstacle_Width[30];
			unsigned short int Obstacle_Length[30];
			short int Obstacle_VehcleX[30];
			short int Obstacle_VehcleY[30];
			*/
		} udpdatarcv;

	double halfwidth;

	// 发送控制量到CAN总线
	UINT sendCtrlParamToCanBus(void);
	//控制喇叭，horn取值：HORN_ON,HORN_OFF
	void setHorn(UINT horn){m_controlParam.hornOn = horn;}; 
	//控制灯光，lgt取值：LAMP_OFF，LEFT_LAMP_ON，RIGHT_LAMP_ON
	void setLight(UINT lgt){m_controlParam.light = lgt;};	
	//控制手刹，epb取值：EPB_NOMAL, EPB_ON, EPB_OFF
	void setEPB(UINT epb){m_controlParam.epb = epb;};		
	// 停车,若是路点结束需要停车，将endFlag赋值true
	void ctrlStop(bool pointEndFlag = false);
	int zStop(CvPoint2D64f end );
	void ClearzStop(bool bclearall);
	//计算预瞄距离
	float GetAimLen(double speed);
	//计算预瞄点
	bool GetAimPoint(CvPoint2D64f currentPosition, double currentDirection, float aimLength,
					 double speed, CvPoint2D64f &aimPosition, double &aimDirection);
	//计算控制量
	void GetCtrlParameters( CvPoint2D64f currentPosition, double currentDirection, 
							CvPoint2D64f *aimPosition, double aimDirection, double speed);

	double SearchFrontOb(I_Map *map,double range)/*正前 絩ange米内有无障碍 */;
	double SearchRearOb(I_Map *map,double range)/*正前 絩ange米内有无障碍 */;
	double SearchRearObForApa(I_Map *map,double range)/*正前 絩ange米内有无障碍 */;
	double SearchrightFrontOb(I_Map *map,double range);
	double SearchleftFrontOb(I_Map *map,double range);

	int Lkahaikanglanecnt;

private://方法
	//计算侧向误差
	double GetLatError( CvPoint2D64f currentPosition, double currentDirection, CvPoint2D64f aimPosition);
	//纵向控制
	void longitudinalControl(double speed, float &torque, UINT &brake);	

	void longitudinalControlLKA(double disirespd, double spdLKAacc, double speed, double &torque, double &brake);
	//侧向控制
	float lateralControl(CvPoint2D64f currentPosition, double currentDirection, 
						 CvPoint2D64f *aimPosition, double aimDirection, double speed);
	float slidingModeControlForSpeed(double speed, double &lastM2);
	// 符号函数
	int sign(double data);
	// 饱和函数，输入乘以factor以后计算饱和，饱和输出-1或1，不饱和输出in*factor
	int sat(double in, double factor=1);
	// 预瞄点滤波
	void filterByTime(CvPoint2D64f *pt, CvPoint2D64f &aimPoint);
	
public://属性
	//HANDLE  m_hEvent;//路径更新消息
	//CvSeq* m_planRoad;//存储决策路点
	double m_uDesire;//期望速度,单位m/s
	UINT m_lastBrake;	//上一次的制动量
	CvSeq* m_roadPoint;	//存储控制使用的路点
	bool m_endControlThreadFlag;//控制线程结束标志，置1结束控制线程
	IVMCONTROL m_controlParam;//控制量

	float laststrldw;
	double LKAlastbrake;
	double LKAlastAcc;
	float fdbhis[5];
	PIDREG3 m_pidLatLdw;

private://属性
	PIDREG3 m_pidLat;	//侧向误差控制器

	PIDREG3 m_pidDrive;	//驱动控制器
	double m_lastM2;// 滑模控制用到的中间变量，上一步的M2值
	bool m_stopAtPoint;//定点停车标志
	CvPoint2D64f m_endPoint;//停止点
	double m_brakeSpeed0;
	bool m_stopAtLine;
	double m_stopLineSpeed0;
	int lasttorque;
	int con;
	int con1;
	int con2;
	int con3;
	int con4;

};

