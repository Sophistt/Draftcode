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

typedef struct //���ƽṹ
{
	float steerAngle;	//[-780,779.9]��
	UINT brake;			//[0,100]%
	float driveTorque;	//[-3000,3000]Nm
	UINT hornOn;		//0��ʾ����,1��ʾ��
	UINT light;			//0��ʾ������1��ʾ��ת�������2��ʾ��ת�����
	UINT epb;			//������ɲ��0��ʾ��̬��1��ʾ������2��ʾ�ɿ�
}IVMCONTROL;

class CControl : public CCANBus, public CVehicle
{
public:
	CControl(void);
	~CControl(void);

public://����

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

	// ���Ϳ�������CAN����
	UINT sendCtrlParamToCanBus(void);
	//�������ȣ�hornȡֵ��HORN_ON,HORN_OFF
	void setHorn(UINT horn){m_controlParam.hornOn = horn;}; 
	//���Ƶƹ⣬lgtȡֵ��LAMP_OFF��LEFT_LAMP_ON��RIGHT_LAMP_ON
	void setLight(UINT lgt){m_controlParam.light = lgt;};	
	//������ɲ��epbȡֵ��EPB_NOMAL, EPB_ON, EPB_OFF
	void setEPB(UINT epb){m_controlParam.epb = epb;};		
	// ͣ��,����·�������Ҫͣ������endFlag��ֵtrue
	void ctrlStop(bool pointEndFlag = false);
	int zStop(CvPoint2D64f end );
	void ClearzStop(bool bclearall);
	//����Ԥ�����
	float GetAimLen(double speed);
	//����Ԥ���
	bool GetAimPoint(CvPoint2D64f currentPosition, double currentDirection, float aimLength,
					 double speed, CvPoint2D64f &aimPosition, double &aimDirection);
	//���������
	void GetCtrlParameters( CvPoint2D64f currentPosition, double currentDirection, 
							CvPoint2D64f *aimPosition, double aimDirection, double speed);

	double SearchFrontOb(I_Map *map,double range)/*��ǰ �range���������ϰ� */;
	double SearchRearOb(I_Map *map,double range)/*��ǰ �range���������ϰ� */;
	double SearchRearObForApa(I_Map *map,double range)/*��ǰ �range���������ϰ� */;
	double SearchrightFrontOb(I_Map *map,double range);
	double SearchleftFrontOb(I_Map *map,double range);

	int Lkahaikanglanecnt;

private://����
	//����������
	double GetLatError( CvPoint2D64f currentPosition, double currentDirection, CvPoint2D64f aimPosition);
	//�������
	void longitudinalControl(double speed, float &torque, UINT &brake);	

	void longitudinalControlLKA(double disirespd, double spdLKAacc, double speed, double &torque, double &brake);
	//�������
	float lateralControl(CvPoint2D64f currentPosition, double currentDirection, 
						 CvPoint2D64f *aimPosition, double aimDirection, double speed);
	float slidingModeControlForSpeed(double speed, double &lastM2);
	// ���ź���
	int sign(double data);
	// ���ͺ������������factor�Ժ���㱥�ͣ��������-1��1�����������in*factor
	int sat(double in, double factor=1);
	// Ԥ����˲�
	void filterByTime(CvPoint2D64f *pt, CvPoint2D64f &aimPoint);
	
public://����
	//HANDLE  m_hEvent;//·��������Ϣ
	//CvSeq* m_planRoad;//�洢����·��
	double m_uDesire;//�����ٶ�,��λm/s
	UINT m_lastBrake;	//��һ�ε��ƶ���
	CvSeq* m_roadPoint;	//�洢����ʹ�õ�·��
	bool m_endControlThreadFlag;//�����߳̽�����־����1���������߳�
	IVMCONTROL m_controlParam;//������

	float laststrldw;
	double LKAlastbrake;
	double LKAlastAcc;
	float fdbhis[5];
	PIDREG3 m_pidLatLdw;

private://����
	PIDREG3 m_pidLat;	//������������

	PIDREG3 m_pidDrive;	//����������
	double m_lastM2;// ��ģ�����õ����м��������һ����M2ֵ
	bool m_stopAtPoint;//����ͣ����־
	CvPoint2D64f m_endPoint;//ֹͣ��
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

