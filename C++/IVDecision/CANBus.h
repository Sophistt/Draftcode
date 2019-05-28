#pragma once
#include "ControlCAN.h"
#include "IVDecision.h"
#include <vector>
using namespace std;

#define CAN_DATA_SIZE 8



class CCANBus
{
public:
	CCANBus(void);
	~CCANBus(void);

public:	
	// 启动CAN
	UINT startCAN(void);
	// 复位CAN
	UINT resetCAN(void);
	// 发送所有信息
	UINT sendMessage();

public:	//仅用于测试
	// 喇叭，time表示喇叭响的时间，
	UINT horn(ULONG time);
	// 左转向灯开
	UINT leftLampOn(void);
	// 左转向灯关
	UINT leftLampOff(void);
	// 右转向灯开
	UINT rightLampOn(void);
	// 右转向灯关
	UINT rightLampOff(void);
	// 发送对地转矩
	UINT sendTorque(float torque);
	// 发送制动程度
	UINT sendBrake(UINT brake);
	// 发送方向盘转角
	UINT sendSteerAngle(float angle);
	// 喇叭关
	UINT hornOff(void);
	// 设置手刹
	UINT setEPB(UINT mode);
	
public:	
	struct CANMessageRecv
	{
		UINT breakSysFault;		//制动系统故障,0表示正常
		UINT EPSSysFault;		//转向系统故障,0表示正常
		UINT VehSoftStop;		//动力不足故障,0表示正常
		UINT driveMode;			//驾驶模式：1人工，2遥控，3自动
		float TqGndPosMax;		//最大正对地扭矩
		float TqGndNegMax;		//最大负对地扭矩，始终小于零
		float realTorque;		//实际对地扭矩
		float realBrakePressure;	//实际制动压力,[0,10]MPa
		float realSteerAngle;	//实际方向盘转角
		float realSpeed;		//实际车速,km/h
		UINT SCUFault;			//SCU故障,0表示正常
		UINT VehEmergencyStop;	//失去动力故障,0表示正常
		UINT EPBMode;			//EPB模式，0常态，1拉紧，2松开
		UINT EVBFault;			//EVB故障，0表示正常
		int offset_l;
		int offset_r;
		int ApaRetInfo;

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

	}m_canMsgRecv;	//接收到的CAN信息
	struct CANMessageSend
	{
		float steerAngle;	//[-780,779.9]°,左正右负
		float driveTorque;	//[-3000,3000]Nm
		UINT brake;			//[0,100]%
		UINT hornOn;		//喇叭, 0表示不响
		UINT turnLamp;		//0表示不亮，1表示左转，2表示右转
		UINT EPBActive;		//电子手刹请求，0常态，1拉紧，2松开
	}m_canMsgSend;	//要发送的CAN信息
	struct AVFault
	{
		UINT HDL_ConnectFault;
		UINT IBEO1_ConnectFault;
		UINT IBEO2_ConnectFault;
		UINT Camera1_ConnectFault;
		UINT Camera2_ConnectFault;
		UINT Camera3_ConnectFault;
		UINT ESR1_ConnectFault;
		UINT ESR2_ConnectFault;
		UINT ESR3_ConnectFault;
		UINT ESR4_ConnectFault;
		UINT CAN_ConnectFault;
		UINT GPS_ConnectFault;
		UINT Ethernet_ConnectFault;
	}m_avFault;	//传感器故障

private:	
	//接收线程
	void startThread(void);
	static DWORD threadEntry(LPVOID lpParam);
	DWORD receiveThread(void);
	// 解析接收到的CAN信号
	UINT CANDecoder(const VCI_CAN_OBJ *frameInfo, ULONG len);
	// 解码0x120
	void decode120(const VCI_CAN_OBJ* frameInfo);
	// 解码0x121
	void decode121(const VCI_CAN_OBJ* frameInfo);
	// 解码0x123
	void decode416(const VCI_CAN_OBJ* frameInfo);
	// 解码0x350
	void decode350(const VCI_CAN_OBJ* frameInfo);

	void decode510(const VCI_CAN_OBJ* frameInfo);
	void decode511(const VCI_CAN_OBJ* frameInfo);
	void decode512(const VCI_CAN_OBJ* frameInfo);
	void decode513(const VCI_CAN_OBJ* frameInfo);
	void decode514(const VCI_CAN_OBJ* frameInfo);

	// 发送值限幅
	UINT sendLimiting(void);
	//导航仪数据解码
	void DecodeADASISData(const BYTE buf[], BYTE cycn, BYTE type);

private:	
	DWORD m_devIndex;// 设备索引号
	DWORD m_CANIndex;// 第几路CAN
	ULONG m_frameID;// 发送帧ID
	BOOL m_canOpenFlag;//设备打开标志
	BYTE m_sendType;	//发送类型，0表示正常发送，2表示自发自收	
public:
	// PC请求加密当前GPS经纬度值
	UINT GPSEncAsk(double GpsLatitude, double GpsLongitude);
	// 初始化传感器故障结构体
	void AVFaultDefault(void);
	// 发送传感器故障
	UINT sendAVFault(void);
	// 请求发送ADASIS地图规划信息和重新发送地图数据
	UINT sendAVReq(UINT8 MapPlaningReq, UINT8 MapDataResendReq);


public://导航仪补充

	//路网点
	struct RNPoint
	{
		double lng;		//经度
		double lat;		//纬度
		double height;	//高度
		int type;		//类型
		int offset;
		int type1;
		int pathidx;
		int lanenum;//车道数
	};

	struct Segment
	{
		int id;
		int pathindex;
		int lanenum;
		int offset;
	};

	struct Position
	{
		int pathindex;
		int offset;
	};

	struct Stub
	{
		int id;
		int pathindex;
		int offset;
		int type;
	};

	struct Longitute
	{
		int pathindex;
		int offset;
		double val;
	};

	struct Latitude
	{
		int pathindex;
		int offset;
		double val;
	};

	Position m_curPosition;

	vector<Segment> m_segQueue;
	vector<Stub> m_stubQueue;
	vector<RNPoint> m_rnPts;

	//15.09.15
	vector<Stub> m_profile_short_type17;
	int m_prs_type17_array[30];
	unsigned int m_prs_type17_array_index;

	RNPoint m_pt;
	RNPoint m_prePt;

	int m_iPreType;
	int m_iSegPreCyCn;
	int m_iStuPreCycn;
	int m_iPrlPreCycn;
	int m_iPreSndType;
	int m_iPrsPreCycn;//hqj
	int m_iErr;
	int m_iRcv;
	int m_iNumStub;
	int m_iNumSeg;
	int m_iNumProLng;
	BOOL m_bJunction;

	double m_dLngOffset;
	double m_dLatOffset;

	double m_dEnLng;
	double m_dEnLat;

	//***********//
	CvMemStorage* storage;

	int adas_id;
};

