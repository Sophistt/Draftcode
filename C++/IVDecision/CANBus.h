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
	// ����CAN
	UINT startCAN(void);
	// ��λCAN
	UINT resetCAN(void);
	// ����������Ϣ
	UINT sendMessage();

public:	//�����ڲ���
	// ���ȣ�time��ʾ�������ʱ�䣬
	UINT horn(ULONG time);
	// ��ת��ƿ�
	UINT leftLampOn(void);
	// ��ת��ƹ�
	UINT leftLampOff(void);
	// ��ת��ƿ�
	UINT rightLampOn(void);
	// ��ת��ƹ�
	UINT rightLampOff(void);
	// ���ͶԵ�ת��
	UINT sendTorque(float torque);
	// �����ƶ��̶�
	UINT sendBrake(UINT brake);
	// ���ͷ�����ת��
	UINT sendSteerAngle(float angle);
	// ���ȹ�
	UINT hornOff(void);
	// ������ɲ
	UINT setEPB(UINT mode);
	
public:	
	struct CANMessageRecv
	{
		UINT breakSysFault;		//�ƶ�ϵͳ����,0��ʾ����
		UINT EPSSysFault;		//ת��ϵͳ����,0��ʾ����
		UINT VehSoftStop;		//�����������,0��ʾ����
		UINT driveMode;			//��ʻģʽ��1�˹���2ң�أ�3�Զ�
		float TqGndPosMax;		//������Ե�Ť��
		float TqGndNegMax;		//��󸺶Ե�Ť�أ�ʼ��С����
		float realTorque;		//ʵ�ʶԵ�Ť��
		float realBrakePressure;	//ʵ���ƶ�ѹ��,[0,10]MPa
		float realSteerAngle;	//ʵ�ʷ�����ת��
		float realSpeed;		//ʵ�ʳ���,km/h
		UINT SCUFault;			//SCU����,0��ʾ����
		UINT VehEmergencyStop;	//ʧȥ��������,0��ʾ����
		UINT EPBMode;			//EPBģʽ��0��̬��1������2�ɿ�
		UINT EVBFault;			//EVB���ϣ�0��ʾ����
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

	}m_canMsgRecv;	//���յ���CAN��Ϣ
	struct CANMessageSend
	{
		float steerAngle;	//[-780,779.9]��,�����Ҹ�
		float driveTorque;	//[-3000,3000]Nm
		UINT brake;			//[0,100]%
		UINT hornOn;		//����, 0��ʾ����
		UINT turnLamp;		//0��ʾ������1��ʾ��ת��2��ʾ��ת
		UINT EPBActive;		//������ɲ����0��̬��1������2�ɿ�
	}m_canMsgSend;	//Ҫ���͵�CAN��Ϣ
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
	}m_avFault;	//����������

private:	
	//�����߳�
	void startThread(void);
	static DWORD threadEntry(LPVOID lpParam);
	DWORD receiveThread(void);
	// �������յ���CAN�ź�
	UINT CANDecoder(const VCI_CAN_OBJ *frameInfo, ULONG len);
	// ����0x120
	void decode120(const VCI_CAN_OBJ* frameInfo);
	// ����0x121
	void decode121(const VCI_CAN_OBJ* frameInfo);
	// ����0x123
	void decode416(const VCI_CAN_OBJ* frameInfo);
	// ����0x350
	void decode350(const VCI_CAN_OBJ* frameInfo);

	void decode510(const VCI_CAN_OBJ* frameInfo);
	void decode511(const VCI_CAN_OBJ* frameInfo);
	void decode512(const VCI_CAN_OBJ* frameInfo);
	void decode513(const VCI_CAN_OBJ* frameInfo);
	void decode514(const VCI_CAN_OBJ* frameInfo);

	// ����ֵ�޷�
	UINT sendLimiting(void);
	//���������ݽ���
	void DecodeADASISData(const BYTE buf[], BYTE cycn, BYTE type);

private:	
	DWORD m_devIndex;// �豸������
	DWORD m_CANIndex;// �ڼ�·CAN
	ULONG m_frameID;// ����֡ID
	BOOL m_canOpenFlag;//�豸�򿪱�־
	BYTE m_sendType;	//�������ͣ�0��ʾ�������ͣ�2��ʾ�Է�����	
public:
	// PC������ܵ�ǰGPS��γ��ֵ
	UINT GPSEncAsk(double GpsLatitude, double GpsLongitude);
	// ��ʼ�����������Ͻṹ��
	void AVFaultDefault(void);
	// ���ʹ���������
	UINT sendAVFault(void);
	// ������ADASIS��ͼ�滮��Ϣ�����·��͵�ͼ����
	UINT sendAVReq(UINT8 MapPlaningReq, UINT8 MapDataResendReq);


public://�����ǲ���

	//·����
	struct RNPoint
	{
		double lng;		//����
		double lat;		//γ��
		double height;	//�߶�
		int type;		//����
		int offset;
		int type1;
		int pathidx;
		int lanenum;//������
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

