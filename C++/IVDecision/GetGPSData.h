#pragma once
#include "IVDecision.h"
#define XJYUE_LENGTH 512 //2048
#define WM_INTERVAL_DATA WM_USER+9999
#define CRC32_POLYNOMIAL 0xEDB88320L
#define  EARTH_RADIUS   6378.137
#define  PI  3.1415926535897932

class CGetGPSData
{

public:
	CGetGPSData(void);
	~CGetGPSData(void);

	bool SetCom(CString strCom,CString strBaud);
    void StartGPSthread();	
	static DWORD theGPSProcess(LPVOID lpParam);
	DWORD StubGPSProcess();
	static DWORD theDataProcess(LPVOID lpParam);
	DWORD StubDataProcess();
	ULONG	ValueCRC32(int i);
	BOOL CalculateBlockCRC32(unsigned long u1Count,unsigned char *ucBuffer);   
	//GPS秒转换
	CString GPSSecond(double gpsSecond);
	
private:
	void DecoderINSPVAData(LPCSTR data, int length);
	void DecoderCORRIMUData(LPCSTR data, int length);
public:
    DWORD dwGPSThreadId;
	HANDLE m_hGPSThread;
	DWORD  dwDataThreadId;  // 数据处理线程Id
	HANDLE  m_hThreadData; // 数据处理线程句柄
private:
    char szRecv[XJYUE_LENGTH];  
    int iRecLength ;
	 //winapi 串口数据定义
	DWORD        m_dwBeginSample; // 开始采样机器数据
	HANDLE       m_hGPSCom;		 // 串口文件句柄
	DCB          dcb;            // 串口波特率设置
	COMMTIMEOUTS CommTimeOuts;   // 串口工作时间参数
	OVERLAPPED   olRead; 

public:
	//CString strGPSQual;
	CString strTime; 
	CString strLatitude; 
	CString strLongitude; 
	//CString strHeight; 
	CString strNorth;     
	CString strEast;     
	CString strUp;  
	//CString strRoll;     
	//CString strPitch;     
	CString strAzimuth;  
	CString strINSstatus;
	// 分析数据帧

	/////%CORRIMUDATASA
	//CString strGPSQual;
	CString strPitchRate; 
	CString strRollRate; 
	CString strYawRate; 
	CString strLateralAcc;     
	CString strLongitudinalAcc;     
	CString strVerticalAcc;  
	//CString strRoll;     
	//CString strPitch;     

	// 分析数据帧
public:
	CvPoint2D64f GetPosition();
	double GetDistance(double lat1, double lng1, double lat2, double lng2);
	Map_Point APiontConver(CvPoint2D64f v,CvPoint2D64f a,double direction);
	CvPoint2D64f APiontConverD(CvPoint2D64f v,CvPoint2D64f a,double direction);
	double GetAngle(double lat1, double lng1, double lat2, double lng2);
	double GetAngle(CvPoint2D64f a, CvPoint2D64f b);
	double GetSpeed();
    double GetDirection();
    double rad(double d) { return d * PI / 180.0; };
	CvPoint2D64f MaptoGPS(CvPoint2D64f v,double dir,CvPoint2D64f a);

	CvPoint2D64f GPS_Point_last;
	bool bGpsReceived;

public:
	HANDLE m_GpsEvent;
};
