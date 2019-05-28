#include "StdAfx.h"
#include "GetGPSData.h"
#include<fstream>
using namespace std;
	 ofstream out2("gps.txt");
	 ofstream out222("gps2.txt");
	ofstream out333("yawrate.txt");
CGetGPSData::CGetGPSData(void)
{
	m_GpsEvent=CreateEvent(NULL,FALSE,FALSE,NULL);
	CvPoint2D64f GPS_Point_last = cvPoint2D64f(0,0);
	bGpsReceived = 0;
}

CGetGPSData::~CGetGPSData(void)
{
}

/***********************************************************
//串口初始化
输入：
    strCom:串口名称
	strBaud:串口波特率
输出：
	返回建立与否
***********************************************************/
bool CGetGPSData::SetCom(CString strCom,CString strBaud)
{  
    
	DWORD dwStoredFlags;	
	// 初始化读串口文件重叠结构参数
	memset(&olRead, 0, sizeof(OVERLAPPED));
	olRead.hEvent = CreateEvent(NULL, FALSE, FALSE, NULL);		
	// 以异步方式打开串口
	m_hGPSCom=CreateFile(strCom, GENERIC_READ|GENERIC_WRITE, 
		0, NULL, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, NULL);
	if (INVALID_HANDLE_VALUE == m_hGPSCom)
	{
		//AfxMessageBox("SDI Timing Software", MB_ICONSTOP);
		//AfxMessageBox("打开串口失败");
		return FALSE;
	}
	//添加或修改Windows所报告的事件列表
	//SetCommMask (m_hCom, EV_RXCHAR|EV_RLSD);
	
	dwStoredFlags = EV_RLSD|EV_RXCHAR ;
	SetCommMask(m_hGPSCom, dwStoredFlags);
	
	// 清除缓冲信息
	PurgeComm (m_hGPSCom, PURGE_TXABORT|PURGE_RXABORT|PURGE_TXCLEAR|PURGE_RXCLEAR);
	
	// 初始化通讯设备参数 
	SetupComm (m_hGPSCom, XJYUE_LENGTH/*读缓冲*/, XJYUE_LENGTH/*写缓冲*/);
	
	/////////////////////////////////////////////////
    // instance an object of COMMTIMEOUTS.
    // Specify time-out between charactor for receiving.
    CommTimeOuts.ReadIntervalTimeout = 1;
    // Specify value that is multiplied 
    // by the requested number of bytes to be read. 
    CommTimeOuts.ReadTotalTimeoutMultiplier = 3;
    // Specify value is added to the product of the 
    // ReadTotalTimeoutMultiplier member
    CommTimeOuts.ReadTotalTimeoutConstant = 2;
    // Specify value that is multiplied 
    // by the requested number of bytes to be sent. 
    CommTimeOuts.WriteTotalTimeoutMultiplier = 3;
    // Specify value is added to the product of the 
    // WriteTotalTimeoutMultiplier member
    CommTimeOuts.WriteTotalTimeoutConstant = 2;
    // set the time-out parameter into device control.
    SetCommTimeouts(m_hGPSCom, &CommTimeOuts);
	
	//获取并设置串口
	GetCommState(m_hGPSCom, &dcb);
    int Rate;
	Rate=atoi(strBaud);
	dcb.BaudRate = Rate ;
	
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT; 
	SetCommState(m_hGPSCom, &dcb); 

	// 保存当前机器数
	m_dwBeginSample = GetTickCount();
	StartGPSthread();
}
void CGetGPSData::StartGPSthread()
{
	//////////////////////////////////////////////////////////////////////////
	// 创建数据处理线程
	m_hThreadData = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)CGetGPSData::theDataProcess,
		this, 0, &dwDataThreadId);
	if (NULL == m_hThreadData)
	{
		AfxMessageBox("程序启动内部数据处理线程失败!!!", MB_ICONSTOP);

		// 关闭串口文件
		CloseHandle(m_hGPSCom);

		return ;
	}

	Sleep(10);
	/////////////////////////////////////////////////////////////////////////
	// 创建串口接收线程线程
	m_hGPSThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)CGetGPSData::theGPSProcess,
		this, 0, &dwGPSThreadId);
	if (NULL == m_hGPSThread)
	{
		AfxMessageBox("程序启动数据接收线程失败!", MB_ICONSTOP);
		// 关闭串口文件
		CloseHandle(m_hGPSCom);
		return ;
	}
}
DWORD CGetGPSData::theGPSProcess(LPVOID lpParam)
  {
	  return (((CGetGPSData*)lpParam)->StubGPSProcess());
  }
DWORD CGetGPSData::StubGPSProcess()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	DWORD      dwEvtMask;
	DWORD      dwErrors;
	DWORD      nLength;
	COMSTAT    ComStat;

	CString    strTemp;
	DWORD      dwTemp;
	DWORD      dwHour;
	DWORD      dwMinute;
	DWORD      dwSecond;
	DWORD      dwMillSec;

	LONG    lLastError = ERROR_SUCCESS;

	// 串口数据相关
	char *pSend = NULL;

	/////////////////////////////////////
	// GPS数据缓存
	char  szSaveData[XJYUE_LENGTH];
	// 已接收到数据长度
	int   iSaveLength;

	// RT数据帧
	char*   pBeginRT = NULL; // 第一次出现
	char*   pEndRT = NULL;   // 第二次出现


	BOOL  bBegin = FALSE;

	// 初始化数据文件保存缓存
	iSaveLength = 0;
	memset(szSaveData, 0, XJYUE_LENGTH);

	// 查询设备状态
	ClearCommError(m_hGPSCom, &dwErrors, &ComStat);
	
  
	while(1)
	{
	
			WaitCommEvent (m_hGPSCom, &dwEvtMask, NULL); 

		// 处理字符到达事件
		if ((dwEvtMask & EV_RXCHAR) == EV_RXCHAR)
		{
			// 查询设备状态
			ClearCommError(m_hGPSCom, &dwErrors, &ComStat);

			// 检查接收队列
			if(ComStat.cbInQue) 
			{
				// 确保队列长度不大于缓存长度
				//ASSERT(ComStat.cbInQue<=XJYUE_LENGTH);

				// 分配数据缓存,为数据安全在堆中做缓存
				DWORD dwBytesRead = 0;

				//pSend = new char[5000];
				pSend = new char[ComStat.cbInQue];
                if (!ReadFile(m_hGPSCom, pSend, ComStat.cbInQue, &nLength, &olRead))
				{
					// 读操作出错处理
					dwErrors = GetLastError();
					if(ERROR_IO_PENDING == dwErrors)
					{
						WaitForSingleObject( olRead.hEvent, 500);
					}
				}

				// 传送到另一线程
				if (ComStat.cbInQue>0)
				{PostThreadMessage(dwDataThreadId, WM_INTERVAL_DATA,
						(WPARAM)pSend,ComStat.cbInQue);
				}
			} // end if
		} // end if ComStat.cbInQue
	}

	return 0;
}
DWORD CGetGPSData::theDataProcess(LPVOID lpParam)
{
	return ((( CGetGPSData*)lpParam)->StubDataProcess());
}

DWORD CGetGPSData::StubDataProcess()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	
	static int num_disp = 0;
	double xm=0.0,ym=0.0,zm=0.0;
	int   iRet;
	MSG   msgThreadData;  // 线程消息

	int iPosPVA=0;

	char  szPVA[4096];     // 语句缓存
	int  iPVALen=0;             // 当前语句数据长度


	char* pBegin = NULL;
	char* pEnd = NULL;

	int iPosIMU=0;

	char  szIMU[4096];     // 语句缓存
	int  iIMULen=0;             // 当前语句数据长度


	char* pBeginIMU = NULL;
	char* pEndIMU = NULL;
	// 初始化缓存数据
	iRecLength = 0;
	memset(szRecv, 0, XJYUE_LENGTH);

	while (1)
	{
		if(app->lukoudirnum>0)
		{
			if(app->lukoudirnum==1)
				out222<<"11111"<<','<<"11111"<<','<<app->laneseq_set<<','<<app->lanenum_set<<','<<app->speediniset<<','<<app->laneexist<<endl;
			else if(app->lukoudirnum==2)
				out222<<"22222"<<','<<"22222"<<','<<app->laneseq_set<<','<<app->lanenum_set<<','<<app->speediniset<<','<<app->laneexist<<endl;
			else if(app->lukoudirnum==3)
				out222<<"33333"<<','<<"33333"<<','<<app->laneseq_set<<','<<app->lanenum_set<<','<<app->speediniset<<','<<app->laneexist<<endl;
			else if(app->lukoudirnum==4)
				out222<<"44444"<<','<<"44444"<<','<<app->laneseq_set<<','<<app->lanenum_set<<','<<app->speediniset<<','<<app->laneexist<<endl;
			else if(app->lukoudirnum==6)
				out222<<"66666"<<','<<"66666"<<','<<app->laneseq_set<<','<<app->lanenum_set<<','<<app->speediniset<<','<<app->laneexist<<endl;
			else if(app->lukoudirnum==7)
				out222<<"77777"<<','<<"77777"<<','<<app->laneseq_set<<','<<app->lanenum_set<<','<<app->speediniset<<','<<app->laneexist<<endl;
			else if(app->lukoudirnum==8)
				out222<<"88888"<<','<<"88888"<<','<<app->laneseq_set<<','<<app->lanenum_set<<','<<app->speediniset<<','<<app->laneexist<<endl;
			else if(app->lukoudirnum==9)
				out222<<"99999"<<','<<"99999"<<','<<app->laneseq_set<<','<<app->lanenum_set<<','<<app->speediniset<<','<<app->laneexist<<endl;
			else if(app->lukoudirnum==29)
				out222<<"292929"<<','<<"292929"<<','<<app->laneseq_set<<','<<app->lanenum_set<<','<<app->speediniset<<','<<app->laneexist<<endl;
			else if(app->lukoudirnum==30)
				out222<<"303030"<<','<<"303030"<<','<<app->laneseq_set<<','<<app->lanenum_set<<','<<app->speediniset<<','<<app->laneexist<<endl;
			app->lukoudirnum=0;
		}
		//if(app->EndCpt)
		//{
		//	//AfxMessageBox("XX");
		//	delete [] (char*)(msgThreadData.wParam);
		//	memset(szPVA, 0, sizeof(szPVA));
		//	return 0;
		//} 
		memset(&msgThreadData, 0, sizeof(MSG));
		iRet = GetMessage(&msgThreadData, NULL, WM_INTERVAL_DATA, WM_INTERVAL_DATA);
		int a = iRet;
		// 成功取得消息
		if (iRet > 0)
		{
			// 保存数据
			if ((msgThreadData.lParam+iRecLength) >= XJYUE_LENGTH)
			{
				memcpy(szRecv+iRecLength, (char*)(msgThreadData.wParam), XJYUE_LENGTH-iRecLength);
				iRecLength = XJYUE_LENGTH;
			}
			else
			{
				memcpy(szRecv+iRecLength, (char*)(msgThreadData.wParam), msgThreadData.lParam);				
				iRecLength += msgThreadData.lParam;
			}
			delete [] (char*)(msgThreadData.wParam);

			//显示
			//m_listboxcom.
			
			// 查找BEST语句帧头
			pBegin = strstr(szRecv, "#INSPVAA");

			// 查找BEST语句帧尾
			if (pBegin!= NULL)
			{
				//再找BESTXYZA
				iPosPVA=pBegin-szRecv;   	
				pEnd = strchr(pBegin, '*');	

				if(pEnd != NULL)
				{				
					// 长度(包括帧尾)
					iPVALen = pEnd - pBegin+9;

					if((iRecLength-iPosPVA)>=iPVALen)
					{			
						// 初始化GPGGA语句缓存数据
						memset(szPVA, 0, 4096);

						// 转存数据
						memcpy(szPVA, pBegin, iPVALen);
                         
						// 校验成功后解析BEST语句数据		
						if(CalculateBlockCRC32(iPVALen-10,(unsigned char*)(szPVA+1)))
							DecoderINSPVAData(szPVA, iPVALen);

						// 清空GPS数据缓存
						iRecLength = iRecLength-iPVALen-iPosPVA;
						memcpy(szRecv,pEnd+9, iRecLength);


					}

				}


				// 清除标志
				pBegin = NULL;  // 帧头
				pEnd = NULL;
				if (iRecLength>= XJYUE_LENGTH)
				{
					iRecLength = 0;
					memset(szRecv, 0, XJYUE_LENGTH);			
				}
			}

			pBeginIMU = strstr(szRecv, "#CORRIMUDATA");

			// 查找BEST语句帧尾
			if (pBeginIMU!= NULL)
			{
				//再找BESTXYZA
				iPosIMU=pBeginIMU-szRecv;   	
				pEndIMU = strchr(pBeginIMU, '*');	

				if(pEndIMU != NULL)
				{				
					// 长度(包括帧尾)
					iIMULen = pEndIMU - pBeginIMU+9;

					if((iRecLength-iPosIMU)>=iIMULen)
					{			
						// 初始化GPGGA语句缓存数据
						memset(szIMU, 0, 4096);

						// 转存数据
						memcpy(szIMU, pBeginIMU, iIMULen);
                         
						// 校验成功后解析BEST语句数据		
						if(CalculateBlockCRC32(iIMULen-10,(unsigned char*)(szIMU+1)))
							DecoderCORRIMUData(szIMU, iIMULen);

						// 清空GPS数据缓存
						iRecLength = iRecLength-iIMULen-iPosIMU;
						memcpy(szRecv,pEndIMU+9, iRecLength);
					}
				}
				// 清除标志
				pBeginIMU = NULL;  // 帧头
				pEndIMU = NULL;
				// 防护处理-->释放缓存数据

				if (iRecLength>= XJYUE_LENGTH)
				{
					iRecLength = 0;
					memset(szRecv, 0, XJYUE_LENGTH);			
				}
			}//if (pBegin!= NULL)
		}//if (iRet > 0)
	}
	return 0;
}
void CGetGPSData::DecoderINSPVAData(LPCSTR data, int length)
{
	// 逗号计数器
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	int  SectionID;
	// #BESTPOSA语句
	
	if (NULL != strstr(data, "#INSPVAA"))
	{
		//strGPSQual = "";
		strTime = ""; 
		strLatitude = ""; 
		strLongitude = ""; 
		//strHeight = ""; 
		strNorth = "";     
		strEast = "";     
		strUp = "";  
		//strRoll = "";     
		//strPitch = "";     
		strAzimuth = "";  
		strINSstatus="";
		// 分析数据帧
		for(int iLoop=0; iLoop<length; iLoop++)
		{
			// 帧头
			if('#' == data[iLoop])
			{
				// SectionID为逗号计数器
				SectionID = 0;
			}

			// 帧尾
			if('*' == data[iLoop])
			{	
				continue;
			}
			// 逗号计数
			if(',' == data[iLoop])
			{
				SectionID++;
			}
			else // 数据
			{
				switch(SectionID)
				{ 
				case 6: 
					strTime += data[iLoop];
					break;

				case 11: 
					strLatitude += data[iLoop];
					break;	

				case 12: 
					strLongitude += data[iLoop];
					break;

			//	case 13: 
					//strHeight += data[iLoop];
					//break;

				case 14: 
					strNorth += data[iLoop];
					break;

				case 15: 
					strEast += data[iLoop];
					break;
				case 16: 
					strUp += data[iLoop];
					break;
				//case 17: 
					//strRoll+= data[iLoop];
				//	break;
				//case 18: 
				//	strPitch += data[iLoop];
					//break;
				case 19: 
					strAzimuth += data[iLoop];
					break;

				case 20: 
					strINSstatus+= data[iLoop];
					break;

				default:
					break;
				}
			}			
		}
		
		double time;
		time=atof(strTime);

		CString time1;
		time1=GPSSecond(time);

		app->critical_section.Lock();//锁住

		//CvPoint2D64f m_Position,m_P;
		//m_Position.x=atof(strLatitude);
		//m_Position.y=atof(strLongitude);
		//m_P.x=atof(strLatitude)*1000000-31900800;
		//m_P.y=atof(strLongitude)*1000000-117170900;
		//app->RT_Station.push(m_Position);
		app->GPS_Point = cvPoint2D64f( (double)atof(strLatitude), (double)atof(strLongitude) );

		double m_UpSpeed,m_NorthSpeed,m_EastSpeed,m_Speed;
		m_UpSpeed=atof(strUp);
		m_NorthSpeed=atof(strNorth);
		m_EastSpeed=atof(strEast);
		m_Speed=sqrt(pow(m_UpSpeed,2)+pow(m_NorthSpeed,2)+pow(m_EastSpeed,2));
		//app->RT_Speed.push(m_Speed);
		app->GPS_Speed = m_Speed;
		app->GPS_NorthSpeed = m_NorthSpeed;
		app->GPS_EastSpeed = m_EastSpeed;
		double m_HangXiang;
		m_HangXiang=atof(strAzimuth);
		//app->RT_Direction.push(m_HangXiang);
		app->GPS_Direction = m_HangXiang;//app->GPS_Direction = m_HangXiang - 3

		//SYSTEMTIME t1;
		//GetLocalTime(&t1);
		//out2<<t1.wHour<<":"<<t1.wMinute<<":"<<t1.wSecond<<",";
		double lastdistance=0;
		if(bGpsReceived)
			lastdistance=GetDistance(GPS_Point_last.x,GPS_Point_last.y,app->GPS_Point.x,app->GPS_Point.y);
		if(app->laneseq_set>=7)
		{
			if((bGpsReceived==0) || (lastdistance>0.2))
			{
				out2<<time1<<','<<strLatitude<<','<<strLongitude<<','<<m_Speed<<','<<m_HangXiang<<endl;
				out222<<strLatitude<<','<<strLongitude<<','<<app->laneseq_set<<','<<app->lanenum_set<<','<<app->speediniset<<','<<app->laneexist<<endl;
				GPS_Point_last=app->GPS_Point;
				bGpsReceived=1;
			}
		}
		else
		{
			if((bGpsReceived==0) || (lastdistance>1))
			{
				out2<<time1<<','<<strLatitude<<','<<strLongitude<<','<<m_Speed<<','<<m_HangXiang<<endl;
				out222<<strLatitude<<','<<strLongitude<<','<<app->laneseq_set<<','<<app->lanenum_set<<','<<app->speediniset<<','<<app->laneexist<<endl;
				GPS_Point_last=app->GPS_Point;
				bGpsReceived=1;
			}
		}
		/*
		if((bGpsReceived==0) || (lastdistance>1))
		{
			out2<<time1<<','<<strLatitude<<','<<strLongitude<<','<<m_Speed<<','<<m_HangXiang<<endl;
			out222<<strLatitude<<','<<strLongitude<<','<<app->laneseq_set<<','<<app->lanenum_set<<endl;
			GPS_Point_last=app->GPS_Point;
			bGpsReceived=1;
		}
		*/
		app->critical_section.Unlock();//释放
		SetEvent(m_GpsEvent);
	} 
}

void CGetGPSData::DecoderCORRIMUData(LPCSTR data, int length)
{
	// 逗号计数器
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	int  SectionID;
	// #BESTPOSA语句
	
	if (NULL != strstr(data, "#CORRIMUDATAA"))
	{
		  	//strGPSQual = "";
		strPitchRate = ""; 
		strRollRate = ""; 
		strYawRate = ""; 
		//strHeight = ""; 
		strLateralAcc = "";     
		strLongitudinalAcc = "";     
		strVerticalAcc = "";  
		//strRoll = "";     
		//strPitch = "";     
  
		strINSstatus="";
		// 分析数据帧
		for(int iLoop=0; iLoop<length; iLoop++)
		{
			// 帧头
			if('#' == data[iLoop])
			{
				// SectionID为逗号计数器
				SectionID = 0;
			}

			// 帧尾
			if('*' == data[iLoop])
			{	
				continue;
			}
			// 逗号计数
			if(',' == data[iLoop])
			{
				SectionID++;
			}
			else // 数据
			{
				switch(SectionID)
				{ 

				case 11: 
					strPitchRate += data[iLoop];
					break;	

				case 12: 
					strRollRate += data[iLoop];
					break;

				case 13: 
					strYawRate += data[iLoop];
					break;

				case 14: 
					strLateralAcc += data[iLoop];
					break;

				case 15: 
					strLongitudinalAcc += data[iLoop];
					break;
				case 16: 
					strVerticalAcc += data[iLoop];
					break;

				default:
					break;
				}
			}			
		}
		


		app->critical_section.Lock();//锁住

	
		double m_YawRate;
		m_YawRate=atof(strYawRate);
		
		app->GPS_YawRate = m_YawRate;
		double m_LonAcc;
		m_LonAcc=atof(strLongitudinalAcc);
		
		app->GPS_LonAcc = m_LonAcc;
		SYSTEMTIME t1;
		GetLocalTime(&t1);
		//out333<<t1.wHour<<":"<<t1.wMinute<<":"<<t1.wSecond<<","<<t1.wMilliseconds<<", ";
		
		//out333<<"GPS: "<<strLatitude<<','<<strLongitude<<", "<<strAzimuth<<", speed: "<<app->GPS_Speed<<", YawRate: "<<strYawRate<<", LonAcc: "<<strLongitudinalAcc<<endl;
		app->critical_section.Unlock();//释放


	} 
}

//校验
ULONG CGetGPSData::ValueCRC32(int i)
{
	int   jLoop;
	ULONG ulCRC;
	ulCRC = i;
	for ( jLoop = 8 ; jLoop > 0; jLoop-- )
	{
		if ( ulCRC & 1 )
		{
			ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
		}
		else
		{
			ulCRC >>= 1;
		}
	}

	return ulCRC;
}

BOOL CGetGPSData::CalculateBlockCRC32(unsigned long u1Count,unsigned char *ucBuffer)
{
	unsigned long u1Temp1;
	unsigned long u1Temp2;
	unsigned long u1CRC=0;
	CString Des;
	char szDes[9];
	char szSrc[9];
	memset(szSrc, 0, 9);

	while(u1Count--!=0)
	{
		u1Temp1=(u1CRC>>8) & 0x00FFFFFFL;
		u1Temp2=ValueCRC32(((int)u1CRC ^ *ucBuffer++) & 0xff);
		u1CRC=u1Temp1 ^ u1Temp2;
	}

	// 计算后结果
	Des.Format("%08X", u1CRC);
	memcpy(szDes,Des,strlen(Des));

	*ucBuffer++;
	// 数据帧中校验数据
	szSrc[0] = *ucBuffer++;
	szSrc[1] = *ucBuffer++;
	szSrc[2] = *ucBuffer++;
	szSrc[3] = *ucBuffer++;
	szSrc[4] = *ucBuffer++;
	szSrc[5] = *ucBuffer++;
	szSrc[6] = *ucBuffer++;
	szSrc[7] = *ucBuffer++;
	Des.MakeLower();

	if (strstr(szSrc, Des))
		return TRUE;
	else
		return FALSE;

}

//gps时间转换
CString CGetGPSData::GPSSecond(double gpsSecond) 
{
	CString a;
	int day,Hour,Second;
	double minute;
	day=int(gpsSecond/86400.0);
	Hour=int((gpsSecond/86400.0-int(gpsSecond/86400.0))*24);	
	minute=(gpsSecond-day*86400-Hour*3600)/60;
	Second=gpsSecond-day*86400-Hour*3600-int(minute)*60;

	a.Format("%02d:%02d:%02d.000",Hour,(int)minute,Second);

	return a;

}

//得到经纬度值
CvPoint2D64f CGetGPSData::GetPosition()
{
	CvPoint2D64f m_Position;
	m_Position.x=atof(strLatitude);
	m_Position.y=atof(strLongitude);
	return m_Position;
}

/*********************************************************************************************
根据经纬度坐标计算实际两点距离
输入：
lat1,lng1:第一点纬度、经度
lat2,lng2:第二点纬度、经度
输出：
返回两点距离
********************************************************************************************/
double CGetGPSData::GetDistance(double lat1, double lng1, double lat2, double lng2)
 {
	 if(lat1 == lat2 && lng1 == lng2)
		 return 0;
    double radLat1 = rad(lat1);
    double radLat2 = rad(lat2);
    double a = radLat1 - radLat2;
    double b = rad(lng1) - rad(lng2);
    double s = 2 * asin(sqrt(pow(sin(a/2),2) + 
		cos(radLat1)*cos(radLat2)*pow(sin(b/2),2)));
    s = s * EARTH_RADIUS*1000;
    //s = round(s * 10000) / 10000;
    return s;
 }

//根据两点经纬度求方向，与北夹角
double CGetGPSData::GetAngle(CvPoint2D64f apoint, CvPoint2D64f bpoint)
{	
	double lat1 = apoint.x;
	double lng1 = apoint.y;
	double lat2 = bpoint.x;
	double lng2 = bpoint.y;
	double radLat1 = rad(lat1);
    double radLat2 = rad(lat2);
    double a = radLat1 - radLat2;
    double b = (rad(lng1) - rad(lng2))*cos(radLat1);
	double t;
	if(a==0)
	{
		if(b>0)
			t = PI/2;
		else
			t = -PI/2;
	}
	else
		t=atan(b/a);
	if (a<0)
		t=t+PI;
	//t=PI/2-t;

	t = t * 180 / PI;
	return t;
}

/*********************************************************************************************
根据经纬度坐标计算实际两点距离
输入：
	lat1,lng1:第一点纬度、经度
	lat2,lng2:第二点纬度、经度
输出：
	返回两点夹角，与正北方向夹角，顺时针正，逆时针负,单位度
********************************************************************************************/
double CGetGPSData::GetAngle(double lat1, double lng1, double lat2, double lng2)
{
	//与北夹角
	if(lat1 == lat2 && lng1 == lng2)
		return 0;
	double radLat1 = rad(lat1);
    double radLat2 = rad(lat2);
    double a = radLat1 - radLat2;
    double b = (rad(lng1) - rad(lng2))*cos(radLat1);
	double t;
	if(a==0)
	{
		if(b>0)
			t = PI/2;
		else
			t = -PI/2;
	}
	else
		t=atan(b/a);
		
	if (a<0)
	{
		t=t+PI;
	
	}

	t = t * 180 / PI;
	return t;
}

/*
目标点的GPS坐标转换为地图坐标。
参数：
     v - 车的GPS坐标
     a - 目标点GPS坐标
	 direction - 车的航向
*/
Map_Point CGetGPSData::APiontConver(CvPoint2D64f v,CvPoint2D64f a,double direction)
{
	double t = GetAngle(a.x,a.y,v.x,v.y);
	double m = direction - t +90;
	m = rad(m);
	double s = GetDistance(v.x,v.y,a.x,a.y);
	Map_Point p;
	p.x = 256 + (int)(s * cos(m) * 5);  //*5换成20cm单位
	p.y = 412 - (int)(s * sin(m) * 5);

	return p;

}

CvPoint2D64f CGetGPSData::APiontConverD(CvPoint2D64f v,CvPoint2D64f a,double direction)
{
	double t = GetAngle(a.x,a.y,v.x,v.y);
	double m = direction - t +90;
	m = rad(m);
	double s = GetDistance(v.x,v.y,a.x,a.y);
	CvPoint2D64f p;
	p.x = 256 + (s * cos(m) * 5);  //*5换成20cm单位
	p.y = 412 - (s * sin(m) * 5);

	return p;

}

//得到航向
double CGetGPSData::GetDirection()
{
	double m_HangXiang;
	m_HangXiang=atof(strAzimuth);
	return m_HangXiang;
}

//得到速度
double CGetGPSData::GetSpeed()
{

	double m_USpeed,m_NSpeed,m_ESpeed,m_Speed;
		m_USpeed=atof(strUp);
		m_NSpeed=atof(strNorth);
		m_ESpeed=atof(strEast);
		m_Speed=sqrt(pow(m_USpeed,2)+pow(m_NSpeed,2)+pow(m_ESpeed,2));
	return m_Speed;
}

//地图坐标转GPS
CvPoint2D64f CGetGPSData::MaptoGPS(CvPoint2D64f v,double dir,CvPoint2D64f a)
{
	if (a.x==256 && a.y==412)
	{
		return v;
	}
	double xq1,yq1,x,y,sita,dire,xa,ya,x1,y1;
	double s1;	
	xq1 = a.x-256;
	yq1 = 412-a.y;	
	s1=sqrt(pow(xq1/5,2)+pow(yq1/5,2));
	if(yq1==0)
	{
		if(xq1>0)
			sita = PI/2;
		else
			sita = -PI/2;
	}
	else
		sita=atan(xq1/yq1);
	if(yq1<0)
		sita += PI;
	dire=sita+rad(dir);
	ya=s1*sin(dire);
	xa=s1*cos(dire);
	x=v.x+(xa*180)/(6378137*3.1415926);
	y=v.y+(ya*180)/(6378137*3.1415926*cos(x*3.1415926/180));
	
	CvPoint2D64f c;
	c.x=x;
	c.y=y;
	return c;
}


//实际坐标(m)转GPS
/*int CGetGPSData::XYtoGPS(double wd, double jd, double dir, double px, double py, double &wd0, double &jd0)
{

	double xq1,yq1,x,y,sita,dire,xa,ya,x1,y1;
	double s1;	
	xq1 = a.x-256;
	yq1 = 256-a.y;	
	s1=sqrt(pow(xq1/5,2)+pow(yq1/5,2));
	sita=atan(xq1/yq1);
	dire=sita+rad(dir);
	ya=s1*sin(dire);
	xa=s1*cos(dire);
	x=wd+(xa*180)/(6378137*3.1415926);
	y=jd+(ya*180)/(6378137*3.1415926*cos(x*3.1415926/180));
	CvPoint2D64f c;
	c.x=x;
	c.y=y;
	return c;
}*/