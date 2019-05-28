#include "StdAfx.h"
#include "CANBus.h"
#include "IVDecision.h"

#include <iostream>
#include <fstream>
#include <iomanip>
ofstream outadas("adasispt.txt");
ofstream outstub("stub.txt");
ofstream outsegment("segment.txt");
ofstream outadas1("adasispt1.txt");
ofstream outnumpost("numofpost.txt");

ofstream outlaneoffset("laneoffset.txt");

ofstream outjiting("outjiting.txt");

CCANBus::CCANBus(void)
	: m_devIndex(0)
	, m_CANIndex(0)
	, m_frameID(0x350)
	, m_canOpenFlag(FALSE)
	, m_sendType(0)
{
	memset(&m_canMsgSend, 0, sizeof(CANMessageSend));
	memset(&m_canMsgRecv, 0, sizeof(CANMessageRecv));
	memset(&m_avFault, 0, sizeof(AVFault));
	m_canMsgRecv.driveMode = 1;
	m_canMsgRecv.TqGndNegMax = -3000;
	m_canMsgRecv.TqGndPosMax = 3000;

}

CCANBus::~CCANBus(void)
{
	if (m_canOpenFlag)
	{
		VCI_CloseDevice(VCI_USBCAN_2E_U, m_devIndex);
	}
}

// ???CAN
UINT CCANBus::startCAN(void)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	m_canMsgRecv.ObstacleTotalNum=0;

	//???????莩?始??
	m_iPreType = 0;
	m_iPrlPreCycn = -1;
	m_iStuPreCycn = -1;
	m_iSegPreCyCn = -1;
	m_iPreSndType = -1;
	m_iPrsPreCycn = -1;
	m_iErr = -1;
	m_dLngOffset = 0.;
	m_dLatOffset = 0.;
	m_iNumStub = 0;
	m_iNumSeg = 0;
	m_iNumProLng = 0;
	m_bJunction = FALSE;
	m_prePt.lng = 0.0;
	m_prePt.lat = 0.0;
	storage = cvCreateMemStorage(0);
	app->ADAS_points = cvCreateSeq( 0, sizeof(CvSeq), sizeof(LEAD), storage);
	adas_id = 0;
	//??
	DWORD iRet = 1;
	iRet = VCI_OpenDevice(VCI_USBCAN_2E_U, m_devIndex, 0);
	if (iRet == 0)
	{
		AfxMessageBox("打开设备失败");
		return 1;
	}
	m_canOpenFlag = TRUE;
	//??????
	int baud = 0x060007;
	if (VCI_SetReference(VCI_USBCAN_2E_U, m_devIndex, m_CANIndex, 0, &baud) == STATUS_ERR)
	{
		AfxMessageBox("设置波特率失败");
		VCI_CloseDevice(VCI_USBCAN_2E_U, m_devIndex);
		return 4;
	}
	//??始??
	VCI_INIT_CONFIG canInitConfig;
	canInitConfig.AccCode = 0x0;	//???
	canInitConfig.AccMask = 0xffffffff;	//????
	canInitConfig.Filter = 0;	//双???
	canInitConfig.Timing0 = 0x0;
	canInitConfig.Timing1 = 0x1c;
	canInitConfig.Mode = 0;		//???模式
	if (VCI_InitCAN(VCI_USBCAN_2E_U, m_devIndex, m_CANIndex, &canInitConfig) == STATUS_ERR)
	{
		VCI_CloseDevice(VCI_USBCAN_2E_U, m_devIndex);
		AfxMessageBox("初始化失败!!!");
		return 2;
	}
	//???
	Sleep(1000);
	if (VCI_StartCAN(VCI_USBCAN_2E_U, m_devIndex, m_CANIndex) == STATUS_ERR)
	{
		VCI_CloseDevice(VCI_USBCAN_2E_U, m_devIndex);
		AfxMessageBox("启动失败");
		return 3;
	}
	//??????叱?
	Sleep(1000);
	startThread();
	return 0;
}

// ???龋?time????????时??
UINT CCANBus::horn(ULONG time)
{
	VCI_CAN_OBJ frame;
	frame.ID = m_frameID;
	frame.SendType = m_sendType;
	frame.RemoteFlag = 0;
	frame.ExternFlag = 0;
	frame.DataLen = 8;

	frame.Data[0] = 0x10;
	frame.Data[1] = 0x0;
	frame.Data[2] = 0x7d;
	frame.Data[3] = 0x0;
	frame.Data[4] = 0x1e;
	frame.Data[5] = 0x78;
	frame.Data[6] = 0x0;
	frame.Data[7] = 0x0;

	ULONG lRet = VCI_Transmit(VCI_USBCAN_2E_U, m_devIndex, m_CANIndex, &frame, 1);
	if (lRet != 1)
	{
		AfxMessageBox("开喇叭发送失败");
		return 1;
	}

	Sleep(time);
	frame.Data[4] = 0x0;
	lRet = VCI_Transmit(VCI_USBCAN_2E_U, m_devIndex, m_CANIndex, &frame, 1);
	if (lRet != 1)
	{
		AfxMessageBox("关喇叭发送失败");
		return 2;
	}
	return 0;
}

// ?转??瓶?
UINT CCANBus::leftLampOn(void)
{
	VCI_CAN_OBJ frame;
	frame.ID = m_frameID;
	frame.SendType = m_sendType;
	frame.RemoteFlag = 0;
	frame.ExternFlag = 0;
	frame.DataLen = 8;

	frame.Data[0] = 0x4;
	frame.Data[1] = 0x0;
	frame.Data[2] = 0x7d;
	frame.Data[3] = 0x0;
	frame.Data[4] = 0x1e;
	frame.Data[5] = 0x78;
	frame.Data[6] = 0x0;
	frame.Data[7] = 0x0;

	ULONG lRet = VCI_Transmit(VCI_USBCAN_2E_U, m_devIndex, m_CANIndex, &frame, 1);
	if (lRet != 1)
	{
		AfxMessageBox("发送失败");
		return 1;
	}
	return 0;
}

// ?转??乒?
UINT CCANBus::leftLampOff(void)
{
	VCI_CAN_OBJ frame;
	frame.ID = m_frameID;
	frame.SendType = m_sendType;
	frame.RemoteFlag = 0;
	frame.ExternFlag = 0;
	frame.DataLen = 8;

	frame.Data[0] = 0x0;
	frame.Data[1] = 0x0;
	frame.Data[2] = 0x7d;
	frame.Data[3] = 0x0;
	frame.Data[4] = 0x1e;
	frame.Data[5] = 0x78;
	frame.Data[6] = 0x0;
	frame.Data[7] = 0x0;

	ULONG lRet = VCI_Transmit(VCI_USBCAN_2E_U, m_devIndex, m_CANIndex, &frame, 1);
	if (lRet != 1)
	{
		AfxMessageBox("发送失败");
		return 1;
	}
	return 0;
}

// ?转??瓶?
UINT CCANBus::rightLampOn(void)
{
	VCI_CAN_OBJ frame;
	frame.ID = m_frameID;
	frame.SendType = m_sendType;
	frame.RemoteFlag = 0;
	frame.ExternFlag = 0;
	frame.DataLen = 8;

	frame.Data[0] = 0x8;
	frame.Data[1] = 0x0;
	frame.Data[2] = 0x7d;
	frame.Data[3] = 0x0;
	frame.Data[4] = 0x1e;
	frame.Data[5] = 0x78;
	frame.Data[6] = 0x0;
	frame.Data[7] = 0x0;

	ULONG lRet = VCI_Transmit(VCI_USBCAN_2E_U, m_devIndex, m_CANIndex, &frame, 1);
	if (lRet != 1)
	{
		AfxMessageBox("发送失败");
		return 1;
	}
	return 0;
}

// ?转??乒?
UINT CCANBus::rightLampOff(void)
{
	VCI_CAN_OBJ frame;
	frame.ID = m_frameID;
	frame.SendType = m_sendType;
	frame.RemoteFlag = 0;
	frame.ExternFlag = 0;
	frame.DataLen = 8;

	frame.Data[0] = 0x0;
	frame.Data[1] = 0x0;
	frame.Data[2] = 0x7d;
	frame.Data[3] = 0x0;
	frame.Data[4] = 0x1e;
	frame.Data[5] = 0x78;
	frame.Data[6] = 0x0;
	frame.Data[7] = 0x0;

	ULONG lRet = VCI_Transmit(VCI_USBCAN_2E_U, m_devIndex, m_CANIndex, &frame, 1);
	if (lRet != 1)
	{
		AfxMessageBox("发送失败");
		return 1;
	}
	return 0;
}

// ????缘????
UINT CCANBus::sendTorque(float torque)
{
	VCI_CAN_OBJ frame;
	frame.ID = m_frameID;
	frame.SendType = m_sendType;
	frame.RemoteFlag = 0;
	frame.ExternFlag = 0;
	frame.DataLen = 8;

	WORD tqr = 0x0;
	tqr = (torque + 3000) / 1.5;

	frame.Data[0] = 0x0;
	frame.Data[1] = 0x0;
	frame.Data[2] = (tqr & 0x0ff0) >> 4;
	frame.Data[3] = (tqr & 0xf) << 4;
	frame.Data[4] = 0x1e;
	frame.Data[5] = 0x78;
	frame.Data[6] = 0x0;
	frame.Data[7] = 0x0;

	ULONG lRet = VCI_Transmit(VCI_USBCAN_2E_U, m_devIndex, m_CANIndex, &frame, 1);
	if (lRet != 1)
	{
		AfxMessageBox("发送失败");
		return 1;
	}
	return 0;
}

// ???????潭?
UINT CCANBus::sendBrake(UINT brake)
{
	VCI_CAN_OBJ frame;
	frame.ID = m_frameID;
	frame.SendType = m_sendType;
	frame.RemoteFlag = 0;
	frame.ExternFlag = 0;
	frame.DataLen = 8;

	BYTE brk = 0x0;
	brk = brake;

	frame.Data[0] = 0x0;
	frame.Data[1] = brk;
	frame.Data[2] = 0x7d;
	frame.Data[3] = 0x0;
	frame.Data[4] = 0x1e;
	frame.Data[5] = 0x78;
	frame.Data[6] = 0x0;
	frame.Data[7] = 0x0;

	ULONG lRet = VCI_Transmit(VCI_USBCAN_2E_U, m_devIndex, m_CANIndex, &frame, 1);
	if (lRet != 1)
	{
		AfxMessageBox("发送失败");
		return 1;
	}
	return 0;
}

// ???????转??
UINT CCANBus::sendSteerAngle(float angle)
{
	VCI_CAN_OBJ frame;
	frame.ID = m_frameID;
	frame.SendType = m_sendType;
	frame.RemoteFlag = 0;
	frame.ExternFlag = 0;
	frame.DataLen = 8;

	WORD str = 0x0;
	str = (angle + 780) / 0.1;

	frame.Data[0] = 0x0;
	frame.Data[1] = 0x0;
	frame.Data[2] = 0x7d;
	frame.Data[3] = 0x0;
	frame.Data[4] = (str & 0xff00) >> 8;
	frame.Data[5] = str & 0xff;
	frame.Data[6] = 0x0;
	frame.Data[7] = 0x0;

	ULONG lRet = VCI_Transmit(VCI_USBCAN_2E_U, m_devIndex, m_CANIndex, &frame, 1);
	if (lRet != 1)
	{
		AfxMessageBox("发送失败");
		return 1;
	}
	return 0;
}

void CCANBus::startThread(void)
{
	DWORD dwSendThreadId;
	CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)CCANBus::threadEntry,
		this, 0, &dwSendThreadId);
}

DWORD CCANBus::threadEntry(LPVOID lpParam)
{
	return (((CCANBus*)lpParam)->receiveThread());
}

DWORD CCANBus::receiveThread(void)
{
	VCI_CAN_OBJ frameinfo[50];
	VCI_ERR_INFO errinfo;

	while(1)
	{
		int nlen = VCI_Receive(VCI_USBCAN_2E_U, m_devIndex, m_CANIndex, frameinfo, 50, 200);
		if(nlen <= 0)
		{
			//注??????没??????????????撕??????取????前?拇???
			//千????省??一??????使???懿??知????????么??
			VCI_ReadErrInfo(VCI_USBCAN_2E_U, m_devIndex, m_CANIndex, &errinfo);
		}
		else
		{
			CANDecoder(frameinfo, nlen);
		}
	}
	return 0;
}

// ?????盏???AN???
UINT CCANBus::CANDecoder(const VCI_CAN_OBJ *frameInfo, ULONG len)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	BYTE buf[CAN_DATA_SIZE] = {0};

	UINT num = 0;	//?效帧?
	for (UINT i = 0; i <len; i++)
	{
		//????????
		unsigned val1 = 0;
		unsigned val2 = 0;
		int autoDriveReq = -1;			//auto driving require message
		BYTE mapPlanReq = 0;		//motion planning require message
		BYTE ibeoReq = 0;			//IBEO require message
		BYTE hdlReq = 0;			//HDL require message
		BYTE laneLineReq = 0;	//Lane line require message
		BYTE traffLightReq = 0;	//traffic light require message
		BYTE traffSignReq = 0;	//traffic signs require message
		BYTE infraredReq = 0;
		BYTE type = 0;//message type: 1, position; 2, segment; 3, stub; 5, profile long (longitude and latitude)
		BYTE cycn = 0;//cycle count:0-1-2-3,...
		BYTE retr = 0; //re-send:0, new; 1: re-send
		BOOL bEnd = TRUE;
		int end = -1;
		double lng, lat;
		//buf = frameInfo[i].Data;

		/*unsigned long x1, x2, x3, x4;*/
		switch (frameInfo[i].ID)
		{
			/*
			case 0x120:
				decode120(&frameInfo[i]);
				num++;
				break;
			case 0x121:
				decode121(&frameInfo[i]);
				num++;
				break;
			case 0x350:	//?????
				decode350(&frameInfo[i]);
				num++;
				break;
			case 0x416:
				decode416(&frameInfo[i]);
				num++;
				break;
				*/
			case 0x510:
				decode510(&frameInfo[i]);
				num++;
				break;
			case 0x511:
				decode511(&frameInfo[i]);
				num++;
				break;
			case 0x512:
				decode512(&frameInfo[i]);
				num++;
				break;
			case 0x513:
				decode513(&frameInfo[i]);
				num++;
				break;
			case 0x514:
				decode514(&frameInfo[i]);
				num++;
				break;
			default:
				break;
		}
	}
	return num;
}

// ?????????
UINT CCANBus::sendMessage()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	VCI_CAN_OBJ frame;
	frame.ID = m_frameID;
	frame.SendType = m_sendType;
	frame.RemoteFlag = 0;
	frame.ExternFlag = 0;
	frame.DataLen = 8;

	UINT limitCode = sendLimiting();

	WORD tqr = (m_canMsgSend.driveTorque + 3000) / 1.5;
	WORD str = (m_canMsgSend.steerAngle + 780) / 0.1;

	frame.Data[0] = (m_canMsgSend.hornOn << 4) | (m_canMsgSend.turnLamp << 2) | m_canMsgSend.EPBActive;
	frame.Data[1] = m_canMsgSend.brake;
	frame.Data[2] = (tqr & 0x0ff0) >> 4;
	frame.Data[3] = (tqr & 0xf) << 4;
	frame.Data[4] = (str & 0xff00) >> 8;
	frame.Data[5] = str & 0xff;
	frame.Data[6] = 0x0;
	if(app->bApaActive==1)
		frame.Data[7] = 0x01;
	else
		frame.Data[7] = 0x0;

	ULONG lRet = VCI_Transmit(VCI_USBCAN_2E_U, m_devIndex, m_CANIndex, &frame, 1);
	if (lRet != 1)
	{
		AfxMessageBox("发送失败");
		return 10;
	}

	return limitCode;
}


// ???裙?
UINT CCANBus::hornOff(void)
{
	VCI_CAN_OBJ frame;
	frame.ID = m_frameID;
	frame.SendType = m_sendType;
	frame.RemoteFlag = 0;
	frame.ExternFlag = 0;
	frame.DataLen = 8;

	frame.Data[0] = 0x0;
	frame.Data[1] = 0x0;
	frame.Data[2] = 0x7d;
	frame.Data[3] = 0x0;
	frame.Data[4] = 0x1e;
	frame.Data[5] = 0x78;
	frame.Data[6] = 0x0;
	frame.Data[7] = 0x0;

	ULONG lRet = VCI_Transmit(VCI_USBCAN_2E_U, m_devIndex, m_CANIndex, &frame, 1);
	if (lRet != 1)
	{
		AfxMessageBox("发送失败");
		return 1;
	}
	return 0;
}


// ???刹
UINT CCANBus::setEPB(UINT mode)
{
	VCI_CAN_OBJ frame;
	frame.ID = m_frameID;
	frame.SendType = m_sendType;
	frame.RemoteFlag = 0;
	frame.ExternFlag = 0;
	frame.DataLen = 8;

	frame.Data[0] = mode;
	frame.Data[1] = 0x0;
	frame.Data[2] = 0x7d;
	frame.Data[3] = 0x0;
	frame.Data[4] = 0x1e;
	frame.Data[5] = 0x78;
	frame.Data[6] = 0x0;
	frame.Data[7] = 0x0;

	ULONG lRet = VCI_Transmit(VCI_USBCAN_2E_U, m_devIndex, m_CANIndex, &frame, 1);
	if (lRet != 1)
	{
		AfxMessageBox("发送失败");
		return 1;
	}
	return 0;
}


// ??位CAN
UINT CCANBus::resetCAN(void)
{
	if(VCI_ResetCAN(VCI_USBCAN_2E_U, m_devIndex, m_CANIndex)==1)
	{
		return 0;
	}
	else
	{
		AfxMessageBox("复位失败");
		return 1;
	}
}


// ???x120
void CCANBus::decode120(const VCI_CAN_OBJ * frameInfo)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	app->systemtimer10ms++;

	WORD high = 0;
	WORD low = 0;
	m_canMsgRecv.SCUFault = (frameInfo->Data[0] & 0x20) >> 5;
	m_canMsgRecv.EPSSysFault = (frameInfo->Data[0] & 0x10) >> 4;
	m_canMsgRecv.EVBFault = (frameInfo->Data[0] & 0x8) >> 3;
	m_canMsgRecv.VehSoftStop = (frameInfo->Data[0] & 0x4) >> 2;
	//m_canMsgRecv.VehEmergencyStop = (frameInfo->Data[0] & 0x2) >> 1;
	m_canMsgRecv.breakSysFault = frameInfo->Data[0] & 0x1;
	m_canMsgRecv.driveMode = (frameInfo->Data[1] & 0xc) >> 2;
	m_canMsgRecv.EPBMode = frameInfo->Data[1] & 0x3;
	high = frameInfo->Data[2] << 3;
	low = (frameInfo->Data[3] & 0xe0) >> 5;
	m_canMsgRecv.TqGndPosMax = (high + low) * 1.5;
	high = (frameInfo->Data[3] & 0x7) << 8;
	low = frameInfo->Data[4];
	//m_canMsgRecv.TqGndNegMax = (high + low) * 1.5 - 3000;
	m_canMsgRecv.TqGndNegMax = -3000;

	m_canMsgRecv.VehEmergencyStop = (frameInfo->Data[0] & 0x40) >> 6;

	//outjiting<<m_canMsgRecv.VehEmergencyStop<<endl;
}


// ???x121
void CCANBus::decode121(const VCI_CAN_OBJ * frameInfo)
{
	WORD high = 0;
	WORD low = 0;
	m_canMsgRecv.realBrakePressure = frameInfo->Data[0] * 0.1;
	high = frameInfo->Data[1] << 4;
	low = (frameInfo->Data[2] & 0xf0) >> 4;
	m_canMsgRecv.realTorque = (high + low) * 1.5 - 3000;
	high = frameInfo->Data[3] << 8;
	low = frameInfo->Data[4];
	m_canMsgRecv.realSteerAngle = (high + low) * 0.1 - 780;
	high = (frameInfo->Data[5] & 0x1f) << 8;
	low = frameInfo->Data[6];
	m_canMsgRecv.realSpeed = (high + low) * 0.05625;
}

void CCANBus::decode416(const VCI_CAN_OBJ * frameInfo)
{
	/*
	m_canMsgRecv.offset_l=(frameInfo->Data[0]<<8)+frameInfo->Data[1];
	m_canMsgRecv.offset_r=(frameInfo->Data[2]<<8)+frameInfo->Data[3];

	outlaneoffset<<m_canMsgRecv.offset_l<<","<<m_canMsgRecv.offset_r<<endl;
	*/
}


// ???x350
void CCANBus::decode350(const VCI_CAN_OBJ * frameInfo)
{
	WORD high = 0;
	WORD low = 0;
	m_canMsgRecv.realBrakePressure = frameInfo->Data[1];
	high = frameInfo->Data[2] << 4;
	low = (frameInfo->Data[3] & 0xf0) >> 4;
	m_canMsgRecv.realTorque = (high + low) * 1.5 - 3000;
	high = frameInfo->Data[4] << 8;
	low = frameInfo->Data[5];
	m_canMsgRecv.realSteerAngle = (high + low) * 0.1 - 780;
}

void CCANBus::decode510(const VCI_CAN_OBJ * frameInfo)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_haikanglock.Lock();

	bool bleftAerr=false;

	m_canMsgRecv.ObstacleTotalNum=0;

	WORD high = 0;
	WORD low = 0;

	outlaneoffset<<"0x510:  "<<hex<<(unsigned int)*(unsigned char *)&(frameInfo->Data[0])<<", "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[1])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[2])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[3])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[4])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[5])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[6])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[7])<<endl;

	high = frameInfo->Data[0] << 8;
	low = frameInfo->Data[1];
	m_canMsgRecv.Left_A_C3 = (high + low) * (4.00e-09)-0.000131072;
	if(m_canMsgRecv.Left_A_C3<=-0.000131072)
	{
		m_canMsgRecv.Left_A_C3=-0.000131072;
		bleftAerr=true;
	}
	if(m_canMsgRecv.Left_A_C3>=0.000131068)
	{
		m_canMsgRecv.Left_A_C3=0.000131068;
		bleftAerr=true;
	}

	high = frameInfo->Data[2] << 8;
	low = frameInfo->Data[3];
	m_canMsgRecv.Left_A_C2 = (high + low) * (1.00e-06)-0.032767;
	if(m_canMsgRecv.Left_A_C2<=-0.032768)
	{
		m_canMsgRecv.Left_A_C2=-0.032768;
		bleftAerr=true;
	}
	if(m_canMsgRecv.Left_A_C2>=0.032767)
	{
		m_canMsgRecv.Left_A_C2=0.032767;
		bleftAerr=true;
	}

	high = frameInfo->Data[4] << 4;
	low = (frameInfo->Data[5] & 0xF0) >> 4;
	m_canMsgRecv.Left_A_C0 = (high + low) * 0.01-20.48;
	if(m_canMsgRecv.Left_A_C0<=-20.48)
	{
		m_canMsgRecv.Left_A_C0=-20.48;
		bleftAerr=true;
	}
	if(m_canMsgRecv.Left_A_C0>=20.47)
	{
		m_canMsgRecv.Left_A_C0=20.47;
		bleftAerr=true;
	}

	high = (frameInfo->Data[5] & 0x0F) << 6;
	low = (frameInfo->Data[6] & 0xFC) >> 2;
	m_canMsgRecv.Left_A_C1 = (high + low) * 0.00097656-0.49951044;
	if(m_canMsgRecv.Left_A_C1<=-0.49951044)
	{
		m_canMsgRecv.Left_A_C1=-0.49951044;
		bleftAerr=true;
	}
	if(m_canMsgRecv.Left_A_C1>=0.49951044)
	{
		m_canMsgRecv.Left_A_C1=0.49951044;
		bleftAerr=true;
	}

	m_canMsgRecv.Left_A_Quality=frameInfo->Data[6] & 0x03;

	if(bleftAerr)
		m_canMsgRecv.Left_A_Quality=0;

	m_canMsgRecv.Left_A_LineType=(frameInfo->Data[7] & 0xE0)>>5;

	m_canMsgRecv.Left_A_ViewRange=(frameInfo->Data[7] & 0x1F)*4;

	outjiting<<"Left_A:   "<<m_canMsgRecv.Left_A_C3<<",  "<<m_canMsgRecv.Left_A_C2<<",  "<<m_canMsgRecv.Left_A_C0<<",  "<<m_canMsgRecv.Left_A_C1<<",  "<<m_canMsgRecv.Left_A_Quality<<",  "<<m_canMsgRecv.Left_A_LineType<<",  "<<m_canMsgRecv.Left_A_ViewRange<<endl;

	app->critical_haikanglock.Unlock();
}

void CCANBus::decode511(const VCI_CAN_OBJ * frameInfo)
{

	outlaneoffset<<"0x511:  "<<hex<<(unsigned int)*(unsigned char *)&(frameInfo->Data[0])<<", "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[1])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[2])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[3])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[4])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[5])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[6])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[7])<<endl;


	WORD high = 0;
	WORD low = 0;

	high = frameInfo->Data[0] << 8;
	low = frameInfo->Data[1];
	m_canMsgRecv.Left_B_C3 = (high + low) * (4.00e-09)-0.000131072;
	if(m_canMsgRecv.Left_B_C3<=-0.000131072)
		m_canMsgRecv.Left_B_C3=-0.000131072;
	if(m_canMsgRecv.Left_B_C3>=0.000131068)
		m_canMsgRecv.Left_B_C3=0.000131068;

	high = frameInfo->Data[2] << 8;
	low = frameInfo->Data[3];
	m_canMsgRecv.Left_B_C2 = (high + low) * (1.00e-06)-0.032767;
	if(m_canMsgRecv.Left_B_C2<=-0.032768)
		m_canMsgRecv.Left_B_C2=-0.032768;
	if(m_canMsgRecv.Left_B_C2>=0.032767)
		m_canMsgRecv.Left_B_C2=0.032767;

	high = frameInfo->Data[4] << 4;
	low = (frameInfo->Data[5] & 0xF0) >> 4;
	m_canMsgRecv.Left_B_C0 = (high + low) * 0.01-20.48;
	if(m_canMsgRecv.Left_B_C0<=-20.48)
		m_canMsgRecv.Left_B_C0=-20.48;
	if(m_canMsgRecv.Left_B_C0>=20.47)
		m_canMsgRecv.Left_B_C0=20.47;

	high = (frameInfo->Data[5] & 0x0F) << 6;
	low = (frameInfo->Data[6] & 0xFC) >> 2;
	m_canMsgRecv.Left_B_C1 = (high + low) * 0.00097656-0.49951044;

	m_canMsgRecv.Left_B_Quality=frameInfo->Data[6] & 0x03;

	m_canMsgRecv.Left_B_LineType=(frameInfo->Data[7] & 0xE0)>>5;

	m_canMsgRecv.Left_B_ViewRange=(frameInfo->Data[7] & 0x1F)*4;
}

void CCANBus::decode512(const VCI_CAN_OBJ * frameInfo)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	outlaneoffset<<"0x512:  "<<hex<<(unsigned int)*(unsigned char *)&(frameInfo->Data[0])<<", "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[1])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[2])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[3])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[4])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[5])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[6])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[7])<<endl;


	app->critical_haikanglock.Lock();

	bool brightAerr=false;

	WORD high = 0;
	WORD low = 0;

	high = frameInfo->Data[0] << 8;
	low = frameInfo->Data[1];
	m_canMsgRecv.Right_A_C3 = (high + low) * (4.00e-09)-0.000131072;
	if(m_canMsgRecv.Right_A_C3<=-0.000131072)
	{
		m_canMsgRecv.Right_A_C3=-0.000131072;
		brightAerr=true;
	}
	if(m_canMsgRecv.Right_A_C3>=0.000131068)
	{
		m_canMsgRecv.Right_A_C3=0.000131068;
		brightAerr=true;
	}

	high = frameInfo->Data[2] << 8;
	low = frameInfo->Data[3];
	m_canMsgRecv.Right_A_C2 = (high + low) * (1.00e-06)-0.032767;
	if(m_canMsgRecv.Right_A_C2<=-0.032768)
	{
		m_canMsgRecv.Right_A_C2=-0.032768;
		brightAerr=true;
	}
	if(m_canMsgRecv.Right_A_C2>=0.032767)
	{
		m_canMsgRecv.Right_A_C2=0.032767;
		brightAerr=true;
	}

	high = frameInfo->Data[4] << 4;
	low = (frameInfo->Data[5] & 0xF0) >> 4;
	m_canMsgRecv.Right_A_C0 = (high + low) * 0.01-20.48;
	if(m_canMsgRecv.Right_A_C0<=-20.48)
	{
		m_canMsgRecv.Right_A_C0=-20.48;
		brightAerr=true;
	}
	if(m_canMsgRecv.Right_A_C0>=20.47)
	{
		m_canMsgRecv.Right_A_C0=20.47;
		brightAerr=true;
	}

	high = (frameInfo->Data[5] & 0x0F) << 6;
	low = (frameInfo->Data[6] & 0xFC) >> 2;
	m_canMsgRecv.Right_A_C1 = (high + low) * 0.00097656-0.49951044;
	if(m_canMsgRecv.Right_A_C1<=-0.49951044)
	{
		m_canMsgRecv.Right_A_C1=-0.49951044;
		brightAerr=true;
	}
	if(m_canMsgRecv.Right_A_C1>=0.49951044)
	{
		m_canMsgRecv.Right_A_C1=0.49951044;
		brightAerr=true;
	}

	m_canMsgRecv.Right_A_Quality=frameInfo->Data[6] & 0x03;

	if(brightAerr)
		m_canMsgRecv.Right_A_Quality=0;

	m_canMsgRecv.Right_A_LineType=(frameInfo->Data[7] & 0xE0)>>5;

	m_canMsgRecv.Right_A_ViewRange=(frameInfo->Data[7] & 0x1F)*4;

	app->critical_haikanglock.Unlock();
}

void CCANBus::decode513(const VCI_CAN_OBJ * frameInfo)
{

	outlaneoffset<<"0x513:  "<<hex<<(unsigned int)*(unsigned char *)&(frameInfo->Data[0])<<", "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[1])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[2])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[3])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[4])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[5])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[6])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[7])<<endl;


	WORD high = 0;
	WORD low = 0;

	high = frameInfo->Data[0] << 8;
	low = frameInfo->Data[1];
	m_canMsgRecv.Right_B_C3 = (high + low) * (4.00e-09)-0.000131072;
	if(m_canMsgRecv.Right_B_C3<=-0.000131072)
		m_canMsgRecv.Right_B_C3=-0.000131072;
	if(m_canMsgRecv.Right_B_C3>=0.000131068)
		m_canMsgRecv.Right_B_C3=0.000131068;

	high = frameInfo->Data[2] << 8;
	low = frameInfo->Data[3];
	m_canMsgRecv.Right_B_C2 = (high + low) * (1.00e-06)-0.032767;
	if(m_canMsgRecv.Right_B_C2<=-0.032768)
		m_canMsgRecv.Right_B_C2=-0.032768;
	if(m_canMsgRecv.Right_B_C2>=0.032767)
		m_canMsgRecv.Right_B_C2=0.032767;

	high = frameInfo->Data[4] << 4;
	low = (frameInfo->Data[5] & 0xF0) >> 4;
	m_canMsgRecv.Right_B_C0 = (high + low) * 0.01-20.48;
	if(m_canMsgRecv.Right_B_C0<=-20.48)
		m_canMsgRecv.Right_B_C0=-20.48;
	if(m_canMsgRecv.Right_B_C0>=20.47)
		m_canMsgRecv.Right_B_C0=20.47;

	high = (frameInfo->Data[5] & 0x0F) << 6;
	low = (frameInfo->Data[6] & 0xFC) >> 2;
	m_canMsgRecv.Right_B_C1 = (high + low) * 0.00097656-0.49951044;

	m_canMsgRecv.Right_B_Quality=frameInfo->Data[6] & 0x03;

	m_canMsgRecv.Right_B_LineType=(frameInfo->Data[7] & 0xE0)>>5;

	m_canMsgRecv.Right_B_ViewRange=(frameInfo->Data[7] & 0x1F)*4;
}


void CCANBus::decode514(const VCI_CAN_OBJ * frameInfo)
{

	outlaneoffset<<"0x514:  "<<hex<<(unsigned int)*(unsigned char *)&(frameInfo->Data[0])<<", "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[1])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[2])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[3])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[4])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[5])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[6])<<" "<<(unsigned int)*(unsigned char *)&(frameInfo->Data[7])<<endl;

	if(m_canMsgRecv.ObstacleTotalNum>=30)
		m_canMsgRecv.ObstacleTotalNum=0;

	WORD high = 0;
	WORD low = 0;

	m_canMsgRecv.Obstacle_ID[m_canMsgRecv.ObstacleTotalNum]=frameInfo->Data[0] & 0x07;
	m_canMsgRecv.Obstacle_Type[m_canMsgRecv.ObstacleTotalNum]=(frameInfo->Data[0] & 0x38)>>3;
	m_canMsgRecv.Obstacle_Quality[m_canMsgRecv.ObstacleTotalNum]=(frameInfo->Data[0] & 0xC0)>>6;

	high = (frameInfo->Data[1] & 0x07) << 8;
	low = frameInfo->Data[2];
	m_canMsgRecv.Obstacle_PosX[m_canMsgRecv.ObstacleTotalNum] = (high + low) * 0.1;

	m_canMsgRecv.Obstacle_PosY[m_canMsgRecv.ObstacleTotalNum] = frameInfo->Data[3] * 0.1 - 12.8;

	m_canMsgRecv.Obstacle_Width[m_canMsgRecv.ObstacleTotalNum] = (frameInfo->Data[4] & 0x3F) * 0.1;

	m_canMsgRecv.Obstacle_Length[m_canMsgRecv.ObstacleTotalNum] = (frameInfo->Data[5] & 0x3F) * 0.1;

	m_canMsgRecv.Obstacle_VehcleX[m_canMsgRecv.ObstacleTotalNum] = (frameInfo->Data[6] & 0x7F) - 80;

	m_canMsgRecv.Obstacle_VehcleY[m_canMsgRecv.ObstacleTotalNum] = (frameInfo->Data[7] & 0x3F) - 32;
}

// ???值???
UINT CCANBus::sendLimiting(void)
{
	if (m_canMsgSend.brake < 0)
	{
		m_canMsgSend.brake = 0;
		return 1;
	}
	if (m_canMsgSend.brake > 100)
	{
		m_canMsgSend.brake = 100;
		return 2;
	}

	if (m_canMsgSend.driveTorque > m_canMsgRecv.TqGndPosMax)
	{
		m_canMsgSend.driveTorque = m_canMsgRecv.TqGndPosMax;
		return 3;
	}
	if (m_canMsgSend.driveTorque < m_canMsgRecv.TqGndNegMax)
	{
		m_canMsgSend.driveTorque = m_canMsgRecv.TqGndNegMax;
		return 4;
	}

	if (m_canMsgSend.EPBActive > 2)
	{
		m_canMsgSend.EPBActive = 0;
		return 5;
	}

	if (m_canMsgSend.hornOn > 1)
	{
		m_canMsgSend.hornOn = 1;
		return 6;
	}

	if (m_canMsgSend.steerAngle > 779.9)
	{
		m_canMsgSend.steerAngle = 779.9;
		return 7;
	}
	if (m_canMsgSend.steerAngle < -780)
	{
		m_canMsgSend.steerAngle = -780;
		return 8;
	}

	if (m_canMsgSend.turnLamp > 2)
	{
		m_canMsgSend.turnLamp = 0;
		return 9;
	}
	return 0;
}


// PC????艿?前GPS??纬???
UINT CCANBus::GPSEncAsk(double GpsLatitude, double GpsLongitude)
{
	VCI_CAN_OBJ frame;
	DWORD32 lat = (GpsLatitude + 90) / 1e-7;
	DWORD32 lng = (GpsLongitude + 180) / 1e-7;

	frame.ID = 0x100;
	frame.SendType = m_sendType;
	frame.RemoteFlag = 0;
	frame.ExternFlag = 0;
	frame.DataLen = 8;

	frame.Data[0] = lat >> 24;
	frame.Data[1] = lat >> 16;
	frame.Data[2] = lat >> 8;
	frame.Data[3] = lat;
	frame.Data[4] = lng >> 24;
	frame.Data[5] = lng >> 16;
	frame.Data[6] = lng >> 8;
	frame.Data[7] = lng;

	ULONG lRet = VCI_Transmit(VCI_USBCAN_2E_U, m_devIndex, m_CANIndex, &frame, 1);
	if (lRet != 1)
	{
		AfxMessageBox("发送失败");
		return 1;
	}
	return 0;
}


// ??始?????????辖?
void CCANBus::AVFaultDefault(void)
{
	memset(&m_avFault, 0, sizeof(AVFault));
}


// ???????????
UINT CCANBus::sendAVFault(void)
{
	VCI_CAN_OBJ frame;
	UINT8 d1 = ((m_avFault.HDL_ConnectFault & 0x1) << 7)
			   | ((m_avFault.IBEO1_ConnectFault & 0x1) << 6)
			   | ((m_avFault.IBEO2_ConnectFault & 0x1) << 5)
			   | ((m_avFault.Camera1_ConnectFault & 0x1) << 4)
			   | ((m_avFault.Camera2_ConnectFault & 0x1) << 3)
			   | ((m_avFault.Camera3_ConnectFault & 0x1) << 2)
			   | ((m_avFault.ESR1_ConnectFault & 0x1) << 1)
			   | (m_avFault.ESR2_ConnectFault & 0x1);
	UINT8 d2 = (((m_avFault.ESR3_ConnectFault & 0x1) << 7)
			   | ((m_avFault.ESR4_ConnectFault & 0x1) << 6)
			   | ((m_avFault.CAN_ConnectFault & 0x1) << 5)
			   | ((m_avFault.GPS_ConnectFault & 0x1) << 4)
			   | ((m_avFault.Ethernet_ConnectFault & 0x1) << 3))
			   & 0xf8;

	frame.ID = 0x98;
	frame.SendType = m_sendType;
	frame.RemoteFlag = 0;
	frame.ExternFlag = 0;
	frame.DataLen = 8;

	frame.Data[0] = d1;
	frame.Data[1] = d2;
	frame.Data[2] = 0;
	frame.Data[3] = 0;
	frame.Data[4] = 0;
	frame.Data[5] = 0;
	frame.Data[6] = 0;
	frame.Data[7] = 0;

	ULONG lRet = VCI_Transmit(VCI_USBCAN_2E_U, m_devIndex, m_CANIndex, &frame, 1);
	if (lRet != 1)
	{
		AfxMessageBox("发送失败");
		return 1;
	}
	return 0;
}


// ?????ADASIS?????息???路????????
UINT CCANBus::sendAVReq(UINT8 MapPlaningReq, UINT8 MapDataResendReq)
{
	VCI_CAN_OBJ frame;

	frame.ID = 0x96;
	frame.SendType = m_sendType;
	frame.RemoteFlag = 0;
	frame.ExternFlag = 0;
	frame.DataLen = 8;

	frame.Data[0] = (MapPlaningReq | (MapDataResendReq << 1)) & 0x3;
	frame.Data[1] = 0;
	frame.Data[2] = 0;
	frame.Data[3] = 0;
	frame.Data[4] = 0;
	frame.Data[5] = 0;
	frame.Data[6] = 0;
	frame.Data[7] = 0;

	ULONG lRet = VCI_Transmit(VCI_USBCAN_2E_U, m_devIndex, m_CANIndex, &frame, 1);
	if (lRet != 1)
	{
		AfxMessageBox("发送失败");
		return 1;
	}
	return 0;
}

void CCANBus:: DecodeADASISData(const BYTE buf[], BYTE cycn, BYTE type)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	//segment message
	if (type == 2)
	{
		if ((m_iSegPreCyCn == -1 && (cycn == 0 || cycn == 1 || cycn == 2 || cycn == 3)) || (m_iSegPreCyCn == 3 && cycn == 0) || (m_iSegPreCyCn == 0 && cycn == 1) || (m_iSegPreCyCn == 1 && cycn == 2) || (m_iSegPreCyCn == 2 && cycn == 3))
		{
			Segment segment;
			int offset = ((buf[0] & 0x1F) << 8) + buf[1];
			if (m_segQueue.size() != 0)
			{
				if ((m_iNumSeg != 0 && offset <  (m_segQueue[m_segQueue.size() - 1].offset % (8191*m_iNumSeg))) || (m_iNumSeg == 0 && offset < m_segQueue[m_segQueue.size() - 1].offset))
				{
					m_iNumSeg++;
				}
			}
			segment.offset = 8191*m_iNumSeg + offset;
			segment.pathindex = buf[2] & 0x3F;
			segment.lanenum = buf[7] & 0x07;
			m_segQueue.push_back(segment);
			m_iSegPreCyCn = cycn;
			outsegment<<segment.offset<<endl;
		}
		else
		{
			m_iErr = 1;
		}
	}
	else if (type == 3)
	{
		//stub message
		if ((m_iStuPreCycn == -1 && (cycn == 0 || cycn == 1 || cycn == 2 || cycn == 3)) || (m_iStuPreCycn == 3 && cycn == 0) || (m_iStuPreCycn == 0 && cycn == 1) || (m_iStuPreCycn == 1 && cycn == 2) || (m_iStuPreCycn == 2 && cycn == 3))
		{
			Stub stub;
			int offset = ((buf[0] & 0x1F) << 8) + buf[1];
			if (m_stubQueue.size() != 0)
			{
				if ((m_iNumStub == 0 && offset < m_stubQueue[m_stubQueue.size() - 1].offset) || (m_iNumStub != 0 && offset < (m_stubQueue[m_stubQueue.size() - 1].offset % (8191*m_iNumStub))))
				{
					m_iNumStub++;
				}
			}
			stub.offset = 8191*m_iNumStub + offset;
			stub.pathindex = buf[2] & 0x3F;
			int type = (buf[5] & 0xF);
			if (type == 6)
			{
				stub.type = 4; // U-Turn
			}
			else if(type == 7)
			{
				stub.type = 3; // Right turn
			}
			else
			{
				int turnangle = buf[6];
				if (turnangle <= 20 || turnangle > 234)
				{
					stub.type = 1;//直?
				}
				else if(turnangle > 107 && turnangle <= 147)
				{
					stub.type = 4;//???
				}
				else if(turnangle > 20 && turnangle <= 107)
				{
					stub.type = 2;//?转
				}
				else if(turnangle > 147 && turnangle <= 234)
				{
					stub.type = 5;//?转
				}
			}
			if (stub.offset != 8191 && stub.pathindex != 0)
			{
				m_stubQueue.push_back(stub);

				//m_iStuPreCycn = cycn;
				outstub<<stub.offset<<endl;
			}			
			else
			{	//hqj 0917
				m_rnPts.clear();
				m_stubQueue.clear();
				m_segQueue.clear();				
			}

			m_iStuPreCycn = cycn;//hqj

				if (stub.offset == 8191 )
					outstub<<" one stub offset=8191"<<endl;
				if ( stub.pathindex == 0)
					outstub<<" one stub pathindex=0"<<endl;


		}
		else
		{
			m_iErr = 1;
		}

	}

	else if (type == 4)
	{
		//profile short message
		if ((m_iPrsPreCycn == -1 && (cycn == 0 || cycn == 1 || cycn == 2 || cycn == 3)) || (m_iPrsPreCycn == 3 && cycn == 0) || (m_iPrsPreCycn == 0 && cycn == 1) || (m_iPrsPreCycn == 1 && cycn == 2) || (m_iPrsPreCycn == 2 && cycn == 3))
		{
			int prstype = buf[3]>>3;
			if (prstype == 17)
			{
				Stub stub;
				int offset = ((buf[0] & 0x1F) << 8) + buf[1];
				stub.offset = 8191*m_iNumStub + offset;
				stub.pathindex = buf[2] & 0x3F;
				stub.type = 1;
                                                    m_stubQueue.push_back(stub);
                                                    outstub<<stub.offset<<endl;
			}
			m_iPrsPreCycn = cycn;
		}
		else
		{
			m_iErr = 1;
			outstub<<"m_iErr= "<<m_iErr<<endl;//hqj
		}

	}

	else if (type == 5)
	{
		//longitude
		if ((buf[3] & 0xF8) >> 3 == 1)
		{
			if((m_iPrlPreCycn == -1 && (cycn == 0 || cycn == 1 || cycn == 2 || cycn == 3)) || (m_iPrlPreCycn == 3 && cycn == 0) || (m_iPrlPreCycn == 0 && cycn == 1) || (m_iPrlPreCycn == 1 && cycn == 2) || (m_iPrlPreCycn == 2 && cycn == 3))
			{
				unsigned int val = (buf[4] << 24) + (buf[5] << 16) + (buf[6] << 8) + buf[7];
				m_pt.lng = val*0.0000001 + m_dLngOffset - 180.;
				m_iPrlPreCycn = cycn;
			}
			else
			{
				m_iErr = 1;
			}
		}
		//latitude
		else if ((buf[3] & 0xF8) >> 3 == 2)
		{
			if (m_iPrlPreCycn == cycn)
			{
				int offset = ((buf[0] & 0x1F) << 8) + buf[1];
				if (m_prePt.lng != 0.0 && m_prePt.lat != 0.0)
				{
					if ((m_iNumProLng == 0 && offset < m_prePt.offset) || (m_iNumProLng != 0 && offset < (m_prePt.offset % (8191*m_iNumProLng))))
					{
						m_iNumProLng++;
					}
				}
				offset = 8191*m_iNumProLng + offset;
				unsigned int val = (buf[4] << 24) + (buf[5] << 16) + (buf[6] << 8) + buf[7];
				m_pt.lat = val*0.0000001 + m_dLatOffset - 90.;
				m_pt.height = 0.0;
				m_pt.offset = offset;
				m_pt.pathidx = buf[2] & 0x3F;
				m_pt.lanenum = -1;



				if (m_prePt.lng != 0.0 && m_prePt.lat != 0.0)
				{
					if (!m_bJunction)
					{
						BOOL flag = FALSE;
						int pos = 0;
						for (int i = 0; i < m_stubQueue.size(); i++)
						{
							if (m_stubQueue[i].offset == m_prePt.offset )
							{
								flag = TRUE;
								pos = i;
								break;
							}
						}
						if (flag)
						{
							m_prePt.type = 3;//路?诘?
							m_prePt.type1 = m_stubQueue[pos].type;
							m_bJunction = TRUE;
						}
						else
						{
							if (m_prePt.pathidx == 7 && (m_prePt.offset % 8191) == 0)
							{
								m_prePt.type = 1;//???
							}
							else
							{
								m_prePt.type = 2;//路???
							}

							m_prePt.type1 = 0;
						}

					}
					else
					{
						if (m_prePt.pathidx == 7 && (m_prePt.offset % 8191) == 0)
						{
							m_prePt.type = 1;//???
						}
						else
						{
							m_prePt.type = 3;
						}

						m_prePt.type1 = 0;
						m_bJunction = FALSE;
					}
					if (m_segQueue.size() == 1)
					{
						m_prePt.lanenum = m_segQueue[0].lanenum;
					}
					else
					{
						for (int i = 0; i < m_segQueue.size() - 1; i++)
						{
							if (m_prePt.offset >= m_segQueue[i].offset && m_prePt.offset < m_segQueue[i + 1].offset)
							{
								m_prePt.lanenum = m_segQueue[i].lanenum;
								break;
							}
							else if ((i + 1 == m_segQueue.size() - 1) && (m_prePt.offset >= m_segQueue[i+1].offset) )
							{
								m_prePt.lanenum = m_segQueue[i + 1].lanenum;
								break;
							}
							else
							{
								continue;
							}
						}
					}
					m_rnPts.push_back(m_prePt);
					m_iPrlPreCycn = cycn;
					adas_id++;
					LEAD lead_temp;
					lead_temp.id = adas_id;
					lead_temp.height = m_prePt.height;
					lead_temp.lat = m_prePt.lat;
					lead_temp.lng = m_prePt.lng;
					lead_temp.param1 = m_prePt.type;
					lead_temp.param2 = m_prePt.type1;
					lead_temp.param3 = m_prePt.lanenum;

					outadas<<setprecision(11)<<m_prePt.lng<<" "<<m_prePt.lat<<" "<<m_prePt.height<<" "<<m_prePt.type<<" "<<m_prePt.type1<<" "<<m_prePt.lanenum<<" "<<m_prePt.offset<<endl;

					for (int ii = 0; ii < m_stubQueue.size(); ii++)
					{
						outadas1<<m_stubQueue[ii].offset<<endl;
					}
					outadas1<<setprecision(11)<<m_prePt.lng - m_dLngOffset<<" "<<m_prePt.lat - m_dLatOffset<<" "<<m_prePt.height<<" "<<m_prePt.type<<" "<<m_prePt.type1<<" "<<m_prePt.lanenum<<" "<<m_prePt.offset<<endl;

					app->critical_adasdata.Lock();//hqj
					cvSeqPush(app->ADAS_points,&lead_temp);//0917
					app->critical_adasdata.Unlock();//hqjs
				}
				m_prePt = m_pt;

			}
			else
			{
				m_iErr = 1;
			}
		}

	}
	//if receive message error
	if (m_iErr == 1)
	{
		adas_id = 0;

		outstub<<"really clear all vector !!"<<endl;//hqj
		m_rnPts.clear();
		m_stubQueue.clear();
		m_segQueue.clear();

		m_iPrsPreCycn = -1;//hqj
		m_iPrlPreCycn = -1;
		m_iSegPreCyCn = -1;
		m_iStuPreCycn = -1;

		outstub<<"really clear all ADAS point !!"<<endl;
		app->critical_adasdata.Lock();
		cvClearSeq(app->ADAS_points);
		app->critical_adasdata.Unlock();
		
		//send message re-send require
		sendAVReq(0, 1);
		PostThreadMessage(app->dwProcThreadId, RETRMSGREQ, 0, 0);
outstub<<"really re-send !!"<<endl;//hqj
	}
}
