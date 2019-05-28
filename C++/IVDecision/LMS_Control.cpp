
#include "stdafx.h"
#include "LMS_Control.h"

using namespace std;
//using namespace System;

#define CRC16_GEN_POL 0x8005
#define MKSHORT(a,b) ((unsigned short) (a) | ((unsigned short)(b) << 8))


LMSControl::LMSControl()
{
	isContinuousOutput = false;
	dataLength = LEN180X5;
	range = RANGE180;
	res = RES5;
}

LMSControl::~LMSControl()
{
	
}

/***************************************************************************
 *
 * Name: 配置COM端口
 *
 ***************************************************************************/
HANDLE LMSControl::ConfigureComPort(LPCSTR comPort)
{
		m_hCommPort = CreateFile( 
								 comPort,
								 GENERIC_ALL, 
								 0, 
								 0, 
								 OPEN_EXISTING, 
								 FILE_FLAG_NO_BUFFERING,
								 0
								);
		
		//无法发现串口或者已被占用-->未找到错误
		if (m_hCommPort == INVALID_HANDLE_VALUE)
		{

			if(GetLastError()==ERROR_NOT_FOUND)
			{
				//throw new exception("Fatal Error!!\nError Not Found");
				printf("Error Not Found - Fatal Error!");
				return 0;
			}

			printf("Cannot find the Serial Port or it may be in use by another device. Close all other applications using the port and restart this Application - Fatal Error");
			return 0;
		}

		//设置COM端口  SETUP COM Port
		ZeroMemory(&dcbSerialParams, sizeof(dcbSerialParams));//测量数据类型长度
		dcbSerialParams.DCBlength  = sizeof(dcbSerialParams);

		//无法得到串口信息
		if(!GetCommState(m_hCommPort, &dcbSerialParams))
		{
			printf("Unable to Retrieve Serial Port Status information - Fatal Error!");
			return 0;
			//throw new exception("Fatal Error!!\nUnable to Retrieve Serial Port Status information.");
		}

		dcbSerialParams.BaudRate				= CBR_500000;
		dcbSerialParams.ByteSize				= 8;
		dcbSerialParams.StopBits				= ONESTOPBIT;
		dcbSerialParams.Parity					= NOPARITY;

		//无法设置串口状态信息
		if(!SetCommState(m_hCommPort, &dcbSerialParams))
		{
			printf("Unable to Set Serial Port Status information - Fatal Error!");
			return 0;
			//throw new exception("\nFatal Error!!\nUnable to Set Serial Port Status information.");
		}

		//设置超时  Set the timeouts we control the timeout overselves using WaitForX()
		COMMTIMEOUTS timeouts;
		timeouts.ReadIntervalTimeout		    = MAXDWORD; 
		timeouts.ReadTotalTimeoutMultiplier		= 0;
		timeouts.ReadTotalTimeoutConstant		= 0;
		timeouts.WriteTotalTimeoutMultiplier	= 0;
		timeouts.WriteTotalTimeoutConstant		= 0;


		//无法设置串口-超时
		if (!SetCommTimeouts(m_hCommPort, &timeouts))
		{
			printf("Unable to Set Serial Port Time-Outs - Fatal Error!");
            return 0;
			// new exception("\nFatal Error!!\nUnable to Set Serial Port Time-Outs.");
		}

		//清除缓冲区 TRY PurgeComm ON THE COM PORT TO FLUSH BUFFER
		int abRet2  = PurgeComm( m_hCommPort, PURGE_RXCLEAR );
		return m_hCommPort;
}


/***************************************************************************
 *
 * Name: 更改LMS角度分辨率
 *
 ***************************************************************************/
void LMSControl::ChangeAngleRes(int mode)
{
	uchar buf[14];	
	
	bool wasContinuous;

	//确定模式为0到5 make sure mode is between 0 and 5
	if (mode<0 || mode>5) mode = 0;//大于或小于都不成立

	wasContinuous = isContinuousOutput;
	if (isContinuousOutput) StopContinuousOutput();

	//写角度分辨率到LMS Write out Angle of Resolution Telegrams to LMS
	do
	{
		SendMessage(11, ANGLE_RES[mode]);
	}
	while (!ReadLMSmsg(14, buf));


	//更新物体距离和分辨率变量 update object range and res variables
	switch (mode)
	{
		case ANGLE_RES_100X1:
			range = RANGE100;
			res = RES1;
			dataLength = LEN100X1;
			break;
		case ANGLE_RES_100X0_5:
			range = RANGE100;
			res = RES5;
			dataLength = LEN100X5;
			break;
		case ANGLE_RES_100X0_25:
			range = RANGE100;
			res = RES25;
			dataLength = LEN100X25;
			break;
		case ANGLE_RES_180X1:
			range = RANGE180;
			res = RES1;
			dataLength = LEN180X1;
			break;
		default: //ANGLE_RES_180X0_5
			range = RANGE180;
			res = RES5;
			dataLength = LEN180X5;
	}

	if (wasContinuous) StartContinuousOutput();

}


/***************************************************************************
 *
 * Name: 改变传输波特率
 *
 ***************************************************************************/
void LMSControl::ChangeBaudRate(int baud)
{
	uchar buf[10];
	bool wasContinuous;

	//确定波特率有效 Make sure baud is valid
	if ((baud != BR_19200)&&(baud != BR_38400)&&(baud != BR_500000)&&(baud != BR_9600))	baud = BR_9600;

	wasContinuous = isContinuousOutput;
	if (isContinuousOutput) StopContinuousOutput();

    //
	do
	{
		SendMessage(8,BAUD_RATE[baud]);
	}
	while (!ReadLMSmsg(10, buf));
	
	dcbSerialParams.BaudRate=CBR[baud];

	//无法设置状态信息
	if(!SetCommState(m_hCommPort, &dcbSerialParams))
	{
		printf("Unable to Set Serial Port Status information - Fatal Error!");
	}

	if (wasContinuous) StartContinuousOutput();

}


/***************************************************************************
 *
 * Name: 开始连续输出
 *
 ***************************************************************************/
void LMSControl::StartContinuousOutput()
{
	uchar buf[10];

	/*if (keepLog || keepStats)
		StartLog();*/

	//如果没有开始，则开始连续输出模式 If it is not already started, start continuous output mode
	if (!isContinuousOutput)
	{
		do
		{
			SendMessage(8, START_CONTINUOUS);
		}
		while (!ReadLMSmsg(10, buf));
		isContinuousOutput = true;
	}
}


/***************************************************************************
 *
 * Name: 停止连续输出
 *
 ***************************************************************************/
void LMSControl::StopContinuousOutput()
{
	uchar buf[10];

	//如果在连续模式则停止连续输出模式 Stop continuous output mode if it is currently in continuous mode
   	if (isContinuousOutput)
	{
		do
		{
			SendMessage(8, STOP_CONTINUOUS);
		}
		while (!ReadLMSmsg(10, buf));
		isContinuousOutput = false;
	}
}


/***************************************************************************
 *
 * Name: 分析数据
 *
 ***************************************************************************/
int LMSControl::ReadLMSData(int *intBuf, int coordinateSystem)
{
	// Purge buffer
	PurgeComm( m_hCommPort, PURGE_RXCLEAR );
	//unsigned short realCRC = 0;
	uchar buf[MAXPACKET];

	int tmpBuf[MAXDATA];
	int packetDataLen=0; //number of data bytes, each of them is 16bits long
	int realCRC, lenBytes;
	unsigned short CRCcalculated;

	buf[0] = 0;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;
	buf[4] = 0;
	//int repeat = 0;

	while (buf[0] != STX) //开始字节STX
	{
		GetByte(buf[0]);
	}

	GetByte(buf[1]); //地址 should be the ADR byte, ADR=0x80 here
	//长度表示从命令字节开始到结尾，不包括校验
	//LEN refers to the packet length in bytes from the command byte to end not including checksum
	GetByte(buf[2]); //数据长度低字节 should be the LEN low byte
	GetByte(buf[3]); //数据长度高字节 should be the LEN high byte
	GetByte(buf[4]); //命令字节 Command byte 0xB0 for continuous, F5 for continuous with reflectivity

	//检查是否得到正确的开始字节、地址字节、命令字节 Check that we have a valid STX byte, valid ADR byte, and valid CMD byte
	while (
		(buf[0] != STX) || 
		(0x80 != buf[1]) || 
		(0xb0 != buf[4])
		)
	{	

		buf[0] = buf[1];
		buf[1] = buf[2];
		buf[2] = buf[3];
		buf[3] = buf[4];
		GetByte(buf[4]);
		
	}

	packetDataLen = buf[2] | ((buf[3] & 0x1f) << 8);
	//减去1个命令字节位 subtract 1 from packetDataLen for the cmd I already read
	packetDataLen--;
	//读除了校验位外所有数据 read in all the data in the packet except the checksum
	ReadLMS(packetDataLen,buf+5);
	//得到CRC get CRC
	GetByte(buf[packetDataLen + 5]); //CRC低字节 should be CRC low byte*/
	GetByte(buf[packetDataLen + 6]); //CRC高字节 should be CRC high byte*/

	realCRC = buf[packetDataLen+5] | (buf[packetDataLen+6] << 8); 
	lenBytes = packetDataLen+5;
	CRCcalculated = LMSCRC(buf, lenBytes);

	if (CRCcalculated != realCRC) 
	{
		return 0;
	}

	//转化测量值到整数数组 convert measured values to integer array
	if (coordinateSystem == RECT)
	{
		//成立交替X Y数组 creates an array of integers alternating X and Y values
		ToIntArrayPolar(dataLength, buf+7, tmpBuf);
		/*
		if (keepLog || keepStats)
		{
			WriteLog(tmpBuf, dataLength);
		}
		*/
		PolarToRect(dataLength, tmpBuf, intBuf);
		return dataLength*2;
	}
	else
	{
		ToIntArrayPolar(dataLength, buf+7, intBuf);
		/*if (keepLog || keepStats)
		{
			WriteLog(intBuf, dataLength);
		}
		*/
		return dataLength;
	}	
}


/***************************************************************************
 *
 * Name: 读COM端口消息
 *
 ***************************************************************************/
bool LMSControl::ReadLMSmsg(int len, uchar *buf)
{
	*buf = 0;
	//等待消息头 wait for the message header
	while(buf[0] != ACKSTX)
	{
		if (!GetByte(buf[0]))
		{
			Reconfigure();
			return false;
		}
	}
	return ReadLMS(len-1, buf+1);
}


/***************************************************************************
 *
 * Name: 读整个LMS数据包
 *
 ***************************************************************************/
bool LMSControl::ReadLMS(int len, uchar *buf)
{
	for (int i=0;i<len;i++)
	{
		if (!GetByte(buf[i]))
		{
			Reconfigure();
			return false;
		}
	}

	return true;
}


/***************************************************************************
 *
 * Name: 初始化
 *
 ***************************************************************************/
void LMSControl::ToIntArrayPolar(int len, uchar *buf, int *intBuf)
{
	for (int i=0;i<len;i++)
	{
		// only upper 12 bits of upper byte are used
		intBuf[i] = buf[2*i] | ((buf[2*i+1] & 0x1f) << 8);
	}
}


/***************************************************************************
 *
 * Name: 得到字节
 *
 ***************************************************************************/
bool LMSControl::GetByte(uchar& c) 
{
	DWORD nBytes = 0;
	time_t finish;
	int success = 0;
	char ch= '\0';

	finish = time(NULL) + 11;
	while ((success == 0) && (time(NULL) < finish)) 
	{
		ReadFile(m_hCommPort, &ch, 1, &nBytes, NULL);
		success = nBytes;
	}

	if (success == 0) 
	{
		return false;
	}

	c = ch;
	return true;
}


/***************************************************************************
 *
 * Name: 发送消息
 *
 ***************************************************************************/
void LMSControl::SendMessage(int len, const uchar* buf) 
{
	for(int i=0;i<len;i++)
	{
		SendByte(&buf[i]);
	}
}


/***************************************************************************
 *
 * Name: 发送字节
 *
 ***************************************************************************/
void LMSControl::SendByte(const uchar* c)
{

	DWORD nBytes = 0;
	time_t finish;
	int success = 0;

	finish = time(NULL) + 5;
	while ((success == 0) && (time(NULL) < finish)) 
	{
		WriteFile(m_hCommPort, c, 1, &nBytes, NULL);
		success = nBytes;
	}

	while (success == 0) 
	{
		Reconfigure();
		finish = time(NULL) + 5;
		while ((success == 0) && (time(NULL) < finish)) 
		{
			WriteFile(m_hCommPort, c, 1, &nBytes, NULL);
			success = nBytes;
		}
	}

}


/***************************************************************************
 *
 * Name: 得到图形
 *
 ***************************************************************************/
int LMSControl::PolarToRect(int length, int* fromArray, int* toArray)
{
	double angle;

	//设置第一个数据元素的角度 set the angle of the first data element
	//第一个数据为LMS最右边的数据 first data element is the right most point relative to the LMS
	if(range == RANGE180)
	{
		angle = 0;
	}
	else
	{
		//第一个数据为100读扫描模式里的10度 first data point is at 40 degrees in 100 degree sweep mode
		angle = 40;
	}


	for (int i=0;i<length;i++)
	{
		//得到X坐标 get X coordinate
		toArray[2*i] = (int)(fromArray[i]*cos(CV_PI * angle/180));
		//得到Y坐标 get Y coordinate
		toArray[2*i+1] = (int)(fromArray[i]*sin(CV_PI * angle/180));
		//角度增加 increment the angle
		angle= angle+res;
	}
	return length*2;
}


int LMSControl::GetDataLength()
{
	return dataLength;
}


/***************************************************************************
 *
 * Name: CRC校验
 *
 ***************************************************************************/
unsigned short LMSControl::LMSCRC(unsigned char* CommData, int lenBytes)
{
	unsigned short uCrc16;
	unsigned char abData[2];

	uCrc16 = 0;
	abData[0] = 0;

	while (lenBytes-- )
	{
		abData[1] = abData[0];
		abData[0] = *CommData++;
		if(uCrc16 & 0x8000)
		{
			uCrc16 = (uCrc16 & 0x7fff) << 1;
			uCrc16 ^= CRC16_GEN_POL;
		}
		else
		{
			uCrc16 <<= 1;
		}
		uCrc16 ^= MKSHORT (abData[0] , abData[1]);
	}
	return uCrc16;
}


/***************************************************************************
 *
 * Name: 重新配置设备
 *
 ***************************************************************************/
void  LMSControl::Reconfigure()
{
	DWORD startBaud;
	uchar buf[MAXPACKET];
	bool success;
	//int realCRC;
	//unsigned short CRCcalculated;
	//int len;

	startBaud = dcbSerialParams.BaudRate;

	do
	{
		success = true;
		//改变波特率 change the baud rate
		if (dcbSerialParams.BaudRate==CBR_500000)
		{
			dcbSerialParams.BaudRate=CBR_38400;
		}
		else if(dcbSerialParams.BaudRate==CBR_9600)
		{
			dcbSerialParams.BaudRate=CBR_500000;
		}
		else if(dcbSerialParams.BaudRate==CBR_19200)
		{
			dcbSerialParams.BaudRate=CBR_9600;	
		}
		else if(dcbSerialParams.BaudRate==CBR_38400)
		{
			dcbSerialParams.BaudRate=CBR_19200;
		}

		//所有波特率通讯失败 if I've checked all baud rates and all have failed send a message. and close the program
		if (dcbSerialParams.BaudRate == startBaud)
		{ 
			printf("Communication at all baudrates has failed. Try turning on the LMS - Communication Failure");
		}

		//设置计算机串口波特率 set the computer serial port baud rate
		if(!SetCommState(m_hCommPort, &dcbSerialParams))
		{
			printf("Unable to set Serial Port information - Fatal Error!");
		}

		//发送操作数据计数测试通讯 send request for Operating Data Counter just to test communication 
		//短消息短回复 is it a short message with a short response
		SendMessage(7,	REQ_TYPE);

		*buf = 0;
		//等待消息头 wait for the message header
		while(buf[0] != ACKSTX)
		{
			if (!GetByte(buf[0]))
			{
				success = false;
				break;
			}
		}

		if (success)
		{
			//得到剩余消息 get the rest of the message
			for (int i=0;i<28;i++)
			{
				if (!GetByte(buf[i]))
				{
					success = false;
					break;
				}
			}
		}
	}
	while (!success);
}


/***************************************************************************
*
* Name:坐标转地图，生成512*512地图
*
***************************************************************************/
int LMSControl::GetSickMap( int *map )
{
	memset(map, 0, 512*512);

	int lidar[MAXDATA];
	memset(lidar, 0, MAXDATA);
	int num = 0;
	num = ReadLMSData(lidar, RECT);

	for(int i=0; i<num/2; i++)
	{
		int x = lidar[2*i];
		int y = lidar[2*i+1];

		if( x >= -256 && x < 256 &&  y >= 0 && y < 512 )
		{
			map[ y*512+x+256 ] = 8;
		}
	}
	return num;
}
/***************************************************************************
*
* Name:坐标转地图，生成512*512地图
*
***************************************************************************/
int LMSControl::GetSickMap( int *map , float scale)
{
	memset(map, 0, 512*512*sizeof(int));

	int lidar[804];
	memset(lidar, 0, 804*sizeof(int));
	int num = ReadLMSData(lidar, POLAR);

	//
	// Convert from polar to Cartesian coordinates using openCV
	//
	CvMat* xpoints = cvCreateMat(num, 1, CV_32FC1);
	CvMat* ypoints = cvCreateMat(num, 1, CV_32FC1);
	CvMat* polar_mags = cvCreateMat(num, 1, CV_32FC1);
	CvMat* polar_angs = cvCreateMat(num, 1, CV_32FC1);
	for(int a = 0; a < num; a++)
	{
		if(num % 180 == 1)
			CV_MAT_ELEM(*polar_angs, float, a, 0) = (float)a / (float)((num-1)/180);
		else
			// Center 100 degree scans around +90 degrees for conversion
			CV_MAT_ELEM(*polar_angs, float, a, 0) = (float)40 + (float)a / (float)((num-1)/100);
		CV_MAT_ELEM(*polar_mags, float, a, 0) = (float)lidar[a]/3;
	}
	cvPolarToCart(polar_mags, polar_angs, xpoints, ypoints, 1);

	for(int i=0; i<num; i++)
	{
		int x = int(cvmGet(xpoints,i,0)*scale)+256;
		int y = int(512-cvmGet(ypoints,i,0)*scale);
		
		if( x>=0 && x<512 && y >=0 && y < 512 )
		    map[y*512 + x] = 8;
	}

	cvReleaseMat( &xpoints );
	cvReleaseMat( &ypoints );
	cvReleaseMat( &polar_mags );
	cvReleaseMat( &polar_angs );

	return num;
}

