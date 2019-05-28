#include "StdAfx.h"
#include "ADASISDataReceiver.h"

#include <iostream>
#include <fstream>
#include <iomanip>
ofstream out1244("ADASdata.txt");
ofstream outceshi("fff.txt");
CADASISDataReceiver::CADASISDataReceiver(void)
{
}

CADASISDataReceiver::~CADASISDataReceiver(void)
{
}

void CADASISDataReceiver::OpenADASISCAN()
{
	InitCAN(0);
	m_hThread = CreateThread(NULL, 0, OnReceive, this, 0, NULL);
}

void CADASISDataReceiver::InitCAN(int channel)
{
	canInitializeLibrary();

	int stat;

	m_hCan = canOpenChannel(channel, canOPEN_ACCEPT_VIRTUAL);
	stat = canSetBusParams(m_hCan, BAUD_500K,0,0,0,0,0);
	stat = canBusOn(m_hCan);
}

DWORD WINAPI CADASISDataReceiver::OnReceive(LPVOID lparam)
{
	CADASISDataReceiver* pDataRecv = (CADASISDataReceiver*) lparam;

	long id = 0;
	int i=0;
	LEAD p1;

	pDataRecv->storage = cvCreateMemStorage(0);
	pDataRecv->ADAS_points = cvCreateSeq( 0, sizeof(CvSeq), sizeof(LEAD), pDataRecv->storage);

	LEAD lead_temp;
	//CvPoint2D64f lead_temp;

	BYTE buf[CAN_DATA_SIZE] = {0};

	while (true)
	{
		if (WAIT_OBJECT_0 == WaitForSingleObject(pDataRecv->m_hEvent,0))
		{
			break;
		}

		canStatus iRet = canReadSync(pDataRecv->m_hCan,CAN_READ_WAIT);

		if (iRet == canOK)
		{
			iRet = canRead(pDataRecv->m_hCan,&id, buf, NULL, NULL, NULL);
		}

		if (iRet == canOK)
		{
			//if (id >= 0X210 && id <= 0X260)
			//{
			BYTE  type = (buf[0] & 0xE0) >> 5;
			//Position message
			if (type == 1)
			{
				Position pos;
				pos.pathindex = buf[2] & 0x3F;
				pos.offset = ((buf[0] & 0x1F) << 8) + buf[1];
				pDataRecv->m_curPosition = pos;
			}
			//Segment message
			else if (type == 2)
			{
				//pDataRecv->m_segid++;
				Segment segment;
				//segment.id = pDataRecv->m_segid;
				segment.offset = ((buf[0] & 0x1F) << 8) + buf[1];
				segment.pathindex = buf[2] & 0x3F;
				segment.lanenum = buf[7] & 0x07;
				pDataRecv->m_segQueue.Push(segment);
			}
			//Stub message
			else if (type == 3)
			{
				//pDataRecv->m_stubid++;
				Stub stub;
				//stub.id = pDataRecv->m_stubid;
				stub.offset = ((buf[0] & 0x1F) << 8) + buf[1];
				stub.pathindex = buf[2] & 0x3F;
				pDataRecv->m_stubQueue.Push(stub);
			}
			//Profile Short message
			else if (type == 4)
			{
				//
			}
			//Profile Long message
			else if (type == 5)
			{
				if ((buf[3] & 0xF8) >> 3 == 1)
				{
					/*Longitute lng;
					lng.offset = ((buf[0] & 0x1F) << 8) + buf[1];
					lng.pathindex = buf[2] & 0x3F;*/
					int val = (buf[4] << 24) + (buf[5] << 16) + (buf[6] << 8) + buf[7];
					//lng.val = 
					pDataRecv->m_pt.lng = val*0.0000001;
					
				}
				else if ((buf[3] & 0xF8) >> 3 == 2)
				{
					//Latitude lat;
					int offset = ((buf[0] & 0x1F) << 8) + buf[1];
					//lat.pathindex = buf[2] & 0x3F;
					int val = (buf[4] << 24) + (buf[5] << 16) + (buf[6] << 8) + buf[7];
					pDataRecv->m_pt.lat = val*0.0000001;
					pDataRecv->m_pt.height = 0.0;
					NODE<Stub> *p = pDataRecv->m_stubQueue.front;  
					BOOL flag = FALSE;
					while(p != pDataRecv->m_stubQueue.rear)
					{
						if (p->data.offset == offset)
						{
							flag = TRUE;
						}
						else
						{
							p = p->next;
						}
						if (flag)
						{
							break;
						}
					}
					if (flag)
					{
						pDataRecv->m_pt.type = 3;
					}
					else
					{
						pDataRecv->m_pt.type = 2;
					}

					i++;
					pDataRecv->m_rnPts.Push(pDataRecv->m_pt);

					lead_temp.id = i;
					lead_temp.height = pDataRecv->m_pt.height;
					lead_temp.lat = pDataRecv->m_pt.lat;
					lead_temp.lng = pDataRecv->m_pt.lng;
					lead_temp.param1 = pDataRecv->m_pt.type;

					////lead_temp.x = pDataRecv->m_pt.lat;
					////lead_temp.y = pDataRecv->m_pt.lng;

					cvSeqPush(pDataRecv->ADAS_points,&lead_temp);
					
					p1 = *(LEAD*)cvGetSeqElem(pDataRecv->ADAS_points,i-1);
					outceshi<<p1.id<<"	"<<setprecision(9)<<p1.lng<<"	"<<setprecision(9)<<p1.lat<<"	"<<p1.param1<<endl;
					out1244<<i<<"	"<<setprecision(9)<<pDataRecv->m_pt.lng<<"	"<<setprecision(9)<<pDataRecv->m_pt.lat<<endl;
				}

			}
			//Meta Data message
			else if (type == 6)
			{
				//
			}
			//}
		}
	}

	canClose(pDataRecv->m_hCan);

	return 0;
}

