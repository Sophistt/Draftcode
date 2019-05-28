#pragma once
#include "afxwin.h"
#include "IVDecision.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef WIN32
#include <winsock2.h>
#else
#include <sys/select.h>
#endif
#include <fcntl.h>
#include <lcm/lcm.h>
class ComuLcm
{
public:

	ComuLcm(void);
	//LCM
	void send_message(CString buf);
	void send_messagegps(CString buf);
	void send_messagerndf(CString buf);
	//void send_message(char *buf);
	void recv_message();
	void recv_messagesignal();
	void Set_RecAddress(CString address);
	void Set_RecChannel(CString channel);
	void Set_SendAddress(CString address);
	void Set_SendChannel(CString channel);
	//static void test_handler ( const lcm_recv_buf_t *rbuf, const char *channel, void *u);
	//�߳�
	int GetMiddleString(CString &string,const char * firststr, const char * endstr,CString &conResult );
	static DWORD theDataProcess(LPVOID lpParam);
	DWORD StubDataProcess();
	DWORD  dwDataThreadId;  // ���ݴ����߳�Id
	HANDLE  m_hThreadData; // ���ݴ����߳̾��

	void recv_messagesign();

	static DWORD theDataProcess1(LPVOID lpParam);
	DWORD StubDataProcess1();
	DWORD  dwDataThreadId1;  // ���ݴ����߳�Id
	HANDLE  m_hThreadData1; // ���ݴ����߳̾��

	static DWORD theDataProcess2(LPVOID lpParam);
	DWORD StubDataProcess2();
	DWORD  dwDataThreadId2;  // ���ݴ����߳�Id
	HANDLE  m_hThreadData2; // ���ݴ����߳̾��
	//HANDLE m_MapEvent;
private:
		lcm_t * lcm_send;
		lcm_t * lcm_send1;
		lcm_t * lcm;
		lcm_t * lcm3;
		lcm_t * lcm4;
		I_Map * mapp;
		
public:
	~ComuLcm(void);
	 CString lcm_address;
	 CString lcm_channel;
	
};
