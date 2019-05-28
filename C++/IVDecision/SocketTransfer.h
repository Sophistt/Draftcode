#include"winsock2.h"
#include "IVDecision.h"
#pragma comment(lib, "wsock32.lib")
#ifndef SOCKET_TRANSFER
#define SOCKET_TRANSFER
class CSocketTransfer
{
public:

	//bool MapData[512][512];
	 CSocketTransfer(void);
public:
	~CSocketTransfer(void);

public:
	SOCKET TestClient;   
	WSADATA TNetData;
//	sockaddr_in local;
    bool SuccSocket;
	bool ClientConnect(CString strIpAddress,int port);
	SOCKET server;//一个结构体，用于调用函数WSAStartup时作为参数
	WSADATA wsaData;
    sockaddr_in local;//存放ip地址的结构体
    CString strIpAddress;
	HANDLE m_VelListenThread;
	HANDLE m_SightListenThread;
	HANDLE m_VelServer;
	HANDLE m_SightServer;
	DWORD VelListenId;
    DWORD SightListenId;
	DWORD VelServerId;
	DWORD SightServerId;
	static DWORD VelListenThread(LPVOID lpParam);
	static DWORD SightListenThread(LPVOID lpParam);
	static UINT VelServerThread(LPVOID lpParam);
	static UINT SightServerThread(LPVOID lpParam);
	void ConnectInit(SOCKET &server,CString strIP);
	DWORD VelListen();
	DWORD SightListen();
	void OpenVelListen();
	void OpenSightListen();
	bool ClientSend(const char *data);
	int GetMiddleString(CString &string,const char * firststr, const char * endstr,CString &conResult );

	void OpenSignListen();
	static DWORD SignListenThread(LPVOID lpParam);
	static UINT SignServerThread(LPVOID lpParam);
	HANDLE m_SignListenThread;
	DWORD SignListenId;
	DWORD SignListen();

	void OpenSignSideListen();
	static DWORD SignSideListenThread(LPVOID lpParam);
	static UINT SignSideServerThread(LPVOID lpParam);
	HANDLE m_SignSideListenThread;
	DWORD SignSideListenId;
	DWORD SignSideListen();
private:
	bool VelListenned;
	bool SightListenned;
	bool SignListenned;
	bool SignSideListenned;
	//DWORD SignListen();
};

#endif
