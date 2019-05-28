
#include "StdAfx.h"
#include "SocketTransfer.h"
#include<fstream>
using namespace std;
	 ofstream out("map.txt");
     ofstream out1("sightmap.txt");
CSocketTransfer::CSocketTransfer(void)
{
	VelListenned=false;
    SightListenned=false;
	SuccSocket=false;
	SignListenned = false;
}

CSocketTransfer::~CSocketTransfer(void)
{
}


//void CSocketTransfer::OpenVelListen()
//{
//	if(!VelListenned)
//	{
//		
//		m_VelListenThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)CSocketTransfer::VelListenThread,
//		this, 0, &VelListenId);
//
//		VelListenned=true;
//	}
//	else
//	{
//		AfxMessageBox("Vel已监听!");
//	}
//}
//void CSocketTransfer::OpenSightListen()
//{
//	if(!SightListenned)
//	{
//		
//		m_SightListenThread=CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)CSocketTransfer::SightListenThread,
//		this, 0, &SightListenId);
//		SightListenned=true;
//	}
//	else
//	{
//		AfxMessageBox("视觉地图已监听!");
//	}
//}
//DWORD CSocketTransfer::VelListenThread(LPVOID lpParam)
//  {
//	  return (((CSocketTransfer*)lpParam)->VelListen());
//  }
//DWORD CSocketTransfer::SightListenThread(LPVOID lpParam)
//  {
//	  return (((CSocketTransfer*)lpParam)->SightListen());
//  }
//DWORD CSocketTransfer::VelListen()
//{
//	AfxMessageBox("正在监听");
//	WSADATA wsd;
//	SOCKET sListen,sClient;
//	int iAddrSize;
//	struct sockaddr_in local,client;
//	if(WSAStartup(MAKEWORD(2,2),&wsd)!=0)
//	{
//		AfxMessageBox("Fail to load Winsock");
//		return 1;
//	}
//	//int sockRcvBufSize = 100;
//	//setsockopt(server, SOL_SOCKET, SO_RCVBUF, (char *)&sockRcvBufSize, sizeof(sockRcvBufSize));
//	local.sin_family=AF_INET;
//	local.sin_port=htons(1088);
//	local.sin_addr.s_addr=htonl(INADDR_ANY);
//	sListen=socket(AF_INET,SOCK_STREAM,IPPROTO_IP);
//	if((bind(sListen,(SOCKADDR*)&local,sizeof(local)))==SOCKET_ERROR)
//	{
//		AfxMessageBox("绑定失败!");
//	}
//	listen(sListen,10);
//	while(1)
//	{
//		iAddrSize=sizeof(client);
//		sClient=accept(sListen,(struct sockaddr*)&client,&iAddrSize);
//		if (sClient==INVALID_SOCKET)
//		{
//			break;
//		}
//		else
//		{
//			CWinThread* hCLientThread=AfxBeginThread(VelServerThread,(LPVOID)sClient,THREAD_PRIORITY_NORMAL);
//		
//		}
//		Sleep(20);
//	}
//	return 0;
//}
//
//
//UINT CSocketTransfer::VelServerThread(LPVOID lpParam)
//  {
//	  
////	  return (((CSocketTransfer*)lpParam)->VelServer());
//		SOCKET sock=(SOCKET) lpParam;
//		CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
//		I_Map * mapp=new I_Map;	
//		char temp[512*512];
//		//DWORD t4,t5,t6;
//		memset( temp, 0, 512*512 );
//			app->sick_num = 0;	
//		while(1)
//		{
//		//memset( temp, 0, 32771 );
//	//int iLen = recv( sock, temp, 32771, 0 );
//			int iLen = recv( sock, temp, 512*512, 0 );
//			if( iLen ==512*512 )
//			{
//				//t4 = GetTickCount();
//				//int ff=0;
//				//app->sick_num = (int)temp[0];
//				for(int k=0;k<512;k++)
//				for(int m=0;m<512;m++)
//				mapp->MapPoint[k][m]=(int)temp[k*512+m];
//				app->critical_map.Lock();
//				app->road_Map = *mapp;
//				app->critical_map.Unlock();
//				/*app->RT_Map.push(*mapp);*/
//				
//
//				//out<<app->sick_num<<endl;
//				//t5 = GetTickCount();
//				//t6= t5 - t4;
//				//out<<"亚队列时间   "<<t6<<endl;
//				//
//				//out<<"收到";
//				//SYSTEMTIME t1;
//				//GetLocalTime(&t1);
//				//out<<t1.wHour<<":"<<t1.wMinute<<":"<<t1.wSecond<<","<<t1.wMilliseconds<<endl;
//				//for(int i=0;i<512;i++)
//				//	{
//				//		
//				//		for(int j=0;j<512;j++)
//				//		{
//	
//				//			  out<<mapp->MapPoint[i][j];
//		
//				//		}                   
//				//			  out<<endl;
//				//	}	
//
//			}
//		}
//		return 0;
// }
//
//
//DWORD CSocketTransfer::SightListen()
//{
//	AfxMessageBox("正在监听");
//	WSADATA wsd;
//	SOCKET sListen,sClient;
//	int iAddrSize;
//	struct sockaddr_in local,client;
//	if(WSAStartup(MAKEWORD(2,2),&wsd)!=0)
//	{
//		AfxMessageBox("Fail to load Winsock");
//		return 1;
//	}
//	local.sin_family=AF_INET;
//	local.sin_port=htons(1234);
//	local.sin_addr.s_addr=htonl(INADDR_ANY);
//	sListen=socket(AF_INET,SOCK_STREAM,IPPROTO_IP);
//	if((bind(sListen,(SOCKADDR*)&local,sizeof(local)))==SOCKET_ERROR)
//	{
//		AfxMessageBox("绑定失败!");
//	}
//	listen(sListen,10);
//	while(1)
//	{
//		iAddrSize=sizeof(client);
//		sClient=accept(sListen,(struct sockaddr*)&client,&iAddrSize);
//		if (sClient==INVALID_SOCKET)
//		{
//			break;
//		}
//		else
//		{
////			m_SightServer=CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)CSocketTransfer::SightServerThread,
////		this, 0, &SightServerId);
//			CWinThread* hCLientThread=AfxBeginThread(SightServerThread,(LPVOID)sClient,THREAD_PRIORITY_NORMAL);
//		}
//		Sleep(20);
//	}
//	return 0;
//}
//UINT CSocketTransfer::SightServerThread(LPVOID lpParam)
//  {
////	  return (((CSocketTransfer*)lpParam)->SightServer());
//	  	SOCKET sock=(SOCKET) lpParam;
//		CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
//		I_Map * mapp=new I_Map;	
//		char temp[512*512];
//		while(1)
//		{
//		//memset( temp, 0, 32771 );
//			memset( temp, 0, 512*512 );
//			//int iLen = recv( sock, temp, 32771, 0 );
//			int iLen = recv( sock, temp, 512*512, 0 );
//			if( iLen ==512*512 )
//			{
//				
//				int ff=0;
//				for(int k=0;k<512;k++)
//				for(int m=0;m<512;m++)
//				mapp->MapPoint[k][m]=(int)temp[k*512+m]-48;
//				app->Way_Map.push(*mapp);
//				out1<<"shou dao"<<endl;
//				I_Map *niuda=new I_Map;						
//			   niuda=&app->Way_Map.front(); 
//				for(int i=0;i<512;i++)
//				//	{
//						
//						for(int j=0;j<512;j++)
//				//		{
//							out1<<niuda->MapPoint[i][j];
//				//			  out1<<'1';
//				//			else
//				//			  out1<<'0';
//				//		}                   
//						  out1<<endl;
//				//	}	
//				}				
//		}
//		return 0;
//  }
//bool CSocketTransfer::ClientSend(const char *data)
//{
//	
//	//CString strIpAddress="127.0.0.1";
//	
//		
//				if(SuccSocket)
//				{
//					  int ret=send(TestClient,data,40,0);
//					  if(ret==SOCKET_ERROR)
//					 {
//						SuccSocket=false;
//						return 0;
//						AfxMessageBox("SOCKET_ERROR_Send");
//					 }
//				}
//		
//}
//bool CSocketTransfer::ClientConnect(CString strIpAddress,int port)
//{
//	//CString strIpAddress="192.168.0.109";
//	int wsaret = WSAStartup( MAKEWORD(2,2), &TNetData );
//		if( wsaret != 0 )
//		{
//			return 0;
//		}
//
//		//填充服务器的ip地址和端口号
//		local.sin_family = AF_INET; //Address family
//		local.sin_addr.s_addr = inet_addr( strIpAddress ); //IP address
//		local.sin_port = htons((u_short)port); //port to use
//
//		//建立一个tcp socket
//		TestClient = socket( AF_INET, SOCK_STREAM, 0 );
//		if( TestClient == INVALID_SOCKET )
//		{
//			return 0;
//		}
//
//		//接受请求后，实际同客户端socket进行交互的SOCKET client
//		
//		{
//			if(connect( TestClient,(struct sockaddr *)&local, sizeof(local))==SOCKET_ERROR)
//		{
//			SuccSocket=false;
//			AfxMessageBox("SOCKET_ERROR_Connect");
//		}
//			else
//		{
//			SuccSocket=true;
//		} 
//
//
//		}
//}





void CSocketTransfer::OpenSignListen()
{
	if(!SignListenned)
	{
		
		m_SignListenThread=CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)CSocketTransfer::SignListenThread,
		this, 0, &SignListenId);
		SignListenned=true;
	}
	else
	{
		AfxMessageBox("视觉地图已监听!");
	}
}
DWORD CSocketTransfer::SignListenThread(LPVOID lpParam)
{
	return (((CSocketTransfer*)lpParam)->SignListen());
}
DWORD CSocketTransfer::SignListen()
{
	//AfxMessageBox("正在监听");
	WSADATA wsd;
	SOCKET sListen,sClient;
	int iAddrSize;
	struct sockaddr_in local,client;
	if(WSAStartup(MAKEWORD(2,2),&wsd)!=0)
	{
		AfxMessageBox("Fail to load Winsock");
		return 1;
	}
	local.sin_family=AF_INET;
	local.sin_port=htons(1234);
	local.sin_addr.s_addr=htonl(INADDR_ANY);
	sListen=socket(AF_INET,SOCK_STREAM,IPPROTO_IP);
	if((bind(sListen,(SOCKADDR*)&local,sizeof(local)))==SOCKET_ERROR)
	{
		AfxMessageBox("绑定失败!");
	}
	listen(sListen,10);
	while(1)
	{
		iAddrSize=sizeof(client);
		sClient=accept(sListen,(struct sockaddr*)&client,&iAddrSize);
		if (sClient==INVALID_SOCKET)
		{
			break;
		}
		else
		{
//			m_SightServer=CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)CSocketTransfer::SightServerThread,
//		this, 0, &SightServerId);
			CWinThread* hCLientThread=AfxBeginThread(SignServerThread,(LPVOID)sClient,THREAD_PRIORITY_NORMAL);
		}
		Sleep(20);
	}
	return 0;
}
UINT CSocketTransfer::SignServerThread(LPVOID lpParam)
  {
//	  return (((CSocketTransfer*)lpParam)->SightServer());
	  	SOCKET sock=(SOCKET) lpParam;
		CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
		int sign;	
		char temp[40];
		int count = 10;
		CString str_sign;
		CString str_rec;
		CSocketTransfer transfer;
		while(1)
		{
		//memset( temp, 0, 32771 );
			memset( temp, 0, 40 );
			//int iLen = recv( sock, temp, 32771, 0 );
			int iLen = recv( sock, temp, 40, 0 );
			//out1<<temp<<endl;
			if( iLen == 40 )
			{
				str_rec.Format("%s",temp);
				
				transfer.GetMiddleString(str_rec,"%LIGHTMID","END",str_sign);
				out<<"bizozhi "<<str_sign<<endl;
				sign = atoi(str_sign);
				
			}
			app->critical_light.Lock();
			if(app->Get_Sign && count>0)
			{

				if(sign!=0)
				{
					app->Traffic_Sign.push(sign);
					count--;
				}
			}
			app->critical_light.Unlock();
			if(count == 0)
			{
				app->Get_Sign = false;
				count = 10;
	
			}
		}
		return 0;
  }
int CSocketTransfer::GetMiddleString(CString &string,const char * firststr, const char * endstr,CString &conResult )
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	const char *pBegin=NULL;
	const char *pEnd=NULL;
	char * temp;
	const char* str = (LPCTSTR)string;
    if((pEnd=strstr(str, endstr))==NULL)
		return 0;
	if((pBegin=strstr(str,firststr))==NULL)
		return 0;
	
	if(pEnd - pBegin > 0 )
	{
		temp = (char*)malloc(pEnd- pBegin +1);   
		if(NULL == temp)
		{
			AfxMessageBox("内存分配失败!");
			return 0;
		}
		memset(temp, 0, pEnd- pBegin +1);  
		int a =strlen(firststr);
		memcpy(temp, pBegin + a, pEnd- pBegin - a); 
		
	}
    conResult.Format("%s", temp);
	free(temp);

	return 1;
}

void CSocketTransfer::OpenSignSideListen()
{
	if(!SignListenned)
	{
		
		m_SignListenThread=CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)CSocketTransfer::SignListenThread,
		this, 0, &SignListenId);
		SignListenned=true;
	}
	else
	{
		//AfxMessageBox("视觉地图已监听!");
	}
}
DWORD CSocketTransfer::SignSideListenThread(LPVOID lpParam)
{
	return (((CSocketTransfer*)lpParam)->SignListen());
}
DWORD CSocketTransfer::SignSideListen()
{
	//AfxMessageBox("正在监听");
	WSADATA wsd;
	SOCKET sListen,sClient;
	int iAddrSize;
	struct sockaddr_in local,client;
	if(WSAStartup(MAKEWORD(2,2),&wsd)!=0)
	{
		AfxMessageBox("Fail to load Winsock");
		return 1;
	}
	local.sin_family=AF_INET;
	local.sin_port=htons(2345);
	local.sin_addr.s_addr=htonl(INADDR_ANY);
	sListen=socket(AF_INET,SOCK_STREAM,IPPROTO_IP);
	if((bind(sListen,(SOCKADDR*)&local,sizeof(local)))==SOCKET_ERROR)
	{
		AfxMessageBox("绑定失败!");
	}
	listen(sListen,10);
	while(1)
	{
		iAddrSize=sizeof(client);
		sClient=accept(sListen,(struct sockaddr*)&client,&iAddrSize);
		if (sClient==INVALID_SOCKET)
		{
			break;
		}
		else
		{
//			m_SightServer=CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)CSocketTransfer::SightServerThread,
//		this, 0, &SightServerId);
			CWinThread* hCLientThread=AfxBeginThread(SignServerThread,(LPVOID)sClient,THREAD_PRIORITY_NORMAL);
		}
		Sleep(20);
	}
	return 0;
}
UINT CSocketTransfer::SignSideServerThread(LPVOID lpParam)
  {
//	  return (((CSocketTransfer*)lpParam)->SightServer());
	  	SOCKET sock=(SOCKET) lpParam;
		CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
		int sign;	
		char temp[40];
		int count = 3;
		CString str_sign;
		CString str_rec;
		CSocketTransfer transfer;
		while(1)
		{
		//memset( temp, 0, 32771 );
			memset( temp, 0, 40 );
			//int iLen = recv( sock, temp, 32771, 0 );
			int iLen = recv( sock, temp, 40, 0 );
			//out1<<temp<<endl;
			if( iLen == 40 )
			{
				str_rec.Format("%s",temp);
				
				transfer.GetMiddleString(str_rec,"%LIGHTMID","END",str_sign);
				sign = atoi(str_sign);
				
			}
			if(app->Get_Sign && count>0)
			{

				if(sign!=0)
				{
					app->Traffic_Sign.push(sign);

					count--;
				}
			}
			if(count == 0)
			{
				app->Get_Sign = false;
				count = 3;
	
			}
		}
		return 0;
  }