
#include "StdAfx.h"
#include "ComuLcm.h"
#include<fstream>
using namespace std;
ofstream out44("recv1.txt");
ofstream stopline("Stoplinedis.txt");
ofstream limitsign("Limitsign.txt");
ofstream perceptime("intertime.txt");

ofstream outpercepmaptime("outpercepmaptime.txt");

I_Map * mapp=new I_Map;
I_Map * mapp1=new I_Map;
I_Map * mapp2=new I_Map;
I_Map * mapp3=new I_Map;
I_Map * mapp4=new I_Map;
I_Map * mapp5=new I_Map;
ComuLcm::ComuLcm(void)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	/*lcm_send = lcm_create(app->SendtoControl.net_address);*/   
	app->m_MapEvent=CreateEvent(NULL,FALSE,FALSE,NULL);
		lcm_send = lcm_create("udpm://239.255.76.70:7670?ttl=1");      	
	if(!lcm_send)       	
	AfxMessageBox("mistake");
	lcm_send1= lcm_create("udpm://239.255.76.75:7675?ttl=1"); 
	lcm = lcm_create("udpm://239.255.76.67:7667?ttl=1");	
		if(!lcm)
		AfxMessageBox("mistake");
	lcm3 = lcm_create("udpm://239.255.76.71:7671?ttl=1");	
		if(!lcm3)
			AfxMessageBox("mistake");
	lcm4 = lcm_create("udpm://239.255.76.72:7672?ttl=1");	
	if(!lcm4)
		AfxMessageBox("mistake");
}

ComuLcm::~ComuLcm(void)
{
	lcm_destroy(lcm);
	lcm_destroy(lcm_send1);
	lcm_destroy(lcm_send);
	lcm_destroy(lcm3);
	lcm_destroy(lcm4);
}


void ComuLcm::send_message(CString buf)
{
    CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
/*	lcm_send = lcm_create("udpm://239.255.76.70:7670?ttl=1");      	
	if(!lcm_send)       	
	AfxMessageBox("mistake");*/ 	
	
	int datalen = strlen(buf)+1;	
	lcm_publish(lcm_send,"roadstate",buf,datalen);
}
void ComuLcm::send_messagegps(CString buf)
{
    CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	
	int datalen = strlen(buf)+1;	
	lcm_publish(lcm_send,"GPS",buf,datalen);
}
void ComuLcm::send_messagerndf(CString buf)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();


	int datalen = strlen(buf)+1;	
	lcm_publish(lcm_send1,"RNDF",buf,datalen);
}
void ComuLcm::recv_message()
{
	// TODO: 在此添加控件通知处理程序代码
	m_hThreadData = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)ComuLcm::theDataProcess,
		 this, 0, &dwDataThreadId);
	if (NULL == m_hThreadData)
	{
		AfxMessageBox("程序启动内部数据处理线程失败!!!", MB_ICONSTOP);

		// 关闭串口文件
		return ;
	}

}
 void test_handler( const lcm_recv_buf_t *rbuf, const char *channel, void *u)
{
    CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
    const char *a;
	CString str,str1,str2;
	str1.Format(_T("%s"), LPCTSTR(channel)); 
	str2.Format(_T("%s"),LPCTSTR((char*)rbuf->data));
	a = LPCTSTR((char*)rbuf->data);
	//
	for(int k=0;k<512;k++)
		for(int m=0;m<512;m++)
		{
			//mapp->MapPoint[k][m]=(unsigned int)a[k*512+m];
			mapp->MapPoint[k][m]=(unsigned int)*(unsigned char *)((unsigned char *)a+k*512+m);
		}

	if(mapp->MapPoint[0][511]!=0)
	{
		app->stopline_dis = ((double)mapp->MapPoint[0][511])*0.05;//0.2
		app->stop_map = true;
		stopline<<"Detect Stopline :"<<app->stopline_dis<<endl;
		mapp->MapPoint[0][511]=0;
	}
	else
		app->stop_map = false;

	
SYSTEMTIME t1;
		GetLocalTime(&t1);
outpercepmaptime<<"time1 "<<t1.wHour<<":"<<t1.wMinute<<":"<<t1.wSecond<<","<<t1.wMilliseconds<<"  "<<endl;

	DWORD cur_time=GetTickCount();
	if(app->perceptimelast)
	{
		perceptime<<cur_time-app->perceptimelast<<endl;
	}
	app->perceptimelast = cur_time;

	app->critical_map.Lock();
	app->PercepMap = mapp;
	app->critical_map.Unlock();
	
	app->mapreceived = true;

}
DWORD ComuLcm::theDataProcess(LPVOID lpParam)
{
	return ((( ComuLcm*)lpParam)->StubDataProcess());
}
DWORD ComuLcm::StubDataProcess()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	/*lcm_t * lcm;*/
	/*lcm = lcm_create(app->GlobalRecNetSet.net_address);	
	if(!lcm)
	  AfxMessageBox("mistake");*/
	lcm_subscription_t * sub =
		lcm_subscribe(lcm,"PERCEPTION",test_handler, 0);
	int fd = lcm_get_fileno (lcm);
	while(1)
	{
		struct timeval timeout = { 1, 0 };
		fd_set readfds;
		FD_ZERO (&readfds);
		FD_SET (fd,&readfds);
		int status=select (fd + 1,&readfds,0,0,&timeout);
		if (FD_ISSET (fd,&readfds))
			{		
				lcm_handle(lcm);
				SetEvent(app->m_MapEvent);
			}
		Sleep(10);
	}		
	lcm_unsubscribe(lcm, sub);
	lcm_destroy(lcm);
}
void ComuLcm::Set_RecAddress(CString address)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->GlobalRecNetSet.net_address = address;
	
}
void  ComuLcm::Set_RecChannel(CString channel)
{	
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->GlobalRecNetSet.net_channel = channel;
}
void ComuLcm::Set_SendAddress(CString address)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->GlobalSendNetSet.net_address = address;
	
}
void  ComuLcm::Set_SendChannel(CString channel)
{	
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->GlobalSendNetSet.net_channel = channel;
}

//void ComuLcm::recv_message1()
//{
//	// TODO: 在此添加控件通知处理程序代码
//	m_hThreadData1 = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)ComuLcm::theDataProcess1,
//		 this, 0, &dwDataThreadId1);
//	if (NULL == m_hThreadData)
//	{
//		AfxMessageBox("程序启动内部数据处理线程失败!!!", MB_ICONSTOP);
//
//		// 关闭串口文件
//		return ;
//	}
//
//}
// void test_handler1( const lcm_recv_buf_t *rbuf, const char *channel, void *u)
//{
//    CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
//	ComuLcm aaa;
//    const char *a;
//	CString str,str1,str2,m_GpsCst,m_Gps;
//	str1.Format(_T("%s"), LPCTSTR(channel)); 
//	str2.Format(_T("%s"),LPCTSTR((char*)rbuf->data));
//	a = LPCTSTR((char*)rbuf->data);
//	int m_num = 0;
//	m_GpsCst.Format(_T("%s"),LPCTSTR((char*)rbuf->data));
//	
//	aaa.GetMiddleString(m_GpsCst,"%GGPS",";",m_Gps);
//	out2<<m_Gps<<endl;
//	string str_path;
//	str_path = (LPCTSTR)m_Gps;
//		int istart =0; 
//		int iend =0; 
//		int num_path =0;
//		int istartpos = 0;
//		int iendpos = 0;
//		
//		
//		{
//			istart = iend;
//			iend = str_path.find(";",istart);
//			
//			string point = str_path.substr(istart,iend);
//			//str_path = str_path.substr(point.length,str_path.length-point.length);
//			iend++;
//			istartpos = 0;
//			iendpos = point.find(",",istartpos);
//			string pointx = point.substr(istartpos,iendpos-istartpos);
//			double x = atof(pointx.c_str());
//			istartpos=iendpos;
//			iendpos=point.find(",",istartpos+1);
//			string pointy = point.substr(istartpos+1,iendpos-istartpos);
//			double y = atof(pointy.c_str());
//			istartpos=iendpos;
//			iendpos=point.length();
//			string pointz = point.substr(istartpos+1,iendpos-istartpos);
//			double z = atof(pointz.c_str());
//				app->m_GpsPoint.x = x;
//				app->m_GpsPoint.y = y;
//				app->m_Direction = z;
//				
//		
//		}
//		
//	app->critical_map.Lock();
//	//app->PercepMap = *mapp;
//	//app->PercepMap.push(*mapp);
//	app->PercepMap = mapp;
//	app->critical_map.Unlock();
//	
//	/*SYSTEMTIME t1;
//		GetLocalTime(&t1);
//		out2<<t1.wHour<<":"<<t1.wMinute<<":"<<t1.wSecond<<":"<<t1.wMilliseconds<<endl;
//		out2<<app->m_num<<endl ;
//		app->m_num++;*/
//	//for(int i = 0;i<512;i++)
//	//{
//	//	for(int j = 0;j<512;j++)
//	//		//if(app->PercepMap->MapPoint[i][j] == 0)
//	//			out333<<app->PercepMap->MapPoint[i][j];
//	//		//else out333<<1;
//	//	out333<<endl;
//	//}
//	app->mapreceived = true;
//}
//DWORD ComuLcm::theDataProcess1(LPVOID lpParam)
//{
//	return ((( ComuLcm*)lpParam)->StubDataProcess1());
//}
//DWORD ComuLcm::StubDataProcess1()
//{
//	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
//	lcm_t * lcm1;
//	lcm1 = lcm_create(app->RecGps.net_address);	
//	if(!lcm1)
//	  AfxMessageBox("mistake");
//	lcm_subscription_t * sub =
//		lcm_subscribe(lcm1, app->RecGps.net_channel,test_handler1, 0);
//	int fd = lcm_get_fileno (lcm1);
//	while(1)
//	{
//		struct timeval timeout = { 1, 0 };
//		fd_set readfds;
//		FD_ZERO (&readfds);
//		FD_SET (fd,&readfds);
//		int status=select (fd + 1,&readfds,0,0,&timeout);
//		if (FD_ISSET (fd,&readfds))
//			{		
//				lcm_handle(lcm1);		
//			}
//			
//	}		
//	lcm_unsubscribe(lcm1, sub);
//	lcm_destroy(lcm1);
//}
int ComuLcm::GetMiddleString(CString &string,const char * firststr, const char * endstr,CString &conResult )
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

void ComuLcm::recv_messagesignal()
{
	// TODO: 在此添加控件通知处理程序代码
	m_hThreadData1 = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)ComuLcm::theDataProcess1,
		 this, 0, &dwDataThreadId1);
	if (NULL == m_hThreadData1)
	{
		AfxMessageBox("程序启动内部数据处理线程失败!!!", MB_ICONSTOP);

		// 关闭串口文件
		return ;
	}

}
 void test_handler1( const lcm_recv_buf_t *rbuf, const char *channel, void *u)
{
    CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	ComuLcm signlcm;
   int sign;	
	char temp[40];
	int count = 10;
	CString str_sign;
	CString str_rec;
	const char *a = LPCTSTR((char*)rbuf->data);
	str_rec = a;
	/*memset( temp, 0, 40 );
	str_rec.Format("%s",temp);*/
	signlcm.GetMiddleString(str_rec,"%LIGHTMID","END",str_sign);
	sign = atoi(str_sign);
	app->critical_light.Lock();
	if(/*app->Get_Sign &&*/ 1)
	{

		if(sign!=0)
		{
			app->Traffic_Sign.push(sign);
			count--;
		}
	}
	app->critical_light.Unlock();
	//if(count == 0)
	//{
	//	//app->Get_Sign = false;
	//	count = 10;

	//}
	
}
DWORD ComuLcm::theDataProcess1(LPVOID lpParam)
{
	return ((( ComuLcm*)lpParam)->StubDataProcess1());
}
DWORD ComuLcm::StubDataProcess1()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	/*lcm_t * lcm3;
	lcm3 = lcm_create("udpm://239.255.76.71:7671?ttl=1");	
	if(!lcm3)
	  AfxMessageBox("mistake");*/
	lcm_subscription_t * sub =
		lcm_subscribe(lcm3, "SIGNAL",test_handler1, 0);
	int fd = lcm_get_fileno (lcm3);
	while(1)
	{
		struct timeval timeout = { 1, 0 };
		fd_set readfds;
		FD_ZERO (&readfds);
		FD_SET (fd,&readfds);
		int status=select (fd + 1,&readfds,0,0,&timeout);
		if (FD_ISSET (fd,&readfds))
			{		
				lcm_handle(lcm3);
			}
		Sleep(10);
	}		
	lcm_unsubscribe(lcm3, sub);
	lcm_destroy(lcm3);
}


void ComuLcm::recv_messagesign()
{
	// TODO: 在此添加控件通知处理程序代码
	m_hThreadData2 = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)ComuLcm::theDataProcess2,
		 this, 0, &dwDataThreadId2);
	if (NULL == m_hThreadData1)
	{
		AfxMessageBox("程序启动内部数据处理线程失败!!!", MB_ICONSTOP);

		// 关闭串口文件
		return ;
	}

}
 void test_handler2( const lcm_recv_buf_t *rbuf, const char *channel, void *u)
{
    CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	ComuLcm signlcm;
   int sign;	
	char temp[40];
	int count = 10;
	CString str_sign;
	CString str_rec;
	const char *a = LPCTSTR((char*)rbuf->data);
	str_rec = a;
	/*memset( temp, 0, 40 );
	str_rec.Format("%s",temp);*/
	signlcm.GetMiddleString(str_rec,"%MARK","END",str_sign);
	sign = atoi(str_sign);
	sign = sign%10;
	app->critical_light.Lock();
	if(/*app->Get_Sign &&*/ 1)
	{

		if(sign!=0)
		{
			app->limit_speed = 40/3.6;
			app->limitflag = true;
			limitsign<<"receive limit speed!"<<endl;
			//app->Traffic_Sign.push(sign);
			//count--;
		}
		else
		{
			app->limitflag = false;
			limitsign<<"There is no limited sign!"<<endl;
		}
			//app->limit_speed = 60/3.6;
	}
	app->critical_light.Unlock();
	//if(count == 0)
	//{
	//	//app->Get_Sign = false;
	//	count = 10;

	//}
	
}
DWORD ComuLcm::theDataProcess2(LPVOID lpParam)
{
	return ((( ComuLcm*)lpParam)->StubDataProcess2());
}
DWORD ComuLcm::StubDataProcess2()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	/*lcm_t * lcm3;
	lcm3 = lcm_create("udpm://239.255.76.71:7671?ttl=1");	
	if(!lcm3)
	  AfxMessageBox("mistake");*/
	lcm_subscription_t * sub =
		lcm_subscribe(lcm4, "SIGN",test_handler2, 0);
	int fd = lcm_get_fileno (lcm4);
	while(1)
	{
		struct timeval timeout = { 1, 0 };
		fd_set readfds;
		FD_ZERO (&readfds);
		FD_SET (fd,&readfds);
		int status=select (fd + 1,&readfds,0,0,&timeout);
		if (FD_ISSET (fd,&readfds))
			{		
				lcm_handle(lcm4);
			}
		Sleep(10);
	}		
	lcm_unsubscribe(lcm4, sub);
	lcm_destroy(lcm4);
}
