#include "StdAfx.h"
#include "IVExecution.h"
#include<fstream>

using namespace std;
	 ofstream outa("jilu.txt");
	 ofstream outb("aaa.txt");
	 ofstream outc("fasong");
	 ofstream outok("ok.txt");

CIVExecution::CIVExecution(void)
{
	tentacles = new GTentacle;
	onoff_flag = true;
	keeplane_flag = false;
	on_uturn = false;
	on_cross = false;
	on_speedup = false;
	on_stopline = false;
	LineMap= new I_Map;
}

CIVExecution::~CIVExecution(void)
{
}
void CIVExecution::OnStart()
{

	m_hThreadData = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE) CIVExecution::theSearch,
		this, 0, &dwDataThreadId);
	//CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	// TODO: 在此添加命令处理程序代码
	//if(AimFlag)
		//InitialTentacleThread=AfxBeginThread( CIVExecution::SearchAtenna,GetSafeHwnd(),THREAD_PRIORITY_NORMAL ); 
	//else
	//{
	//	AfxMessageBox("目标点未载入!");
	//}
}
void CIVExecution::OnEnd()
{
     onoff_flag=false;
}


DWORD CIVExecution::theSearch(LPVOID lpParam)
{
	//return (((CIVExecution*)lpParam)->SearchAtenna_GPS());
	//return (((CIVExecution*)lpParam)->SearchAtenna_GPS_u());
	//return (((CIVExecution*)lpParam)->SearchAtenna_SMap());
	return (((CIVExecution*)lpParam)->SearchAtenna_VMap());
}
//CvPoint2D64f CIVExecution::MapPointConv(Map_Point a,CvPoint2D64f b)
//{
	//double xq1,yq1,x,y,sita,dire,xa,ya;
	//double s1;
	//xq1=a.x;
	//yq1=a.y;
	//xq1 = xq1-256;
	//yq1 = 256-yq1;
	//dire=b.Dir;
	//s1=sqrt(pow(xq1/5.0,2)+pow(yq1/5.0,2));
 //   sita=atan(xq1/yq1);
	//dire=sita+b.Dir*3.1415926/180;
	//xa=s1*sin(dire);
	//ya=s1*cos(dire);
	//x=b.x+(ya*180)/(6378137*3.1415926);
	//y=b.y+(xa*180)/(6378137*3.1415926*cos(x*3.1415926/180));
	//CvPoint2D64f c;
	//c.x=x;
	//c.y=y;
	//c.Dir=b.Dir;
	//return c;
//	return 1;
//}
//a是车辆位置，b是目标点
//void CIVExecution::CountJiaoDian(CvPoint2D64f a,CvPoint2D64f b)
//{
//	Map_Point Apoint;
//	Apoint=GpsData.APiontConver(a,b,rDirection);
//    double Jiajiao=0;
////	Jiajiao=a.Dir-b.Dir;
//    double l=0;
//	l=abs(Apoint.x-256)/tan(GpsData.rad(Jiajiao));
//	Map_Point Jiaodian;
//	Jiaodian.x=256;
//	Jiaodian.y=A_Point.y+l;
//	//if(Jiaodian.y<Apoint.y||Jiaodian.y>240)
//	{
//		Jiaodian.x=Apoint.x+35*sin(GpsData.rad(Jiajiao));
//		Jiaodian.y=Apoint.y+35*cos(GpsData.rad(Jiajiao));
//	}
//    CvPoint2D64f Jiao_dian;
//	Jiao_dian=MapPointConv(Jiaodian,rPosition);
	//return 1;

//}

/*

*/
void CIVExecution::SetTrafficSign( )
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	int sign[13] = {1,2,1,3,3,3,3,4,2,1,1,1,3};
	for(int i = 0; i<13; i++)
	app->Traffic_Sign.push(sign[i]);

}


void CIVExecution::UTurn(double lane_angle )
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();	

	outa<<"utrun开始"<<endl;
	rResult.TentacleID = 1;
	rResult.brake=-40000;
	app->RT_Result.push(rResult);
	Sleep(6000);
	

	double u_Direction;

	while(1)
	{
		rResult.TentacleID = 0;
		rResult.brake=-25500;
		app->RT_Result.push(rResult);
		
		u_Direction = GpsData.GetDirection();
		//u_Direction = 180;
		if(abs(u_Direction-lane_angle)<40||abs(abs(u_Direction-lane_angle)-360)<40)
		{
			outa<<"utrun结束"<<endl;
		
			on_uturn = false;
			break;
		}
		Sleep(100);
		outa<<"uturn: id  "<<rResult.TentacleID <<endl;
	}
	rResult.TentacleID = 40;
	rResult.brake=-40000;
	app->RT_Result.push(rResult);
	Sleep(6000);
}

Map_Point CIVExecution::Crossing(int n, int num, Map_Point Point[3],GTentacle *A_Tentacles)//n: 1,2,3 = 左，中，右。使用路点规划
{
	int a[3];
	Map_Point temp[3];
	for(int i = 0; i<num; i++)
	{
		a[i] = GetAID(A_Tentacles,Point[i]);
		if(a[i]<25)temp[1] = Point[i];
		else if(abs(a[i]-40)<15)temp[2] = Point[i];
		else temp[3] = Point[i];
	}
	return temp[n];
}


int CIVExecution::Crossing(int n,I_Map *RT_Map,GTentacle *A_Tentacles)//n: 1,2,3 = 左，中，右。使用地图规划
{
	int x1,y1;
	int temp;
	int id = -1;
	int i,j;
	if(n == 1)
	{
		temp = 20;
		for(i = 35;i>0;i--)
		{
			for(j = 0; j<200; j++)
			{	
				x1 = (int)A_Tentacles->OneTentacle[i].OnePoint[j].x;
				y1 = (int)A_Tentacles->OneTentacle[i].OnePoint[j].y;
				if(GoEnable(RT_Map,x1,y1) != 0) break;	
			}	
			
			if(j>temp)
			{
				temp = j;
				id = i;
			}		
		}
		return id;
	}

	if(n == 3)
	{
		temp = 20;
		for(i = 80;i>45;i--)
		{
			for(j = 0; j<200; j++)
			{	
				x1 = (int)A_Tentacles->OneTentacle[i].OnePoint[j].x;
				y1 = (int)A_Tentacles->OneTentacle[i].OnePoint[j].y;
				if(GoEnable(RT_Map,x1,y1) != 0) break;	
			}	
			
			if(j>temp)
			{
				temp = j;
				id = i;
			}		
		}
		return id;
	}
	else
	{
		temp = 20;
		for(i = 30;i<50;i++)
		{
			for(j = 0; j<200; j++)
			{	
				x1 = (int)A_Tentacles->OneTentacle[i].OnePoint[j].x;
				y1 = (int)A_Tentacles->OneTentacle[i].OnePoint[j].y;
				if(GoEnable(RT_Map,x1,y1) != 0) break;	
			}	
			
			if(j>temp)
			{
				temp = j;
				id = i;
			}		
		}
		return id;
	}
	
}



DWORD CIVExecution::SearchAtenna_GPS( )
 {
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();	
	onoff_flag=true;

	SetTrafficSign();
	CvPoint2D64f apoint;
	Map_Point Apoint,Apoint2;
	tentacles = GetTentacles(1);
	double lane_angle;
	double lane_angle1to2,lane_angle2to3;
	CSocketTransfer sendstopline;
	int seg_id = 17;
	int lan_id = 2;
	int point_id = 3;

	int sign = 0; //1,2,3 = 左，中，右

	while(onoff_flag)
	{
		//-------执行U-Turn--------
		if(on_uturn)
		{
		
			lane_angle = GpsData.GetAngle(RNDF.GetPoint(seg_id,lan_id,point_id+1),RNDF.GetPoint(seg_id,lan_id,point_id));

			UTurn(lane_angle);
			on_uturn=false;
			point_id++;
		}	

		rDirection = GpsData.GetDirection();		
		rPosition = GpsData.GetPosition();
		if(lan_id == 0)
		{
			apoint = RNDF.GetPerimeter(seg_id,point_id);
			AfxMessageBox("进入区域！");
			return 0;
		}
		else
			apoint = RNDF.GetPoint(seg_id,lan_id,point_id);
		apoint.y+=0.00005;
		Apoint = GpsData.APiontConver(rPosition,apoint,rDirection);

		
		if(!on_cross)
		{
			//Apoint.x = (Apoint.x-256)*0.6+0.5+256;
			Apoint.y = 256 - (256 - Apoint.y)*0.6;
		}

		//-------执行路口--------
		if(on_cross)
		{
		
			lane_angle = GpsData.GetAngle(RNDF.GetPoint(seg_id,lan_id,point_id+1),RNDF.GetPoint(seg_id,lan_id,point_id));

			if(abs(rDirection-lane_angle)<20||abs(abs(rDirection-lane_angle)-360)<20) //按照下一路口的方向判断是否进入路口。
			{
				outa<<"路口已通过"<<endl;
			
				on_cross = false;
				point_id++;
				continue;
			}

		}

		//-------判断能否加速--------
		//int temp = point_id; 
		//if(RNDF.m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].waypoints_num-temp<2)
		//{
		//	temp=RNDF.m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].waypoints_num-2;
		//}
		//lane_angle1to2 = GpsData.GetAngle(RNDF.GetPoint(seg_id,lan_id,1+temp),RNDF.GetPoint(seg_id,lan_id,temp));
		//lane_angle2to3 = GpsData.GetAngle(RNDF.GetPoint(seg_id,lan_id,2+temp),RNDF.GetPoint(seg_id,lan_id,1+temp));
		//if(point_id>1&&(abs(lane_angle1to2-lane_angle2to3)<10||abs(abs(lane_angle1to2-lane_angle2to3)-360)<10))
		//	on_speedup = true;
		//if(point_id+1>RNDF.m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].waypoints_num||point_id == RNDF.m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].exit_id)
		//	on_speedup = false;
		
		//-------发送结果-------
		rResult.TentacleID = GetAID(tentacles,Apoint);
		if(on_speedup)
			rResult.speed = 20;
		else
			rResult.speed = 5;

		rResult.brake = -25500;
		rResult.way_switch = 3;
		app->RT_Result.push(rResult);		
		

		Sleep(80);

		outa<<seg_id<<','<<lan_id<<','<<point_id<<"  目标点:  "<<Apoint.x<<", "<<Apoint.y;
		outa<<" 触须id  "<<rResult.TentacleID <<endl;
		SYSTEMTIME t1;
		GetLocalTime(&t1);
		outa<<t1.wHour<<':'<<t1.wMinute<<':'<<t1.wSecond<<':'<<t1.wMilliseconds<<',';
		if((256-Apoint.y)<60&&point_id == RNDF.m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].stop_id)
		{
			
			
			sendstopline.ClientConnect("192.168.0.106",2222);
			const char * sl="ok";
			sendstopline.ClientSend(sl);
			outa<<seg_id<<"   "<<endl;
			StopLine();
			on_stopline = true;
		}
		//-------从RNDF取下一路点--------
		if(abs(Apoint.x-256)<10&&abs(256-Apoint.y)<15||on_stopline)
		{
			
			on_cross=false;
			on_stopline=false;
			//-------转下一个lane的情况--------
			if(point_id == RNDF.m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].exit_id)
			{
				
				sign = app->Traffic_Sign.front();
				app->Traffic_Sign.pop();
				lane_angle = GpsData.GetAngle(RNDF.GetPoint(seg_id,lan_id,point_id),RNDF.GetPoint(seg_id,lan_id,point_id-1));
				int num = RNDF.m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].exit_num;
				if(num==1)
				{
					
						int m = RNDF.m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].pExit[0].m;
						int n = RNDF.m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].pExit[0].n;
						int p = RNDF.m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].pExit[0].p;
						seg_id = m;
						lan_id = n;
						point_id = p;
						
						double exit_angle = GpsData.GetAngle(RNDF.GetPoint(m,n,p+1),RNDF.GetPoint(m,n,p));
						//-------根据方向判断U-Turn--------
						if((abs((exit_angle-lane_angle)-180)<30 ||abs(abs((exit_angle-lane_angle)-180)-360)<30))
							on_uturn = true;
	
				}
				else 
				{
					on_cross=true;
				
					//Sleep(5000);
					double exit_angle[4];
					for(int i = 0; i<num;i++)
					{
						int m = RNDF.m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].pExit[i].m;
						int n = RNDF.m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].pExit[i].n;
						int p = RNDF.m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].pExit[i].p;
						
						exit_angle[i] = GpsData.GetAngle(RNDF.GetPoint(m,n,p+1),RNDF.GetPoint(m,n,p));
						if((abs((exit_angle[i]-lane_angle)-270)<60 ||abs(abs((exit_angle[i]-lane_angle)-270)-360)<60)&&(sign == 1))
						{
							seg_id = m;
							lan_id = n;
							point_id = p;
							break;
						}
						else if((abs((exit_angle[i]-lane_angle))<30 ||abs(abs((exit_angle[i]-lane_angle))-360)<30)&&(sign == 2))
						{
							seg_id = m;
							lan_id = n;
							point_id = p;
							break;
						}
						else if((abs((exit_angle[i]-lane_angle)-90)<60 ||abs(abs((exit_angle[i]-lane_angle)-90)-360)<60)&&(sign == 3))
						{
							seg_id = m;
							lan_id = n;
							point_id = p;
							break;
						}
						else if((abs((exit_angle[i]-lane_angle)-180)<30 ||abs(abs((exit_angle[i]-lane_angle)-180)-360)<30)&&(sign == 3))
						{
							//-------根据方向判断U-Turn--------
							on_uturn = true;
							seg_id = m;
							lan_id = n;
							point_id = p;
							break;
						}
					}

				}

			}

			//-------同一lane中去下一个点的情况--------
			else
			{
				point_id++;
				if(point_id>RNDF.m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].waypoints_num) 
				{
					AfxMessageBox("行程结束，线程关闭。");
					return 0;
				}
			}
				

		}

	}

    AfxMessageBox("Search线程关闭");
	return 0;
	
}

DWORD CIVExecution::SearchAtenna_GPS_u( )
 {
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();	
	onoff_flag=true;
	

	CvPoint2D64f apoint;
	Map_Point Apoint,Apoint2;
	tentacles = GetTentacles(1);
	int seg_id = 1;
	int lan_id = 1;
	int point_id = 1;

	while(onoff_flag)
	{

		rDirection = GpsData.GetDirection();		
		rPosition = GpsData.GetPosition();
		apoint = RNDF.GetPoint(seg_id,lan_id,point_id);
		Apoint = GpsData.APiontConver(rPosition,apoint,rDirection);

		if(on_uturn == true)
		{
			double lane_angle;
			lane_angle = GpsData.GetAngle(RNDF.GetPoint(seg_id,lan_id,point_id),RNDF.GetPoint(seg_id,lan_id,point_id+1));
			rResult.TentacleID = 0;
			rResult.contrl=1;
			app->RT_Result.push(rResult);
			Sleep(6000);
			outa<<"utrun开始"<<endl;
			UTurn(lane_angle);
			rResult.TentacleID = 40;
			rResult.contrl=1;
			app->RT_Result.push(rResult);
			Sleep(6000);
			point_id++;

		}
		else
		{
			
			Apoint2.x = (Apoint.x-256)*0.5+0.5+256;
			Apoint2.y = 256+0.5-(256-Apoint.y)*0.5;
			rResult.TentacleID = GetAID(tentacles,Apoint);
			rResult.contrl=3;
			app->RT_Result.push(rResult);		
		}


		
		Sleep(200);
		outa<<"目标点:  "<<Apoint.x<<", "<<Apoint.y<<endl;;
		outa<<"目标点触须id  "<<rResult.TentacleID <<endl;
		SYSTEMTIME t1;
		GetLocalTime(&t1);
		outa<<t1.wHour<<':'<<t1.wMinute<<':'<<t1.wSecond<<':'<<t1.wMilliseconds<<',';

		if(abs(Apoint.x-256)<10&&abs(256-Apoint.y)<10)
		{
			if(point_id == RNDF.m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].exit_id)
			{
				on_uturn = true;
				int m = RNDF.m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].pExit[0].m;
				int n = RNDF.m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].pExit[0].n;
				int p = RNDF.m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].pExit[0].p;
				seg_id = m;
				lan_id = n;
				point_id = p;
			}
			else
			{
				point_id++;
				if(point_id>RNDF.m_mapInfo.pSegment[seg_id-1].pLane[lan_id-1].waypoints_num) 
				{
					AfxMessageBox("行程结束，线程关闭。");
					return 0;
				}
			}
				
		}

	}

    AfxMessageBox("Search线程关闭");
	return 0;
	
}
DWORD CIVExecution::SearchAtenna_SMap( )
 {
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();	
	onoff_flag=true;
	I_Map* WayMap;
	WayMap = new I_Map;
	Map_Point Apoint[200] = {0};

	tentacles = GetTentacles(1);

	while(onoff_flag)
	{
	
		
		KeepLane.WaitMap(app->Way_Map,*WayMap);
		KeepLane.GetLane(WayMap,Apoint);

		rResult.TentacleID = GetWay(tentacles,Apoint);
		
			
		rResult.contrl=3;
		app->RT_Result.push(rResult);


		Sleep(100);

		outa<<"目标点触须id  "<<rResult.TentacleID <<endl;
		SYSTEMTIME t1;
		GetLocalTime(&t1);
		outa<<t1.wHour<<':'<<t1.wMinute<<':'<<t1.wSecond<<':'<<t1.wMilliseconds<<',';

	}

    AfxMessageBox("Search线程关闭");
	return 0;
	
}

DWORD CIVExecution::SearchAtenna_VMap( )
 {
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();	
	onoff_flag=true;
	I_Map* rMap;
	rMap = new I_Map;
	CvPoint2D64f Apoint[200] = {0};
	CvPoint2D64f GPSpoint[200] = {0};
	tentacles = GetTentacles(1);
	int data[4]={40,40,40,40};
	int id;
	CvPoint2D64f cGpsConv[200];
	double s=0;
	double s10=0;
	int hisid=40;
	CvPoint2D64f cMap,cTenMap;
	S_Result rResult;
	CvPoint2D64f cTenMGps;
	bool cbreakflag=false;
	while(onoff_flag)
	{
		/*rResult.way_switch=3;
		rResult.brake=-25500;
		rResult.speed=10;*/

		rDirection = GpsData.GetDirection();		
		rPosition = GpsData.GetPosition();

//		KeepLane.WaitMap(app->RT_Map,*rMap);
		
		//KeepLane.GetLane(rMap,Apoint);
		//rResult.TentacleID = GetWay(tentacles,Apoint);
		LocalPath.Get10Sample(rMap);
		LocalPath.Get200Point(Apoint);
		
		for(int i=0;i<200;i++)
		{
			cGpsConv[i]=GpsData.MaptoGPS(rPosition,rDirection,Apoint[i]);
		}
		//200个点转GPS
		for(int j=0;j<200;j++)
		{
			s10=GpsData.GetDistance(rPosition.x,rPosition.y,cGpsConv[j].x,cGpsConv[j].y);
			if(s10 >= 15)
			{
				cTenMGps.x=cGpsConv[j].x;
				cTenMGps.y=cGpsConv[j].y;
					break;
			}
		}
		
		int k=0;
		
		while(1)
		{
			rDirection = GpsData.GetDirection();		
			rPosition = GpsData.GetPosition();
			for(k;k<200;k++)
			{
				s=GpsData.GetDistance(rPosition.x,rPosition.y,cGpsConv[k].x,cGpsConv[k].y);
				if(s >= 7)
				{
					cMap=GpsData.APiontConverD(rPosition,cGpsConv[k],rDirection);
					id=GetAID(tentacles,cMap);
				    if(abs(hisid-id)>20)
					{
						cbreakflag=true;
						break;
					}
					break;
				}
			}
			if(cbreakflag)
			{
				cbreakflag=false;
				break;
			}
			hisid=id;
			rResult.TentacleID=id;
			rResult.way_switch=3;
			rResult.brake=-25500;
			rResult.speed=5;
			app->RT_Result.push(rResult);
			cTenMap=GpsData.APiontConverD(rPosition,cTenMGps,rDirection);
			if(abs(256-cTenMap.x)<5&&abs(256-cTenMap.y)<10)
				break;
	
			Sleep(10);
		}
		


		outa<<"目标点触须id  "<<rResult.TentacleID<<"原id  "<<id <<endl;
		SYSTEMTIME t1;
		GetLocalTime(&t1);
		outa<<t1.wHour<<':'<<t1.wMinute<<':'<<t1.wSecond<<':'<<t1.wMilliseconds<<',';

	}
	delete rMap;
    AfxMessageBox("Search线程关闭");
	return 0;
	
}
/*
计算一组触须，返回。
输入：num - 当前车辆速度取整
*/

 GTentacle *CIVExecution::GetTentacles(int num)
 {

	
	 GTentacle *t;
	 t = new GTentacle;

	
	 for(int i = 0; i<81; i++)
	 {
		 
		 t->OneTentacle[i].Angle = -68+1.7*i;
		 for(int j = 0; j<400 ;j++)
		 {
			 t->OneTentacle[i].OnePoint[j].y = 256-5*r[i]*sin(j/(10*r[i])); 
			 t->OneTentacle[i].OnePoint[j].x = 256+5*r[i]*(1-cos(j/(10*r[i])));
			 t->OneTentacle[i].Angle = r[i];
		 }
	 }
	 t->SpeedGroup = num;
	
	//从Built类中取值！

	 return t;
 }



/*计算两点距离*/
double CIVExecution::D(CvPoint2D64f p1, Map_Point p2)
{
	double temp;
	temp = pow(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2),0.5);
	return temp;
}
double CIVExecution::D(CvPoint2D64f p1, CvPoint2D64f p2)
{
	double temp;
	temp = pow(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2),0.5);
	return temp;
}

/*
搜索通向目标的一条触须，返回此条触须ID。
输入：A_Tentacles - 当前速度下整组触须，
	  A_Position - 目标点地图坐标。
*/
int CIVExecution::GetAID(GTentacle *A_Tentacles,Map_Point A_Position)
{
	int aID;
	aID = -1;
	double d1 = 0, d2 = 0;
	double temp;
	temp = 1000.0;

	for(int i = 0; i < 41; i++)  //从中间向两边搜目标
	{
		for(int j = 0; j<400;j++) 
		{
		
			d1 = D(A_Tentacles->OneTentacle[40+i].OnePoint[j],A_Position);
			
			if(d1<temp)
			{
				temp = d1;
				aID = 40+i;
				
			}

		}
		for(int j = 0; j<400;j++) 
		{
		
			d1 = D(A_Tentacles->OneTentacle[40-i].OnePoint[j],A_Position);
			
			if(d1<temp)
			{
				temp = d1;
				aID = 40-i;
				
			}

		}
	}
	if (aID == -1)aID = 40;
	if (abs(A_Position.x-256)<4&&abs(A_Position.x-256)<40)aID = 40;
	return aID;
}
int CIVExecution::GetAID(GTentacle *A_Tentacles,CvPoint2D64f A_Position)
{
	int aID;
	aID = -1;
	double d1 = 0, d2 = 0;
	double temp;
	temp = 1000.0;

	for(int i = 0; i < 41; i++)  //从中间向两边搜目标
	{
		for(int j = 0; j<400;j++) 
		{
		
			d1 = D(A_Tentacles->OneTentacle[40+i].OnePoint[j],A_Position);
			
			if(d1<temp)
			{
				temp = d1;
				aID = 40+i;
				
			}

		}
		for(int j = 0; j<400;j++) 
		{
		
			d1 = D(A_Tentacles->OneTentacle[40-i].OnePoint[j],A_Position);
			
			if(d1<temp)
			{
				temp = d1;
				aID = 40-i;
				
			}

		}
	}
	if (aID == -1)aID = 40;
	//if (abs(A_Position.x-256)<4&&abs(A_Position.y-256)<40)aID = 40;
	return aID;
}
/*
返回停止指令：刹车，方向盘回正。
*/
S_Result CIVExecution::Stop()
{
	S_Result temp;
	temp.contrl = 1;
	temp.Distance = 0;
	temp.TentacleID = 40;
	return temp;
}


/*开启发送数据线程*/
void CIVExecution::StartSend()
{
		m_hThreadSend = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE) CIVExecution::theSendThread,
		this, 0, &dwSendThreadId);
}

/*关闭发送数据线程*/
void CIVExecution::EndSend()
{
}
DWORD CIVExecution::theSendThread(LPVOID lpParam)
{
		return (((CIVExecution*)lpParam)->SendThread());
}
DWORD CIVExecution::SendThread()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	
	int t=0;
	int v=5;
	
	int b=-25500;
	int s=3;
	int k=0;
	int a=0;
	int h=0;
	CString l="0000";
	
	S_Result result;
	S_Result hisresult={0};
	CString str;
	while(1)
	{	
		// %G100 关键字
		// T 方向盘转向值（-1050000 ~ 1050000）qc
		// V 控制速度值 （0 ~ 40）qc
		// B 刹车位置值（-25500 ~ -42500）qc (-25500为等待位，-42500为刹车到底位)
		// S 换挡（0-P停车 1-R倒档  2-N空挡  3-D行车）
		// K 钥匙（暂时不需要）
		// A 油门（暂时不需要）
		// H 喇叭（暂时不需要）
		// L 灯光（4位二进制 0表示左转 1表示右转 2表示远光 3表示喇叭）0000
		// END 结束符

		CSocketTransfer send;
		send.ClientConnect("192.168.0.109",3456);
		int j=100;
		while(j--)
		{
			int size = app->RT_Result.size();
			while(1)
			{
				if(size>0)
					break;
				Sleep(5);
			}
			for(int i=0;i<size-1;i++)
			{
					app->RT_Result.pop();
			}
			result = app->RT_Result.front();
			outc<<"result"<<result.brake<<","<<result.speed<<","<<result.TentacleID<<","<<result.way_switch<<endl;
	//		app->RT_Result.pop();
			if(result.TentacleID==hisresult.TentacleID&&result.way_switch==hisresult.way_switch&&result.speed==hisresult.speed)
			{
				Sleep(5);
				continue;
				
			}
			t = int(code[result.TentacleID]/2.33);
			v=result.speed;
			b=result.brake;
			s=result.way_switch;
			if(t<-450000)
				t=-450000;
			if(t>450000)
				t=450000;
			str.Format("%sG100T%dV%dB%dK%dA%dH%dL%sEND;","%",t,v,b,k,a,h,l);
			const char* sttrr=str.GetString();
		
		    if(send.SuccSocket)
 		    send.ClientSend(sttrr);
		    hisresult=result;
			outc<<"hisresult"<<hisresult.brake<<","<<hisresult.speed<<","<<hisresult.TentacleID<<","<<hisresult.way_switch<<endl;
		    Sleep(200);
		
	  }
	}
	return 0;
}


/*开启接受GPS数据线程*/
void CIVExecution::StartGPS()
{
	GpsData.SetCom("COM3","115200");
}


/*
已知行车轨迹半径的情况下，搜索触须ID
输入：R - 半径(m)
返回：触须ID
*/
int CIVExecution::GetRID(double R)
{

	double temp = 1000;
	int count = -1;
	for(int i = 0; i<81; i++)
	{
		if(temp>abs(R-r[i]))
		{
			temp = abs(R-r[i]);
			count = i;
		}
	}
	return count;
}

int CIVExecution::GetWay(GTentacle *A_Tentacles,Map_Point *A_Position)
{
    
	int aID = 0; 
	aID = -1;
	

	double a[81] = {0};//记录通过目标点的触须ID
	for(int i = 0;i<81;i++)
		a[i]=i;
	a[0]=-1;
	int num=81;//记录通过目标点的触须的条数

	int temp_a;
	int temp_n;
	int t;


	for(int m = 0; m<40; m++)
	{
		temp_n=0;
		for(int i = 0; i < num; i++)  
		{
			temp_a=0;
			
			for(int k = 0; k<5; k++)
			{
				for(int j = 0; j<400;j++) 
				{
					t = a[i];
					if (t == -1)t = 0;
					if(abs(A_Tentacles->OneTentacle[t].OnePoint[j].x-A_Position[k+m*k].x)<5
						&&abs(A_Tentacles->OneTentacle[t].OnePoint[j].y-A_Position[k+m*k].y)<5)
					{
						temp_a++;
						break;
					}
					 
				}
			
			}
			if(temp_a == 5)
			{
				a[temp_n]=a[i];
				temp_n+=1;
				
			}
			
		}
		num = temp_n;
		if(num == 0)break;
	}
	aID=a[0];
	if (aID == -1)
		aID = GetAID(A_Tentacles,A_Position[30]);
	return aID;

}

bool CIVExecution::GoEnable(I_Map *p,int x,int y)
{

	//int temp = p->MapPoint[x][y];
	for(int i = -4; i < 4; i++)
	{
		if(p->MapPoint[y][x+i] <= 0)
			return 0;
	}

	for(int j = -4; j < 4; j++)
	{
		if(y+j>255)continue;
		//if(y+j<0)continue;
		if(p->MapPoint[y+j][x] <= 0)
			return 0;
	}

	return 1;
				
}

int CIVExecution::GetWay_V(GTentacle *A_Tentacles,I_Map *map)
{
    
	int aID = 0; 
	aID = -1;
	

	//double a[81] = {0};//记录通过目标点的触须ID
	//for(int i = 0;i<81;i++)
	//	a[i]=i;
	//a[0]=-1;
	//int num=81;//记录通过目标点的触须的条数
	int temp = 0;
	int temp_a[81] = {0};
	int temp_n;
	int t;
	int x,y;
	temp_n = 0;

	for(int i = 0; i < 81; i++)  
	{
		
		for(int j = 0; j<400;j++) 
		{
			
			x = A_Tentacles->OneTentacle[i].OnePoint[j].x+0.5;
			y = A_Tentacles->OneTentacle[i].OnePoint[j].y+0.5;
			temp_a[i] = j;
			if(GoEnable(map,x,y) == 0)
				break;
		}
		
	}

	for(int i = 0; i<81; i++)
	{
		if(temp_a[i]>temp)
		{
			temp = temp_a[i];
			temp_n = i;
		}
	}
	
	aID = temp_n;
	if(aID == -1) aID = 40;//这里要出错处理

	return aID;

}
void CIVExecution::Convert200(CvPoint2D64f v,CvPoint2D64f *a,double direction,Map_Point (&Ap)[200])
{
	//Map_Point *Ap;
	//Ap = new Map_Point;

	for(int i = 0; i<200; i++)
	{
		Ap[i] = GpsData.APiontConver(v,*(a+i),direction);
	}
	//return Ap;
}
void CIVExecution::Convert200(CvPoint2D64f v,CvPoint2D64f *a,double direction,CvPoint2D64f (&Ap)[200])
{
	//Map_Point *Ap;
	//Ap = new Map_Point;

	for(int i = 0; i<200; i++)
	{
		Ap[i] = GpsData.APiontConverD(v,*(a+i),direction);
	}
	//return Ap;
}
void CIVExecution::StopLine()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();	
	//I_Map *LineMap;
	//LineMap= new I_Map;
	S_Result rResult;
	CvPoint2D64f x;//用于记录上一帧的位置
	CvPoint2D64f gps;
	double s1=0;
	double s2=0;
	double s=0;
	bool stopflag=true;
	double speed=0; 
	int stopline[8]={1,0,1,0,1,0,1,0};
	while(1)
	{
		KeepLane.WaitMap(app->Way_Map,*LineMap);
		for(int j=0;j<8;j++)
		outa<<LineMap->MapPoint[0][j]<<',';
		outa<<endl;
		for(int i=0;i<8;i++)
		{
			if(stopline[i]!=LineMap->MapPoint[0][i])
			{
				stopflag=false;
				break;
			}
			stopflag=true;
		}
		if(stopflag)
		{
			gps=GpsData.GetPosition();
			speed=GpsData.GetSpeed();
			outa<<"停止线停车";
			s=9.59-speed*0.9;
			s2=speed*0.71+0.5*1.357*pow((speed/1.357),2);
			outa<<"speed"<<speed<<','<<s2<<','<<s<<endl;
			while(1)
			{
				//x=gps;
				//gps=app->RT_Station.front();
				//gps=GpsData.GetPosition();
				x=GpsData.GetPosition();
				s1=GpsData.GetDistance(x.x,x.y,gps.x,gps.y);//调整，以前延时200ms才计算S1.

				if((abs(s1-s+s2)<=0.75)||speed>2.8) //新增，车速高于临界车速时，直接输出制动
					{
						rResult.TentacleID=40;
						rResult.brake=-42400;
						rResult.speed=0;
						rResult.way_switch=3;
						app->RT_Result.push(rResult);
							outa<<s1<<endl;
							outa<<"制动了"<<endl;
						//AfxMessageBox("zhidongle");
						Sleep(10000);
						rResult.TentacleID=40;
						rResult.brake=-25500;
						rResult.speed=5;
						rResult.way_switch=3;
						app->RT_Result.push(rResult);
						Sleep(100);	
						return ;
					}
					rResult.TentacleID=40;
					rResult.brake=-25500;
					rResult.speed=5;
					rResult.way_switch=3;
					app->RT_Result.push(rResult);
					Sleep(200);	
			}
		}
	}
}