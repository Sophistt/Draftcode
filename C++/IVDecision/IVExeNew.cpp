#include "StdAfx.h"
#include "IVExeNew.h"

#include<fstream>

using namespace std;
	 ofstream outn1("chuli.txt");
	 ofstream outn2("dist.txt");
	 ofstream outn5("tc.txt");
	 //ofstream outn3("fasong");
	 //ofstream outn4("ok.txt");
	 ofstream outn7("testt.txt");
	 ofstream outn8("testym.txt");
	ofstream outn9("lujin.txt");

	ofstream outaaa(L"C:\\Documents and Settings\\Administrator\\桌面\\经过的点.txt",ios::app);



CIVExeNew::CIVExeNew(void)
{
	//存储路径点
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	/*CvMemStorage* */storage_road = cvCreateMemStorage(0);
	seq_road = cvCreateSeq( CV_64FC2, sizeof(CvSeq), sizeof(CvPoint2D64f), storage_road );
	plan_road = cvCreateSeq( CV_64FC2, sizeof(CvSeq), sizeof(CvPoint2D64f), storage_road );

	sendflag = false;
 	lastturn = 0;
	m_ctrl.turn=0;
	m_ctrl.velocity=0;
	m_ctrl.brake=-25500;
	m_ctrl.mswitch=0;
	m_ctrl.key=0;
	m_ctrl.accelerator=37;
	m_ctrl.horn=0;
	m_ctrl.light=0;
	m_end.x = 0;
	m_end.y = 0;
//	m_end.z = 0;

	aim_length = 10;
	lastaim_length = 10;
	lastu = 0;
	u_desire = 10;
	lastaim_s = 10;
	lasts = 0;

	laterr_y[0]=0;
	laterr_u[0]=0;
	laterr_y[1]=0;
	laterr_u[1]=0;
	laterr_y[2]=0;
	laterr_u[2]=0;

	last_gps.x = 0;
	last_gps.y = 0;
	last_dir = 0;
	last_t = 0;
	//gps.Set_RecAddress("udpm://239.255.76.69:7669?ttl=1");
	//gps.Set_RecChannel("GPS");
	m_GpsDecision.Set_RecAddress("udpm://239.255.76.68:7668?ttl=1");
	m_GpsDecision.Set_RecChannel("CONTROL");
	app->rec_flag = false;
	lcm_send = lcm_create("udpm://239.255.76.70:7670?ttl=1");      	
	if(!lcm_send)  
	{
		//AfxMessageBox("sendmistake"); 
		return;
	}	
	//////////
	//vel_Map = new I_Map;
	mid_Map = new I_Map;
	lane_Map = new I_Map;
	/*for( int i=0; i<512; i++)
	{
		for( int j=0; j<512; j++)
		{
			vel_Map->MapPoint[i][j] = 0;
			
		}
	}*/
	for( int i=0; i<512; i++)
	{
		for( int j=0; j<512; j++)
		{
			mid_Map->MapPoint[i][j] = 0;
			
		}
	}
	//lane_Map = vel_Map;
	for( int i=0; i<512; i++)
	{
		for( int j=0; j<512; j++)
		{
			lane_Map->MapPoint[i][j] = 0;
			
		}
	}
	
	s_new = 0;
	s_old = 0;
	for(int i = 0;i<200;i++)
	{
		MidPoint[i] = cvPoint2D64f(256,412);
		MidGpsPoint[i] = cvPoint2D64f(0,0);
		his_MidPoint[i] = cvPoint2D64f(0,0);
		shift_MidPoint[i] =  cvPoint2D64f(0,0);
	}
	///
	app->tj_flag = false;
	on_obs = false;
	ob_count = 0;
	stop_flag = false;
	way_out = false; 
	seq_num = -1;
	app->Get_Sign=false;
	app->stop_lane = false;
	pass_count =0;
}

CIVExeNew::~CIVExeNew(void)
{
	cvReleaseMemStorage(&storage_road);
}

/*******************************************************************************************
控制部分的发送线程
*******************************************************************************************/
/*开启发送数据线程*/
//void CIVExeNew::StartCtrl()
//{
//	m_hThreadSend = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE) CControl::theCtrlThread,
//		this, 0, &dwSendThreadId);
//}
//
///*关闭发送数据线程*/
//
//DWORD CIVExeNew::theCtrlThread(LPVOID lpParam)
//{
//	return (((CIVExeNew*)lpParam)->CtrlThread());
//}
//
////控制转换线程
//DWORD CIVExeNew::CtrlThread()
//{
//	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
//	MSG   msgThreadData;
//	PeekMessage(&msgThreadData, NULL, WM_INTERVAL_EXE, WM_INTERVAL_EXE,PM_NOREMOVE);
//	  // 线程消息
//	//IVMCONTROL m_ctrl={0};
//	IVMCONTROL his_ctrl = {0};
//
//	int count = 0;
//	
//	//send.ClientConnect("192.168.0.109",3456);
//	while(1)
//	{	
//		if(!app->sendflag1 || stop_flag)
//		{
//			Sleep(10);
//			continue;
//		}
//		// %G100 关键字
//		// T 方向盘转向值（-450000 ~ 450000）qc
//		// V 控制速度值 （0 ~ 40）qc
//		// B 刹车位置值（-25500 ~ -42500）qc (-25500为等待位，-42500为刹车到底位)
//		// S 换挡（0-P停车 1-R倒档  2-N空挡  3-D行车）
//		// K 钥匙（暂时不需要）
//		// A 油门（暂时不需要）
//		// H 喇叭（暂时不需要）
//		// L 灯光（4位二进制 0表示左转 1表示右转 2表示远光 3表示喇叭）0000
//		// END 结束符
//		//取当前的GPS坐标和方向
//		CvPoint2D64f m_gps = cvPoint2D64f(0, 0);
//		double m_gpsdir = 0;
//		double speed = 0;
//		app->critical_section.Lock();//锁住
//		m_gps = app->GPS_Point;
//		m_gpsdir = app->GPS_Direction;
//		speed = app->GPS_Speed;
//		outn7<<"位置： "<<(m_gps.x-31)*10000<<", "<<(m_gps.y-117)*10000<<"  "<<"航向： "<<m_gpsdir<<"  ";
//		app->critical_section.Unlock();//解锁
//
//		///接收路径规划消息，更新路点信息，根据新的路点更新控制，不更新按原路径走
//		//int iRet = PeekMessage(&msgThreadData, NULL, WM_INTERVAL_EXE, WM_INTERVAL_EXE,PM_REMOVE);
//		int iRet = WaitForSingleObject(app->m_hEvent,10);
//
//		//取得消息
//		if( iRet == 0)
//		{
//			//cvClearSeq( seq_road  );
//			seq_road = cvCloneSeq (CVehicle::planning_road) ;
//			
//
//		}
//		//outn2<<count<<"   "<<iRet<<endl;
//		///////////////////////////////////////////////
//		//确定预瞄距离
//		zGetAimLen(speed);
//		//确定目标点
//		CvPoint2D64f m_dstgps;
//		// m_dstgps = *(CvPoint2D64f*)cvGetSeqElem( seq_road, 0 );
//		zGetAimPoint( m_gps, m_gpsdir, seq_road, m_dstgps );
//		outn7<<"  目标点  "<<(m_dstgps.x-31)*10000<<", "<<(m_dstgps.y-117)*10000<<"  ";
//		//u_desire = m_dstgps.z;
//		//到目标点就停止
//		count++;
//		
//		if(( m_dstgps.x == m_gps.x) && (m_dstgps.y == m_gps.y))
//		{
//
//			//zStop();
//			m_ctrl.turn = 0;
//			m_ctrl.velocity = 0;
//			m_ctrl.accelerator = 37;
//			m_ctrl.brake = -47500;/////////////////////////////////////刹车
//			//m_ctrl.horn = 0;
//			//m_ctrl.key = 0;
//			//m_ctrl.light = 0;
//			m_ctrl.mswitch = 3;
//		}
//		else
//		{
//			zGetCtrlParameters( m_gps, m_gpsdir, m_dstgps, m_ctrl );
//			//zGetCtrlParameters( m_gps, m_gpsdir, m_dstgps, m_ctrl );//得到到目标点的控制量
//		}
//		//if(m_ctrl.turn == his_ctrl.turn && m_ctrl.velocity == his_ctrl.velocity && m_ctrl.brake == his_ctrl.brake && m_ctrl.mswitch == his_ctrl.mswitch && m_ctrl.accelerator == his_ctrl.accelerator)
//		//{
//		//		Sleep(5);
//		//		continue;
//
//		//}
//		//发送控制量
//		if((abs(m_ctrl.turn)>30000&&app->GPS_Speed*3.6>18)||((app->GPS_Speed*3.6-u_desire)>4))
//		{
//			m_ctrl.turn = 0;
//			m_ctrl.accelerator = 37;
//			m_ctrl.brake = -45000;
//			Sleep(10);
//			continue;
//		}
//		if(m_ctrl.turn<-430000)
//			m_ctrl.turn=-430000;
//		if(m_ctrl.turn>430000)
//			m_ctrl.turn=430000;
//		if(m_ctrl.accelerator<37)
//			m_ctrl.accelerator=37;
//		//if(m_ctrl.accelerator>110)
//		//	m_ctrl.accelerator=110;
//		//CString str;
//		//str.Format("%sG100T%dV%dB%dS%dK%dA%dH%dL%sEND;","%",m_ctrl.turn,m_ctrl.velocity,m_ctrl.brake,m_ctrl.mswitch,0,m_ctrl.accelerator,0,0);
//		//app->sick_num = m_ctrl.turn;
//		//const char* sttrr=str.GetString();
//		//sttrr=str.GetString();
//		//if(send.SuccSocket)
//		//	send.ClientSend(sttrr);
//	
//		//app->sick1 = m_ctrl.turn;
//		his_ctrl=m_ctrl;
//		Sleep(10);	
//		
//	}
//	return 0;
//}

/***********************************************************************
控制线程结束
***********************************************************************/

/***********************************************************************
车辆控制状态计算，根据当前点和目标点计算车辆 转向、制动、速度等
根据当前点、方向和目标点，确定前轮转弯半径，确定控制量
输入：
	m_gps:当前GPS点，x-纬度，y-经度
	m_direction:当前车辆方向
	m_dstgps:目标GPS点
输出：
	m_strl:输出控制量
************************************************************************/

void CIVExeNew::zGetCtrlParameters( CvPoint2D64f m_gps, double m_direction, CvPoint2D64f m_dstgps/*double m_dstdir,*/, IVMCONTROL &m_ctrl )
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	int tc = 0,throttle = 0,brake = 0;
	double speed = app->GPS_Speed*3.6;
	//double speed = app->cspeed;

	zGetTurn(m_gps, m_direction, m_dstgps,tc);//计算转向
	//outn1<<"turn  "<<tc<<endl;

	zGetThrottle(speed,throttle,brake);//计算油门刹车

	m_ctrl.turn = tc;
	//lastturn = m_ctrl.turn;
	//m_ctrl.turn = 0;
	m_ctrl.velocity = 5;
	m_ctrl.accelerator = throttle;
	//m_ctrl.accelerator = 45;

	m_ctrl.brake = brake;
	//m_ctrl.horn = 0;
	//m_ctrl.key = 0;
	//m_ctrl.light = 0;
	m_ctrl.mswitch = 3;
}

/*****************************************************
已知行车轨迹半径的情况下，搜索触须ID
输入：r - 半径(m)
输出：t - 触须电机脉冲数量
******************************************************/
int CIVExeNew::GetTencaleByVr(double r, int &t)
{
	t = 430000*5.5/r;

	return 1;
}

/*******************************************************
通过路径轨迹，确定目标点
根据当前GPS点，当前方向，路点序列，输出目标点
输入：
	m_curgps:当前GPS，x-纬度，y-经度
	m_direction:当前方向
	m_roadseq:系列路点
输出：
	m_dstgps:输出前方路点（1点）
********************************************************/
int CIVExeNew::zGetAimPoint( CvPoint2D64f m_curgps, double m_direction, CvSeq * m_roadseq, CvPoint2D64f &m_dstgps )
{
	double dist;
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	
	int n = m_roadseq->total;//总路点数，路点存储按行走顺序
	if(n<1)
	{
		m_dstgps.x = m_curgps.x;
		m_dstgps.y = m_curgps.y;//停止
		return 0;
	}
	CvPoint2D64f pt = {0};
	CvPoint2D64f pt1 = {0};
	CvPoint2D64f pt2 = {0};
	CvPoint2D64f pt3 = {0};
	pt = *(CvPoint2D64f*)cvGetSeqElem( m_roadseq, n-1 );
	m_end.x = pt.x;
	m_end.y = pt.y;
	dist = m_GpsData.GetDistance( m_curgps.x, m_curgps.y, pt.x, pt.y );
	if(dist <10)
	{
		m_dstgps.x = m_curgps.x;
		m_dstgps.y = m_curgps.y;//停止
		return 0;
	}
	
	//确定当前点的位置。用当前方向来比较，前方的路点在车辆前方
	while( m_roadseq->total > 0 )
	{
		pt = *(CvPoint2D64f*)cvGetSeqElem( m_roadseq, 0 );

		if(m_direction>180)
			m_direction-=360;

		double north_dir = m_GpsData.GetAngle(pt.x, pt.y, m_curgps.x, m_curgps.y);
		double head_path_angle = north_dir - m_direction;
		if(head_path_angle >180)
			head_path_angle-=360;
		if(head_path_angle < -180)
			head_path_angle+=360;

		dist = m_GpsData.GetDistance( m_curgps.x, m_curgps.y, pt.x, pt.y );//计算距离，距离过大，需要插值，距离过小，找下一点
		if(dist <2  || ( (head_path_angle < -135) || (head_path_angle > 135) ) )
		{
			cvSeqPopFront( m_roadseq,NULL );
			continue;
		}
		int count=0;
		for(int j = 0; j<m_roadseq->total;j++)
		{
			pt1 = *(CvPoint2D64f*)cvGetSeqElem( m_roadseq, j );
			dist = m_GpsData.GetDistance( m_curgps.x, m_curgps.y, pt1.x, pt1.y );
			if(dist < aim_length)
			{
				count = j;
				break;
			}
		}
		for(int k = 0;k<count;k++)
		{
			cvSeqPopFront( m_roadseq,NULL );
		}

		int i = 0;	

		//if(dir3<0.5||abs(dir3-360)<0.5)state = 1; //直线
		//else if(dir3<8||abs(dir3-360)<8)state = 2; //大曲线
		//else state = 0; //转弯
		state = 1;
		app->state1 = state;
		//计算预瞄距离
		for( i=0; i<m_roadseq->total; i++)
		{
			pt = *(CvPoint2D64f*)cvGetSeqElem( m_roadseq, i );
			dist = m_GpsData.GetDistance( m_curgps.x, m_curgps.y, pt.x, pt.y );
			if(dist > aim_length)break;
		}
		m_dstgps = pt;
		return 1;		

		//double dist2 = m_GpsData.GetDistance( m_curgps.x, m_curgps.y, m_end.x, m_end.y );
		//if(dist2 < 20 )
		//{
		//	m_dstgps = m_curgps;//停止
		//	return 0;
		//}
	
	}
	m_dstgps = pt;
	dist = m_GpsData.GetDistance( m_curgps.x, m_curgps.y, pt.x, pt.y );
	
	if(dist < 5|| n <= 1 )
	{
		m_dstgps.x = m_curgps.x;
		m_dstgps.y = m_curgps.y;//停止
	}
	return 0;

}

/******************************************************************************************
综合控制流程、路径规划及系列目标点生成
******************************************************************************************/

/*开启处理线程*/
void CIVExeNew::StartProc()
{
	m_hThreadProc = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE) CIVExeNew::theProcThread,
		this, 0, &dwProcThreadId);
}

DWORD CIVExeNew::theProcThread(LPVOID lpParam)
{
	return (((CIVExeNew*)lpParam)->ProcThread());
}

//综合运行流程
DWORD CIVExeNew::ProcThread()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	m_MissionEvent = CreateEvent(NULL,FALSE,FALSE,NULL);

	m_hEvent=CreateEvent(NULL,FALSE,FALSE,NULL);
	MSG   msgThreadData;
	PeekMessage(&msgThreadData, NULL, WM_INTERVAL_EXE, WM_INTERVAL_EXE,PM_NOREMOVE);

	///////
	rec_map.Set_RecAddress("udpm://239.255.76.67:7667?ttl=1"); 
	rec_map.Set_RecChannel("PERCEPTION");
	rec_map.recv_message();


	if(app->Lead_pt.size()<=0)
	{
		AfxMessageBox("未加载引导点文件");
		return 0;
	}
	
	if(seq_num == -1)
	{
		AfxMessageBox("未加载引导点文件");
		return 0;
	}

	int mission_num = GetMissionNum();

	LeadPoint = (LEAD *)malloc( mission_num*sizeof(LEAD) );
	GetMissionPoint(LeadPoint,mission_num);
	
	
	CvPoint2D64f Apoint[20] = {0};//任务路点
	CvPoint2D64f GPSpoint[200] = {0};//插值路径点


	double lane_len;
	app->u_t =false;
	//app->stop_sign = false;
	int mid_num;
	int waypoint_num;
	int count = 0;

	
	
	int return_value = 0;
	int t_seg;
	 
	bool cross_out = false; 
	bool lane_out = false;
	bool on_uturn = false;
	bool stopped = false;
	m_task = IVTASK_LANE;
	
	CvPoint2D64f m_gps = cvPoint2D64f(0,0);
	double m_gpsdir = 0;
	double m_speed = 0;
	
	m_gps = app->GPS_Point;
	//GetSegmentNumberByGPS( RNDFGPS, m_gps.x, m_gps.y, n_seg );
	//输入几点序号
		
	CvPoint2D64f LeftPoint[200]={0};
	CvPoint2D64f RightPoint[200]={0};
	int ob = 0;
	int ob_x = 0;
	int ob_y = 0;
	
	CvPoint2D64f V_StartP = cvPoint2D64f(256,412);
	ArrayFuZhi(V_StartP,MidPoint);	

	LEAD StartPoint;
	StartPoint = LeadPoint[seq_num-1];
	int iRet_miss;

	//2012新加
	bool ob_cross = false;
	//if(StartPoint.param1 == 0||StartPoint.param1 == 1||StartPoint.param1 == 3)
	//	m_task = IVTASK_LANE;
	//else if(StartPoint.param1 == 2)
	//	m_task = IVTASK_CROSS;
	while(1)
	{
		Sleep(10);
		if(app->mapreceived&&app->GPS_Point.x != 0)
			break;
	}
	int sign[4] = {0};
	way_out = true;
	while(1)
	{	
		
	
		app->critical_map.Lock();
		vel_Map = app->PercepMap;
		app->critical_map.Unlock();
		
		ProcCenterLane(vel_Map,mid_Map);
		
	
		app->critical_section.Lock();//锁住
		m_gps = app->GPS_Point;
		m_gpsdir = app->GPS_Direction;
		m_speed = app->GPS_Speed;
		app->critical_section.Unlock();//解锁


		//if(!sendflag||cross_out||lane_out)
		//{
		//	way_out = true;
		//	cross_out = false;
		//	lane_out = false;
		//}
		

		//if(way_out&&sendflag)
		//{
		//	
		//	if(m_task == IVTASK_LANE)
		//		m_task = IVTASK_CROSS;
		//	else
		//		m_task = IVTASK_LANE;
		//}

		
		//KeepLane.WaitMap(app->RT_Map,*vel_Map);
		//判断路径正向、反向，路段和路口两种情况。。。。。

		//获得各种感知信息，包括RNDF路网信息
		//根据感知信息判断下一步的任务及路径
		if(/*app->stop_lane*/0)
		{
			CvPoint2D64f end_point;
			end_point.x = LeadPoint[mission_num-1].lat;
			end_point.y = LeadPoint[mission_num-1].lng;
			double dist = m_GpsData.GetDistance(m_gps.x,m_gps.y,end_point.x,end_point.y);
			if(dist<100)
			{
				u_desire = 10;
				on_obs = true;
			}
			if(isStopLine())
			{
				//outn1<<"停止县"<<endl;
				stopped = true;
				double s1 = 9.5;
				m_ctrl.turn = 0;
				StopLine(s1);
				return 0;
			}
		}
		
		double dist2,dist3  = 0;
		switch( m_task )
		{
			case IVTASK_LANE://需要根据感知信息规划路径，包括曲线和变更车道、路口转弯。发送获取感知信息指令。
				//判断上次路径中的当前车辆位置

				//根据一次感知地图生成路径，如果是大于一定长度的路径，就用新路径替换旧路径执行。如果不好的路径判断当前车辆执行的位置，如果接近路径终点，按全局路径执行

				//生成路径过程：1、车道线大于一定长度，生成路径
				//              2、小于一定长度，并且到路径执行终点前，采用全局路径，并加上障碍物信息规划路径。

				//获取正前方路点
				//KeepLane();
			
				if(way_out)
				{	
					way_out = false;
					//那些变量置零
					//取路段的起止点
					for(int i = 0;i<200;i++)
					{
						his_MidPoint[i] = cvPoint2D64f(0,0);
					}
					for(int i = 0;i<200;i++)
					{
						MidGpsPoint[i] = cvPoint2D64f(0,0);
					}
				}

				//ArrayFuZhi(m_gps,GPSpoint);


				///修正rndf路网线，加入到keeplane里面，判断长度来修正。
				
				KeepLane(GPSpoint,MidGpsPoint);

				//搜障碍
				ob=SearchObstacle(MidGpsPoint,vel_Map,10/*30*/,1,ob_x,ob_y);
				if(ob)
				{
				//	outn1<<"有章减速"<<"    ";
					on_obs = true;
					
					u_desire = 15;
		
					if(SearchObstacle(MidGpsPoint,vel_Map,15,1))
					{
						ob_count++;
						if(ob_count == 2)
						{
							ob_count = 0;
							
							MoveLeft(MidGpsPoint,LeftPoint,4.5);
							MoveLeft(MidGpsPoint,RightPoint,-4.5);
							int ob_l = SearchObstacle(LeftPoint,vel_Map,15,-5);
							int ob_r = SearchObstacle(RightPoint,vel_Map,15,-5);
							if(ob_l&&ob_r)//左边有障碍物
							{
								outn1<<"有停车"<<"    ";
								//ArrayFuZhi(m_gps,MidGpsPoint);
								Stop(m_gps);
								Sleep(1000);
								break;
							}
							else if(!ob_l)
							{
								outn1<<"左换道"<<"    ";
								ChangeLeft_RightLane(MidGpsPoint,1);
							}
							else
							{
								outn1<<"右换道"<<"    ";
								ChangeLeft_RightLane(MidGpsPoint,-1);
							}
						}
					}
				}
				else
					on_obs = false;
				
				if(app->zuo_huandao)
				{
					ChangeLeft_RightLane(MidGpsPoint,1);//1表示左边换道，-1表示右边换道()
				}
				if(app->you_hundao == true)
				{
					ChangeLeft_RightLane(MidGpsPoint,-1);//1表示左边换道，-1表示右边换道()
				}
				//MoveWay(m_gps,m_gpsdir,GPSpoint);
				//ob=SearchObstacle(GPSpoint,vel_Map,40,5,ob_x,ob_y);
				//if(ob)
				//{
				//	outn1<<"有章减速"<<"    ";
				//	on_obs = true;
				//	u_desire = 10;
				//	//aim_length = 10;
				//	if(SearchObstacle(GPSpoint,vel_Map,25,5))
				//	{
				//		ob_count++;
				//		if(ob_count == 5)
				//		{
				//			ob_count = 0;
				//			if(app->GPS_Speed*3.6 > 15)
				//			{
				//				Stop(app->GPS_Point);
				//				Sleep(3000);
				//			}
				//			MoveLeft(GPSpoint,LeftPoint,4.5);
				//			if(SearchObstacle(LeftPoint,vel_Map,25,-10)||vel_Map->MapPoint[0][0] == 10)//左边有障碍物
				//			{
				//				outn1<<"有停车"<<"    ";
				//				ArrayFuZhi(m_gps,MidGpsPoint);
				//				Stop(m_gps);
				//				Sleep(1000);
				//				break;
				//			}
				//			else
				//			{
				//				outn1<<"有章换道"<<"    ";
				//				//AvoidObstacle(GPSpoint,LeftPoint);
				//				ChangeLeftLane(GPSpoint,LeftPoint);
				//				for(int i = 0;i<200;i++)
				//				{
				//					his_MidPoint[i] = cvPoint2D64f(0,0);
				//				}
				//			}
				//		}
				//	}
				//}
				//
				//else
				//	on_obs = false;
				/*GetLocalTime(&t1);
				outn1<<"liang ci3 "<<t1.wMinute<<","<<t1.wSecond<<","<<t1.wMilliseconds<<endl;*/
				/*dist3 = m_GpsData.GetDistance(m_gps.x,m_gps.y,RNDFGPS.m_mapInfo.pSegment[n_seg].pLane[0].pPoint[n_exit-1].x,RNDFGPS.m_mapInfo.pSegment[n_seg].pLane[0].pPoint[n_exit-1].y);
				if(dist3 < 40)
				{
					lane_out = true;
					n_poi = n_exit;
				}*/
				
				break;
			case IVTASK_CROSS://规划到路口的路径，发送标识识别指令

				//can_speed = false ;
				////标识识别
				////.......分类控制
				////直行
				////左转
				////右转
				////停止线停车
				////获取sign
				//outn1<<"lu kou"<<","<<way_out<<endl;
				int fx=0;
				int ob_front = 8;
				if(way_out)
				{
					way_out = false;
					//outn1<<"在路口"<<"    ";
					for(int i = 0;i<200;i++)
					{
						his_MidPoint[i] = cvPoint2D64f(0,0);
					}
					//规划路径：按当前车辆位置、当前路段方向、目标路段方向、障碍物信息规划路径
					//取路口的参考点

					GetCross_new(seq_num,Apoint,GPSpoint,fx);
					//outn1<<"feng ge xian"<<endl;//////////////////////
					ob_cross=SearchObstacle(GPSpoint,vel_Map,3/*30*/,1);
					//if(!ob_cross)
					//{
					//	int ob_count_count = 0;
					//	ob_front =	8; 
					//}
					//if(ob_cross)
					//{
					////	outn1<<"有章减速"<<"    ";						
					//	u_desire = 15;
					//	if(SearchObstacle(GPSpoint,vel_Map,3,1))
					//	{
					//		ob_count++;
					//		if(ob_count == 2)
					//		{	
					//			ob_count = 0;
					//			Stop(m_gps);
					//			ob_count_count++;
					//			
					//		}
					//		if(ob_count_count == 6)
					//			ob_front =	2;
					//		/*if(ob_count == 10)
					//		{
					//			ob_front =	2;
					//		}*/
					//	}

					//}
					//////////////////////////////
					//if(app->u_t)
					//{
					//	Stop(m_gps);
					//	Sleep(3000);
					//	outn1<<"isUTurn"<<endl;
					//	//app->u_t;
					//	outn1<<"seq num"<<seq_num<<endl;
					//	/*for(int i = 0;i<7;i++)
					//	{
					//		outn1<<"Apoint "<<(Apoint[i].x-31)*100000<<","<<(Apoint[i].y-117)*100000<<endl;
					//	}*/
					//	GetUTurn2(MidGpsPoint,Apoint);
					//	cross_out = true;
					//	app->u_t =0;
					//	outn1<<"U-Turn wan"<<endl;
					//		break;
					//}
			
					MoveWay(m_gps,m_gpsdir,Apoint,7,GPSpoint);
					
				
					ArrayFuZhi(GPSpoint,MidGpsPoint);
				
				
				if(/*1*/LeadPoint[seq_num-2].param2 > 0 /*|| LeadPoint[seq_num-2].param2 == 5*/)
				{
					app->Get_Sign = true;
					Stop(m_gps);//待完善 注意
					Sleep(1000);
					while(1)
					{	
						//fx 1.直行 3。右转 2。左传 
						//sign 的顺序
						//sign[4] = {0,0,0,0};{sign[0]大饼，sign[1]右，sign[2]直，sign[3]左}
						for(int i =0;i<4;i++)
						{
							sign[i] = 0;
						}
						app->Get_Sign = true;
						GetSignal(sign);
						outn1<<"fx  "<<fx<<"  sign  "<<sign[0]<<sign[1]<<sign[2]<<sign[3]<<endl;
						if(sign[1] == 0 &&sign[2] == 0 &&sign[3] == 0 )
						{	
							if(sign[0] != 2)
								break;
						}
						else if(fx == 1)//直行
						{
						
							if(sign[2] != 2)
							break;
							Sleep(20);
						}
						else if(fx == 2)//左转
						{
						
							if(sign[3] != 2)
							break;
							Sleep(20);
						}
						else if(fx == 3)//右转`
						{
						
							if(sign[1] != 2)
							break;
							Sleep(20);
						}
						else
							break;
					}


					

				}
				}

				break;
				
			//case IVTASK_STOPLINE://发送停止线停车指令
			
			//break;
		}
		
		//if(!way_out&&!m)
		//	continue;
		//if(!way_out&&on_uturn)
		//	continue;
		//if(return_value>6000)
		//	continue;
		//if (cross_out||lane_out)
		//{
		//	sendflag = true;
		//	continue;
		//}
		//根据事件进行触发
		//获得各种感知信息，包括RNDF路网信息
		//根据感知信息判断下一步的任务，包括 单路段行驶、停止线停车、路口行驶、变更车道
		//规划路径，并发送更新消息
		cvClearSeq( plan_road );

		for(int i = 0; i<200; i++)
		{
			cvSeqPush( plan_road, &MidGpsPoint[i] );
			
			//outn2<<(GPSpoint[i].x-31)*10000<<","<<(GPSpoint[i].y-117)*10000<<endl;
		}
		//double distance = m_GpsData.GetDistance( m_gps.x, m_gps.y, GPSpoint[199].x, GPSpoint[199].y );
		//if(his_task != m_task||his_seg != n_seg)
		SetEvent(m_hEvent);//路径更新后，发送消息
		
		
		sendflag = true;
		Sleep(10);		


	}
	return 0;
}

void CIVExeNew::StartSendGPS()
{
	m_hThreadSendGPS = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE) CIVExeNew::theSendGPSThread,
		this, 0, &dwSendGPSThreadId);
}

DWORD CIVExeNew::theSendGPSThread(LPVOID lpParam)
{
	return (((CIVExeNew*)lpParam)->SendGPSThread());
}

//综合运行流程
DWORD CIVExeNew::SendGPSThread()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	while(1)
	{
		CString gpstodecision,strAzimuth1,strLatitude,strLongitude,lichen;
		lichen.Format("%f",lichen);
		strAzimuth1.Format("%f",app->GPS_Direction);
		strLatitude.Format("%f",app->GPS_Point.x);
		strLongitude.Format("%f",app->GPS_Point.y);
		gpstodecision =  strAzimuth1+","+strLatitude+ "," + strLongitude+ ";";
		//gps.send_message(gpstodecision);
		send_message(gpstodecision);
		Sleep(100);
	}
	return 0;
}

bool CIVExeNew::Path_Error(CvPoint2D64f GPSpoint[],CvSeq* plan_road)
{
	int check_num = 10;
	int n = 200/check_num;
	int count = n;
	while(count < 150)
	{
		
		double x0 = GPSpoint[count].x;
		double y0 = GPSpoint[count].y;
		CvPoint2D64f pt = *(CvPoint2D64f*)cvGetSeqElem( plan_road, count );
		double x1 = pt.x;
		double y1 = pt.y;
		double dist = m_GpsData.GetDistance(x0,y0,x1,y1);
		if(dist>1)
			return 1;
		count += n;
	}
	return 0;
}
int CIVExeNew::zGetArrTime( CvPoint2D64f m_curgps, double m_direction, CvPoint2D64f m_dstgps, double &m_arrtime)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
    double north_dir = m_GpsData.GetAngle(m_dstgps.x, m_dstgps.y, m_curgps.x, m_curgps.y);
	double head_path_angle = (north_dir - m_direction)*PI/180;
	double dist = m_GpsData.GetDistance( m_curgps.x, m_curgps.y, m_dstgps.x, m_dstgps.y );

	double turn_r = dist/2.0/sin( head_path_angle );
	double length = turn_r * head_path_angle * 2;
	double speed = app->GPS_Speed;
	m_arrtime = length/speed;
	return 0;
}
int CIVExeNew::zGetLatError( CvPoint2D64f m_curgps, double m_direction, CvPoint2D64f m_dstgps, double &m_laterr)
{
	//CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
    double north_dir = m_GpsData.GetAngle(m_dstgps.x, m_dstgps.y, m_curgps.x, m_curgps.y);
	double head_path_angle = (north_dir - m_direction)*PI/180;
	double dist = m_GpsData.GetDistance( m_curgps.x, m_curgps.y, m_dstgps.x, m_dstgps.y );

	m_laterr = dist*sin( head_path_angle );

	return 0;
}
int CIVExeNew::zFilterLatError( double &m_laterr )
{
	//CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	laterr_u[0] = laterr_u[1];
	laterr_u[1] = laterr_u[2];
	laterr_u[2] = m_laterr;

	laterr_y[0] = laterr_y[1];
	laterr_y[1] = laterr_y[2];
	laterr_y[2] = 1.801*laterr_y[1] - 0.819*laterr_y[0] + 0.0045*laterr_u[2] + 0.0091*laterr_u[1] + 0.0045*laterr_u[0];
	
	m_laterr = laterr_y[2];
	return 0;
}
int CIVExeNew::zGetLatSpeed( double m_direction, double &m_latspeed )
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	double north_speed = app->GPS_NorthSpeed;
	double east_speed = app->GPS_EastSpeed;
	double speed = app->GPS_Speed;
	double dir_speed = atan( east_speed/north_speed );
	if(north_speed<0)
		dir_speed += PI;
	double rad_direction = m_direction * PI/180;
	double dir_err = -dir_speed + rad_direction;
	m_latspeed = sin(dir_err) * speed;
	if(speed<0.1&&speed>-0.1)m_latspeed=0;
	return 0;
}

int CIVExeNew::zGetLatSpeed(CvPoint2D64f m_gps, double m_direction,double &m_latspeed )
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	double speed = app->GPS_Speed;
	if(last_gps.x == 0 || (speed<0.3&&speed>-0.3))
	{
		DWORD t = GetTickCount();
		m_latspeed = 0;
		last_gps = m_gps;
		last_dir = m_direction;
		last_t = t;
		return 1;
	}

	double north_dir = m_GpsData.GetAngle(m_gps.x, m_gps.y, last_gps.x, last_gps.y);
	double head_path_angle = (north_dir - last_dir)*PI/180;
	double dist = m_GpsData.GetDistance( m_gps.x, m_gps.y, last_gps.x, last_gps.y );
	
	DWORD t = GetTickCount();
	double m_laterr = dist*sin( head_path_angle );
	double t1 = double(t - last_t)/1000.0;

	m_latspeed = -m_laterr/t1;
	if(dist==0)
		m_latspeed=0;
	last_gps = m_gps;
	last_dir = m_direction;
	last_t = t;

	return 1;
}

int CIVExeNew::zGetTurn(CvPoint2D64f m_gps, double m_direction, CvPoint2D64f m_dstgps,int &turn)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
    double north_dir = m_GpsData.GetAngle(m_dstgps.x, m_dstgps.y, m_gps.x, m_gps.y);
	double head_path_angle = (north_dir - m_direction)*PI/180;
	double dist = m_GpsData.GetDistance( m_gps.x, m_gps.y, m_dstgps.x, m_dstgps.y );
	//前轮转向角
	//double turn_angle = atan( 2.51*sin(head_path_angle)/(dist/2.0 + 1.2*cos(head_path_angle)));//弧度
	PIDREG3 pid1 = PIDREG3_DEFAULTS;
	if(state==1)//直线参数
	{
		pid1.Kp = 0.2; // Pass _iq parameters to pid1
		pid1.Ki = 3; // Pass _iq parameters to pid1
		pid1.Kd = 10; // Pass _iq parameters to pid1
		pid1.Kc = 0;
	}
	if(state==0 && state==2 )
	{
		pid1.Kp = 0.2; // Pass _iq parameters to pid1
		pid1.Ki = 0; // Pass _iq parameters to pid1
		pid1.Kd = 10; // Pass _iq parameters to pid1
		pid1.Kc = 0;
	}

	double m_laterr = 0;
	double m_latspeed = 0;
	zGetLatError(m_gps, m_direction, m_dstgps, m_laterr);
	//zFilterLatError(m_laterr);
	//if(m_laterr<0.5&&m_laterr>-0.5)m_laterr=0;
	SYSTEMTIME t1;
	GetLocalTime(&t1);
	outn7<<"time "<<t1.wHour<<":"<<t1.wMinute<<":"<<t1.wSecond<<","<<t1.wMilliseconds<<"  ";
	outn7<<"速度：　"<<app->GPS_Speed<<"  横向偏差: "<<m_laterr<<"   ";
	zGetLatSpeed(m_direction, m_latspeed );
	outn7<<"(原)横向速度: "<<m_latspeed<<"    ";
	//zGetLatSpeed(m_gps,m_direction, m_latspeed );
	//outn7<<"横向速度: "<<m_latspeed<<"    ";

	pid1.Ref = m_laterr; // Pass _iq inputs to pid1
	pid1.Fdb = 0;//m_latspeed * (dist/app->GPS_Speed);
	pid_reg3_calc(&pid1);
	double a = pid1.Out;
	//head_path_angle = asin(a/dist);
	outn7<<"输出 "<<pid1.Out<<"  "<<"距离  "<<dist<<"  ";

	//转弯半径
	//double turn_r = dist/2.0/sin( head_path_angle );
	double turn_r = 1/(a*2/(dist*dist));
	double curv = a*2/(dist*dist);
	//outn1<<turn_r<<"  车: "<<(m_gps.x-31)*10000<<","<<(m_gps.y-117)*10000<<", "<<m_direction<<"目标  "<<(m_dstgps.x-31)*10000<<","<<(m_dstgps.y-117)*10000<<", "<<north_dir<<" 夹角 "<<head_path_angle*180/PI<<endl;
	//turn = curv*2098728-5541;
	outn7<<"半径 "<<turn_r<<"      "<<endl;
	int tc;
	if ( !GetTencaleByVr(turn_r, tc) )
		tc = 0;
	turn = tc;
	return 0;
}

int CIVExeNew::zGetThrottle(double speed,int &throttle,int &brake)
{

	if ((speed -u_desire)>=0.5)//过程值大于设定值1 
	{
		PIDREG3 pid2 = PIDREG3_DEFAULTS;
		pid2.Kp=120;
		pid2.Ki=30;
		pid2.Kd=30;
		pid2.Kc=0.03;
		pid2.OutMax=0;//输出最大值
		pid2.OutMin=-17500;//输出最小值
		pid2.Ref=u_desire;//过程值
		pid2.Fdb=speed;//反馈值
		pid_reg3_calc(&pid2);
		int a1 = pid2.Out-25500;
		double a2 = 0.37; 
		brake = a1;
		throttle = a2*100;

	 } 
	 else if ((speed -u_desire)<=-0.5)           
	 {
		PIDREG3 pid2=PIDREG3_DEFAULTS;
		if (abs(speed-u_desire)<=3)
		{
			pid2.Kp=0.125;
			pid2.Ki=0.06;
			pid2.Kd=2;
		} 
		/*else if (abs(speed-u_desire)>3 && abs(speed-u_desire)<=6)
		{
			pid2.Kp=0.03;
			pid2.Ki=0.0;
			pid2.Kd=3;
		} 	*/	   
		else if (abs(speed-u_desire)>3 )
		{ 
			pid2.Kp=0.05;
			pid2.Ki=0.0;
			pid2.Kd=6; 
		} 		   
		pid2.Kc=0.03;
		pid2.OutMax=0.45;//输出最大值
		pid2.OutMin=0;//输出最小值
		pid2.Ref=u_desire;//过程值
		pid2.Fdb=speed;//反馈值
		pid_reg3_calc(&pid2);
		double a4 = pid2.Out+0.4;
		brake = -25500;
		throttle = a4*100;

	 } 
		else
		{
			double a7;
			if (u_desire >= 10)
				a7 = 0.5;
			else
				a7=0.4;
			brake = -25500;
			throttle = a7*100;
		}
	
	return 0;
}

int CIVExeNew::zStop( )
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	double speed;
	double dist;

	CvPoint2D64f m_stop;
	m_stop = app->GPS_Point;

	double di = 0;
	
	while(1)
	{
		
		//GetLocalTime(&t1);
		CvPoint2D64f m_gps = app->GPS_Point;
		double m_direction = app->GPS_Direction;
		speed = app->GPS_Speed;
		//outn2<<"速度, "<<speed<<", ";
		di = (app->cardr + app->cardr)/2;

		double north_dir = m_GpsData.GetAngle(m_end.x, m_end.y, m_gps.x, m_gps.y);
		double head_path_angle = (north_dir - m_direction)*PI/180;


		//dist = 10-di;
		dist = m_GpsData.GetDistance( m_gps.x, m_gps.y, m_end.x, m_end.y );
		dist = dist*cos( head_path_angle );
		//outn2<<"距离, "<<dist<<", ";
		if(dist < 0.45|| ( (head_path_angle < -135) || (head_path_angle > 135) ))
			return 0;
		

		int throttle = 37,brake = -25500;

		if(speed>1)
		{
			brake = -43000;
			
			m_stop = app->GPS_Point;
		}
		else	
		{
			
			double dist3 = m_GpsData.GetDistance( m_gps.x, m_gps.y, m_stop.x, m_stop.y );
			//outn2<<"距离3, "<<dist3<<", ";

			//double out = dist-speed*t;
			
			PIDREG3 pid2 = PIDREG3_DEFAULTS;
			pid2.Kp=0.15;
			pid2.Ki=0;
			pid2.Kd=0;
			pid2.Kc=0.03;
			pid2.OutMax=30;//输出最大值
			pid2.OutMin=-30;//输出最小值
			pid2.Ref=dist;//过程值
			pid2.Fdb=0;//反馈值
			pid_reg3_calc(&pid2);
			double out = pid2.Out;
			
			double a = (speed*speed)/(2*out);
			if(a>1.5)a=1.5;
			if(a<0)a=0;

			//outn2<<"a, "<<a<<", ";
			brake = 10000*(a + 5.8)/(-1.6445);
			//outn2<<"brake, "<<brake<<", ";
			if(brake>-36000)brake = -35000;


			if(brake<-46000)brake = -46000;
			if(brake>-25500)brake = -25500;
			
		}
		

		//m_ctrl.turn = tc;
		//outn1<<"turn  "<<m_ctrl.turn<<"  tc "<<tc<<"  last  "<<lastturn<<endl;
		//lastturn = m_ctrl.turn;

		//int a = zGetTurn(m_gps,m_direction,m_end,m_ctrl.turn);
		m_ctrl.turn = 0;
		m_ctrl.velocity = 5;
		m_ctrl.accelerator = 37;
		m_ctrl.brake = brake;
		//m_ctrl.horn = 0;
		//m_ctrl.key = 0;
		//m_ctrl.light = 0;
		m_ctrl.mswitch = 3;

		//CString str;
		//str.Format("%sG100T%dV%dB%dS%dK%dA%dH%dL%sEND;","%",m_ctrl.turn,m_ctrl.velocity,m_ctrl.brake,m_ctrl.mswitch,0,m_ctrl.accelerator,0,0);

		//const char* sttrr=str.GetString();
		//sttrr=str.GetString();
		//if(send.SuccSocket)
		//	send.ClientSend(sttrr);
	
		Sleep(70);
	}
	return 0;
}
int CIVExeNew::zGetAimLen(double speed)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	
	//aim_length = 0.5*speed*3.6 + 5;
	if(on_obs)
	{
		return 0; 
	}
	if(app->u_t)
	{
		aim_length = 8;
		u_desire = 5;
		return 0;
	}
	if(app->onroad)
	{
		aim_length = 20;
		u_desire = 35;
		return 0;
	}
	if(!app->onroad)	
	{
		aim_length = 15;
		u_desire = 15;
		return 0;
	}

	//u_desire = 30;


	//double u;
	//double u_s;
	//
	//if(app->state1 == 1)
	//{
	//	u = 20;
	//	u_s = 30;
	//}
	//else if(app->state1 == 2)
	//{
	//	u = 13;
	//	u_s = 25;
	//}
	//else
	//{
	//	u = 10;
	//	u_s = 20;
	//}
	//aim_length = 0.9355*lastaim_length + 0.03225*u + 0.03225 *lastu;
	//lastaim_length = aim_length;
	//lastu = u;
	//outn8<<"预瞄 "<<aim_length;
	//
	//u_desire = 0.8824*lastaim_s + 0.05882*u_s + 0.05883 *lasts;
	//lastaim_s = u_desire;
	//lasts = u_s;
	////outn8<<"  预瞄u "<<u_desire<<endl;
	//
	return 0;
	
}


int CIVExeNew::GetMiddleString(CString &string,const char * firststr, const char * endstr,CString &conResult )
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
void CIVExeNew::send_message(CString buf)
{
   CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	 
	
	int datalen = strlen(buf)+1;	
	lcm_publish(lcm_send,"lichen",buf,datalen);
}
//int CIVExeNew::KeepLane(CvPoint2D64f *Rndf_MidPoint,CvPoint2D64f (&MidGpsPoint)[200])
//{
//	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
//	//init
//	int search_res  = 0;
//	//CvPoint2D64f V_StartP = cvPoint2D64f(256,412);
//	//
//	//while(!app->mapreceived)
//	//{
//	//	Sleep(5);
//	//	
//	//}
//	//app->critical_map.Lock();
//	//vel_Map = app->PercepMap;
//	//app->critical_map.Unlock();
//
//	//	//GetLaneMap(vel_Map,lane_Map);	
//	//	//ProcCenterLane(lane_Map,mid_Map);
//	//	ProcCenterLane(vel_Map,mid_Map);	
//		app->critical_section.Lock();
//	//	vel_Map = app->PercepMap;
//		rPosition = app->GPS_Point;
//		rDirection = app->GPS_Direction;
//		app->critical_section.Unlock();
//		//ArrayFuZhi(V_StartP,MidPoint);	
//		//outn1<<"meizou"<<endl;
//		int iRet = WaitForSingleObject(rec_map.m_MapEvent,100);
//
//		//取得消息
//		if( iRet == 0)
//		{
//			//cvClearSeq( seq_road  );
//			 
//			search_res = keeplane.SearchMidLane(mid_Map,MidPoint,1);
//			/*for(int i = 0;i<200;i++)
//			{
//				outn2<<MidPoint[i].x<<","<<MidPoint[i].y<<endl;
//			}*/
//			//search_res = keeplane.SearchOneMidLane(mid_Map,MidPoint,1);
//			
//			
//			s_old = m_GpsData.GetDistance(rPosition.x,rPosition.y,his_MidPoint[199].x,his_MidPoint[199].y);
//			if(his_MidPoint[199].x==0)
//				s_old = 0;
//			if(search_res != 2)
//			{
//				GetSendGps(rPosition,rDirection,MidPoint,MidGpsPoint);
//				s_new = m_GpsData.GetDistance(rPosition.x,rPosition.y,MidGpsPoint[199].x,MidGpsPoint[199].y);
//				if(s_old > s_new)
//					ArrayFuZhi(his_MidPoint,MidGpsPoint);
//				else
//					ArrayFuZhi(MidGpsPoint,his_MidPoint);
//				return 1;
//			}
//		
//			if(search_res == 2 && s_old < 15)
//			{
//				//AfxMessageBox("zou lu wang");
//			//	ArrayFuZhi(his_MidPoint,MidGpsPoint);
//				ArrayFuZhi(Rndf_MidPoint,MidGpsPoint);
//				for(int i = 0;i<200;i++)
//				{
//					his_MidPoint[i] = cvPoint2D64f(0,0);
//				}
//				return 1;
//			}
//		}
//		//else
//		//{
//		//	s_old = m_GpsData.GetDistance(rPosition.x,rPosition.y,his_MidPoint[199].x,his_MidPoint[199].y);
//		//	if(his_MidPoint[199].x==0)
//		//		s_old = 0;
//		//	if(s_old < 15)
//		//	{
//		//		//AfxMessageBox("zou lu wang");
//		//	//	ArrayFuZhi(his_MidPoint,MidGpsPoint);
//		//		ArrayFuZhi(Rndf_MidPoint,MidGpsPoint);
//		//		for(int i = 0;i<200;i++)
//		//		{
//		//			his_MidPoint[i] = cvPoint2D64f(0,0);
//		//		}
//		//		return 1;
//		//	}
//		//}
//		
//	return 0;
//	
//}
int CIVExeNew::KeepLane(CvPoint2D64f *Rndf_MidPoint,CvPoint2D64f (&MidGpsPoint)[200])
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	//init
	//int search_res  = 0;
	////CvPoint2D64f V_StartP = cvPoint2D64f(256,412);
	////
	////while(!app->mapreceived)
	////{
	////	Sleep(5);
	////	
	////}
	////app->critical_map.Lock();
	////vel_Map = app->PercepMap;
	////app->critical_map.Unlock();

	////	//GetLaneMap(vel_Map,lane_Map);	
	////	//ProcCenterLane(lane_Map,mid_Map);
	////	ProcCenterLane(vel_Map,mid_Map);	
	//	app->critical_section.Lock();
	////	vel_Map = app->PercepMap;
	//	rPosition = app->GPS_Point;
	//	rDirection = app->GPS_Direction;
	//	app->critical_section.Unlock();
	//	//ArrayFuZhi(V_StartP,MidPoint);	
	//	//outn1<<"meizou"<<endl;
	//	int iRet = WaitForSingleObject(app->m_MapEvent,100);

	//	//取得消息
	//	if( iRet == 0)
	//	{			 
	//		
	//		search_res = keeplane.SearchMidLane(mid_Map,MidPoint);
	//		YanChang(MidPoint,20,shift_MidPoint);
	//		s_old = m_GpsData.GetDistance(rPosition.x,rPosition.y,his_MidPoint[199].x,his_MidPoint[199].y);
	//		if(his_MidPoint[199].x==0)
	//			s_old = 0;
	//		if(search_res != 2)
	//		{
	//			//GetSendGps(rPosition,rDirection,MidPoint,MidGpsPoint);
	//			GetSendGps(rPosition,rDirection,shift_MidPoint,MidGpsPoint);
	//			s_new = m_GpsData.GetDistance(rPosition.x,rPosition.y,MidGpsPoint[199].x,MidGpsPoint[199].y);
	//			if(s_old > s_new)
	//				ArrayFuZhi(his_MidPoint,MidGpsPoint);
	//			else
	//				ArrayFuZhi(MidGpsPoint,his_MidPoint);
	//			return 1;
	//		}
	//	}
	return 0;
	
}
int CIVExeNew::GetLaneMap(I_Map *vel_Map,I_Map *lane_Map)
{
	for( int i=0; i<512; i++)
	{
		for( int j=0; j<512; j++)
		{
			lane_Map->MapPoint[i][j] = vel_Map->MapPoint[i][j];
			if(lane_Map->MapPoint[i][j] == 9)
			lane_Map->MapPoint[i][j] = 0;
			
		}
	}
	return 0;
}
void CIVExeNew::ProcCenterLane(I_Map *map_src,I_Map *map_dst)
{
	IplImage *src_tmp = cvCreateImage(cvSize(512, 512), 8, 1);
//	memcpy( src_tmp->imageData, (uchar *)map_src->MapPoint, 512*512*sizeof(uchar) );
	
	
	//读取图像数据,i-列坐标----x，j行坐标-----y
	for(int j = 0;j<512;j++)
	{for(int i = 0;i<512;i++)
		{
			//a = ((uchar*)(frameImage->imageData + frameImage->widthStep*j))[i];
			//out4<<(int)a;
			if(map_src->MapPoint[j][i] == 1)
			{
				((uchar*)(src_tmp->imageData + src_tmp->widthStep*j))[i] = 255;
			}
			else
			{
				((uchar*)(src_tmp->imageData + src_tmp->widthStep*j))[i] = 0;
			}
		}

	}
		/*for(int i = 0;i<512;i++)
		{
			for(int j = 0;j<512;j++)
				out4<<map_src->MapPoint[i][j];
			out4<<endl;
		}*/
	//cvNameWindow("aa",1);
	//cvShowImage( "aa", src_tmp ); cvWaitKey();

	zFitLanesCenter(src_tmp, src_tmp);
	//cvShowImage( "aa", src_tmp ); cvWaitKey();
	
	//读取图像数据,i-列坐标----x，j行坐标-----y
	for(int j = 0;j<512;j++)
	{for(int i = 0;i<512;i++)
		{
			uchar a = ((uchar*)(src_tmp->imageData + src_tmp->widthStep*j))[i];
			//out4<<(int)a;
			if(a==255)
			{
				map_dst->MapPoint[j][i] = 1;
			}
			else
			{
				map_dst->MapPoint[j][i] = 0;
			}
			//out4<<map_dst->MapPoint[j][i];
		}
		//out4<<endl;
	}
	/*for(int j = 0;j<512;j++)
	{	
		for(int i = 0;i<512;i++)
		{
			if(map_dst->MapPoint[j][i] == 1)

		}
	}*/
	cvReleaseImage(&src_tmp);
}
void CIVExeNew::zFitLanesCenter(IplImage *src, IplImage *dst)
{
	if(!src) return;
	IplImage *tmpsrc = cvCloneImage(src);
	cvThreshold( tmpsrc, tmpsrc, 0, 255, CV_THRESH_BINARY_INV);
	IplImage* tmpdist = cvCreateImage( cvGetSize(tmpsrc), IPL_DEPTH_32F,1);
	cvDistTransform(tmpsrc,tmpdist,CV_DIST_L2,3);
	IplImage* tmp = cvCreateImage( cvGetSize(tmpsrc), 8,1);
	IplImage* tmp1 = cvCreateImage( cvGetSize(tmpsrc), 8,1);
	IplImage* tmp_dist = cvCreateImage( cvGetSize(tmpsrc), 8,1);
	cvConvertScaleAbs( tmpdist, tmp_dist );
//	cvSaveImage("aa.jpg",tmp_dist);
//	cvSaveImage("bb.bmp",tmp_dist);
 	cvInRangeS(tmp_dist, cvScalarAll(0), cvScalarAll(40), tmp);
	cvLaplace(tmpdist,tmpdist,3); 
	cvZero(tmp1);
	cvConvertScaleAbs( tmpdist, tmp1 );
	cvThreshold(tmp1, tmp1, 6, 255, CV_THRESH_BINARY);//0.
	cvZero(dst);
	cvCopy( tmp1, dst, tmp );
	cvReleaseImage(&tmpdist);
	cvReleaseImage(&tmpsrc);
	cvReleaseImage(&tmp);
	cvReleaseImage(&tmp1);
	cvReleaseImage(&tmp_dist);
}
void CIVExeNew::ArrayFuZhi(CvPoint2D64f a,CvPoint2D64f *b)
{
	for(int i = 0;i<200;i++)
	{
		b[i].x = 0;
		b[i].y = 0;
		b[i].x = b[i].x + a.x;
		b[i].y = b[i].y + a.y;
	}
}
void CIVExeNew::ArrayFuZhi(CvPoint2D64f *a,CvPoint2D64f *b)
{
	for(int i = 0;i<200;i++)
	{
		b[i].x = 0;
		b[i].y = 0;
		b[i].x = a[i].x;
		b[i].y = a[i].y;
	}
}
int CIVExeNew::GetSendGps(CvPoint2D64f m_gps,double m_gpsdir,CvPoint2D64f *MidPoint,CvPoint2D64f (&MidGpsPoint)[200])
{
	MidGpsPoint[0] = m_gps;
	for(int i = 1;i<200;i++)
		{
			MidGpsPoint[i] = m_GpsData.MaptoGPS(m_gps,m_gpsdir,MidPoint[i]);
		}
	return 0;
}
int CIVExeNew::GetLanePoint(int n_seg, int n_poi,int &exit_id,CvPoint2D64f WayPoint[],int &n)
{


	//CvPoint2D64f WayPoint[10];//选出的路点
	
	int temp = 9999;

	int wp_id = 0;

	int wp_num;//选出路点个数

	int nwaypoint = RNDFGPS.m_mapInfo.pSegment[n_seg].pLane[0].waypoints_num;//路点
	int nexitpoint = RNDFGPS.m_mapInfo.pSegment[n_seg].pLane[0].exit_num;//退出点



	for ( int i=0; i<nexitpoint; i++)
	{
		int id = RNDFGPS.m_mapInfo.pSegment[n_seg].pLane[0].pExit[i].exit_Id;
		//double wx0 = RNDFGPS.m_mapInfo.pSegment[n_seg].pLane[0].pCharpoint[i].x;
		//double wy0 = RNDFGPS.m_mapInfo.pSegment[n_seg].pLane[0].pCharpoint[i].y;

		//double dist = m_GpsData.GetDistance(wx0,wy0,wx1,wy1);
		if(id>=n_poi&&id<temp)
		{
			temp = id;
			
		}

	}
	exit_id = temp;
	//根据特征点判断是在路上还是需识别标识

	for(int k = n_poi; k<nwaypoint; k++)
	{
		WayPoint[k-n_poi].x = RNDFGPS.m_mapInfo.pSegment[n_seg].pLane[0].pPoint[k].x;
		WayPoint[k-n_poi].y = RNDFGPS.m_mapInfo.pSegment[n_seg].pLane[0].pPoint[k].y;
	}
	n = exit_id - n_poi;
	return 1;
}	
int CIVExeNew::GetLane(int &n_seg, int &n_poi,int exit_id, CvPoint2D64f WayPoint[],int n,CvPoint2D64f (&Point)[200])
{
	//LocalPath.BSpline(WayPoint,n+2,Point);//生产连续路径
	////LocalPath.threeSpline(WayPoint,n,Point);//生产连续路径
	////for(int i=0;i<n;i++)
	////{
	////	outn1<<(WayPoint[i].x-31)*100000<<", "<<(WayPoint[i].y-117)*100000<<endl;
	////}
	////outn1<<"华丽的分割县"<<endl;
	////for(int i=0;i<200;i++)
	////{
	////	outn1<<(Point[i].x-31)*100000<<", "<<(Point[i].y-117)*100000<<endl;
	////}
	////outn1<<"华丽的分割县"<<endl;
	////去掉距特征点10m内的点
	//for(int i = 0; i<200; i++)
	//{
	//	//double wx0 = RNDFGPS.m_mapInfo.pSegment[n_seg].pLane[0].pCharpoint[char_id].x;
	//	//double wy0 = RNDFGPS.m_mapInfo.pSegment[n_seg].pLane[0].pCharpoint[char_id].y;
	//	double wx0 = RNDFGPS.m_mapInfo.pSegment[n_seg].pLane[0].pPoint[exit_id].x;
	//	double wy0 = RNDFGPS.m_mapInfo.pSegment[n_seg].pLane[0].pPoint[exit_id].y;
	//	double wx1 = Point[i].x;
	//	double wy1 = Point[i].y;

	//	double alph = m_GpsData.GetAngle(wx0,wy0,wx1,wy1) - RNDFGPS.GetLaneDir(n_seg+1,1);
	//	double dist = m_GpsData.GetDistance(wx0,wy0,wx1,wy1);
	//	dist = dist * cos(alph*PI/180.0);
	//	
	//	if( abs(dist)<17 )
	//		Point[i] = Point[i-1];
	//}
	
	return 1;
}
int CIVExeNew::PassCheckPoint(CvPoint2D64f m_gps)
{
	int chk_id = MDF.Check_Points.front();
	CvPoint2D64f chk_pt;
	RNDFGPS.GetCheckPoint(chk_id,chk_pt);
	double dist = m_GpsData.GetDistance(m_gps.x,m_gps.y,chk_pt.x,chk_pt.y);
	if(dist<10)
	{
		MDF.Check_Points.pop();
		return 1;
	}
	return 0;
}
int CIVExeNew::PassMissionPoint(CvPoint2D64f m_gps,double dir)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	if(pass_count>0)
	{
		pass_count--;
		return 2;
	}
	LEAD cur_lead = LeadPoint[seq_num-1];
	CvPoint2D64f leadpt;
	leadpt.x=cur_lead.lat;
	leadpt.y=cur_lead.lng;
	
	if(cur_lead.param1 == 0||cur_lead.param1 == 1||cur_lead.param1 == 3)
		m_task = IVTASK_LANE;
	else if(cur_lead.param1 == 2)
		m_task = IVTASK_CROSS;
	double dist_vert = VertDist(m_gps,dir,leadpt);
	double dist = m_GpsData.GetDistance(m_gps.x,m_gps.y,leadpt.x,leadpt.y);
	if(dist_vert</*15*/10&&abs(dist)<50)
	{
		if(cur_lead.param2==4||cur_lead.param2==5.4||cur_lead.param2==0.4)
			app->u_t=true;
		if(cur_lead.param1 == 3)
			app->stop_lane = true;
		pass_count=50;
		outaaa<<"经过的点序号: "<<seq_num<<endl;
		seq_num++;
		way_out = true;
		cur_lead = LeadPoint[seq_num-1];
		if(cur_lead.param1 == 0||cur_lead.param1 == 1||cur_lead.param1 == 3)
			m_task = IVTASK_LANE;
		else if(cur_lead.param1 == 2&&LeadPoint[seq_num-2].param1==1)
			m_task = IVTASK_CROSS;
		else
			m_task = IVTASK_LANE;
		return 1;
	}

	return 0;
}
int CIVExeNew::GetCrossPoint(CvPoint2D64f cur_gps,double cur_direction,int &next_seg, int &n_poi,CvPoint2D64f WayPoint[])
{
	CvPoint2D64f NextPoint0[3],NextPoint1[3];
	int Nexthead[3];
	CvPoint2D64f ConnectPoint;
	//CvPoint2D64f WayPoint[6];
	
	
	
	NextPoint1[0].x = RNDFGPS.m_mapInfo.pSegment[next_seg].pLane[0].pPoint[0].x;
	NextPoint1[0].y = RNDFGPS.m_mapInfo.pSegment[next_seg].pLane[0].pPoint[0].y;

	NextPoint1[1].x = RNDFGPS.m_mapInfo.pSegment[next_seg].pLane[0].pPoint[1].x;
	NextPoint1[1].y = RNDFGPS.m_mapInfo.pSegment[next_seg].pLane[0].pPoint[1].y;
		

	//按路点取最后两点


	WayPoint[0].x = RNDFGPS.m_mapInfo.pSegment[n_seg].pLane[0].pPoint[n_poi-2].x;
	WayPoint[0].y = RNDFGPS.m_mapInfo.pSegment[n_seg].pLane[0].pPoint[n_poi-2].y;
	

	WayPoint[1].x = RNDFGPS.m_mapInfo.pSegment[n_seg].pLane[0].pPoint[n_poi-1].x;
	WayPoint[1].y = RNDFGPS.m_mapInfo.pSegment[n_seg].pLane[0].pPoint[n_poi-1].y;
	


	WayPoint[5] = NextPoint1[0];
	WayPoint[6] = NextPoint1[1];

	//求两端路的交点
	double x1 = WayPoint[0].x;
	double y1 = WayPoint[0].y;
	double x2 = WayPoint[1].x;
	double y2 = WayPoint[1].y;
	double A1 = y2-y1;
	double B1 = -(x2-x1);
	double C1 = x2*y1-x1*y2;

	x1 = WayPoint[6].x;
	y1 = WayPoint[6].y;
	x2 = WayPoint[5].x;
	y2 = WayPoint[5].y;
	double A2 = y2-y1;
	double B2 = -(x2-x1);
	double C2 = x2*y1-x1*y2;

	WayPoint[3].x = (B1*C2-B2*C1)/(A1*B2-A2*B1);
	WayPoint[3].y = (A1*C2-A2*C1)/(A2*B1-A1*B2);

	WayPoint[2].x = (WayPoint[1].x+4*WayPoint[3].x)/5;
	WayPoint[2].y = (WayPoint[1].y+4*WayPoint[3].y)/5;
	WayPoint[4].x = (WayPoint[5].x+4*WayPoint[3].x)/5;
	WayPoint[4].y = (WayPoint[5].y+4*WayPoint[3].y)/5;
	
	


	n_seg = next_seg;
	n_poi = 0;

	return 1;
}

int CIVExeNew::GetCross(CvPoint2D64f WPoint[], CvPoint2D64f (&Point)[200])
{
//	LocalPath.Bezier(WPoint,6,Point);
	//Point
//for(int i=0;i<7;i++)
//outn1<<(WPoint[i].x-31)*100000<<","<<(WPoint[i].y-117)*100000<<endl;
//outn1<<"fengge xian"<<endl;
//for(int i = 0;i<200;i++)
//outn1<<(Point[i].x-31)*100000<<","<<(Point[i].y-117)*100000<<endl;
//outn1<<"fenge xian"<<endl;
	return 1;
}

int CIVExeNew::GetSegmentNumberByGPS( CLoadRNDF RNDF, double wd, double jd, int &n_seg )
{
	int nsegment = RNDF.m_mapInfo.segment_num;

	double *dist_segment;
	dist_segment = (double *)calloc(nsegment, sizeof(double));

	for(int i=0; i<nsegment; i++)//每段
	{
		int nlane = RNDF.m_mapInfo.pSegment[i].lane_num;

		//每条路上取最短值
		//面积公式：area=根号下s(s-a)(s-b)(s-c).其中s=(a+b+c)/2
		//垂线长度：area/c/2;
		double min_dist = 999999;
		for(int j=0; j<nlane; j++)//每条路
		{
			int nwaypoint = RNDF.m_mapInfo.pSegment[i].pLane[j].waypoints_num;//路点
			

			//每条路上的相邻两点连线，到当前点的距离
			//先求三边边长
			for (int k1=0; k1<nwaypoint-1; k1++)
			{
				double wx0 = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1].x;
				double wy0 = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1].y;

				double wx1 = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1+1].x;
				double wy1 = RNDF.m_mapInfo.pSegment[i].pLane[j].pPoint[k1+1].y;

				//计算点到路段距离
				double dd = pointToLine(wx0, wy0, wx1, wy1, wd, jd);

				if(min_dist > dd)
				{
					min_dist = dd;
				}

			}

		}
		dist_segment[i] = min_dist;
	}

	int min_n_seg=0;
	double min_dist = 999999;
	for(int i=0; i<nsegment; i++)//每段
	{
		if( dist_segment[i] < min_dist )
		{
			min_dist = dist_segment[i];
			min_n_seg = i;
		}
	}

	delete []dist_segment;

	n_seg = min_n_seg;

	return min_n_seg;
}

double CIVExeNew::pointToLine(double x1, double y1, double x2, double y2, double x0, double y0) 
{
	double space = 0;
	double a, b, c;
	//a = lineSpace(x1, y1, x2, y2);// 线段的长度
	//b = lineSpace(x1, y1, x0, y0);// (x1,y1)到点的距离
	//c = lineSpace(x2, y2, x0, y0);// (x2,y2)到点的距离
	//if (c <= 0.000001 || b <= 0.000001) {
	//	space = 0;
	//	return space;
	//}
	//if (a <= 0.000001) {
	//	space = b;
	//	return space;
	//}
	//if (c * c >= a * a + b * b) {
	//	space = b;
	//	return space;
	//}
	//if (b * b >= a * a + c * c) {
	//	space = c;
	//	return space;
	//}
	//double p = (a + b + c) / 2;// 半周长
	//double s = sqrt(p * (p - a) * (p - b) * (p - c));// 海伦公式求面积
	//space = 2 * s / a;// 返回点到线的距离（利用三角形面积公式求高）
	//return space;
	return 0;
}
int CIVExeNew::GetUTurn(CvPoint2D64f GpsPoint[],CvPoint2D64f WPoint[])
{
	//先发一个20m的路径
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	//app->u_t = true;
	//I_Map *map;
	//map = new I_Map;
	CvPoint2D64f uturn[11] ={0};
	CvPoint2D64f m_gps;
	double m_gpsdir;
	int end_num =0;
	double dis = 0;

	m_gps = app->GPS_Point;

	CvPoint2D64f firstpoint[3];
	CvPoint2D64f firstway[200];
	//firstpoint[0] = m_gps;
	firstpoint[0] = WPoint[0];
	firstpoint[1] = WPoint[1];

	//LocalPath.Bezier(firstpoint,1,firstway);
	//outn1<<1<<endl;
	for( int i=199; i>=0; i-- )
	{
		
		dis = m_GpsData.GetDistance(m_gps.x,m_gps.y,GpsPoint[i].x,GpsPoint[i].y);
		if(dis < 35)
		{
			end_num = i;
			break;
		}
	}
	cvClearSeq( plan_road );

	for(int i = 0; i<end_num; i++)
	{
		cvSeqPush( plan_road, &GpsPoint[i] );
	}

	SetEvent(m_hEvent);//路径更新后，发送消息
	outn1<<2<<endl;
	sendflag = true;
	while(1)
	{
		outn1<<3<<endl;
		m_gps = app->GPS_Point;
		dis = m_GpsData.GetDistance(m_gps.x,m_gps.y,GpsPoint[end_num-1].x,GpsPoint[end_num-1].y);
		if(dis<10)
			break;
		Sleep(20);
	}
	
	cvClearSeq( plan_road );

	for(int i = 0; i<200; i++)
	{
		cvSeqPush( plan_road, &m_gps);
	}

	SetEvent(m_hEvent);//路径更新后，发送消息
	Sleep(3000);

	double waydir = m_GpsData.GetAngle(WPoint[1].x,WPoint[1].y,WPoint[0].x,WPoint[0].y);
	Map_Point MidPoint = {0};
	while(1)
	{
		outn1<<4<<endl;
		Sleep(200);

		app->critical_map.Lock();
		vel_Map = app->PercepMap;
		app->critical_map.Unlock();
		m_gps = app->GPS_Point;
		m_gpsdir = app->GPS_Direction;

		double k = waydir - m_gpsdir;

	
		if(CountUTurn(vel_Map,k,MidPoint))
			break;

	}

	m_gps = app->GPS_Point;
	m_gpsdir = app->GPS_Direction;
	CvPoint2D64f MidPoint1 = cvPoint2D64f((double)MidPoint.x,(double)MidPoint.y);
	CvPoint2D64f MidGps = m_GpsData.MaptoGPS(m_gps,m_gpsdir,MidPoint1);

	CvPoint2D64f chuizu1,chuizu2;
	double x1 = WPoint[0].x;
	double y1 = WPoint[0].y;
	double x2 = WPoint[1].x;
	double y2 = WPoint[1].y;
	double A = y2-y1;
	double B = -(x2-x1);
	double C = x2*y1-x1*y2;


	double x0 = (B*B*MidGps.x - A*B*MidGps.y - A*C)/(A*A + B*B);
	double y0 = (-A*B*MidGps.x + A*A*MidGps.y - B*C)/(A*A + B*B);
	chuizu1.x = x0;
	chuizu1.y = y0;
	
	
	x1 = WPoint[5].x;
	y1 = WPoint[5].y;
	x2 = WPoint[6].x;
	y2 = WPoint[6].y;
	A = y2-y1;
	B = -(x2-x1);
	C = x2*y1-x1*y2;


	x0 = (B*B*MidGps.x - A*B*MidGps.y - A*C)/(A*A + B*B);
	y0 = (-A*B*MidGps.x + A*A*MidGps.y - B*C)/(A*A + B*B);
	chuizu2.x = x0;
	chuizu2.y = y0;

	uturn[0].x =  2*WPoint[1].x - chuizu1.x;
	uturn[0].y =  2*WPoint[1].y - chuizu1.y;
	uturn[1] = WPoint[1];
	uturn[2] = chuizu1;
	uturn[3].x = (MidGps.x + 2*chuizu1.x)/3.;
	uturn[3].y = (MidGps.y + 2*chuizu1.y)/3.;
	uturn[4].x = (2*MidGps.x + chuizu1.x)/3.;
	uturn[4].y = (2*MidGps.y + chuizu1.y)/3.;
	uturn[5] = MidGps;
	uturn[6].x = (MidGps.x + 2*chuizu2.x)/3.;
	uturn[6].y = (MidGps.y + 2*chuizu2.y)/3.;
	uturn[7].x = (2*MidGps.x + chuizu2.x)/3.;
	uturn[7].y = (2*MidGps.y + chuizu2.y)/3.;
	uturn[8] = chuizu2;
	uturn[9] = WPoint[5];
	uturn[10].x =  2*WPoint[6].x - chuizu2.x;
	uturn[10].y =  2*WPoint[6].y - chuizu2.y;



//	LocalPath.Bezier(uturn,10,firstway);



	cvClearSeq( plan_road );

	for(int i = 0; i<200; i++)
	{
		cvSeqPush( plan_road, &firstway[i] );
	}

	SetEvent(m_hEvent);//路径更新后，发送消息
	int countdd = 0;
	double angle = m_GpsData.GetAngle(WPoint[6].x,WPoint[6].y,WPoint[5].x,WPoint[5].y);
   while(1)
   {   
	   outn1<<5<<endl;
		m_gps = app->GPS_Point;
		m_gpsdir = app->GPS_Direction;
		//double dd = m_GpsData.GetDistance(m_gps.x,m_gps.y,WPoint[4].x,WPoint[4].y);
		if(abs(m_gpsdir - angle) < 20 || abs(360 - abs(m_gpsdir - angle)) < 20)
			break ;
		/*if(dd < 5)
			break;*/
		if(countdd > 600)
		{
			break;
		}
		Sleep(100);
		countdd++;
   }
   
   	cvClearSeq( plan_road );

	for(int i = 0; i<200; i++)
	{
		cvSeqPush( plan_road, &m_gps);
	}

	SetEvent(m_hEvent);//路径更新后，发送消息
	Sleep(3000);
	return 1;
}

int CIVExeNew::GetUTurn2(CvPoint2D64f GpsPoint[],CvPoint2D64f WPoint[])
{
	//先发一个20m的路径
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	//app->u_t = true;
	//I_Map *map;
	//map = new I_Map;
	CvPoint2D64f uturn[11] ={0};
	CvPoint2D64f m_gps;
	double m_gpsdir;
	int end_num =0;
	double dis = 0;

	m_gps = app->GPS_Point;

	CvPoint2D64f firstpoint[3];
	CvPoint2D64f firstway[200];
	//firstpoint[0] = m_gps;
	firstpoint[0] = WPoint[0];
	firstpoint[1] = WPoint[1];

	//LocalPath.Bezier(firstpoint,1,firstway);
	//outn1<<1<<endl;
	

	cvClearSeq( plan_road );

	for(int i = 0; i<200; i++)
	{
		cvSeqPush( plan_road, &GpsPoint[i] );
	}

	SetEvent(m_hEvent);//路径更新后，发送消息
	
	

	double waydir = m_GpsData.GetAngle(WPoint[1].x,WPoint[1].y,WPoint[0].x,WPoint[0].y);
	Map_Point MidPoint = {0};
	while(1)
	{
		app->critical_map.Lock();
		vel_Map = app->PercepMap;
		app->critical_map.Unlock();
		m_gps = app->GPS_Point;
		m_gpsdir = app->GPS_Direction;

		double k = waydir - m_gpsdir;
		if(CountUTurn(vel_Map,k,MidPoint))
			break;
	}
	/*Stop(m_gps);
	Sleep(2000);*/
	m_gps = app->GPS_Point;
	m_gpsdir = app->GPS_Direction;
	CvPoint2D64f MidPoint1 = cvPoint2D64f((double)MidPoint.x,(double)MidPoint.y);
	CvPoint2D64f MidGps = m_GpsData.MaptoGPS(m_gps,m_gpsdir,MidPoint1);
	
	CvPoint2D64f chuizu1,chuizu2;
	double x1 = WPoint[0].x;
	double y1 = WPoint[0].y;
	double x2 = WPoint[1].x;
	double y2 = WPoint[1].y;
	double A = y2-y1;
	double B = -(x2-x1);
	double C = x2*y1-x1*y2;


	double x0 = (B*B*MidGps.x - A*B*MidGps.y - A*C)/(A*A + B*B);
	double y0 = (-A*B*MidGps.x + A*A*MidGps.y - B*C)/(A*A + B*B);
	chuizu1.x = x0;
	chuizu1.y = y0;                                                        
	
	
	x1 = WPoint[5].x;
	y1 = WPoint[5].y;
	x2 = WPoint[6].x;
	y2 = WPoint[6].y;
	A = y2-y1;
	B = -(x2-x1);
	C = x2*y1-x1*y2;


	x0 = (B*B*MidGps.x - A*B*MidGps.y - A*C)/(A*A + B*B);
	y0 = (-A*B*MidGps.x + A*A*MidGps.y - B*C)/(A*A + B*B);
	chuizu2.x = x0;
	chuizu2.y = y0;

	uturn[0].x =  2*WPoint[1].x - chuizu1.x;
	uturn[0].y =  2*WPoint[1].y - chuizu1.y;
	uturn[1] = WPoint[1];
	uturn[2] = chuizu1;
	uturn[3].x = (MidGps.x + 2*chuizu1.x)/3.;
	uturn[3].y = (MidGps.y + 2*chuizu1.y)/3.;
	uturn[4].x = (2*MidGps.x + chuizu1.x)/3.;
	uturn[4].y = (2*MidGps.y + chuizu1.y)/3.;
	uturn[5] = MidGps;
	uturn[6].x = (MidGps.x + 2*chuizu2.x)/3.;
	uturn[6].y = (MidGps.y + 2*chuizu2.y)/3.;
	uturn[7].x = (2*MidGps.x + chuizu2.x)/3.;
	uturn[7].y = (2*MidGps.y + chuizu2.y)/3.;
	uturn[8] = chuizu2;
	uturn[9] = WPoint[5];
	uturn[10].x =  2*WPoint[6].x - chuizu2.x;
	uturn[10].y =  2*WPoint[6].y - chuizu2.y;



//	LocalPath.Bezier(uturn,10,firstway);
	m_gps = app->GPS_Point;
	m_gpsdir = app->GPS_Direction;
	MoveWay(m_gps,m_gpsdir,WPoint,7,firstway);

	cvClearSeq( plan_road );

	for(int i = 0; i<200; i++)
	{
		cvSeqPush( plan_road, &firstway[i] );
	}

	SetEvent(m_hEvent);//路径更新后，发送消息
	int countdd = 0;
	double angle = m_GpsData.GetAngle(WPoint[6].x,WPoint[6].y,WPoint[5].x,WPoint[5].y);
   while(1)
   {   
	   outn1<<5<<endl;
		m_gps = app->GPS_Point;
		m_gpsdir = app->GPS_Direction;
		//double dd = m_GpsData.GetDistance(m_gps.x,m_gps.y,WPoint[4].x,WPoint[4].y);
		if(abs(m_gpsdir - angle) < 20 || abs(360 - abs(m_gpsdir - angle)) < 20)
			break ;
		/*if(dd < 5)
			break;*/
		if(countdd > 600)
		{
			break;
		}
		Sleep(100);
		countdd++;
   }
   
   	cvClearSeq( plan_road );

	for(int i = 0; i<200; i++)
	{
		cvSeqPush( plan_road, &m_gps);
	}

	SetEvent(m_hEvent);//路径更新后，发送消息
	Sleep(3000);
	return 1;
}

int CIVExeNew::CountUTurn(I_Map *map,double k,Map_Point &a)
{

	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	int shanghuokou_up = 12;
	int shanghuokou_down = 372;
	const int xia_num = 20;
	const int shang_num = 343;
	double k_nihe=0;
	double b_nihe=0;
	int x[xia_num]={0};
	int y[xia_num]={0};
	double xl = 0;
	double vel_angle = 0;
	double lud_angle = 90-k;
	int count = 0;
	int j=256;
	int x_ave=0,y_ave=0;
	int sumx=0,sumy=0;
	int count_ux = 0;
	int count_kou=0;
	Map_Point u_twaypoint;
	CvPoint2D64f apoint = {0};
	if(LeadPoint[seq_num-2].param2 == 4)
	{
		apoint.x = LeadPoint[seq_num-1].lat;
		apoint.y = LeadPoint[seq_num-1].lng;
	}
	else if(LeadPoint[seq_num-3].param2 == 4)
	{
		apoint.x = LeadPoint[seq_num-2].lat;
		apoint.y = LeadPoint[seq_num-2].lng;
	}
	u_twaypoint = m_GpsData.APiontConver(app->GPS_Point,apoint,app->GPS_Direction);
	
	outn1<<"u_twaypoint"<<u_twaypoint.x<<","<<u_twaypoint.y<<endl;
	/*for(int i=240;i>220;i--)
	{
		for(j=256;j>0;j--)
		{
			if(map->MapPoint[i][j] == 1)
				break;
		}
		x[240-i] = j;
		y[240-i] = i;
	}*/

	for(int i = 392;i>372;i--)
	{
		for(j=256;j>0;j--)
		{
			if(map->MapPoint[i][j] == 8&&j > u_twaypoint.x)
				break;
			
		}
		x[392-i] = j;
		y[392-i] = i;
	}
	for(int i = 0;i<xia_num;i++)
	{
		if(x[i] == 0 )
			count_ux++;

	}
	if(count_ux == xia_num)
	{
		a.x = u_twaypoint.x  + (256 -  u_twaypoint.x)/2;
		a.y = 360;
		outn1<<"a "<<a.x<<","<<a.y<<endl;
		return 1;
	}
	//for(int i=0;i<40;i++)
	//	outn1<<x[i]<<",  "<<y[i]<<endl;
	int count_x=0;
	for(int i=0;i<xia_num;i++)
	{
		if(x[i]>u_twaypoint.x)
		{
			sumx+=x[i];
			sumy+=y[i];
			count_x++;
		}
	}
	x_ave = sumx/count_x;
	y_ave = sumy/count_x;
	k_nihe = tan((90-k)*PI/180);
	b_nihe = y_ave - k_nihe*x_ave;
	
	int count1 = 0,count2=0,count3=0;
    j = 256;
	double s = 0;
	int xx[shang_num] = {0};
	int yy[shang_num] = {0};
	/*for(int i=200;i>50;i--)
	{
		for(j = 256;j>0;j--)
		{
			if(map->MapPoint[i][j] == 1)
				break;
		}
		xx[200-i] = j;
		yy[200-i] = i;
	}*/
	for(int i=shanghuokou_down;i>shanghuokou_up;i--)
	{
		for(j = 256;j>0;j--)
		{
			if(map->MapPoint[i][j] == 8)
			{
				
				break;
			}

		}
		
		xx[shanghuokou_down-i] = j;
		yy[shanghuokou_down-i] = i;
		//if(j > u_twaypoint.x)
		//	count_kou=0;
		//if(j < u_twaypoint.x)
		//	count_kou++;
		s=(abs(k_nihe*xx[shanghuokou_down-i]-yy[shanghuokou_down-i]+b_nihe)/sqrt(pow(k_nihe,2)+pow(double(1),2)))*0.2 ;//算距离，怎么算
		if(s>4)
			count_kou++;
		else
			count_kou=0;
		if(count_kou>40)
		{
			a.y = i+0*count_kou/8;
			a.x = (a.y - b_nihe)/k_nihe;
			return 1;
		}
	}
		//for(int i=0;i<150;i++)
		//outn1<<xx[i]<<",  "<<yy[i]<<endl;
	//Map_Point b;
	//for(int i=shanghuokou_down;i>shanghuokou_up;i--)
	//{
	//	
	//		s=(abs(k_nihe*xx[shanghuokou_down-i]-yy[shanghuokou_down-i]+b_nihe)/sqrt(pow(k_nihe,2)+pow(double(1),2)))*0.2 ;//算距离，怎么算
	//	
	//	/*if(s>0 && s<7)
	//		{
	//			app->tj_flag = false;
	//			if(count2 > 10 && count2 <50)
	//				return count2*0.2;
	//			else 
	//				count2 = 0;
	//			continue;
	//		}*/
	//	if(s>2)
	//	{					
	//       app->tj_flag = true;
	//		
	//		  if(app->tj_flag)
	//		  {
	//			  count2++;
	//		  }
	//		  continue;
	//	}
	//		
	//	else
	//	{
	//		app->tj_flag = false;
	//		if(count2 > 20)
	//		{  
	//			a.y = i+count2/6;
	//			a.x = (a.y - b_nihe)/k_nihe;
	//			return 1;
	//		}
	//		else
	//			count2 = 0;
	//		continue;
	//	}
	//	
	//}
	//if(count2 > 20)
	//{  
	//	a.y = shanghuokou_up+count2/6;
	//	a.x = (a.y - b_nihe)/k_nihe;
	//	return 1;
	//}
	return 0;


}
int CIVExeNew::isUTurn(CvPoint2D64f WPoint[])
{
	double dir1 = m_GpsData.GetAngle(WPoint[0],WPoint[1]);
	double dir2 = m_GpsData.GetAngle(WPoint[5],WPoint[6]);
	if(abs(dir1-dir2)>160&&abs(dir1-dir2)<200)
		return 1;
	return 0;
}

int CIVExeNew::MoveWay(CvPoint2D64f m_gps, double m_gpsdir,CvPoint2D64f APoint[],int num,CvPoint2D64f WayPoint[])
{
	double mindis = 100000;
	int v_num = 0;
	double dis;
	for(int i=0;i<200;i++)
	{
		dis = m_GpsData.GetDistance(m_gps.x,m_gps.y,WayPoint[i].x,WayPoint[i].y);
		if(dis < mindis)
		{
			mindis= dis;
			v_num = i;
		}
	}

	double dx = WayPoint[v_num].x - m_gps.x;
	double dy = WayPoint[v_num].y - m_gps.y;
	
	//outn1<<"偏差"<<dx*100000<<", "<<dy*100000<<endl;
	for(int i=0;i<200;i++)
	{
		WayPoint[i].x -= dx;
		WayPoint[i].y -= dy;

	}

	for(int i=0;i<num;i++)
	{
		APoint[i].x -= dx;
		APoint[i].y -= dy;

	}
	return 1;
}
bool CIVExeNew::SearchObstacle(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	
	app->critical_section.Lock();//锁住
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;

	app->critical_section.Unlock();

	CvPoint2D64f Rndf_MapPoint[200]={0};
	for(int i=0;i<200;i++)
	{

		Rndf_MapPoint[i] = m_GpsData.APiontConverD(m_gps,GPSpoint[i],m_gpsdir);
		if(m_gps.x==GPSpoint[i].x&&m_gps.y==GPSpoint[i].y)
		{
			Rndf_MapPoint[i].x = 256;
			Rndf_MapPoint[i].y = 411;
		}
	}

	int m_up = 412-up*10;
	int m_down = 411 -down*10;

	int x = 0;
	int y = 0;
	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 512||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(int m=-6;m<6;m++)
			for(int n=-9;n<9;n++)
			{
				/*if(y+m>360&&y+m<430)
					continue;*/
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == 8)
				{
					
					return true;

				}
			}
	}
	return false;
}
bool CIVExeNew::SearchObstacle(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down,int &ob_x,int &ob_y)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	
	app->critical_section.Lock();//锁住
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;
	
	app->critical_section.Unlock();

	CvPoint2D64f Rndf_MapPoint[200]={0};
	for(int i=0;i<200;i++)
	{
		
		Rndf_MapPoint[i] = m_GpsData.APiontConverD(m_gps,GPSpoint[i],m_gpsdir);
		if(m_gps.x==GPSpoint[i].x&&m_gps.y==GPSpoint[i].y)
		{
			Rndf_MapPoint[i].x = 256;
			Rndf_MapPoint[i].y = 411;
		}
	}

	int m_up = 412-up*10;
	int m_down = 411 -down*10;

	int x = 0;
	int y = 0;
	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 512||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
					continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(int m=-6;m<6;m++)
			for(int n=-10;n<10;n++)
			{
				if(y+m>360&&y+m<430)
					continue;
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == 8)
				{
					ob_x = x+n;
					ob_y = y+m;
					return true;
			
				}
			}
	}
	return false;
}
bool CIVExeNew::AvoidObstacle(CvPoint2D64f cur_point[],CvPoint2D64f left_point[])
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	double dir = m_GpsData.GetAngle(left_point[199],left_point[0]);
	cvClearSeq( plan_road );
	int i_youzhuan = 0;
	for(int i = 0; i<200; i++)
	{
		cvSeqPush( plan_road, &left_point[i]);
	}

	SetEvent(m_hEvent);//路径更新后，发送消息
	Sleep(2000);

	int countdd = 0;
	bool ob = false;
	double angle = m_GpsData.GetAngle(left_point[199].x,left_point[199].y,left_point[0].x,left_point[0].y);
	while(1)
	{   
	   
		CvPoint2D64f m_gps = app->GPS_Point;
		
		double m_gpsdir = app->GPS_Direction;
		ob=SearchObstacle(left_point,vel_Map,40,5);
		if(ob)			
			{
				Stop(m_gps);
				Sleep(1000);
				break;
			}
		//double dd = m_GpsData.GetDistance(m_gps.x,m_gps.y,WPoint[4].x,WPoint[4].y);
		if(abs(m_gpsdir - angle) < 5 || abs(360 - abs(m_gpsdir - angle)) < 5)
			break ;
		/*if(dd < 5)
			break;*/
		if(countdd > 600)
		{
			break;
		}
		Sleep(100);
		countdd++;
	}//换道完成
	Sleep(2000);
	u_desire = 25;
	//aim_length = 18;
	
	while(1)//判断右边
	{
		app->critical_map.Lock();
		vel_Map = app->PercepMap;
		app->critical_map.Unlock();
		CvPoint2D64f m_gps = app->GPS_Point;
		
		if(SearchObstacle(cur_point,vel_Map,35,-8))
		{
			i_youzhuan = 0;
			CvPoint2D64f m_gps = app->GPS_Point;
			
			if(SearchObstacle(left_point,vel_Map,25,5))
			{
				Stop(m_gps);
				Sleep(1000);
			}
			else
			{
				cvClearSeq( plan_road );
				for(int i = 0; i<200; i++)
				{
					cvSeqPush( plan_road, &left_point[i]);
				}

				SetEvent(m_hEvent);//路径更新后，发送消息
			}
			Sleep(50);
			continue;
		}
		else
		{
			i_youzhuan++;
			Sleep(30);
		}
		if(i_youzhuan>30)
			u_desire = 10;
		if(i_youzhuan>50)break;
	}
	u_desire = 8;
	//aim_length = 10;
	//Sleep(2000);
	cvClearSeq( plan_road );
	for(int i = 0; i<200; i++)
	{
		cvSeqPush( plan_road, &cur_point[i]);
	}

	SetEvent(m_hEvent);//路径更新后，发送消息
	Sleep(5000);

	angle = m_GpsData.GetAngle(left_point[199].x,left_point[199].y,left_point[0].x,left_point[0].y);
	while(1)
	{   
	   
		CvPoint2D64f m_gps = app->GPS_Point;
		
		double m_gpsdir = app->GPS_Direction;
		//double dd = m_GpsData.GetDistance(m_gps.x,m_gps.y,WPoint[4].x,WPoint[4].y);
		if(abs(m_gpsdir - angle) < 3 || abs(360 - abs(m_gpsdir - angle)) < 3)
			break ;
		/*if(dd < 5)
			break;*/
		Sleep(100);
		
	}//换道完成
	u_desire = 20;
	//aim_length = 18;
	return 0;
}
int CIVExeNew::MoveWay(CvPoint2D64f m_gps, double m_gpsdir,CvPoint2D64f WayPoint[])
{
	double mindis = 100000;
	int v_num = 0;
	double dis;
	for(int i=0;i<200;i++)
	{
		dis = m_GpsData.GetDistance(m_gps.x,m_gps.y,WayPoint[i].x,WayPoint[i].y);
		if(dis < mindis)
		{
			mindis= dis;
			v_num = i;
		}
	}

	double dx = WayPoint[v_num].x - m_gps.x;
	double dy = WayPoint[v_num].y - m_gps.y;
	
	//outn1<<"偏差"<<dx*100000<<", "<<dy*100000<<endl;
	for(int i=0;i<200;i++)
	{
		WayPoint[i].x -= dx;
		WayPoint[i].y -= dy;

	}
	return 1;
}

int CIVExeNew::MoveLeft(CvPoint2D64f WayPoint[],CvPoint2D64f LeftWayPoint[],double length)
{
	double dir1 = m_GpsData.GetAngle(WayPoint[199],WayPoint[0]);
	double dir2 = dir1-90;
	double rad_dir2 = dir2*PI/180;

	double x = length * cos(rad_dir2);
	double y = length * sin(rad_dir2);

	double dx = (x*180)/(6378137*PI);
	double dy = (y*180)/(6378137*PI*cos(x*PI/180));

	
	//outn1<<"偏差"<<dx*100000<<", "<<dy*100000<<endl;
	for(int i=0;i<200;i++)
	{
		LeftWayPoint[i].x = WayPoint[i].x + dx;
		LeftWayPoint[i].y = WayPoint[i].y + dy;

	}
	return 1;
}
int CIVExeNew::MoveRight(CvPoint2D64f WayPoint[],CvPoint2D64f RightWayPoint[],double length)
{
	double dir1 = m_GpsData.GetAngle(WayPoint[199],WayPoint[0]);
	double dir2 = dir1 + 90;
	double rad_dir2 = dir2*PI/180;

	double x = length * sin(rad_dir2);
	double y = length * cos(rad_dir2);

	double dx = (x*180)/(6378137*PI);
	double dy = (y*180)/(6378137*PI*cos(x*PI/180));
		
	//outn1<<"偏差"<<dx*100000<<", "<<dy*100000<<endl;
	for(int i=0;i<200;i++)
	{
		RightWayPoint[i].x = WayPoint[i].x + dx;
		RightWayPoint[i].y = WayPoint[i].y + dy;

	}
	return 1;
}
int CIVExeNew::Stop(CvPoint2D64f m_gps)
{
   	cvClearSeq( plan_road );

	for(int i = 0; i<200; i++)
	{
		cvSeqPush( plan_road, &m_gps);
	}

	SetEvent(m_hEvent);//路径更新后，发送消息
	Sleep(1000);
	return 1;
}

int CIVExeNew::StopBreak(CvPoint2D64f gps[])
{
   	cvClearSeq( plan_road );

	for(int i = 0; i<200; i++)
	{
		cvSeqPush( plan_road, &gps[i]);
	}

	SetEvent(m_hEvent);//路径更新后，发送消息
	Sleep(30);
	return 1;
}
bool CIVExeNew::ChangeLeftLane(CvPoint2D64f cur_point[])
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	on_obs = true;
	u_desire = 10;
	aim_length = 15;
	CvPoint2D64f left_point[200] = {0};
	MoveLeft(cur_point,left_point,4.5);
	double dir = m_GpsData.GetAngle(left_point[199],left_point[0]);
	cvClearSeq( plan_road );
	int i_youzhuan = 0;
	for(int i = 0; i<200; i++)
	{
		cvSeqPush( plan_road, &left_point[i]);
	}

	SetEvent(m_hEvent);//路径更新后，发送消息
	Sleep(2000);

	int countdd = 0;
	//bool ob = false;
	double angle = m_GpsData.GetAngle(left_point[199].x,left_point[199].y,left_point[0].x,left_point[0].y);
	while(1)
	{   
	   
		CvPoint2D64f m_gps = app->GPS_Point;
		double m_gpsdir = app->GPS_Direction;
		//app->critical_map.Lock();
		//vel_Map = app->PercepMap;
		//app->critical_map.Unlock();
		////double dd = m_GpsData.GetDistance(m_gps.x,m_gps.y,WPoint[4].x,WPoint[4].y);
		//ob=SearchObstacle(left_point,vel_Map,40,5);
		//if(ob)			
		//	{
		//		Stop(m_gps);
		//		Sleep(1000);
		//		break;
		//	}


		if(abs(m_gpsdir - angle) < 5 || abs(360 - abs(m_gpsdir - angle)) < 5)
			break ;
		/*if(dd < 5)
			break;*/
		if(countdd > 600)
		{
			break;
		}
		
		Sleep(100);
		countdd++;
	}//换道完成
	Sleep(2000);
	//on_obs = false;
	return 0;
	//aim_length = 18;
}
bool CIVExeNew::ChangeRightLane(CvPoint2D64f cur_point[])
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	on_obs = true;
	u_desire = 10;
	aim_length = 15;
	CvPoint2D64f right_point[200] = {0};
	MoveRight(cur_point,right_point,4.5);
	double dir = m_GpsData.GetAngle(right_point[199],right_point[0]);
	cvClearSeq( plan_road );
	int i_youzhuan = 0;
	for(int i = 0; i<200; i++)
	{
		cvSeqPush( plan_road, &right_point[i]);
	}

	SetEvent(m_hEvent);//路径更新后，发送消息
	Sleep(2000);

	int countdd = 0;
	//bool ob = false;
	double angle = m_GpsData.GetAngle(right_point[199].x,right_point[199].y,right_point[0].x,right_point[0].y);
	while(1)
	{   
	   
		CvPoint2D64f m_gps = app->GPS_Point;
		double m_gpsdir = app->GPS_Direction;
		app->critical_map.Lock();
		//vel_Map = app->PercepMap;
		//app->critical_map.Unlock();
		////double dd = m_GpsData.GetDistance(m_gps.x,m_gps.y,WPoint[4].x,WPoint[4].y);
		//ob=SearchObstacle(right_point,vel_Map,40,5);
		//if(ob)			
		//	{
		//		Stop(m_gps);
		//		Sleep(1000);
		//		break;
		//	}


		if(abs(m_gpsdir - angle) < 5 || abs(360 - abs(m_gpsdir - angle)) < 5)
			break ;
		/*if(dd < 5)
			break;*/
		if(countdd > 600)
		{
			break;
		}
		
		Sleep(100);
		countdd++;
	}//换道完成
	Sleep(2000);
	//on_obs = false;
	return 0;
	//aim_length = 18;
}
bool CIVExeNew::GetMidLFroMap(I_Map *OriMap,I_Map *mid_Map)
{
	for(int i = 0;i<512;i++)
		for(int j = 0;j<512;j++)
		{
			if(OriMap->MapPoint[i][j] > 1)
				OriMap->MapPoint[i][j] = 0;
			mid_Map->MapPoint[i][j] = OriMap->MapPoint[i][j];
		}	
	return 1;
}
int CIVExeNew::StopLine(double s1)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();	
	stop_flag = true;
	double di = 0;
	double start_pt = app->cardl;
	while(1)
	{
		di = abs(start_pt - app->cardl);
		if(di>s1-6)
			break;
		Sleep(5);
	}
	zStop_Odo(6);
	m_ctrl.brake = -48000;
	Sleep(5000);
	stop_flag =false;
	return 0;
}
int CIVExeNew::zStop_Odo(double Stop_Lenth )
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	double speed;
	double dist;
	//double Stop_Lenth = 2;
	CvPoint2D64f m_stop;
	m_stop = app->GPS_Point;

	double di = 0;
	double start_pt = app->cardl;
	while(1)
	{
		SYSTEMTIME t1;
		GetLocalTime(&t1);
		//outn2<<endl<<t1.wHour<<":"<<t1.wMinute<<":"<<t1.wSecond<<","<<t1.wMilliseconds<<" ";
		//outn2<<"轮速值 "<<app->cardl<< " "; 
		
		CvPoint2D64f m_gps = app->GPS_Point;
		double m_direction = app->GPS_Direction;
		
		speed = abs(app->cspeed);
		//outn2<<"速度, "<<speed<<", ";
		/*di = (app->cardr + app->cardr)/2 - start_pt;*/
		di = abs(app->cardl - start_pt);

		dist = Stop_Lenth - di;
		
		//dist = m_GpsData.GetDistance( m_gps.x, m_gps.y, m_end.x, m_end.y );
		/*if(abs(dist)>6)
			continue;*/
		//dist = -dist*cos( head_path_angle );
		//outn2<<"距离, "<<dist<<", ";
		if(dist < 0.1/*|| ( (head_path_angle < -135) || (head_path_angle > 135) )*/)
			return 0;


		int throttle = 37,brake = -25500;

		if(speed>2)
		{
			brake = -47500;
			m_ctrl.brake = brake;
			//outn2<<"brake"<<brake<<" ";
			continue;
			

			//m_stop = app->GPS_Point;
		}
		else	
		{

			double dist3 = m_GpsData.GetDistance( m_gps.x, m_gps.y, m_stop.x, m_stop.y );
			//outn2<<"距离3, "<<dist3<<", ";

			//double out = dist-speed*t;

			PIDREG3 pid2 = PIDREG3_DEFAULTS;
			pid2.Kp=0.15;
			pid2.Ki=0;
			pid2.Kd=0;
			pid2.Kc=0.03;
			pid2.OutMax=30;//输出最大值
			pid2.OutMin=-30;//输出最小值
			pid2.Ref=dist;//过程值
			pid2.Fdb=0;//反馈值
			pid_reg3_calc(&pid2);
			double out = pid2.Out;

			double a = (speed*speed)/(2*out);
			if(a>1.5)a=1.5;
			if(a<0)a=0;

			//outn2<<"a, "<<a<<", ";
			brake = 10000*(a + 5.8)/(-1.6445);
			//outn2<<"brake, "<<brake<<", ";
			if(brake>-30000)brake = -30000;


			if(brake<-45000)brake = -45000;
			if(brake>-25500)brake = -25500;

		}


		//m_ctrl.turn = tc;
		//outn1<<"turn  "<<m_ctrl.turn<<"  tc "<<tc<<"  last  "<<lastturn<<endl;
		//lastturn = m_ctrl.turn;

		//int a = zGetTurn(m_gps,m_direction,m_end,m_ctrl.turn);
		//m_ctrl.turn = 0;
		m_ctrl.velocity = 5;
		m_ctrl.accelerator = 37;
		m_ctrl.brake = brake;
		//m_ctrl.horn = 0;
		//m_ctrl.key = 0;
		//m_ctrl.light = 0;
		//m_ctrl.mswitch = 3;

		//CString str;
		//str.Format("%sG100T%dV%dB%dS%dK%dA%dH%dL%sEND;","%",m_ctrl.turn,m_ctrl.velocity,m_ctrl.brake,m_ctrl.mswitch,0,m_ctrl.accelerator,0,0);

		//const char* sttrr=str.GetString();
		//sttrr=str.GetString();
		//if(send.SuccSocket)
		//	send.ClientSend(sttrr);

		Sleep(10);
	}
	return 0;
}
int CIVExeNew::isStopLine()
{
	if(vel_Map->MapPoint[0][511] == 11)
		return 1;
	else
		return 0;
	
}
//int CIVExeNew::ChangeLeftLine
bool CIVExeNew::ChangeLeft_RightLane(CvPoint2D64f cur_point[],int left_right)//1表示左边换道，-1表示右边换道
{
		
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->zuo_huandao = false;

	app->you_hundao = false;
	CvPoint2D64f m_gps;
	double dist = 0;
	on_obs = true;
	u_desire = 10;
	aim_length = 15;
	CvPoint2D64f temp_point[200] = {0};
	if(left_right == 1)
	MoveLeft(cur_point,temp_point,4.5);
	if(left_right == -1)
	MoveLeft(cur_point,temp_point,-4.5);
	double dir = m_GpsData.GetAngle(temp_point[199],temp_point[0]);
	ArrayFuZhi(temp_point,cur_point);
	cvClearSeq( plan_road );
	for(int i = 0; i<200; i++)
	{
		cvSeqPush( plan_road, &temp_point[i]);
	}

	SetEvent(m_hEvent);//路径更新后，发送消息
	Sleep(7000);

	int countdd = 0;
	//bool ob = false;
	CvPoint2D64f his_gps = app->GPS_Point;
	double his_dir = app->GPS_Direction;
	double angle = m_GpsData.GetAngle(temp_point[199].x,temp_point[199].y,temp_point[0].x,temp_point[0].y);
	while(1)
	{   
	   
		CvPoint2D64f m_gps = app->GPS_Point;
		double m_gpsdir = app->GPS_Direction;
		//app->critical_map.Lock();
		//vel_Map = app->PercepMap;
		//app->critical_map.Unlock();
		////double dd = m_GpsData.GetDistance(m_gps.x,m_gps.y,WPoint[4].x,WPoint[4].y);
		//ob=SearchObstacle(left_point,vel_Map,40,5);
		//if(ob)			
		//	{
		//		Stop(m_gps);
		//		Sleep(1000);
		//		break;
		//	}

		//outn1<<"huandao zhong"<<endl;
		app->critical_section.Lock();//锁住
		m_gps = app->GPS_Point;
		app->critical_section.Unlock();//解锁
		dist = abs(ParaDist(his_gps,his_dir,m_gps));

		if(abs(m_gpsdir - angle) < 15 || abs(360 - abs(m_gpsdir - angle)) < 15 && dist > 1.5)
			break ;
		//outn2<<(m_gpsdir - angle)<<endl;
		/*if(dd < 5)
			break;*/
	/*	if(countdd > 2000)
		{
			break;
		}*/
		
		Sleep(30);
		countdd++;
	}//换道完成

	if(left_right == 1)
		app->zuo_huandao = false;
	if(left_right == -1)
		app->you_hundao = false;
	on_obs = false;
	return 0;
	//aim_length = 18;
}

int CIVExeNew::YanChang(CvPoint2D64f midpoint[],int length,CvPoint2D64f (&shift_point)[200])
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	CvPoint2D64f *shift_midmap = (CvPoint2D64f *)malloc( (200+length)*sizeof(CvPoint2D64f) );

	CvPoint2D64f a1 = midpoint[150];
	CvPoint2D64f a2 = midpoint[199];
	for(int i = 0;i>-length;i--)
	{
		shift_midmap[-i+200].x = (i*length-a1.y)/(a2.y-a1.y)*(a2.x-a1.x)+a1.x;
		shift_midmap[-i+200].y = i*length;
	}
	for(int i = 0;i<200;i++)
	{
		shift_midmap[i] = midpoint[i];
	}
//	keeplane.path.Bezier(shift_midmap,199+length,shift_point);
	return 1;
}

double CIVExeNew::ParaDist(CvPoint2D64f ori_gps,double ori_dir,CvPoint2D64f cur_gps)
{
		double north_dir = m_GpsData.GetAngle(cur_gps,ori_gps);
		double head_path_angle = (north_dir - ori_dir)*PI/180;


		//dist = 10-di;
		double dist = m_GpsData.GetDistance( cur_gps.x,cur_gps.y,ori_gps.x, ori_gps.y);
		dist = dist*sin( head_path_angle );
		//outn2<<"距离, "<<dist<<", ";
		return dist;
	
}
double CIVExeNew::VertDist(CvPoint2D64f ori_gps,double ori_dir,CvPoint2D64f cur_gps)
{
		double north_dir = m_GpsData.GetAngle(cur_gps,ori_gps);
		double head_path_angle = (north_dir - ori_dir)*PI/180;


		//dist = 10-di;
		double dist = m_GpsData.GetDistance( cur_gps.x,cur_gps.y,ori_gps.x, ori_gps.y);
		dist = dist*cos( head_path_angle );
		//outn2<<"距离, "<<dist<<", ";
		return dist;
}

int CIVExeNew::GetMissionNum( )
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	return app->Lead_pt.size();
}
int CIVExeNew::GetMissionPoint(LEAD lead[],int num)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	for(int i = 0; i<num;i++)
	{
		lead[i] = app->Lead_pt.front();
		app->Lead_pt.pop();
	}
	return 1;
}
int CIVExeNew::GetCross_new(int seq,CvPoint2D64f WayPoint[],CvPoint2D64f (&GpsPoint)[200],int &fx)
{
	CvPoint2D64f Point[4];
	CvPoint2D64f Cross_Pt[8];
	Point[0].x = LeadPoint[seq-3].lat;
	Point[0].y = LeadPoint[seq-3].lng;
	
	Point[1].x = LeadPoint[seq-2].lat;
	Point[1].y = LeadPoint[seq-2].lng;	
	
	Point[2].x = LeadPoint[seq-1].lat;
	Point[2].y = LeadPoint[seq-1].lng;

	Point[3].x = LeadPoint[seq].lat;
	Point[3].y = LeadPoint[seq].lng;


	//按路点取最后两点
	

	WayPoint[1] = Point[1];
	WayPoint[0] = Point[0];
	WayPoint[6] = Point[3];
	//
	double dist = m_GpsData.GetDistance( Point[0].x,Point[0].y,Point[1].x, Point[1].y);
	WayPoint[0].x = ((dist-30)*Point[1].x + 30*Point[0].x)/dist;
	WayPoint[0].y = ((dist-30)*Point[1].y + 30*Point[0].y)/dist;	


	WayPoint[5] = Point[2];

	dist = m_GpsData.GetDistance( Point[2].x,Point[2].y,Point[3].x, Point[3].y);
	WayPoint[6].x = ((dist-30)*Point[2].x + 30*Point[3].x)/dist;
	WayPoint[6].y = ((dist-30)*Point[2].y + 30*Point[3].y)/dist;	

	//求两端路的交点
	double x1 = Point[0].x;
	double y1 = Point[0].y;
	double x2 = Point[1].x;
	double y2 = Point[1].y;
	double A1 = y2-y1;
	double B1 = -(x2-x1);
	double C1 = x2*y1-x1*y2;

	x1 = Point[2].x;
	y1 = Point[2].y;
	x2 = Point[3].x;
	y2 = Point[3].y;
	double A2 = y2-y1;
	double B2 = -(x2-x1);
	double C2 =x2*y1-x1*y2;

	WayPoint[3].x = (B1*C2-B2*C1)/(A1*B2-A2*B1);
	WayPoint[3].y = (A1*C2-A2*C1)/(A2*B1-A1*B2);

	double dir1 = m_GpsData.GetAngle(WayPoint[0],WayPoint[1]);
	double dir2 = m_GpsData.GetAngle(WayPoint[5],WayPoint[6]);
	if(/*abs(dir1-dir2)<10||abs(360-abs(dir1-dir2))<10*/  LeadPoint[seq-2].param2 == 1)
	{
		fx = 1;
		WayPoint[3].x = (WayPoint[1].x+WayPoint[5].x)/2.0;
		WayPoint[3].y = (WayPoint[1].y+WayPoint[5].y)/2.0;
	}
	if(/*abs(abs(dir1-dir2-270)<30||abs(90+dir1-dir2))<30*/ LeadPoint[seq-2].param2 == 2/*abs(dir1-dir2-270)<30||abs(90+dir1-dir2)<30*/)
	{
		fx = 2/*2*/;
		
	}
	if(/*abs(abs(dir1-dir2-90)<30||abs(270+dir1-dir2))<30*/  LeadPoint[seq-2].param2 == 3/*abs(dir1-dir2-90)<30||abs(270+dir1-dir2)<30*/)
	{
		fx = 3/*3*/;
		
	}
	
	double dist1 = m_GpsData.GetDistance(WayPoint[3].x,WayPoint[3].y,WayPoint[5].x,WayPoint[5].y);//w3,w5之间距离
	if(dist1>10)
	{
		WayPoint[4].x = (dist1*WayPoint[3].x+0*(WayPoint[5].x-WayPoint[3].x))/dist1;
		WayPoint[4].y = (dist1*WayPoint[3].y+0*(WayPoint[5].y-WayPoint[3].y))/dist1;
	}
	else WayPoint[4] = WayPoint[3];
	double dist2 =  m_GpsData.GetDistance(WayPoint[3].x,WayPoint[3].y,WayPoint[1].x,WayPoint[1].y);//w1,w3之间距离
	if(dist2>10)
	{
		WayPoint[2].x = (dist2*WayPoint[3].x+0*(WayPoint[1].x-WayPoint[3].x))/dist2;
		WayPoint[2].y = (dist2*WayPoint[3].y+0*(WayPoint[1].y-WayPoint[3].y))/dist2;
	}
	else WayPoint[2] = WayPoint[3];
	for(int i = 0;i<3;i++)
	{
		Cross_Pt[i] = WayPoint[i];
	}
	for(int i = 3;i<6;i++)
	{
		Cross_Pt[i] = WayPoint[i+1];
	}
	
	/*WayPoint[2].x = (WayPoint[1].x+9*WayPoint[3].x)/10;
	WayPoint[2].y = (WayPoint[1].y+9*WayPoint[3].y)/10;
	WayPoint[4].x = (WayPoint[5].x+9*WayPoint[3].x)/10;
	WayPoint[4].y = (WayPoint[5].y+9*WayPoint[3].y)/10;*/
	//LocalPath.BSpline(Cross_Pt,8,GpsPoint);
//	LocalPath.Bezier(WayPoint,6,GpsPoint);
	/*for(int i = 0;i<6;i++)
	{
		outn1<<(Cross_Pt[i].x-31)*100000<<","<<(Cross_Pt[i].y-117)*100000<<endl;
	}
	outn1<<"feng ge xian"<<endl;
	for(int i = 0;i<200;i++)
	{
		outn1<<(GpsPoint[i].x-31)*100000<<","<<(GpsPoint[i].y-117)*100000<<endl;
	}*/
	
	return 1;
}

bool CIVExeNew::GetSignal(int (&sign)[4])
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	int GREEN[4]={0};
	int RED[4]={0};
	
	//5/4/3/2/1->STOP/NOTLEFT/NOTRIGHT/LEFT/RIGHT
	int time = 1000;
	while(1)
	{
		if(app->Traffic_Sign.size()>=3)
			break;
		Sleep(5);
		time--;
		if(time == 0)
			return false;
	}
	
	for(int i = 0; i<3; i++)
	{
		int temp = app->Traffic_Sign.front();
		outn1<<temp<<endl;
		int temp0 = temp;
		for(int j = 0; j<4; j++)
		{
			int temp0 = temp%10;
			if(temp0 == 1)
				GREEN[j]++;
			else if(temp0 == 2)
				RED[j]++;
			temp=temp/10;
		}
		app->Traffic_Sign.pop();
	}
	
	int count = 0;
	
	for(int j = 0;j<4;j++)
	{
		if(GREEN[j]>=2)
		{
			sign[j]=1;
			
		}
		if(RED[j]>=2)
		{
			sign[j]=2;
			//sign = j;
		}

	}
	
	//app->stop_sign = true;
	//sign = 1,2,3 //1左转，2直行，3右转。
	app->Get_Sign = false;
	
	int sign_size = app->Traffic_Sign.size();
	for(int i = 0; i < sign_size;i++)
	{
		app->Traffic_Sign.pop();
	}
	return true;

}