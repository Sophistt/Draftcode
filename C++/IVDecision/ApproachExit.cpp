#include "StdAfx.h"
#include "ApproachExit.h"
#include "LaneDriver.h"
#include "UrbanRoadContext.h"
CApproachExit::CApproachExit(void)
{
	temp_map = new I_Map;
}

CApproachExit::~CApproachExit(void)
{
	delete temp_map;
}


void CApproachExit::Approach_intersection()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->secondDecison = "接近路口";
	CvPoint2D64f P;//起始点的延长点
	double r;//起始点在地图坐标系中与x轴正方向的夹角
	double d = 0;//设置起始点向前延长距离，每延长一米增加-5，如延长10米，则设置d为-50.
	double e = 300;//设置轨迹点向前延长距离，每延长一米增加5，如延长10米，则设置e为50.
	double k4;

	if( LeadPoint[seq_num-1].param2 ==3)
		intersection_direction = 3/*fx*/;//1 直行 2 左转 3 右转
	else if( LeadPoint[seq_num-1].param2 ==2)
		intersection_direction = 2;
	else
		intersection_direction = 1;
	//startpoint = cvPoint2D64f(31.847466884,117.120801086 );//31.847466884,117.120901086
	//as = 179.206547962;

	lane_appro.GetLaneMap(*CVehicle::vel_Map,*temp_map);
	lane_appro.ProcCenterLane(temp_map,CVehicle::lane_mid_map);
	lane_appro.KeepLane(realtime_Gps,realtime_Dir,MidGpsPoint);
	double dir = m_GpsData.GetAngle(MidGpsPoint[199].x,MidGpsPoint[199].y,MidGpsPoint[180].x,MidGpsPoint[180].y);


	Search_intersection_startpoint(vel_Map,intersection_direction,startpoint,as);//求出的as是起始点的GPS方向角
	as = dir;

	app->critical_section.Lock();//锁住
	currentpoint = app->GPS_Point;
	ac = app->GPS_Direction;//读取当前位姿

	app->critical_section.Unlock();

	
	
	WayPoint1[0].x = 256;
	WayPoint1[0].y = 412;//当前位置在地图坐标系中的坐标，方向角是270°

	WayPoint1[4] = m_GpsData.APiontConverD(currentpoint,startpoint,ac);//把起始点GPS坐标转换为地图坐标
	//r = ((int)(270 + as - ac))%360;//把起始点的方向角转换为地图坐标系中与x轴正方向夹角;
	r = 270 + as - ac;//把起始点的方向角转换为地图坐标系中与x轴正方向夹角;


	if(r != 270)
	{
		r = rad(r);
	    k4 = tan(r);//把起始点的方向角转换为地图坐标系中的斜率

		P.x = WayPoint1[4].x - d*cos(r);
		P.y = WayPoint1[4].y - d*sin(r);//求出起始点的延长点的地图坐标


		WayPoint1[4] = P;

		solvepoints(WayPoint1[0],WayPoint1[4],k4,WayPoint1[2],WayPoint1[1],WayPoint1[3]);//在接近路口的过程中，求当前点、起始点的延长点、交点和两个线段上的插入点的地图坐标

		if( (WayPoint1[2].y < 412) && ( ((WayPoint1[4].x < 256)&&((r > 3.1415926 )&&(r < 1.5*3.1415926))) || ((WayPoint1[4].x > 256)&&((r > 1.5*3.1415926)&&(r < 2*3.1415926))) ))//此种情况下，可以直接用这五点生成贝塞尔轨迹
		{
			WayPoint1[0] = currentpoint;//又将这五个点的地图坐标转换为GPS坐标
			WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[1]);
			WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[2]);
			WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[3]);
			WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[4]);
		
			Bezier(WayPoint1,4,MidGpsPoint);//在接近路口的过程中，用当前点、起始点的延长点、交点和两个线段上的插入点生成的贝塞尔曲线
		}
		else
		{
			solvepoints1(WayPoint1[0],WayPoint1[4],k4,WayPoint1[2],WayPoint1[1],WayPoint1[3]);
			WayPoint1[0] = currentpoint;//又将这五个点的地图坐标转换为GPS坐标
			WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[1]);
			WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[2]);
			WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[3]);
			WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[4]);

			Bezier(WayPoint1,4,MidGpsPoint);
		}

	}

	else if(r == 270)
	{
		r = rad(r);

		P.x = WayPoint1[4].x - d*cos(r);
		P.y = WayPoint1[4].y - d*sin(r);//求出起始点的延长点的地图坐标


		WayPoint1[4] = P;

		solvepoints2(WayPoint1[0],WayPoint1[4],WayPoint1[2],WayPoint1[1],WayPoint1[3]);
		WayPoint1[0] = currentpoint;//又将这五个点的地图坐标转换为GPS坐标
		WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[1]);
		WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[2]);
		WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[3]);
		WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[4]);

		Bezier(WayPoint1,4,MidGpsPoint);
	}

	bool edge;
	int edgepoint_x,edgepoint_y;
	edge = Search_edge(MidGpsPoint,vel_Map,40,2,edgepoint_x,edgepoint_y);//判断生成的轨迹是否伸出路沿
	while(edge == 1)
	{
		//avoid_edge(startpoint, as, MidGpsPoint);
		startpoint = correct_point(startpoint, as);//生成的轨迹如果伸出路沿，则修正路口进入点的GPS坐标（向左平移1米），重新生成轨迹

		app->critical_section.Lock();//锁住
		currentpoint = app->GPS_Point;
		ac = app->GPS_Direction;//读取当前位姿

		app->critical_section.Unlock();



		WayPoint1[0].x = 256;
		WayPoint1[0].y = 412;//当前位置在地图坐标系中的坐标，方向角是270°

		WayPoint1[4] = m_GpsData.APiontConverD(currentpoint,startpoint,ac);//把起始点GPS坐标转换为地图坐标
		//r = ((int)(270 + as - ac))%360;//把起始点的方向角转换为地图坐标系中与x轴正方向夹角;
		r = 270 + as - ac;//把起始点的方向角转换为地图坐标系中与x轴正方向夹角;


		if(r != 270)
		{
			r = rad(r);
			k4 = tan(r);//把起始点的方向角转换为地图坐标系中的斜率

			P.x = WayPoint1[4].x - d*cos(r);
			P.y = WayPoint1[4].y - d*sin(r);//求出起始点的延长点的地图坐标		


			WayPoint1[4] = P;

			solvepoints(WayPoint1[0],WayPoint1[4],k4,WayPoint1[2],WayPoint1[1],WayPoint1[3]);//在接近路口的过程中，求当前点、起始点的延长点、交点和两个线段上的插入点的地图坐标

			if( (WayPoint1[2].y < 412) && ( ((WayPoint1[4].x < 256)&&((r > 3.1415926 )&&(r < 1.5*3.1415926))) || ((WayPoint1[4].x > 256)&&((r > 1.5*3.1415926)&&(r < 2*3.1415926))) ))//此种情况下，可以直接用这五点生成贝塞尔轨迹
			{
				WayPoint1[0] = currentpoint;//又将这五个点的地图坐标转换为GPS坐标
				WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[1]);
				WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[2]);
				WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[3]);
				WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[4]);

				Bezier(WayPoint1,4,MidGpsPoint);//在接近路口的过程中，用当前点、起始点的延长点、交点和两个线段上的插入点生成的贝塞尔曲线
			}
			else
			{
				solvepoints1(WayPoint1[0],WayPoint1[4],k4,WayPoint1[2],WayPoint1[1],WayPoint1[3]);
				WayPoint1[0] = currentpoint;//又将这五个点的地图坐标转换为GPS坐标
				WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[1]);
				WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[2]);
				WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[3]);
				WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[4]);

				Bezier(WayPoint1,4,MidGpsPoint);
			}

		}

		else if(r == 270)
		{
			r = rad(r);

			P.x = WayPoint1[4].x - d*cos(r);
			P.y = WayPoint1[4].y - d*sin(r);//求出起始点的延长点的地图坐标


			WayPoint1[4] = P;

			solvepoints2(WayPoint1[0],WayPoint1[4],WayPoint1[2],WayPoint1[1],WayPoint1[3]);
			WayPoint1[0] = currentpoint;//又将这五个点的地图坐标转换为GPS坐标
			WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[1]);
			WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[2]);
			WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[3]);
			WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[4]);

			Bezier(WayPoint1,4,MidGpsPoint);
		}

		edge = Search_edge(MidGpsPoint,vel_Map,40,2,edgepoint_x,edgepoint_y);
	}


	app->critical_section.Lock();//锁住
	currentpoint = app->GPS_Point;
	ac = app->GPS_Direction;//读取当前位姿

	app->critical_section.Unlock();

	CvPoint2D64f m_startpoint = m_GpsData.APiontConverD(currentpoint,startpoint,ac);//把起始点GPS坐标转换为地图坐标
	CvPoint2D64f y_startpoint;//延长的轨迹点
	r = 270 + as - ac;//把起始点的方向角转换为地图坐标系中与x轴正方向夹角;
	if(r!=270)
	{
		r = r - 180;
		r = rad(r);
		y_startpoint.x = m_startpoint.x - e*cos(r);
		y_startpoint.y = m_startpoint.y - e*sin(r);
	}
	else if (r==270)
	{
		y_startpoint.x = m_startpoint.x;
		y_startpoint.y = m_startpoint.y - e;
	}
	y_startpoint = m_GpsData.MaptoGPS(currentpoint,ac,y_startpoint);
	double dx = (y_startpoint.x - startpoint.x)/200;
	double dy = (y_startpoint.y - startpoint.y)/200;
	CvPoint2D64f y_MidGpsPoint[400];
	for(int i =0;i<200;i++)
	{
		y_MidGpsPoint[i] = MidGpsPoint[i];
	}
	for (int j=200;j<400;j++)
	{
		y_MidGpsPoint[j].x = y_MidGpsPoint[j-1].x + dx;
		y_MidGpsPoint[j].y = y_MidGpsPoint[j-1].y + dy;
	}
	edge = Search_edge(y_MidGpsPoint,vel_Map,40,2,edgepoint_x,edgepoint_y);//判断延长的轨迹是否伸出路沿

	while (edge == 1)
	{
		as = as - 1;//修正获取的起始点的GPS方向角，重新生成轨迹

		app->critical_section.Lock();//锁住
		currentpoint = app->GPS_Point;
		ac = app->GPS_Direction;//读取当前位姿

		app->critical_section.Unlock();



		WayPoint1[0].x = 256;
		WayPoint1[0].y = 412;//当前位置在地图坐标系中的坐标，方向角是270°

		WayPoint1[4] = m_GpsData.APiontConverD(currentpoint,startpoint,ac);//把起始点GPS坐标转换为地图坐标
		//r = ((int)(270 + as - ac))%360;//把起始点的方向角转换为地图坐标系中与x轴正方向夹角;
		r = 270 + as - ac;//把起始点的方向角转换为地图坐标系中与x轴正方向夹角;


		if(r != 270)
		{
			r = rad(r);
			k4 = tan(r);//把起始点的方向角转换为地图坐标系中的斜率

			P.x = WayPoint1[4].x - d*cos(r);
			P.y = WayPoint1[4].y - d*sin(r);//求出起始点的延长点的地图坐标


			WayPoint1[4] = P;

			solvepoints(WayPoint1[0],WayPoint1[4],k4,WayPoint1[2],WayPoint1[1],WayPoint1[3]);//在接近路口的过程中，求当前点、起始点的延长点、交点和两个线段上的插入点的地图坐标

			if( (WayPoint1[2].y < 412) && ( ((WayPoint1[4].x < 256)&&((r > 3.1415926 )&&(r < 1.5*3.1415926))) || ((WayPoint1[4].x > 256)&&((r > 1.5*3.1415926)&&(r < 2*3.1415926))) ))//此种情况下，可以直接用这五点生成贝塞尔轨迹
			{
				WayPoint1[0] = currentpoint;//又将这五个点的地图坐标转换为GPS坐标
				WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[1]);
				WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[2]);
				WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[3]);
				WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[4]);

				Bezier(WayPoint1,4,MidGpsPoint);//在接近路口的过程中，用当前点、起始点的延长点、交点和两个线段上的插入点生成的贝塞尔曲线
			}
			else
			{
				solvepoints1(WayPoint1[0],WayPoint1[4],k4,WayPoint1[2],WayPoint1[1],WayPoint1[3]);
				WayPoint1[0] = currentpoint;//又将这五个点的地图坐标转换为GPS坐标
				WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[1]);
				WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[2]);
				WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[3]);
				WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[4]);

				Bezier(WayPoint1,4,MidGpsPoint);
			}

		}

		else if(r == 270)
		{
			r = rad(r);

			P.x = WayPoint1[4].x - d*cos(r);
			P.y = WayPoint1[4].y - d*sin(r);//求出起始点的延长点的地图坐标


			WayPoint1[4] = P;

			solvepoints2(WayPoint1[0],WayPoint1[4],WayPoint1[2],WayPoint1[1],WayPoint1[3]);
			WayPoint1[0] = currentpoint;//又将这五个点的地图坐标转换为GPS坐标
			WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[1]);
			WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[2]);
			WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[3]);
			WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],ac,WayPoint1[4]);

			Bezier(WayPoint1,4,MidGpsPoint);
		}

		m_startpoint = m_GpsData.APiontConverD(currentpoint,startpoint,ac);//把起始点GPS坐标转换为地图坐标
		r = 270 + as - ac;//把起始点的方向角转换为地图坐标系中与x轴正方向夹角;
		if(r!=270)
		{
			r = r - 180;
			r = rad(r);
			y_startpoint.x = m_startpoint.x - e*cos(r);
			y_startpoint.y = m_startpoint.y - e*sin(r);
		}
		else if (r==270)
		{
			y_startpoint.x = m_startpoint.x;
			y_startpoint.y = m_startpoint.y - e;
		}
		y_startpoint = m_GpsData.MaptoGPS(currentpoint,ac,y_startpoint);
		dx = (y_startpoint.x - startpoint.x)/200;
		dy = (y_startpoint.y - startpoint.y)/200;
		y_MidGpsPoint[400];
		for(int i =0;i<200;i++)
		{
			y_MidGpsPoint[i] = MidGpsPoint[i];
		}
		for (int j=200;j<400;j++)
		{
			y_MidGpsPoint[j].x = y_MidGpsPoint[j-1].x + dx;
			y_MidGpsPoint[j].y = y_MidGpsPoint[j-1].y + dy;
		}
		edge = Search_edge(y_MidGpsPoint,vel_Map,40,2,edgepoint_x,edgepoint_y);//判断延长的轨迹是否伸出路沿
	}
	MidGpsPoint[199] = y_startpoint;
	Bezier(MidGpsPoint,199,MidGpsPoint);
	

	/*暂时不考虑红绿灯情况*/

	//检测红绿灯;
	//if(到达P && 红灯)
	//{
	//	停在起始点;
	//	检测红绿灯;
	//	if(红灯)
	//	{
	//		wait();
	//		检测红绿灯;
	//	}
	//}
	//if(到达P && 绿灯)
	//{
	//	Intersection_Driver();
	//}

	//if(P.y >= 412)//到达起始点前3米，切换到过路口状态
	//{
	//    Intersection_Driver();
	//}

}



void CApproachExit::solvepoints(CvPoint2D64f P1,CvPoint2D64f P2,double a2,CvPoint2D64f &P3,CvPoint2D64f &P4,CvPoint2D64f &P5)//初始位姿和目标位姿满足第一种情况下，由当前位姿和终止位姿求出用来生成贝塞尔曲线的其它三个点
{
	P3.x = 256;
	P3.y = a2*256 - a2*P2.x + P2.y;
	P4.x = (P3.x + P1.x)/2;
	P4.y = (P3.y + P1.y)/2;
	P5.x = (P3.x + P2.x)/2;
	P5.y = (P3.y + P2.y)/2;
}

void CApproachExit::solvepoints1(CvPoint2D64f P1,CvPoint2D64f P2,double a2,CvPoint2D64f &P3,CvPoint2D64f &P4,CvPoint2D64f &P5)//初始位姿和目标位姿满足第二种情况下，由当前位姿和终止位姿求出用来生成贝塞尔曲线的其它三个点
{
	P3.x = (256 + P2.x)/2;
	P3.y = (412 + P2.y)/2;
	P4.x = 256;
	P4.y = P3.y;
	P5.x = (a2*P2.x - P2.y + P3.x/a2 + P3.y)/(a2 + 1/a2);
	P5.y = a2*(P5.x - P2.x) + P2.y;
}

void CApproachExit::solvepoints2(CvPoint2D64f P1,CvPoint2D64f P2,CvPoint2D64f &P3,CvPoint2D64f &P4,CvPoint2D64f &P5)//初始位姿和目标位姿满足第三种情况下，由当前位姿和终止位姿求出用来生成贝塞尔曲线的其它三个点
{
	P3.x = (256 + P2.x)/2;
	P3.y = (412 + P2.y)/2;
	P4.x = 256;
	P4.y = P3.y;
	P5.x = P2.x;
	P5.y = P3.y;
}

void CApproachExit::Search_intersection_startpoint(I_Map *map, int t,CvPoint2D64f &qqq,double &kkk)//t:转向，0左转1直行2左转；qqq：获取的点；kkk：获取点的GPS方向角,S:表示方向角是否是90或者270度，0不是，1是
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	app->critical_section.Lock();//锁住
	/*currentpoint = app->GPS_Point;
	ac = app->GPS_Direction;*/

	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;
	
	app->critical_section.Unlock();

	CvPoint2D64f Q1,Q2;
	double U;//获取的斜率
	double V;//获取的点在地图坐标中的方向角
	int S;//S=0代表获取的点在地图坐标中的方向角不是270度或者90度，1则代表是270度或者90度

	if(t==3)//右转
	{
		for (int j=0;j<412;j++)
		{
			for(int i=512;i>0;i--)
			{
				if(map->MapPoint[j][i] == 1 || map->MapPoint[j][i]==11)
				{
					Q1.x = i;
					Q1.y = j;
					goto search11_over;
				}
			}
		}
search11_over:
		for(int j=0;j<412;j++)
		{
			for(int i=0;i<512;i++)
			{
				if(map->MapPoint[j][i] == 2 || map->MapPoint[j][i]==12)
				{
					Q2.x = i;
					Q2.y = j;
					goto search12_over;
				}
			}
		}
	}
search12_over:
	if(t==1)//直行
	{
		for(int j=0;j<412;j++)
		{
			for(int i=512;i>0;i--)
			{
				if(map->MapPoint[j][i] == 2 || map->MapPoint[j][i]==12)
				{
					Q1.x = i;
					Q1.y = j;
					goto search21_over;
				}
			}
		}
search21_over:
		for(int j=0;j<412;j++)
		{
			for(int i=0;i<512;i++)
			{
				if(map->MapPoint[j][i] == 3 || map->MapPoint[j][i]==13)
				{
					Q2.x = i;
					Q2.y = j;
					goto search22_over;
				}
			}
		}
	}
search22_over:
	if(t==2)//左转
	{
		for(int j=0;j<412;j++)
		{
			for(int i=512;i>0;i--)
			{
				if(map->MapPoint[j][i] == 2 || map->MapPoint[j][i]==12)
				{
					Q1.x = i;
					Q1.y = j;
					goto search31_over;
				}
			}
		}
search31_over:
		for(int j=0;j<412;j++)
		{
			for(int i=0;i<512;i++)
			{
				if(map->MapPoint[j][i] == 3 || map->MapPoint[j][i]==13)
				{
					Q2.x = i;
					Q2.y = j;
					goto search32_over;
				}
			}
		}
	}
search32_over:
	qqq.x = (Q1.x +Q2.x)/2;
	qqq.y = (Q1.y +Q2.y)/2;

	qqq = m_GpsData.MaptoGPS(m_gps,m_gpsdir,qqq);//转换为GPS坐标


	//用最小二乘法拟合第一条车道线的斜率
	int X[20]={0};
	int Y[20]={0};
	int n=0;
	double x_sum_average=0;//数组X20个元素的平均值
	double y_sum_average=0;//数组Y20个元素的平均值
	double x_square_sum=0;//数组X20个元素的平方和
	double x_multiply_y=0;//数组X和Y对应元素乘积之和

	for (int j=1;j<512;j++)
	{
		for(int i=1;i<512;i++)
		{
			if(map->MapPoint[j][i] == 1 || map->MapPoint[j][i] == 11)
			{
				X[n]=i;
				Y[n]=j;
				n++;	
				if(n>19)
				{
					goto search_roadline_over;
				}
				break;
			}
		}
	}
search_roadline_over:
	for(int i=0;i<n;i++)
	{
		x_sum_average = x_sum_average + X[i];
		y_sum_average = y_sum_average + Y[i];
		x_square_sum = x_square_sum + X[i]*X[i];
		x_multiply_y = x_multiply_y + X[i]*Y[i];
	}
	x_sum_average = x_sum_average/n;
	y_sum_average = y_sum_average/n;

	if((x_square_sum - n * x_sum_average * x_sum_average)!=0)
	{
		U = (x_multiply_y - n * x_sum_average * y_sum_average)/(x_square_sum - n * x_sum_average * x_sum_average);
		S=0;
	}
	else if((x_square_sum - n * x_sum_average * x_sum_average)==0)
	{
		S=1;
	}


	if(S == 0)
	{
		V = atan(U)*180/3.1415926;
		if(V>=0)
		{
			V = V+180;
		}
		else if(V<0)
		{
			V = V+360;
		}
	}
	else if(S == 1)
	{
		V = 270;
	}

	kkk = V + m_gpsdir - 270;//转换为GPS方向角
}

bool CApproachExit::Search_edge(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down,int &edge_x,int &edge_y)
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
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
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
				if(map->MapPoint[y+m][x+n] == 18)
				{
					edge_x = x+n;
					edge_y = y+m;
					return true;
			
				}
			}
	}
	return false;
}

//int CApproachExit::avoid_edge(CvPoint2D64f s_gps, double s_gpsdir,CvPoint2D64f WayPoint[])//此处的参数与moveway函数意义不同，s_gps和s_gpsdir是路过进入点坐标和方向，本来就在轨迹之上，将轨迹向左平移1米(在地图坐标中为5格)
//{
//	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
//	
//	app->critical_section.Lock();//锁住
//	CvPoint2D64f m_gps = app->GPS_Point;
//	double m_gpsdir = app->GPS_Direction;
//	
//	app->critical_section.Unlock();
//
//	s_gps = m_GpsData.APiontConverD(m_gps,s_gps,m_gpsdir);
//	s_gpsdir = s_gpsdir + 270 - m_gpsdir - 180;
//	s_gpsdir = rad(s_gpsdir);
//
//	CvPoint2D64f s1_gps;
//	s1_gps.x = (int)( s_gps.x - 5*sin(s_gpsdir) );
//	s1_gps.y = (int)( s_gps.y + 5*cos(s_gpsdir) );//平移1米
//
//	s1_gps = m_GpsData.MaptoGPS(m_gps,m_gpsdir,s1_gps);
//	s_gps = m_GpsData.MaptoGPS(m_gps,m_gpsdir,s_gps);
//	
//	double dx = s_gps.x - s1_gps.x;
//	double dy = s_gps.y - s1_gps.y;
//	
//	//outn1<<"偏差"<<dx*100000<<", "<<dy*100000<<endl;
//	for(int i=0;i<200;i++)
//	{
//		WayPoint[i].x -= dx;
//		WayPoint[i].y -= dy;
//
//	}
//	return 1;
//}


CvPoint2D64f CApproachExit::correct_point(CvPoint2D64f s_gps, double s_gpsdir)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
		
	app->critical_section.Lock();//锁住
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;
		
	app->critical_section.Unlock();
	
	s_gps = m_GpsData.APiontConverD(m_gps,s_gps,m_gpsdir);
	s_gpsdir = s_gpsdir + 270 - m_gpsdir - 180;
	s_gpsdir = rad(s_gpsdir);
	
	CvPoint2D64f s1_gps;
	s1_gps.x = (int)( s_gps.x - 5*sin(s_gpsdir) );
	s1_gps.y = (int)( s_gps.y + 5*cos(s_gpsdir) );//平移1米

	s1_gps = m_GpsData.MaptoGPS(m_gps,m_gpsdir,s1_gps);

	return s1_gps;
}

