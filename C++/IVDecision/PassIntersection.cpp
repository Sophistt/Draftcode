#include "StdAfx.h"
#include "PassIntersection.h"
#include <fstream>
using namespace std;
ofstream out111("lukou.txt");
CPassIntersection::CPassIntersection(void)
{
	temp_map = new I_Map;
	ae_history[0] = -100;
	ae_history[1] = -100;
}

CPassIntersection::~CPassIntersection(void)
{
	delete temp_map;
}


void CPassIntersection::PassIntersectionDirve()
{
	///缺少一个将IntersectionMidGpsPoint赋0的步骤
	PassIntersectionDirvecjj();
	//PassIntersectionDirvexy();
}
void CPassIntersection::PassIntersectionDirvecjj()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	CvPoint2D64f point[4] = {0};
	if(way_out)
	{
		way_out = false;
		//GetCross_newtest();
		
		//GetCross_new(point);
		intersection_obnum = 0;
	}

	
}

void CPassIntersection::PassIntersectionDirvexy()
{
	///缺少一个将IntersectionMidGpsPoint赋0的步骤
		CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	CvPoint2D64f point[4] = {0};
	if(way_out)
	{
		way_out = false;
		zuohuandao = 0;
		intersection_obnum = 0;
//		GetCross_new(point);

		next_direction[10] = -100;
		
		endpoint = point[2]/* cvPoint2D64f(LeadPoint[seq_num-1].lat,LeadPoint[seq_num-1].lng)*/;
		
		next_intersection_point = MidGpsPoint[199];
	/*按下路段是直路建立路口模型估计下路段方向ae*/

		//ArrayFuZhi(MidGpsPoint,MidGpsPoint_2);

		ae =  m_GpsData.GetAngle(next_intersection_point,endpoint);
		possible_ae = ae;
		
		ArrayFuZhi(MidGpsPoint,MidGpsPoint_4);
		//ae = 0;
	}
	//ArrayFuZhi(MidGpsPoint,IntersectionMidGpsPoint);
	app->critical_map.Lock();
	temp_map = app->PercepMap;
	app->critical_map.Unlock();
	Intersection_Driver();
	//ArrayFuZhi(MidGpsPoint_1,MidGpsPoint_2);
	

	//CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	//app->critical_section.Lock();//锁住
	//currentpoint = app->GPS_Point;
	//ac = app->GPS_Direction;//读取当前位姿
	//app->critical_section.Unlock();


	//ae_map = 270 + ae - ac;
	//if(ae_map < 0)
	//{
	//	ae_map = ae_map + 360;
	//}
	//else if(ae_map > 360)
	//{
	//	ae_map = ae_map - 360;
	//}
	//endpoint_map = m_GpsData.APiontConverD(currentpoint,endpoint,ac);
	//ae_map = ae_map - 180;
	//ae_map = rad(ae_map);
	//MidGpsPoint_3[199].x = endpoint_map.x - 2*75*cos(ae_map);//15米
	//MidGpsPoint_3[199].y = endpoint_map.y - 2*75*sin(ae_map);
	//MidGpsPoint_3[199] = m_GpsData.MaptoGPS(currentpoint,ac,MidGpsPoint_3[200]);
	//for(int i = 0;i<199;i++)
	//{
	//	MidGpsPoint_3[i] = MidGpsPoint[i];
	//}
	//ArrayFuZhi(MidGpsPoint_3,MidGpsPoint);
	//Bezier(MidGpsPoint_3,200,MidGpsPoint);
} 

int CPassIntersection::GetCross_new(CvSeq *gp,CvPoint2D64f *Point,CvPoint2D64f (&WayPoint)[7],CvPoint2D64f (&IntersectionPath)[200])
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	
	CvPoint2D64f Cross_Pt[8];
	CvPoint2D64f uturnpath[200];
	CvPoint2D64f upath[4];
	//CvPoint2D64f WayPoint[7];
	int seq = 0;
	seq = seq_num;

	Point[0].x = LeadPoint[seq-3].lat;
	Point[0].y = LeadPoint[seq-3].lng;
	
	Point[1].x = LeadPoint[seq-2].lat;
	Point[1].y = LeadPoint[seq-2].lng;	
	
	Point[2].x = LeadPoint[seq-1].lat;
	Point[2].y = LeadPoint[seq-1].lng;

	Point[3].x = LeadPoint[seq].lat;
	Point[3].y = LeadPoint[seq].lng;

	if (Point[2].x == Point[3].x && Point[2].y == Point[3].y)
	{
		Stop();
		return 99;
	}
	double uturn_dir = 0;
	uturn_dir = m_GpsData.GetAngle(Point[1],Point[0]);
	//int lane_num = RNDFGPS.m_mapInfo.pSegment[id11-1].lane_num/2;
	//qiudian(Point,seq);
	//qiudian1(Point,seq,double gps_fangxiang);
	//Point[0] = *(CvPoint2D64f*)cvGetSeqElem( gp, seq-2);
	//Point[1] = *(CvPoint2D64f*)cvGetSeqElem( gp, seq-1);
	//Point[2] = *(CvPoint2D64f*)cvGetSeqElem( gp, seq);
	//Point[3] = *(CvPoint2D64f*)cvGetSeqElem( gp, seq+1);
	
	

	WayPoint[1] = Point[1];
	WayPoint[0] = Point[0];
	WayPoint[6] = Point[3];
	//
	double dist = m_GpsData.GetDistance( Point[0].x,Point[0].y,Point[1].x, Point[1].y);
	WayPoint[0].x = ((dist-50)*Point[1].x + 50*Point[0].x)/dist;
	WayPoint[0].y = ((dist-50)*Point[1].y + 50*Point[0].y)/dist;	


	WayPoint[5] = Point[2];

	dist = m_GpsData.GetDistance( Point[2].x,Point[2].y,Point[3].x, Point[3].y);
	WayPoint[6].x = ((-10)*Point[2].x + (dist+10)*Point[3].x)/dist;
	WayPoint[6].y = ((-10)*Point[2].y + (dist+10)*Point[3].y)/dist;	
	upath[0] = WayPoint[0];
	upath[1] = WayPoint[1];
	upath[2] = WayPoint[5];
	upath[3] = WayPoint[6];
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

	double dir1 = m_GpsData.GetAngle(WayPoint[1],WayPoint[0]);
	double dir2 = m_GpsData.GetAngle(WayPoint[6],WayPoint[5]);
	double dmb_dir = m_GpsData.GetAngle(WayPoint[0],WayPoint[1]);
	if(abs(dir1-dir2)<30||abs(360-abs(dir1-dir2))<30  )
	{
		fx = 1;
	
		WayPoint[3].x = (WayPoint[1].x+WayPoint[5].x)/2.0;
		WayPoint[3].y = (WayPoint[1].y+WayPoint[5].y)/2.0;
	}
	else if(abs(abs(dir1-dir2)-180)<10)
	{
		//uturn.ReturnUTurnPathRNDF(gp,seq,IntersectionPath);
		uturn.ReturnUTurnPathRNDF(seq_num,IntersectionPath,upath,3);
		//app->inter_small = false;
		app->inter_small = true;
		fx = 4;
		return 4;
	}
	else if(abs(dir1-dir2+270)<80||abs(90-dir1+dir2)<80 )
	{
		fx = 3;	
	}
	else if(abs(dir1-dir2+90)<80||abs(270-dir1+dir2)<80)
	{
		fx = 2;
	}
//	int lane_num = RNDFGPS.m_mapInfo.pSegment[i].lane_num;
	
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
	

	WayPoint[2].x = (3*WayPoint[1].x+7*WayPoint[3].x)/10;
	WayPoint[2].y = (3*WayPoint[1].y+7*WayPoint[3].y)/10;
	WayPoint[4].x = (3*WayPoint[5].x+7*WayPoint[3].x)/10;
	WayPoint[4].y = (3*WayPoint[5].y+7*WayPoint[3].y)/10;

	if (fx==3)
	{
		WayPoint[2].x = (2*WayPoint[1].x+8*WayPoint[3].x)/10;
		WayPoint[2].y = (2*WayPoint[1].y+8*WayPoint[3].y)/10;
		WayPoint[4].x = (2*WayPoint[5].x+8*WayPoint[3].x)/10;
		WayPoint[4].y = (2*WayPoint[5].y+8*WayPoint[3].y)/10;
	}
	//**********1206需要对锐角情况特殊处理*********//
	//if (cos((dmb_dir-dir2)*PI/180) > cos(95*PI/180))
	//{

	//}
	if (cos((dmb_dir-dir2)*PI/180) > cos(55*PI/180))
	{
		//WayPoint[2].x = (7*WayPoint[1].x+3*WayPoint[3].x)/10;
		//WayPoint[2].y = (7*WayPoint[1].y+3*WayPoint[3].y)/10;
		//WayPoint[4].x = (7*WayPoint[5].x+3*WayPoint[3].x)/10;
		//WayPoint[4].y = (7*WayPoint[5].y+3*WayPoint[3].y)/10;
		app->inter_small = true;
		double dist1 = m_GpsData.GetDistance(WayPoint[3].x,WayPoint[3].y,WayPoint[5].x,WayPoint[5].y);//w3,w5之间距离
		if(dist1>10)
		{
			WayPoint[4].x = (dist1*WayPoint[3].x+2*(WayPoint[5].x-WayPoint[3].x))/dist1;
			WayPoint[4].y = (dist1*WayPoint[3].y+2*(WayPoint[5].y-WayPoint[3].y))/dist1;
			if(fx==3)
			{
				WayPoint[4].x = (dist1*WayPoint[3].x+6*(WayPoint[5].x-WayPoint[3].x))/dist1;
				WayPoint[4].y = (dist1*WayPoint[3].y+6*(WayPoint[5].y-WayPoint[3].y))/dist1;
			}
		}
		else WayPoint[4] = WayPoint[3];
		double dist2 =  m_GpsData.GetDistance(WayPoint[3].x,WayPoint[3].y,WayPoint[1].x,WayPoint[1].y);//w1,w3之间距离
		if(dist2>10)
		{
			WayPoint[2].x = (dist2*WayPoint[3].x+6*(WayPoint[1].x-WayPoint[3].x))/dist2;
			WayPoint[2].y = (dist2*WayPoint[3].y+6*(WayPoint[1].y-WayPoint[3].y))/dist2;
		}
		else WayPoint[2] = WayPoint[3];

		CvPoint2D64f ruipts[6];
		for (int i=0;i<3;i++)
		{
			ruipts[i].x = WayPoint[i].x;
			ruipts[i].y = WayPoint[i].y;
		}
		for (int i=3;i<6;i++)
		{
			ruipts[i].x = WayPoint[i+1].x;
			ruipts[i].y = WayPoint[i+1].y;
		}

		Bezier(ruipts,5,IntersectionPath);
		app->critical_section.Lock();//锁住
		CvPoint2D64f m_gps = app->GPS_Point;
		double m_gpsdir = app->GPS_Direction;
		app->critical_section.Unlock();//解锁
		//MoveWay(m_gps,m_gpsdir,MidGpsPoint);
		//MoveWay(m_gps,m_gpsdir,WayPoint,4,IntersectionPath);
		MoveWay(m_gps,uturn_dir,ruipts,4,IntersectionPath);

	}
	else
	{
		app->inter_small = false;
		Bezier(WayPoint,6,IntersectionPath);
		app->critical_section.Lock();//锁住
		CvPoint2D64f m_gps = app->GPS_Point;
		double m_gpsdir = app->GPS_Direction;
		app->critical_section.Unlock();//解锁
		//MoveWay(m_gps,m_gpsdir,MidGpsPoint);
		//MoveWay(m_gps,m_gpsdir,WayPoint,4,IntersectionPath);
		MoveWay(m_gps,uturn_dir,WayPoint,4,IntersectionPath);
	}
	double yan_dir =  m_GpsData.GetAngle(IntersectionPath[199],IntersectionPath[195]);;
	//**********1219路口拟合曲线延长**********//
	CvPoint2D64f inter_yanchang[200];
	for (int i = 0;i<100;i++)
	{
		inter_yanchang[i]=IntersectionPath[2*i];
	}
	for(int i=100;i<200;i++)
	{
		inter_yanchang[i]=m_GpsData.MaptoGPS(IntersectionPath[199],yan_dir,cvPoint2D64f(256,412-(i-100)*2));
	}
	for (int i = 0;i<200;i++)
	{
		IntersectionPath[i].x=inter_yanchang[i].x;
		IntersectionPath[i].y=inter_yanchang[i].y;
	}
	//
	//***********1206********//
	//LocalPath.BSpline(Cross_Pt,8,GpsPoint);
	//Bezier(WayPoint,6,IntersectionPath);
	//app->critical_section.Lock();//锁住
	//CvPoint2D64f m_gps = app->GPS_Point;
	//double m_gpsdir = app->GPS_Direction;
	//app->critical_section.Unlock();//解锁
	////MoveWay(m_gps,m_gpsdir,MidGpsPoint);
	////MoveWay(m_gps,m_gpsdir,WayPoint,4,IntersectionPath);
	//MoveWay(m_gps,uturn_dir,WayPoint,4,IntersectionPath);
	
	///*for(int i = 0;i<6;i++)
	//{
	//	outn1<<(Cross_Pt[i].x-31)*100000<<","<<(Cross_Pt[i].y-117)*100000<<endl;
	//}
	//outn1<<"feng ge xian"<<endl;
	//for(int i = 0;i<200;i++)
	//{
	//	outn1<<(GpsPoint[i].x-31)*100000<<","<<(GpsPoint[i].y-117)*100000<<endl;
	//}*/
	//
	return 1;
}

int CPassIntersection::GetCross_lrguihua(CvPoint2D64f (&upoint)[4],CvPoint2D64f (&IntersectionPath)[200])
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	app->critical_section.Lock();
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;
	app->critical_section.Unlock();

	CvPoint2D64f WayPoint[7];

	WayPoint[1] = upoint[1];
	WayPoint[0] = upoint[0];
	WayPoint[6] = upoint[3];

	double dist = m_GpsData.GetDistance( upoint[0].x,upoint[0].y,upoint[1].x, upoint[1].y);
	if(dist<50)
	{
		WayPoint[0].x = ((dist-50)*upoint[1].x + 50*upoint[0].x)/dist;
		WayPoint[0].y = ((dist-50)*upoint[1].y + 50*upoint[0].y)/dist;	
	}

	CvPoint2D64f waypointtmp[200];
	for(int i=0;i<200;i++)
	{
		waypointtmp[i].x= WayPoint[0].x+i*(WayPoint[1].x-WayPoint[0].x)/199.0;
		waypointtmp[i].y= WayPoint[0].y+i*(WayPoint[1].y-WayPoint[0].y)/199.0;
	}
	waypointtmp[199].x=WayPoint[1].x;
	waypointtmp[199].y=WayPoint[1].y;
	double mindis = 100000;
	int v_num = 0;
	double dis;
	for(int i=0;i<200;i++)
	{
		dis = m_GpsData.GetDistance(m_gps.x,m_gps.y,waypointtmp[i].x,waypointtmp[i].y);
		if(dis < mindis)
		{
			mindis= dis;
			v_num = i;
		}
	}
	double dx = waypointtmp[v_num].x - m_gps.x;
	double dy = waypointtmp[v_num].y - m_gps.y;
	for(int i=0;i<2;i++)
	{
		WayPoint[i].x -= dx;
		WayPoint[i].y -= dy;
	}

	WayPoint[5] = upoint[2];

	dist = m_GpsData.GetDistance( upoint[2].x,upoint[2].y,upoint[3].x, upoint[3].y);
	WayPoint[6].x = ((-10)*upoint[2].x + (dist+10)*upoint[3].x)/dist;
	WayPoint[6].y = ((-10)*upoint[2].y + (dist+10)*upoint[3].y)/dist;

	double x1 = WayPoint[0].x;
	double y1 = WayPoint[0].y;
	double x2 = WayPoint[1].x;
	double y2 = WayPoint[1].y;
	double A1 = y2-y1;
	double B1 = -(x2-x1);
	double C1 = x2*y1-x1*y2;
	x1 = WayPoint[5].x;
	y1 = WayPoint[5].y;
	x2 = WayPoint[6].x;
	y2 = WayPoint[6].y;
	double A2 = y2-y1;
	double B2 = -(x2-x1);
	double C2 =x2*y1-x1*y2;

	WayPoint[3].x = (B1*C2-B2*C1)/(A1*B2-A2*B1);
	WayPoint[3].y = (A1*C2-A2*C1)/(A2*B1-A1*B2);

	double dir1 = m_GpsData.GetAngle(WayPoint[1],WayPoint[0]);
	double dir2 = m_GpsData.GetAngle(WayPoint[6],WayPoint[5]);
	double dmb_dir = m_GpsData.GetAngle(WayPoint[0],WayPoint[1]);

	int fxtmp;
	
	if(abs(dir1-dir2+270)<90||abs(90-dir1+dir2)<90 )
	{
		fxtmp = 3;	
	}
	else
	{
		fxtmp = 2;
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

	WayPoint[2].x = (3*WayPoint[1].x+7*WayPoint[3].x)/10;
	WayPoint[2].y = (3*WayPoint[1].y+7*WayPoint[3].y)/10;
	WayPoint[4].x = (3*WayPoint[5].x+7*WayPoint[3].x)/10;
	WayPoint[4].y = (3*WayPoint[5].y+7*WayPoint[3].y)/10;

	//if (fxtmp==3)
	{
		WayPoint[2].x = (2*WayPoint[1].x+8*WayPoint[3].x)/10;
		WayPoint[2].y = (2*WayPoint[1].y+8*WayPoint[3].y)/10;
		WayPoint[4].x = (2*WayPoint[5].x+8*WayPoint[3].x)/10;
		WayPoint[4].y = (2*WayPoint[5].y+8*WayPoint[3].y)/10;
	}
	/*
	if (cos((dmb_dir-dir2)*PI/180) > cos(55*PI/180))
	{
		app->inter_small = true;
		double dist1 = m_GpsData.GetDistance(WayPoint[3].x,WayPoint[3].y,WayPoint[5].x,WayPoint[5].y);
		if(dist1>10)
		{
			WayPoint[4].x = (dist1*WayPoint[3].x+2*(WayPoint[5].x-WayPoint[3].x))/dist1;
			WayPoint[4].y = (dist1*WayPoint[3].y+2*(WayPoint[5].y-WayPoint[3].y))/dist1;
			if(fx==3)
			{
				WayPoint[4].x = (dist1*WayPoint[3].x+6*(WayPoint[5].x-WayPoint[3].x))/dist1;
				WayPoint[4].y = (dist1*WayPoint[3].y+6*(WayPoint[5].y-WayPoint[3].y))/dist1;
			}
		}
		else WayPoint[4] = WayPoint[3];
		double dist2 =  m_GpsData.GetDistance(WayPoint[3].x,WayPoint[3].y,WayPoint[1].x,WayPoint[1].y);
		if(dist2>10)
		{
			WayPoint[2].x = (dist2*WayPoint[3].x+6*(WayPoint[1].x-WayPoint[3].x))/dist2;
			WayPoint[2].y = (dist2*WayPoint[3].y+6*(WayPoint[1].y-WayPoint[3].y))/dist2;
		}
		else WayPoint[2] = WayPoint[3];

		CvPoint2D64f ruipts[6];
		for (int i=0;i<3;i++)
		{
			ruipts[i].x = WayPoint[i].x;
			ruipts[i].y = WayPoint[i].y;
		}
		for (int i=3;i<6;i++)
		{
			ruipts[i].x = WayPoint[i+1].x;
			ruipts[i].y = WayPoint[i+1].y;
		}

		Bezier(ruipts,5,IntersectionPath);
	}
	else
	*/
	{
		app->inter_small = false;
		Bezier(WayPoint,6,IntersectionPath);
	}
	double yan_dir =  m_GpsData.GetAngle(IntersectionPath[199],IntersectionPath[195]);

	CvPoint2D64f inter_yanchang[200];
	for (int i = 0;i<100;i++)
	{
		inter_yanchang[i]=IntersectionPath[2*i];
	}
	for(int i=100;i<200;i++)
	{
		inter_yanchang[i]=m_GpsData.MaptoGPS(IntersectionPath[199],yan_dir,cvPoint2D64f(256,412-(i-100)*2));
	}
	for (int i = 0;i<200;i++)
	{
		IntersectionPath[i].x=inter_yanchang[i].x;
		IntersectionPath[i].y=inter_yanchang[i].y;
	}
	return 1;
}
int CPassIntersection::GetCross_newexe(CvPoint2D64f (&IntersectionPath)[200])
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	CvPoint2D64f Point[4];
	CvPoint2D64f Cross_Pt[8];
	CvPoint2D64f WayPoint[7];
	int seq = 0;
	seq = seq_num;
	Point[0].x = LeadPoint[seq-3].lat;
	Point[0].y = LeadPoint[seq-3].lng;
	
	Point[1].x = LeadPoint[seq-2].lat;
	Point[1].y = LeadPoint[seq-2].lng;	
	
	Point[2].x = LeadPoint[seq-1].lat;
	Point[2].y = LeadPoint[seq-1].lng;

	Point[3].x = LeadPoint[seq].lat;
	Point[3].y = LeadPoint[seq].lng;


	//按路点取最后两点
	
	//int lane_num = RNDFGPS.m_mapInfo.pSegment[ID1-1].lane_num/2;
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
	if( LeadPoint[seq-2].param2 == 4)
	{
			//uturn.ReturnUTurnPath(seq_num,IntersectionPath);
			return 4;
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
	
	WayPoint[2].x = (WayPoint[1].x+9*WayPoint[3].x)/10;
	WayPoint[2].y = (WayPoint[1].y+9*WayPoint[3].y)/10;
	WayPoint[4].x = (WayPoint[5].x+9*WayPoint[3].x)/10;
	WayPoint[4].y = (WayPoint[5].y+9*WayPoint[3].y)/10;
	//LocalPath.BSpline(Cross_Pt,8,GpsPoint);
	Bezier(WayPoint,6,IntersectionPath);
	app->critical_section.Lock();//锁住
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;
	app->critical_section.Unlock();//解锁
	MoveWay(m_gps,m_gpsdir,IntersectionPath);
	///*for(int i = 0;i<6;i++)
	//{
	//	outn1<<(Cross_Pt[i].x-31)*100000<<","<<(Cross_Pt[i].y-117)*100000<<endl;
	//}
	//outn1<<"feng ge xian"<<endl;
	//for(int i = 0;i<200;i++)
	//{
	//	outn1<<(GpsPoint[i].x-31)*100000<<","<<(GpsPoint[i].y-117)*100000<<endl;
	//}*/
	//
	return 1;

}



void CPassIntersection::Intersection_Driver()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	double r;//终止点在地图坐标系中与x轴正方向的夹角
	double k4;
	CvPoint2D64f P;//终止点的延长点

	double d = 0;//设置终止点向前延长距离，每延长一米增加-5，如延长10米，则设置d为-50.
	double e = 300;//设置轨迹点向前延长距离，每延长一米增加5，如延长10米，则设置e为50.

	app->critical_section.Lock();//锁住
	currentpoint = app->GPS_Point;
	ac = app->GPS_Direction;//读取当前位姿

	app->critical_section.Unlock();
	////////////////////////假定路口，路上路点连续//////////////
	endpoint = cvPoint2D64f(LeadPoint[seq_num-1].lat,LeadPoint[seq_num-1].lng);
	next_intersection_point = cvPoint2D64f(LeadPoint[seq_num].lat,LeadPoint[seq_num].lng);
	


	//bool ob_fangxiang = SearchObstacle(MidGpsPoint_4,temp_map,50,2);

	double ae1 = correct_endpoint_direction_1(temp_map,endpoint,seq_num,ae);
	//double ae2 = correct_endpoint_direction_2(temp_map,endpoint,seq_num,ae);

	/*if( (fangxianggengxin1 == 0) && (fangxianggengxin2 == 0) )
	{}
	else if( (fangxianggengxin1 == 1)&&(fangxianggengxin2 == 0) )
	{
		if( (difference_ae1 > -90) && (difference_ae1<90) )
		{
			ae = ae1;
		}
	}
	else if( (fangxianggengxin1 == 0)&&(fangxianggengxin2 == 1) )
	{
		if( (difference_ae2 > -90) && (difference_ae2<90) )
		{
			ae = ae2;
		}
	}
	else if( (fangxianggengxin1 == 1)&&(fangxianggengxin2 == 1) )
	{
		if( abs(difference_ae1) <= abs(difference_ae2) )
		{
			ae = ae1;
		}
		else if( abs(difference_ae1) > abs(difference_ae2) )
		{
			ae = ae2;
		}
	}*/
	//possible_ae;

	if(fangxianggengxin1)
	{
		if( abs(difference_ae1) < 70)
		{
			ae = ae1;
		}
		planpath(endpoint,ae);
		ArrayFuZhi(MidGpsPoint_1,MidGpsPoint);
	}



	bool ob = SearchObstacle(MidGpsPoint,temp_map,app->GPS_Speed*2+20,0);
	out111<<"ob "<<ob<<endl;
	
	if(ob)
	{
		app->drive_obstacle = 15/3.6;

		for(int i = 0;i<10;i++)
		{
			Stop();

			//app->stop = true;
			Sleep(100);
			vel_Map = app->PercepMap;
			if( SearchObstacle(MidGpsPoint,vel_Map,app->GPS_Speed*2+20,0))
			intersection_obnum++;
			if(intersection_obnum > 6)
				break;
		}

		out111<<"intersection_obnum "<<intersection_obnum<<endl;
		if(intersection_obnum > 6)
		{
			endpoint = correct_endpoint_location(endpoint,next_intersection_point,ae);
			out111<<"xiu zheng le "<<endl;
			ArrayFuZhi(MidGpsPoint_1,MidGpsPoint);


			

			//MidGpsPoint[199].x = endpoint_map.x - 75*cos(ae_map);//15米
			//MidGpsPoint[199].y = endpoint_map.y - 75*sin(ae_map);
			//MidGpsPoint[199] = m_GpsData.MaptoGPS(currentpoint,ac,MidGpsPoint[199]);
			
			
			intersection_obnum = 0;	
			//app->intersec_stopflag = false;
		}
		/*app->intersec_stopflag = true;*/
	}
	else
		app->drive_obstacle = 40/3.6;
	intersection_obnum = 0;


	//int edgepoint_x,edgepoint_y;
	//ob = Search_edge(MidGpsPoint_1,temp_map,30,0,edgepoint_x,edgepoint_y);
	//if(ob)
	//{
	//	ArrayFuZhi(MidGpsPoint_1,MidGpsPoint);
	//	Sleep(1000);
	//}


	//correct_endpoint_direction(endpoint,ae);
	
	
	

	

}

void CPassIntersection::solvepoints(CvPoint2D64f P1,CvPoint2D64f P2,double a2,CvPoint2D64f &P3,CvPoint2D64f &P4,CvPoint2D64f &P5)//初始位姿和目标位姿满足第一种情况下，由当前位姿和终止位姿求出用来生成贝塞尔曲线的其它三个点
{            
	P3.x = 256;
	P3.y = a2*256 - a2*P2.x + P2.y;
	P4.x = (3*P3.x + P1.x)/4;
	P4.y = (3*P3.y + P1.y)/4;
	P5.x = (3*P3.x + P2.x)/4;
	P5.y = (3*P3.y + P2.y)/4;
}

void CPassIntersection::solvepoints1(CvPoint2D64f P1,CvPoint2D64f P2,double a2,CvPoint2D64f &P3,CvPoint2D64f &P4,CvPoint2D64f &P5)//初始位姿和目标位姿满足第二种情况下，由当前位姿和终止位姿求出用来生成贝塞尔曲线的其它三个点
{
	P3.x = (256 + P2.x)/2;
	P3.y = (412 + P2.y)/2;
	P4.x = 256;
	P4.y = P3.y;
	P5.x = (a2*P2.x - P2.y + P3.x/a2 + P3.y)/(a2 + 1/a2);
	P5.y = a2*(P5.x - P2.x) + P2.y;
}

void CPassIntersection::solvepoints2(CvPoint2D64f P1,CvPoint2D64f P2,CvPoint2D64f &P3,CvPoint2D64f &P4,CvPoint2D64f &P5)//初始位姿和目标位姿满足第三种情况下，由当前位姿和终止位姿求出用来生成贝塞尔曲线的其它三个点
{
	P3.x = (256 + P2.x)/2;
	P3.y = (412 + P2.y)/2;
	P4.x = 256;
	P4.y = P3.y;
	P5.x = P2.x;
	P5.y = P3.y;
}

void CPassIntersection::correct_endpoint_direction(CvPoint2D64f ppp,double &lll)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	app->critical_section.Lock();//锁住
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;

	app->critical_section.Unlock();

	CvPoint2D64f next_road_GpsPoint[400];

	next_road_GpsPoint[0] = m_GpsData.APiontConverD(m_gps,ppp,m_gpsdir);
	double map_lll =  270 + lll - m_gpsdir;//转化为地图坐标中的角度
	double map_lll_0;
	map_lll = rad(map_lll - 270);
	map_lll_0 = map_lll;
	double dx = sin(map_lll);//每20cm长设置一个点
	double dy = -cos(map_lll);

	for (int i=1;i<400;i++)
	{
		next_road_GpsPoint[i].x = next_road_GpsPoint[i-1].x + dx;
		next_road_GpsPoint[i].y = next_road_GpsPoint[i-1].y + dy;		
	}

	for (int i=0;i<400;i++)
	{
		next_road_GpsPoint[i] = m_GpsData.MaptoGPS(m_gps,m_gpsdir,next_road_GpsPoint[i]);	
	}

	int next_edgepoint_x,next_edgepoint_y;
	int change_angle = 2;//每次改变两度

	next_road_edge0 = Search_edge(next_road_GpsPoint,vel_Map,40,2,next_edgepoint_x,next_edgepoint_y);
	bool next_road_edge1,next_road_edge2;

	if (next_road_edge0 == 1)
	{
		next_road_edge1 = 1;
		next_road_edge2 = 1;
	}
	else if (next_road_edge0 == 0)
	{
		next_road_edge1 = 0;
		next_road_edge2 = 0;
	}

	double map_lll_1 = map_lll;
	double map_lll_2 = map_lll;

	while ((next_road_edge1 == 1)&&(next_road_edge2 == 1))
	{
		CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

		app->critical_section.Lock();//锁住
		m_gps = app->GPS_Point;
		m_gpsdir = app->GPS_Direction;

		app->critical_section.Unlock();

		map_lll_1 = map_lll_1 - rad(1)*change_angle;//先向左改变两度试试看
		dx = sin(map_lll_1);
		dy = -cos(map_lll_1);
		next_road_GpsPoint[0] = m_GpsData.APiontConverD(m_gps,ppp,m_gpsdir);

		for (int i=1;i<400;i++)
		{
			next_road_GpsPoint[i].x = next_road_GpsPoint[i-1].x + dx;
			next_road_GpsPoint[i].y = next_road_GpsPoint[i-1].y + dy;		
		}
		for (int i=0;i<400;i++)
		{
			next_road_GpsPoint[i] = m_GpsData.MaptoGPS(m_gps,m_gpsdir,next_road_GpsPoint[i]);	
		}
		next_road_edge1 = Search_edge(next_road_GpsPoint,vel_Map,40,2,next_edgepoint_x,next_edgepoint_y);

		map_lll_2 = map_lll_2 + rad(1)*change_angle;//再向右改变两度试试看
		dx = 2*sin(map_lll_2);
		dy = -2*cos(map_lll_2);
		next_road_GpsPoint[0] = m_GpsData.APiontConverD(m_gps,ppp,m_gpsdir);

		for (int i=1;i<400;i++)
		{
			next_road_GpsPoint[i].x = next_road_GpsPoint[i-1].x + dx;
			next_road_GpsPoint[i].y = next_road_GpsPoint[i-1].y + dy;		
		}
		for (int i=0;i<400;i++)
		{
			next_road_GpsPoint[i] = m_GpsData.MaptoGPS(m_gps,m_gpsdir,next_road_GpsPoint[i]);	
		}
		next_road_edge2 = Search_edge(next_road_GpsPoint,vel_Map,40,2,next_edgepoint_x,next_edgepoint_y);
	}

	if((next_road_edge1 == 0) || (next_road_edge2==0))
	{
		next_road_edge0 = 0;
	}

	if (next_road_edge1 == 0)
	{
		lll = map_lll_1;
	}
	else if (next_road_edge2 == 0)
	{
		lll = map_lll_2;
	}
	double b = lll - map_lll_0;
	if((b > -rad(30)) && (b < rad(30)))
	{}
	else
	{
		lll = map_lll_0;
	}

	lll = lll*180/3.1415926;
	lll = lll + m_gpsdir;
	if (lll < 0)
	{
		lll = lll + 360;
	}
	else if ( lll > 360)
	{
		lll = lll - 360;
	}
}

/*左右平移法修正路口出点的位置*/

//CvPoint2D64f CPassIntersection::correct_endpoint_location(CvPoint2D64f ppp,double lll)
//{
//	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
//
//	app->critical_section.Lock();//锁住
//	CvPoint2D64f m_gps = app->GPS_Point;
//	double m_gpsdir = app->GPS_Direction;
//
//	app->critical_section.Unlock();
//
//	int next_edgepoint_x,next_edgepoint_y;
//	double map_lll,k4,r;
//	double d = 0;
//
//	CvPoint2D64f ppp_correct,map_ppp,map_ppp1,map_ppp2,P;
//	CvPoint2D64f MidGpsPoint_1[200];
//
//	map_ppp = m_GpsData.APiontConverD(m_gps,ppp,m_gpsdir);
//	map_ppp1 = map_ppp;
//	map_ppp2 = map_ppp;
//
//	for (int i=0;i<200;i++)
//	{
//		MidGpsPoint_1[i] = MidGpsPoint[i];
//	}
//
//	bool next_road_edge = Search_edge(MidGpsPoint_1,vel_Map,40,2,next_edgepoint_x,next_edgepoint_y);
//
//	bool next_road_edge1,next_road_edge2;
//
//	next_road_edge1 = next_road_edge;
//	next_road_edge2 = next_road_edge;
//
//	while ((next_road_edge1 == 1)&&(next_road_edge2 == 1))
//	{
//		CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
//
//		app->critical_section.Lock();//锁住
//		m_gps = app->GPS_Point;
//		m_gpsdir = app->GPS_Direction;
//
//		app->critical_section.Unlock();
//
//		map_lll = lll - m_gpsdir;
//		map_lll = rad(map_lll);
//		map_ppp1.x = map_ppp1.x - 5*cos(map_lll);//先向左平移1米试试看
//		map_ppp1.y = map_ppp1.y - 5*sin(map_lll);
//
//		WayPoint1[0].x = 256;
//		WayPoint1[0].y = 412;
//
//		WayPoint1[4] = m_GpsData.APiontConverD(m_gps,map_ppp1,m_gpsdir);
//		
//		r = 270 + lll - m_gpsdir;
//		
//
//		if (r < 0)
//		{
//			r = r + 360;
//		}
//		else if (r > 360)
//		{
//			r = r - 360;
//		}
//
//
//		if(r != 270)
//		{
//			r = rad(r);
//			k4 = tan(r);
//
//			P.x = WayPoint1[4].x - d*cos(r);
//			P.y = WayPoint1[4].y - d*sin(r);
//
//
//			WayPoint1[4] = P;
//
//			solvepoints(WayPoint1[0],WayPoint1[4],k4,WayPoint1[2],WayPoint1[1],WayPoint1[3]);
//
//			if( (WayPoint1[2].y < 412) && ( ((WayPoint1[4].x < 256)&&((r > 3.1415926 )&&(r < 1.5*3.1415926))) || ((WayPoint1[4].x > 256)&&((r > 1.5*3.1415926)&&(r < 2*3.1415926))) ))
//			{
//				WayPoint1[0] = m_gps;
//				WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[1]);
//				WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[2]);
//				WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[3]);
//				WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[4]);
//
//				Bezier(WayPoint1,4,MidGpsPoint_1);
//			}
//			else
//			{
//				solvepoints1(WayPoint1[0],WayPoint1[4],k4,WayPoint1[2],WayPoint1[1],WayPoint1[3]);
//				WayPoint1[0] = m_gps;
//				WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[1]);
//				WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[2]);
//				WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[3]);
//				WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[4]);
//
//				Bezier(WayPoint1,4,MidGpsPoint_1);
//			}
//
//		}
//
//		else if(r == 270)
//		{
//			r = rad(r);
//
//			P.x = WayPoint1[4].x - d*cos(r);
//			P.y = WayPoint1[4].y - d*sin(r);
//
//
//			WayPoint1[4] = P;
//
//			solvepoints2(WayPoint1[0],WayPoint1[4],WayPoint1[2],WayPoint1[1],WayPoint1[3]);
//			WayPoint1[0] = m_gps;
//			WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[1]);
//			WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[2]);
//			WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[3]);
//			WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[4]);
//
//			Bezier(WayPoint1,4,MidGpsPoint_1);
//		}
//		next_road_edge1 = Search_edge(MidGpsPoint_1,vel_Map,40,2,next_edgepoint_x,next_edgepoint_y);
//
//
//		map_ppp2.x = map_ppp2.x + 5*cos(map_lll);//再向右平移1米试试看
//		map_ppp2.y = map_ppp2.y + 5*sin(map_lll);
//
//		WayPoint1[0].x = 256;
//		WayPoint1[0].y = 412;
//
//		WayPoint1[4] = m_GpsData.APiontConverD(m_gps,map_ppp2,m_gpsdir);
//
//		r = 270 + lll - m_gpsdir;
//
//
//		if (r < 0)
//		{
//			r = r + 360;
//		}
//		else if (r > 360)
//		{
//			r = r - 360;
//		}
//
//
//		if(r != 270)
//		{
//			r = rad(r);
//			k4 = tan(r);
//
//			P.x = WayPoint1[4].x - d*cos(r);
//			P.y = WayPoint1[4].y - d*sin(r);
//
//
//			WayPoint1[4] = P;
//
//			solvepoints(WayPoint1[0],WayPoint1[4],k4,WayPoint1[2],WayPoint1[1],WayPoint1[3]);
//
//			if( (WayPoint1[2].y < 412) && ( ((WayPoint1[4].x < 256)&&((r > 3.1415926 )&&(r < 1.5*3.1415926))) || ((WayPoint1[4].x > 256)&&((r > 1.5*3.1415926)&&(r < 2*3.1415926))) ))
//			{
//				WayPoint1[0] = m_gps;
//				WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[1]);
//				WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[2]);
//				WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[3]);
//				WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[4]);
//
//				Bezier(WayPoint1,4,MidGpsPoint_1);
//			}
//			else
//			{
//				solvepoints1(WayPoint1[0],WayPoint1[4],k4,WayPoint1[2],WayPoint1[1],WayPoint1[3]);
//				WayPoint1[0] = m_gps;
//				WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[1]);
//				WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[2]);
//				WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[3]);
//				WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[4]);
//
//				Bezier(WayPoint1,4,MidGpsPoint_1);
//			}
//
//		}
//
//		else if(r == 270)
//		{
//			r = rad(r);
//
//			P.x = WayPoint1[4].x - d*cos(r);
//			P.y = WayPoint1[4].y - d*sin(r);
//
//
//			WayPoint1[4] = P;
//
//			solvepoints2(WayPoint1[0],WayPoint1[4],WayPoint1[2],WayPoint1[1],WayPoint1[3]);
//			WayPoint1[0] = m_gps;
//			WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[1]);
//			WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[2]);
//			WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[3]);
//			WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[4]);
//
//			Bezier(WayPoint1,4,MidGpsPoint_1);
//		}
//		next_road_edge2 = Search_edge(MidGpsPoint_1,vel_Map,40,2,next_edgepoint_x,next_edgepoint_y);
//	}
//
//	if (next_road_edge1 == 0)
//	{
//		ppp_correct = map_ppp1;
//	}
//	else if (next_road_edge2 == 0)
//	{
//		ppp_correct = map_ppp2;
//	}
//	ppp_correct = m_GpsData.MaptoGPS(m_gps,m_gpsdir,ppp_correct);
//	return ppp_correct;
//}

CvPoint2D64f CPassIntersection::correct_endpoint_location(CvPoint2D64f ppp,CvPoint2D64f &qqq,double lll)//按向外扩展圆的方法进行修正
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	app->critical_section.Lock();//锁住
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;
 
	app->critical_section.Unlock();

	int next_edgepoint_x,next_edgepoint_y;
	
	double map_lll,k4,r;
	double d = 0;
	int yichu = 1;
	zuohuandao = 0;

	CvPoint2D64f ppp_correct,map_ppp,map_ppp_ex,P;
	

	map_ppp = m_GpsData.APiontConverD(m_gps,ppp,m_gpsdir);
	map_ppp_ex = map_ppp;

	for (int i=0;i<200;i++)
	{
		MidGpsPoint_1[i] = MidGpsPoint[i];
		MidGpsPoint_6[i] = MidGpsPoint[i];
	}
	temp_map = app->PercepMap;
	next_road_edge = Search_edge(MidGpsPoint_1,temp_map,app->GPS_Speed*2+20,0,next_edgepoint_x,next_edgepoint_y);
	int j = 1;
	int t;
	double u;

	//double distance;
	//int num;
	//double min_distance = 100000;
	//CvPoint2D64f yanchang_endpoint;
	//for(int i=0;i<200;i++)//求出规划轨迹上离车当前位置最近的点
	//{
	//	distance = sqrt((MidGpsPoint_1[i].x - m_gps.x)*(MidGpsPoint_1[i].x - m_gps.x) + (MidGpsPoint_1[i].y - m_gps.y)*(MidGpsPoint_1[i].y - m_gps.y));
	//	if (min_distance > distance)
	//	{
	//		min_distance = distance;
	//		num = i;
	//	}
	//}
	//for(int i=0;i<=num;i++)
	//{
	//	MidGpsPoint_2[i] = MidGpsPoint_1[i];
	//}

	while (next_road_edge == 1)
	{
		CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
		vel_Map = app->PercepMap;
		app->critical_section.Lock();//锁住
		m_gps = app->GPS_Point;
		m_gpsdir = app->GPS_Direction;

		app->critical_section.Unlock();

		t = 7*j-1;//在扩展的圆上考察点的个数，半径每增加1，考察的点的个数增加7（大概每一米远考察一个点）

		for (int k=t;k>=0;k--)
		{
			u = k*360/(t+1);
			u = rad(u);
			map_ppp_ex.x = 5*j*cos(u) + map_ppp.x;
			map_ppp_ex.y = 5*j*sin(u) + map_ppp.y;

			WayPoint1[0].x = 256;
			WayPoint1[0].y = 412;
			WayPoint1[4] = map_ppp_ex;

			r = 270 + lll - m_gpsdir;

			if (r < 0)
			{
				r = r + 360; 
			}
			else if (r > 360)
			{
				r = r - 360;
			}


			if(r != 270)
			{
				r = rad(r);
				k4 = tan(r);

				P.x = WayPoint1[4].x - d*cos(r);
				P.y = WayPoint1[4].y - d*sin(r);


				WayPoint1[4] = P;

				solvepoints(WayPoint1[0],WayPoint1[4],k4,WayPoint1[2],WayPoint1[1],WayPoint1[3]);

				if( (WayPoint1[2].y < 412) && ( ((WayPoint1[4].x < 256)&&((r > 0.5*3.1415926 )&&(r < 1.5*3.1415926))) || ((WayPoint1[4].x > 256)&&( ((r > 1.5*3.1415926)&&(r < 2*3.1415926))||((r >= 0)&&(r < 0.5*3.1415926)))) ))
				{
					WayPoint1[0] = m_gps;
					WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[1]);
					WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[2]);
					WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[3]);
					WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[4]);

					Bezier(WayPoint1,4,MidGpsPoint_1);
				}
				else
				{
					solvepoints1(WayPoint1[0],WayPoint1[4],k4,WayPoint1[2],WayPoint1[1],WayPoint1[3]);
					WayPoint1[0] = m_gps;
					WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[1]);
					WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[2]);
					WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[3]);
					WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[4]);

					Bezier(WayPoint1,4,MidGpsPoint_1);
				}

			}

			else if(r == 270)
			{
				r = rad(r);

				P.x = WayPoint1[4].x - d*cos(r);
				P.y = WayPoint1[4].y - d*sin(r);


				WayPoint1[4] = P;

				solvepoints2(WayPoint1[0],WayPoint1[4],WayPoint1[2],WayPoint1[1],WayPoint1[3]);
				WayPoint1[0] = m_gps;
				WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[1]);
				WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[2]);
				WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[3]);
				WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[4]);

				Bezier(WayPoint1,4,MidGpsPoint_1);
			}

			app->critical_section.Lock();//锁住
			currentpoint = app->GPS_Point;
			ac = app->GPS_Direction;//读取当前位姿
			app->critical_section.Unlock();
			
			
			ae_map = 270 + lll - ac;
			if(ae_map < 0)
			{
				ae_map = ae_map + 360;
			}
			else if(ae_map > 360)
			{
				ae_map = ae_map - 360;
			}
			ArrayFuZhi(MidGpsPoint_1,MidGpsPoint_4);
			endpoint_map = m_GpsData.APiontConverD(currentpoint,MidGpsPoint_1[179],ac);
			ae_map = ae_map - 180;
			ae_map = rad(ae_map);
			//MidGpsPoint_3[200].x = endpoint_map.x - 75*cos(ae_map);//15米
			//MidGpsPoint_3[200].y = endpoint_map.y - 75*sin(ae_map);
			//MidGpsPoint_3[200] = m_GpsData.MaptoGPS(currentpoint,ac,MidGpsPoint_3[200]);
			//for(int i = 0;i<200;i++)
			//{
			//	MidGpsPoint_3[i] = MidGpsPoint_1[i];
			//}
			//Bezier(MidGpsPoint_3,200,MidGpsPoint_1);

			MidGpsPoint_1[199].x = endpoint_map.x - 100*cos(ae_map);//15米
			MidGpsPoint_1[199].y = endpoint_map.y - 100*sin(ae_map);

			endpoint_map = m_GpsData.APiontConverD(currentpoint,MidGpsPoint_1[159],ac);

			MidGpsPoint_4[199].x = endpoint_map.x - 200*cos(ae_map);//15米
			MidGpsPoint_4[199].y = endpoint_map.y - 200*sin(ae_map);

			for(int i=198;i>179;i--)
			{
				MidGpsPoint_1[i].x = MidGpsPoint_1[i+1].x + 5*cos(ae_map);
				MidGpsPoint_1[i].y = MidGpsPoint_1[i+1].y + 5*sin(ae_map);
			}
			for(int i=198;i>159;i--)
			{
				MidGpsPoint_4[i].x = MidGpsPoint_4[i+1].x + 5*cos(ae_map);
				MidGpsPoint_4[i].y = MidGpsPoint_4[i+1].y + 5*sin(ae_map);
			}
			
			/*MidGpsPoint_1[199] = m_GpsData.MaptoGPS(currentpoint,ac,MidGpsPoint_1[199]);
			MidGpsPoint_1[198].x = (MidGpsPoint_1[197].x + MidGpsPoint_1[199].x)/2;
			MidGpsPoint_1[198].y = (MidGpsPoint_1[197].y + MidGpsPoint_1[199].y)/2;*/

			for(int i=180;i<200;i++)
			{
				MidGpsPoint_1[i] = m_GpsData.MaptoGPS(currentpoint,ac,MidGpsPoint_1[i]);
			}

			for(int i=160;i<200;i++)
			{
				MidGpsPoint_4[i] = m_GpsData.MaptoGPS(currentpoint,ac,MidGpsPoint_4[i]);
			}


			temp_map = app->PercepMap;
			next_road_edge = Search_edge(MidGpsPoint_1,temp_map,app->GPS_Speed*2+20,0,next_edgepoint_x,next_edgepoint_y);

			if (next_road_edge == 0)
			{
				break;
			}

		}
		ArrayFuZhi(MidGpsPoint_1,MidGpsPoint_5);
		
		j++;

		if((j>=15)&&(next_road_edge == 1))
		{
			//ArrayFuZhi(MidGpsPoint_6,MidGpsPoint_1);
			ArrayFuZhi(MidGpsPoint_6,MidGpsPoint_5);
			map_ppp_ex = map_ppp;
			yichu = 0;
			break;
		}
		
	}

	j = 1;


	if(next_road_edge == 1)
	{
		CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

		app->critical_section.Lock();//锁住
		m_gps = app->GPS_Point;
		m_gpsdir = app->GPS_Direction;

		app->critical_section.Unlock();


		WayPoint1[0].x = 256;
		WayPoint1[0].y = 412;
		map_ppp = m_GpsData.APiontConverD(m_gps,ppp,m_gpsdir);
		WayPoint1[4] = map_ppp;

		r = 270 + lll - m_gpsdir;

		if (r < 0)
		{
			r = r + 360; 
		}
		else if (r > 360)
		{
			r = r - 360;
		}


		if(r != 270)
		{
			r = rad(r);
			k4 = tan(r);

			P.x = WayPoint1[4].x - d*cos(r);
			P.y = WayPoint1[4].y - d*sin(r);


			WayPoint1[4] = P;

			solvepoints(WayPoint1[0],WayPoint1[4],k4,WayPoint1[2],WayPoint1[1],WayPoint1[3]);

			if( (WayPoint1[2].y < 412) && ( ((WayPoint1[4].x < 256)&&((r > 0.5*3.1415926 )&&(r < 1.5*3.1415926))) || ((WayPoint1[4].x > 256)&&( ((r > 1.5*3.1415926)&&(r < 2*3.1415926))||((r >= 0)&&(r < 0.5*3.1415926)))) ))
			{
				WayPoint1[0] = m_gps;
				WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[1]);
				WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[2]);
				WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[3]);
				WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[4]);

				Bezier(WayPoint1,4,MidGpsPoint_1);
			}
			else
			{
				solvepoints1(WayPoint1[0],WayPoint1[4],k4,WayPoint1[2],WayPoint1[1],WayPoint1[3]);
				WayPoint1[0] = m_gps;
				WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[1]);
				WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[2]);
				WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[3]);
				WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[4]);

				Bezier(WayPoint1,4,MidGpsPoint_1);
			}

		}

		else if(r == 270)
		{
			r = rad(r);

			P.x = WayPoint1[4].x - d*cos(r);
			P.y = WayPoint1[4].y - d*sin(r);


			WayPoint1[4] = P;

			solvepoints2(WayPoint1[0],WayPoint1[4],WayPoint1[2],WayPoint1[1],WayPoint1[3]);
			WayPoint1[0] = m_gps;
			WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[1]);
			WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[2]);
			WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[3]);
			WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[4]);

			Bezier(WayPoint1,4,MidGpsPoint_1);
		}

		app->critical_section.Lock();//锁住
		currentpoint = app->GPS_Point;
		ac = app->GPS_Direction;//读取当前位姿
		app->critical_section.Unlock();
		
		
		ae_map = 270 + lll - ac;
		if(ae_map < 0)
		{
			ae_map = ae_map + 360;
		}
		else if(ae_map > 360)
		{
			ae_map = ae_map - 360;
		}
		ArrayFuZhi(MidGpsPoint_1,MidGpsPoint_4);
		endpoint_map = m_GpsData.APiontConverD(currentpoint,MidGpsPoint_1[179],ac);
		ae_map = ae_map - 180;
		ae_map = rad(ae_map);
		//MidGpsPoint_3[200].x = endpoint_map.x - 75*cos(ae_map);//15米
		//MidGpsPoint_3[200].y = endpoint_map.y - 75*sin(ae_map);
		//MidGpsPoint_3[200] = m_GpsData.MaptoGPS(currentpoint,ac,MidGpsPoint_3[200]);
		//for(int i = 0;i<200;i++)
		//{
		//	MidGpsPoint_3[i] = MidGpsPoint_1[i];
		//}
		//Bezier(MidGpsPoint_3,200,MidGpsPoint_1);

		MidGpsPoint_1[199].x = endpoint_map.x - 100*cos(ae_map);//15米
		MidGpsPoint_1[199].y = endpoint_map.y - 100*sin(ae_map);

		endpoint_map = m_GpsData.APiontConverD(currentpoint,MidGpsPoint_1[159],ac);

		MidGpsPoint_4[199].x = endpoint_map.x - 200*cos(ae_map);//15米
		MidGpsPoint_4[199].y = endpoint_map.y - 200*sin(ae_map);

		for(int i=198;i>179;i--)
		{
			MidGpsPoint_1[i].x = MidGpsPoint_1[i+1].x + 5*cos(ae_map);
			MidGpsPoint_1[i].y = MidGpsPoint_1[i+1].y + 5*sin(ae_map);
		}
		for(int i=198;i>159;i--)
		{
			MidGpsPoint_4[i].x = MidGpsPoint_4[i+1].x + 5*cos(ae_map);
			MidGpsPoint_4[i].y = MidGpsPoint_4[i+1].y + 5*sin(ae_map);
		}
		
		/*MidGpsPoint_1[199] = m_GpsData.MaptoGPS(currentpoint,ac,MidGpsPoint_1[199]);
		MidGpsPoint_1[198].x = (MidGpsPoint_1[197].x + MidGpsPoint_1[199].x)/2;
		MidGpsPoint_1[198].y = (MidGpsPoint_1[197].y + MidGpsPoint_1[199].y)/2;*/

		for(int i=180;i<200;i++)
		{
			MidGpsPoint_1[i] = m_GpsData.MaptoGPS(currentpoint,ac,MidGpsPoint_1[i]);
		}

		for(int i=160;i<200;i++)
		{
			MidGpsPoint_4[i] = m_GpsData.MaptoGPS(currentpoint,ac,MidGpsPoint_4[i]);
		}
	}
	
	ArrayFuZhi(MidGpsPoint_1,MidGpsPoint_7);
	while(next_road_edge == 1)
	{
		CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
		MoveLeft(/*OriGpsPoint*/MidGpsPoint_7,MidGpsPoint_1,j);

		/*cpoint_xiugai.x = 256 - 5*j;
		cpoint_xiugai.y = 412;

		cpoint_xiugai = m_GpsData.MaptoGPS(m_gps,m_gpsdir,cpoint_xiugai);*/

		temp_map = app->PercepMap;
		next_road_edge = Search_edge(MidGpsPoint_1,temp_map,app->GPS_Speed*2+20,0,next_edgepoint_x,next_edgepoint_y);

		if(next_road_edge == 0)
		{
			zuohuandao = 1;
			break;
		}
		MoveLeft(/*OriGpsPoint*/MidGpsPoint_7,MidGpsPoint_1,-j);
		next_road_edge = Search_edge(MidGpsPoint_1,temp_map,app->GPS_Speed*2+20,0,next_edgepoint_x,next_edgepoint_y);
	    j++;
		if((j>15)&&(next_road_edge == 1))
		{
			ArrayFuZhi(MidGpsPoint_7,MidGpsPoint_1);
			zuohuandao = 0;
			break;
		}
	
	}

	/*if(next_road_edge == 1)
	{
		temp_map = app->PercepMap;
		next_road_edge = Search_edge(MidGpsPoint_1,temp_map,app->GPS_Speed*2+15,0,next_edgepoint_x,next_edgepoint_y);
		while(next_road_edge == 0)
		{
			ArrayFuZhi(MidGpsPoint_6,MidGpsPoint_1);
			Sleep(100);
			temp_map = app->PercepMap;
			next_road_edge = Search_edge(MidGpsPoint_1,temp_map,app->GPS_Speed*2+15,0,next_edgepoint_x,next_edgepoint_y);
		}
	}*/


	app->critical_section.Lock();//锁住
	m_gps = app->GPS_Point;
	m_gpsdir = app->GPS_Direction;

	app->critical_section.Unlock();

	double ae_map = 270 + ae - m_gpsdir;
	if(ae_map < 0)
	{
		ae_map = ae_map + 360;
	}
	else if(ae_map > 360)
	{
		ae_map = ae_map - 360;
	}

	double x111,y111;

	if(yichu == 1)
	{
		if((ae_map != 90)&&(ae_map != 270))
		{
			ae_map = rad(ae_map);
			ae_map = tan(ae_map);
			x111 = (ae_map*map_ppp_ex.x - map_ppp_ex.y + (map_ppp.x/ae_map) + map_ppp.y )/(ae_map + (1/ae_map));
			y111 = ae_map*(x111 - map_ppp_ex.x) + map_ppp_ex.y;
		}
		else if((ae_map == 90)||(ae_map == 270))
		{
			x111 = map_ppp_ex.x;
			y111 = map_ppp.y;
		}
		map_ppp_ex.x = x111;
		map_ppp_ex.y = y111;
	
		double z = sqrt((map_ppp_ex.x - map_ppp.x)*(map_ppp_ex.x - map_ppp.x) + (map_ppp_ex.y - map_ppp.y)*(map_ppp_ex.y - map_ppp.y));

		z = 15/(15+z);//平移4米
	
		ppp_correct.x = (map_ppp_ex.x - z*map_ppp.x)/(1-z);
		ppp_correct.y = (map_ppp_ex.y - z*map_ppp.y)/(1-z);

		/*ppp_correct.x = map_ppp_ex.x;
		ppp_correct.y = map_ppp_ex.y;*/


		double dx = ppp_correct.x - map_ppp.x;
		double dy = ppp_correct.y - map_ppp.y;

		qqq = m_GpsData.APiontConverD(m_gps,qqq,m_gpsdir);
		qqq.x = qqq.x + dx;
		qqq.y = qqq.y + dy;

		ppp_correct = m_GpsData.MaptoGPS(m_gps,m_gpsdir,ppp_correct);
		qqq = m_GpsData.MaptoGPS(m_gps,m_gpsdir,qqq);
		return ppp_correct;
	}

	else if(yichu == 0)
	{
		return ppp;
	}
}

bool CPassIntersection::Search_edge(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down,int &edge_x,int &edge_y)
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
	/*	if(m_GpsData.GetDistance(GPSpoint[i].x,GPSpoint[i].y,m_gps.x,m_gps.y)>up)
			continue;*/
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(int m=-8;m<8;m++)
			for(int n=-8;n<8;n++)
			{
				if(y+m>380&&y+m<511)
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
bool CPassIntersection::SearchObstacle(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down)
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
	if(Rndf_MapPoint[199].x == 256 && Rndf_MapPoint[199].y == 411)
	{
		return false;
	}
	int m_up = 412-up*5;
	int m_down = 411 -down*5;

	int x = 0;
	int y = 0;
	for(int i = 0;i<200;i++)
	{
	/*	if(m_GpsData.GetDistance(GPSpoint[i].x,GPSpoint[i].y,m_gps.x,m_gps.y)>up)
			continue;*/
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
					continue;
		x = Rndf_MapPoint[i].x ;//2012.7.18xiugai -10
		y = Rndf_MapPoint[i].y;
		for(int m=-3;m<3;m++)
			for(int n=-3;n<3;n++)
			{
				if(y+m>380&&y+m<511)
					continue;
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == 18)
				{

					return true;
			
				}
			}
	}
	return false;
}


//取一个y为352的点



double CPassIntersection::correct_endpoint_direction_1(I_Map *map,CvPoint2D64f ggg,int luduan1,double jjj)
{

	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	CvPoint2D64f m_gps;
	double m_gpsdir;
	double angle;
	int num;
	int panduan_3;
	num = 0;
	panduan_1 = 0;//前10次获取到的角度中是否有几次差不多
	panduan_2 = 0;//是否拐过一半
	panduan_3 = 0;//路径上是否有障碍物


	for(int i = 0;i<10;i++)
	{

		
		vel_Map=app->PercepMap;
		

		angle = vel_Map->MapPoint[511][511];
		if(angle<0)
		{
			angle = angle + 256;
		}
		angle = 360 - angle;
	
		out111<<"angle "<<angle<<endl;
		
	
		app->critical_section.Lock();//锁住
		m_gps = app->GPS_Point;
		m_gpsdir = app->GPS_Direction;
	
		app->critical_section.Unlock();

		if(angle != 360)
		{
			angle = angle + m_gpsdir - 270;
		}
		else if(angle == 360)
		{
			angle = -100;
		}

		for(int i=1;i<10;i++)
		{
			next_direction[i] = next_direction[i-1];
		}

		next_direction[0] = angle;
		
		for(int i=1;i<10;i++)
		{
			if(((next_direction[i]-next_direction[0]) > -3)&&((next_direction[i]-next_direction[0])<3)&&(next_direction[0] != -100))
			{
				num++;
			}
		}
		if(num>4)
		{
			panduan_1 = 1;
		}
		Sleep(100);
	}

	//for(int i = 0;i<5;i++)
	//{

	//	
	//	vel_Map=app->PercepMap;
	//	

	//	angle = vel_Map->MapPoint[511][511];
	//	angle = 360 - angle;
	//
	//	out111<<"angle "<<angle<<endl;
	//	
	//
	//	app->critical_section.Lock();//锁住
	//	m_gps = app->GPS_Point;
	//	m_gpsdir = app->GPS_Direction;
	//
	//	app->critical_section.Unlock();

	//	if(angle != 360)
	//	{
	//		angle = angle + m_gpsdir - 270;
	//	}
	//	else if(angle == 360)
	//	{
	//		angle = -100;
	//	}

	//	for(int i=1;i<5;i++)
	//	{
	//		next_direction[i] = next_direction[i-1];
	//	}

	//	next_direction[0] = angle;
	//	
	//	for(int i=1;i<5;i++)
	//	{
	//		if(((next_direction[i]-next_direction[0]) > -3)&&((next_direction[i]-next_direction[0])<3)&&(next_direction[0] != -100))
	//		{
	//			num++;
	//		}
	//	}
	//	if(num>2)
	//	{
	//		panduan_1 = 1;
	//	}
	//	Sleep(100);
	//}
	
	CvPoint2D64f spoint,epoint;
	spoint.x = LeadPoint[luduan1-2].lat;
	spoint.y = LeadPoint[luduan1-2].lng;
	epoint = ggg;

	spoint = m_GpsData.APiontConverD(m_gps,spoint,m_gpsdir);
	epoint = m_GpsData.APiontConverD(m_gps,epoint,m_gpsdir);
	double dis_s,dis_e;
	dis_s = sqrt((spoint.x - 256)*(spoint.x - 256) + (spoint.y - 412)*(spoint.y - 412));
	dis_e = sqrt((epoint.x - 256)*(epoint.x - 256) + (epoint.y - 412)*(epoint.y - 412));
	if(dis_s > dis_e)
	{
		panduan_2 = 1;
	}

	bool ob = SearchObstacle(MidGpsPoint,vel_Map,30,2);
	if(ob == 1)
	{
		panduan_3 = 1;
	}

	if((panduan_1 == 1)&&(panduan_2 == 1)/*&&(panduan_3 == 1)*/)
	{
		fangxianggengxin1 = 1;
		difference_ae1 = angle - possible_ae;
		return angle;
	}
	else
	{
		fangxianggengxin1 = 0;
		return jjj;
	}
	
}

double CPassIntersection::correct_endpoint_direction_2(I_Map *map,CvPoint2D64f ggggg,int luduan2,double jjjjj)
{

	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	CvPoint2D64f m_gps;
	double m_gpsdir;
	double angle11;
	int num11;
	int panduan_3;
	num11 = 0;
	panduan_11 = 0;//前10次获取到的角度中是否有几次差不多
	panduan_22 = 0;//是否拐过一半
	panduan_3 = 0;//路径上是否有障碍物


	for(int i = 0;i<10;i++)
	{

		
		vel_Map=app->PercepMap;
		

		angle11 = vel_Map->MapPoint[510][510];
		if(angle11 < 0)
		{
			angle11 = angle11 + 256;
		}
		angle11 = 360 - angle11;
	
		out111<<"angle11 "<<angle11<<endl;
		
	
		app->critical_section.Lock();//锁住
		m_gps = app->GPS_Point;
		m_gpsdir = app->GPS_Direction;
	
		app->critical_section.Unlock();

		if(angle11 != 360)
		{
			angle11 = angle11 + m_gpsdir - 270;
		}
		else if(angle11 == 360)
		{
			angle11 = -100;
		}

		for(int i=1;i<10;i++)
		{
			next_direction[i] = next_direction[i-1];
		}

		next_direction[0] = angle11;
		
		for(int i=1;i<10;i++)
		{
			if(((next_direction[i]-next_direction[0]) > -3)&&((next_direction[i]-next_direction[0])<3)&&(next_direction[0] != -100))
			{
				num11++;
			}
		}
		if(num11>4)
		{
			panduan_11 = 1;
		}
		Sleep(100);
	}

	CvPoint2D64f spoint,epoint;
	spoint.x = LeadPoint[luduan2-2].lat;
	spoint.y = LeadPoint[luduan2-2].lng;
	epoint = ggggg;

	spoint = m_GpsData.APiontConverD(m_gps,spoint,m_gpsdir);
	epoint = m_GpsData.APiontConverD(m_gps,epoint,m_gpsdir);
	double dis_s,dis_e;
	dis_s = sqrt((spoint.x - 256)*(spoint.x - 256) + (spoint.y - 412)*(spoint.y - 412));
	dis_e = sqrt((epoint.x - 256)*(epoint.x - 256) + (epoint.y - 412)*(epoint.y - 412));
	if(dis_s > dis_e)
	{
		panduan_22 = 1;
	}

	bool ob = SearchObstacle(MidGpsPoint,vel_Map,30,2);
	if(ob == 1)
	{
		panduan_3 = 1;
	}

	if((panduan_11 == 1)&&(panduan_22 == 1)/*&&(panduan_3 == 1)*/)
	{
		fangxianggengxin2 = 1;
		difference_ae2 = angle11 - possible_ae;
		return angle11;
	}
	else
	{
		fangxianggengxin2 = 0;
		return jjjjj;
	}
	
}

void CPassIntersection::qiudian(CvPoint2D64f *Point,int luduan)
{
	//double dir = m_GpsData.GetAngle(MidGpsPoint[199].x,MidGpsPoint[199].y,MidGpsPoint[180].x,MidGpsPoint[180].y);
	double dir_end;

	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	
	app->critical_section.Lock();//锁住
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;
	
	app->critical_section.Unlock();

	double dir = /*180;*/m_gpsdir;

	

	//Point[0].x = LeadPoint[luduan-3].lat;
	//Point[0].y = LeadPoint[luduan-3].lng;
	
	Point[1].x = LeadPoint[luduan-2].lat;
	Point[1].y = LeadPoint[luduan-2].lng;	

	Point[1] = m_GpsData.APiontConverD(m_gps,Point[1],m_gpsdir);
	
	Point[2].x = LeadPoint[luduan-1].lat;
	Point[2].y = LeadPoint[luduan-1].lng;

	Point[2] = m_GpsData.APiontConverD(m_gps,Point[2],m_gpsdir);

	//Point[3].x = LeadPoint[luduan].lat;
	//Point[3].y = LeadPoint[luduan].lng;

	if(LeadPoint[luduan-2].param2 == 1)
	{
		dir_end = dir;
	}
	else if(LeadPoint[luduan-2].param2 == 3)
	{
		dir_end = dir + 90;
		if(dir_end > 360)
		{
			dir_end = dir_end - 360;
		}
	}
	else if(LeadPoint[luduan-2].param2 == 2)
	{
		dir_end = dir - 90;
		if(dir_end < 0)
		{
			dir_end = dir_end + 360;
		}
	}

	dir = rad(90 + dir - m_gpsdir);
	dir_end = rad(90 + dir_end - m_gpsdir);

	Point[0].x = Point[1].x + 300*cos(dir);
	Point[0].y = Point[1].y + 300*sin(dir);

	Point[3].x = Point[2].x - 300*cos(dir_end);
	Point[3].y = Point[2].y - 300*sin(dir_end);

	Point[0] = m_GpsData.MaptoGPS(m_gps,m_gpsdir,Point[0]);
	Point[1] = m_GpsData.MaptoGPS(m_gps,m_gpsdir,Point[1]);
	Point[2] = m_GpsData.MaptoGPS(m_gps,m_gpsdir,Point[2]);
	Point[3] = m_GpsData.MaptoGPS(m_gps,m_gpsdir,Point[3]);
}

void CPassIntersection::qiudian1(CvPoint2D64f *Point,int luduan,double gps_fangxiang)
{
	//double dir = m_GpsData.GetAngle(MidGpsPoint[199].x,MidGpsPoint[199].y,MidGpsPoint[180].x,MidGpsPoint[180].y);
	double dir_end;

	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	
	app->critical_section.Lock();//锁住
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;
	
	app->critical_section.Unlock();

	double dir = /*180;*/m_gpsdir;

	

	//Point[0].x = LeadPoint[luduan-3].lat;
	//Point[0].y = LeadPoint[luduan-3].lng;
	
	Point[1].x = LeadPoint[luduan-2].lat;
	Point[1].y = LeadPoint[luduan-2].lng;	

	Point[1] = m_GpsData.APiontConverD(m_gps,Point[1],m_gpsdir);
	
	Point[2].x = LeadPoint[luduan-1].lat;
	Point[2].y = LeadPoint[luduan-1].lng;

	Point[2] = m_GpsData.APiontConverD(m_gps,Point[2],m_gpsdir);

	//Point[3].x = LeadPoint[luduan].lat;
	//Point[3].y = LeadPoint[luduan].lng;

	dir_end = gps_fangxiang;

	dir = rad(90 + dir - m_gpsdir);
	dir_end = rad(90 + dir_end - m_gpsdir);

	Point[0].x = Point[1].x + 300*cos(dir);
	Point[0].y = Point[1].y + 300*sin(dir);

	Point[3].x = Point[2].x - 300*cos(dir_end);
	Point[3].y = Point[2].y - 300*sin(dir_end);

	Point[0] = m_GpsData.MaptoGPS(m_gps,m_gpsdir,Point[0]);
	Point[1] = m_GpsData.MaptoGPS(m_gps,m_gpsdir,Point[1]);
	Point[2] = m_GpsData.MaptoGPS(m_gps,m_gpsdir,Point[2]);
	Point[3] = m_GpsData.MaptoGPS(m_gps,m_gpsdir,Point[3]);
}


void  CPassIntersection::planpath(CvPoint2D64f ccc,double ddd)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	app->critical_section.Lock();//锁住
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;
 
	app->critical_section.Unlock();

	double k4,r;
	CvPoint2D64f map_ccc,P;

	map_ccc = m_GpsData.APiontConverD(m_gps,ccc,m_gpsdir);
	double d = 0;

	for (int i=0;i<200;i++)
	{
		MidGpsPoint_1[i] = MidGpsPoint[i];
	}

	/*if(zuohuandao == 0)
	{*/
	
		WayPoint1[0].x = 256;
		WayPoint1[0].y = 412;
		WayPoint1[4] = map_ccc;

		r = 270 + ddd - m_gpsdir;

		if (r < 0)
		{
			r = r + 360; 
		}
		else if (r > 360)
		{
			r = r - 360;
		}


		if(r != 270)
		{
			r = rad(r);
			k4 = tan(r);

			P.x = WayPoint1[4].x - d*cos(r);
			P.y = WayPoint1[4].y - d*sin(r);


			WayPoint1[4] = P;

			solvepoints(WayPoint1[0],WayPoint1[4],k4,WayPoint1[2],WayPoint1[1],WayPoint1[3]);

			if( (WayPoint1[2].y < 412) && ( ((WayPoint1[4].x < 256)&&((r > 0.5*3.1415926 )&&(r < 1.5*3.1415926))) || ((WayPoint1[4].x > 256)&&( ((r > 1.5*3.1415926)&&(r < 2*3.1415926))||((r >= 0)&&(r < 0.5*3.1415926)))) ))
			{
				WayPoint1[0] = m_gps;
				WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[1]);
				WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[2]);
				WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[3]);
				WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[4]);
				
				Bezier(WayPoint1,4,MidGpsPoint_1);
			}
			else
			{
				solvepoints1(WayPoint1[0],WayPoint1[4],k4,WayPoint1[2],WayPoint1[1],WayPoint1[3]);
				WayPoint1[0] = m_gps;
				WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[1]);
				WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[2]);
				WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[3]);
				WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[4]);
				
				Bezier(WayPoint1,4,MidGpsPoint_1);
			}

		}

		else if(r == 270)
		{
			r = rad(r);

			P.x = WayPoint1[4].x - d*cos(r);
			P.y = WayPoint1[4].y - d*sin(r);


			WayPoint1[4] = P;

			solvepoints2(WayPoint1[0],WayPoint1[4],WayPoint1[2],WayPoint1[1],WayPoint1[3]);
			WayPoint1[0] = m_gps;
			WayPoint1[1] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[1]);
			WayPoint1[2] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[2]);
			WayPoint1[3] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[3]);
			WayPoint1[4] = m_GpsData.MaptoGPS(WayPoint1[0],m_gpsdir,WayPoint1[4]);

			Bezier(WayPoint1,4,MidGpsPoint_1);
		}
	

		app->critical_section.Lock();//锁住
		currentpoint = app->GPS_Point;
		ac = app->GPS_Direction;//读取当前位姿
		app->critical_section.Unlock();
			
			
		ae_map = 270 + ddd - ac;
		if(ae_map < 0)
		{
			ae_map = ae_map + 360;
		}
		else if(ae_map > 360)
		{
			ae_map = ae_map - 360;
		}

		ArrayFuZhi(MidGpsPoint_1,MidGpsPoint_4);
		endpoint_map = m_GpsData.APiontConverD(currentpoint,MidGpsPoint_1[179],ac);
		ae_map = ae_map - 180;
		ae_map = rad(ae_map);
			//MidGpsPoint_3[200].x = endpoint_map.x - 75*cos(ae_map);//15米
			//MidGpsPoint_3[200].y = endpoint_map.y - 75*sin(ae_map);
			//MidGpsPoint_3[200] = m_GpsData.MaptoGPS(currentpoint,ac,MidGpsPoint_3[200]);
			//for(int i = 0;i<200;i++)
			//{
			//	MidGpsPoint_3[i] = MidGpsPoint_1[i];
			//}
			//Bezier(MidGpsPoint_3,200,MidGpsPoint_1);

		MidGpsPoint_1[199].x = endpoint_map.x - 100*cos(ae_map);//20米
		MidGpsPoint_1[199].y = endpoint_map.y - 100*sin(ae_map);

		endpoint_map = m_GpsData.APiontConverD(currentpoint,MidGpsPoint_1[159],ac);

		MidGpsPoint_4[199].x = endpoint_map.x - 200*cos(ae_map);//40米
		MidGpsPoint_4[199].y = endpoint_map.y - 200*sin(ae_map);

		for(int i=198;i>179;i--)
		{
			MidGpsPoint_1[i].x = MidGpsPoint_1[i+1].x + 5*cos(ae_map);
			MidGpsPoint_1[i].y = MidGpsPoint_1[i+1].y + 5*sin(ae_map);
		}
		for(int i=198;i>159;i--)
		{
		MidGpsPoint_4[i].x = MidGpsPoint_4[i+1].x + 5*cos(ae_map);
				MidGpsPoint_4[i].y = MidGpsPoint_4[i+1].y + 5*sin(ae_map);
		}
			
			/*MidGpsPoint_1[199] = m_GpsData.MaptoGPS(currentpoint,ac,MidGpsPoint_1[199]);
			MidGpsPoint_1[198].x = (MidGpsPoint_1[197].x + MidGpsPoint_1[199].x)/2;
			MidGpsPoint_1[198].y = (MidGpsPoint_1[197].y + MidGpsPoint_1[199].y)/2;*/

		for(int i=180;i<200;i++)
		{
			MidGpsPoint_1[i] = m_GpsData.MaptoGPS(currentpoint,ac,MidGpsPoint_1[i]);
		}

		for(int i=160;i<200;i++)
		{
			MidGpsPoint_4[i] = m_GpsData.MaptoGPS(currentpoint,ac,MidGpsPoint_4[i]);
		}
		Bezier(MidGpsPoint_1,199,MidGpsPoint_1);
	/*}

	else if(zuohuandao == 1)
	{
	}*/
}

int CPassIntersection::GetCross_newtest()
{
	

	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	CvPoint2D64f Point[4];
	CvPoint2D64f Cross_Pt[8];
	CvPoint2D64f WayPoint[7];
	int seq = 0;
	seq = seq_num;
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
	
	WayPoint[2].x = (WayPoint[1].x+9*WayPoint[3].x)/10;
	WayPoint[2].y = (WayPoint[1].y+9*WayPoint[3].y)/10;
	WayPoint[4].x = (WayPoint[5].x+9*WayPoint[3].x)/10;
	WayPoint[4].y = (WayPoint[5].y+9*WayPoint[3].y)/10;
	//LocalPath.BSpline(Cross_Pt,8,GpsPoint);
	Bezier(WayPoint,6,MidGpsPoint);
	app->critical_section.Lock();//锁住
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;
	app->critical_section.Unlock();//解锁
	MoveWay(m_gps,m_gpsdir,MidGpsPoint);
	///*for(int i = 0;i<6;i++)
	//{
	//	outn1<<(Cross_Pt[i].x-31)*100000<<","<<(Cross_Pt[i].y-117)*100000<<endl;
	//}
	//outn1<<"feng ge xian"<<endl;
	//for(int i = 0;i<200;i++)
	//{
	//	outn1<<(GpsPoint[i].x-31)*100000<<","<<(GpsPoint[i].y-117)*100000<<endl;
	//}*/
	//
	return 1;

}
CvPoint2D64f CPassIntersection::GetCrossPoint(CvPoint2D64f *Point,CvPoint2D64f (&WayPoint)[7])
{
//	GetCross_new(Point,WayPoint);
	return WayPoint[3];
}
CvPoint2D64f CPassIntersection::findInterAimPoint(CvPoint2D64f m_gps,double dir,CvPoint2D64f (&CrossPath)[200],double s,double &goal_dir)
{
	double len = 0;
	double min_len = 99999;
	CvPoint2D64f goal;
	int n = 0;
	for(int i = 0;i<200;i++)
	{
		len = m_GpsData.GetDistance(m_gps.x,m_gps.y,CrossPath[i].x,CrossPath[i].y);
		if(len < min_len)
		{
			min_len = len;
			n = i;
		}
	}
	int t=199;
	for(int i = n;i<200;i++)
	{
		len = m_GpsData.GetDistance(m_gps.x,m_gps.y,CrossPath[i].x,CrossPath[i].y);
		if(len > s)
		{
			t = i;
			break;
		}

	}
	goal = CrossPath[t];
	if(t+2<200)
		/*goal_dir = m_GpsData.GetAngle(CrossPath[n].x,CrossPath[n].y,CrossPath[n+5].x,CrossPath[n+5].y);*/
		goal_dir = m_GpsData.GetAngle(CrossPath[t+2].x,CrossPath[t+2].y,CrossPath[t].x,CrossPath[t].y);
	else
		goal_dir = m_GpsData.GetAngle(CrossPath[199].x,CrossPath[199].y,CrossPath[197].x,CrossPath[197].y);
	return goal;
}
CvPoint2D64f CPassIntersection::findUturnAimPoint(CvPoint2D64f m_gps,double dir,CvPoint2D64f (&CrossPath)[200],double s,double &goal_dir)
{
	double len = 0;
	double min_len = 99999;
	CvPoint2D64f goal;
	int n = 0;
	for(int i = 0;i<200;i++)
	{
		len = m_GpsData.GetDistance(m_gps.x,m_gps.y,CrossPath[i].x,CrossPath[i].y);
		if(len < min_len)
		{
			min_len = len;
			n = i;
		}
	}
	int t=199;
	len = 0;
	for(int i = n;i<199;i++)
	{
		len += m_GpsData.GetDistance(CrossPath[i].x,CrossPath[i].y,CrossPath[i+1].x,CrossPath[i+1].y);
	
		if(len > s)
		{
			t = i+1;//t = i;
			break;
		}

	}
	goal = CrossPath[t];
	if(t+2<200)
		/*goal_dir = m_GpsData.GetAngle(CrossPath[n].x,CrossPath[n].y,CrossPath[n+5].x,CrossPath[n+5].y);*/
		goal_dir = m_GpsData.GetAngle(CrossPath[t+2].x,CrossPath[t+2].y,CrossPath[t].x,CrossPath[t].y);
	else
		goal_dir = m_GpsData.GetAngle(CrossPath[199].x,CrossPath[199].y,CrossPath[197].x,CrossPath[197].y);
	return goal;
}
CvPoint2D64f CPassIntersection::findUturnAimPointlukou(CvPoint2D64f m_gps,double dir,CvPoint2D64f (&CrossPath)[200],double s,double &goal_dir)
{
	double len = 0;
	double min_len = 99999;
	CvPoint2D64f goal;
	int n = 0;
	for(int i = 0;i<200;i++)
	{
		len = m_GpsData.GetDistance(m_gps.x,m_gps.y,CrossPath[i].x,CrossPath[i].y);
		if(len < min_len)
		{
			min_len = len;
			n = i;
		}
	}
	int t=199;
	len = VertDist(m_gps,dir,CrossPath[n]);
	if(len<0)
		len=0;
	double ll=0;
	for(int i = n;i<199;i++)
	{
		len += m_GpsData.GetDistance(CrossPath[i].x,CrossPath[i].y,CrossPath[i+1].x,CrossPath[i+1].y);
		ll=m_GpsData.GetDistance(m_gps.x,m_gps.y,CrossPath[i].x,CrossPath[i].y);
		if(len > s && ll>5)
		{
			t = i;
			break;
		}
	}
	double anglethis = m_GpsData.GetAngle(CrossPath[t].x,CrossPath[t].y,m_gps.x,m_gps.y);
	int tt=t;
	double anglett=0;
	for(int i = t+1;i<199;i++)
	{
		anglett = m_GpsData.GetAngle(CrossPath[i].x,CrossPath[i].y,m_gps.x,m_gps.y);
		ll=m_GpsData.GetDistance(m_gps.x,m_gps.y,CrossPath[i].x,CrossPath[i].y);
		if((cos(PI*(anglett - dir)/180)>cos(PI*(anglethis - dir)/180))&&(ll<s+1))
		{
			t=i;
			anglethis=anglett;
		}
		else
			break;
	}
	goal = CrossPath[t];
	if(t+2<200)
		/*goal_dir = m_GpsData.GetAngle(CrossPath[n].x,CrossPath[n].y,CrossPath[n+5].x,CrossPath[n+5].y);*/
		goal_dir = m_GpsData.GetAngle(CrossPath[t+2].x,CrossPath[t+2].y,CrossPath[t].x,CrossPath[t].y);
	else
		goal_dir = m_GpsData.GetAngle(CrossPath[199].x,CrossPath[199].y,CrossPath[197].x,CrossPath[197].y);
	return goal;
}