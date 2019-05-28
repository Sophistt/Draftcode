#include "StdAfx.h"
#include "Vehicle.h"
#include<fstream>
using namespace std;
ofstream out777("777.txt");

I_Map *CVehicle::vel_Map = new I_Map;
I_Map *CVehicle::vel_Map_path = new I_Map;
CvMemStorage* CVehicle::storage_road = cvCreateMemStorage(0);
CvSeq* CVehicle::planning_road = cvCreateSeq( CV_64FC2, sizeof(CvSeq), sizeof(CvPoint2D64f), storage_road );
CvSeq* CVehicle::global_path = cvCreateSeq( CV_64FC2, sizeof(CvSeq), sizeof(CvPoint2D64f), storage_road );
I_Map *CVehicle::lane_mid_map = new I_Map;
CvPoint2D64f CVehicle::MidGpsPoint[200];
CvPoint2D64f CVehicle::search_obGpsPoint[200];
CvPoint2D64f CVehicle::IntersectionMidGpsPoint[200];
int CVehicle::mission_num;
int CVehicle::mission_num2;
LEAD *CVehicle::LeadPoint/* = (LEAD *)malloc( mission_num*sizeof(LEAD) )*/;
LEAD *CVehicle::MissionPoint;
int CVehicle::seq_num = -1;
int CVehicle::fx = 0;
CGetGPSData CVehicle::m_GpsData;
CvPoint2D64f CVehicle::OriGpsPoint[200];
int CVehicle::vehicleob_x = 0;
int CVehicle::vehicleob_y = 0;
bool CVehicle::way_out = 1;
int CVehicle::intersection_obnum = 0;
I_Map* CVehicle::map_ts;
I_Map* CVehicle::dy_map;
I_Map* CVehicle::map_dan;
I_Map* CVehicle::map_v;
I_Map* CVehicle::map_dir;
I_Map* CVehicle::map_ts1;

v_Map* CVehicle::map_v_x;
v_Map* CVehicle::map_v_y;
I_Map* CVehicle::static_Map;
I_Map* CVehicle::dynamic_Map;
v_Map* CVehicle::dynamicmap_v_x;
v_Map* CVehicle::dynamicmap_v_y;

I_Map* CVehicle::lukou_Map;
v_Map* CVehicle::lukou_v_x;
v_Map* CVehicle::lukou_v_y;

/////实时处理所用信息
I_Map *CVehicle::realtime_Map = new I_Map;
CvPoint2D64f CVehicle::realtime_Gps = cvPoint2D64f(0,0);
double CVehicle::realtime_Dir = 0;
//CvPoint2D64f CVehicle::MidPoint1[512];
CVehicle::CVehicle(void)
{
	vehicle_upstate = false;
	vehicle_pos = cvPoint2D64f(0,0);
	vehicle_orientation = 0;
	
	veiw_storage_road = cvCreateMemStorage(0);
	view_vechile_planning_road = cvCreateSeq( CV_64FC2, sizeof(CvSeq), sizeof(CvPoint2D64f), veiw_storage_road );
	for( int i=0; i<512; i++)
	{
		for( int j=0; j<512; j++)
		{
			vel_Map->MapPoint[i][j] = 0;
			
		}
	}
	//view_vechile_vel_map = new I_Map;
	//view_vechile_mid_map = new I_Map;
	/*vel_Map = new I_Map;
	
	/*lane_mid_map = new I_Map;
	for( int i=0; i<512; i++)
	{
		for( int j=0; j<512; j++)
		{
			lane_mid_map->MapPoint[i][j] = 0;
			
		}
	}*/
}

CVehicle::~CVehicle(void)
{
	//delete view_vechile_vel_map;
	//delete view_vechile_mid_map;
	cvReleaseMemStorage(&storage_road);
}
void CVehicle::setEnable(bool option)
{
	vehicle_upstate = option;
}
bool CVehicle::getEnable()
{
	return vehicle_upstate;
}
void CVehicle::setPerceptionNetwork (void) 
{
	//perception_network.Set_RecAddress("udpm://239.255.76.67:7667?ttl=1"); 
	//perception_network.Set_RecChannel("PERCEPTION");
	perception_network.recv_message();
	
}
void CVehicle::setSignalNetwork (void) 
{
	//perception_network.Set_RecAddress("udpm://239.255.76.71:7671?ttl=1"); 
	//perception_network.Set_RecChannel("SIGNAl");
	perception_network.recv_messagesignal();
	perception_network.recv_messagesign();
}
I_Map* CVehicle::getPerceptionMap() 
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_map.Lock();
	perception_map = app->PercepMap;
	//vel_Map = app->PercepMap;
	app->critical_map.Unlock();
	return perception_map;
}
//void CVehicle::setRoadNet (const char *rndf_name, bool hash)
//{}
bool CVehicle::setStartState (CDriveState *state)
{
	return 0;
}




void CVehicle::ArrayFuZhi(CvPoint2D64f a,CvPoint2D64f *b)
{
	for(int i = 0;i<200;i++)
	{
		b[i].x = 0;
		b[i].y = 0;
		b[i].x = a.x;
		b[i].y = a.y;
	}
}
void CVehicle::ArrayFuZhi(CvPoint2D64f *a,CvPoint2D64f *b)
{
	for(int i = 0;i<200;i++)
	{
		b[i].x = 0;
		b[i].y = 0;
		b[i].x = a[i].x;
		b[i].y = a[i].y;
	}
}
int CVehicle::GetSendGps(CvPoint2D64f m_gps,double m_gpsdir,CvPoint2D64f *MidPoint,CvPoint2D64f (&MidGpsPoint)[200],int num)
{
	MidGpsPoint[0] = m_gps;
	for(int i = 1;i<num;i++)
		{
			MidGpsPoint[i] = m_GpsData.MaptoGPS(m_gps,m_gpsdir,MidPoint[i]);
		}
	return 0;
}
void CVehicle::Bezier(CvPoint2D64f p[],int n,CvPoint2D64f *MPoint,int m)
{

	CvPoint2D64f *pc=new CvPoint2D64f[n+1];   
	int i,r;   
	float u; 
	int count = 0;
	//u的步长决定了曲线点的精度    
	for(int k = 0;k<m;k++)
	{   
		u=k/double(m-1);
		for(i=0;i<=n;i++)pc[i]=p[i];   
     
		for(r=1;r<=n;r++)
		{   
	      for(i=0;i<=n-r;i++)
		  {   
                pc[i].x=(1-u)*pc[i].x+u*pc[i+1].x;   
                pc[i].y=(1-u)*pc[i].y+u*pc[i+1].y;   
			}   
		}   
		MPoint[k] = pc[0];   

	}   
delete [] pc;  
	
}
void CVehicle::Bezier(CvPoint2D64f p[],int n,CvPoint2D64f (&MPoint)[200])
{

	CvPoint2D64f *pc=new CvPoint2D64f[n+1];   
	int i,r;   
	float u; 
	int count = 0;
	//u的步长决定了曲线点的精度    
	for(int k = 0;k<200;k++)
	{   
		u=k/199.0;
		for(i=0;i<=n;i++)pc[i]=p[i];   
     
		for(r=1;r<=n;r++)
		{   
	      for(i=0;i<=n-r;i++)
		  {   
                pc[i].x=(1-u)*pc[i].x+u*pc[i+1].x;   
                pc[i].y=(1-u)*pc[i].y+u*pc[i+1].y;   
			}   
		}   
		MPoint[k] = pc[0];   
	}   
delete [] pc;  
	
}

void CVehicle::Bezierlukou(CvPoint2D64f p[],int n,CvPoint2D64f (&MPoint)[200])
{

	CvPoint2D64f *pc = (CvPoint2D64f *)malloc( (n*199)*sizeof(CvPoint2D64f) );
	int chadiannum=0;
	for(int i=0;i<n;i++)
	{
		for(int kk=0;kk<199;kk++)
		{
			pc[chadiannum].x=(p[i].x*(199-kk)+p[i+1].x*kk)/199;
			pc[chadiannum].y=(p[i].y*(199-kk)+p[i+1].y*kk)/199;
			chadiannum++;
		}
	}
	MPoint[0]=p[0];
	MPoint[199]=p[n];
	for(int i=1;i<199;i++)
	{
		MPoint[i]=pc[i*n];
	}
	free(pc);
}

void CVehicle::Beziercazhi(CvPoint2D64f p[],int n,CvPoint2D64f (&MPoint)[200])
{

	CvPoint2D64f *pc = (CvPoint2D64f *)malloc( (n*199)*sizeof(CvPoint2D64f) );
	int chadiannum=0;
	for(int i=0;i<n;i++)
	{
		for(int kk=0;kk<199;kk++)
		{
			pc[chadiannum].x=(p[i].x*(199-kk)+p[i+1].x*kk)/199;
			pc[chadiannum].y=(p[i].y*(199-kk)+p[i+1].y*kk)/199;
			chadiannum++;
		}
	}
	MPoint[0]=p[0];
	MPoint[199]=p[n];
	for(int i=1;i<199;i++)
	{
		MPoint[i]=pc[i*n];
	}
	free(pc);
}


double CVehicle::ParaDist(CvPoint2D64f ori_gps,double ori_dir,CvPoint2D64f cur_gps)
{
		double north_dir = m_GpsData.GetAngle(cur_gps,ori_gps);
		double head_path_angle = (north_dir - ori_dir)*PI/180;


		//dist = 10-di;
		double dist = m_GpsData.GetDistance( cur_gps.x,cur_gps.y,ori_gps.x, ori_gps.y);
		dist = dist*sin( head_path_angle );
		//outn2<<"距离, "<<dist<<", ";
		return dist;
	
}
int CVehicle::Stop()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
//	app->secondDecison = "停车";
	app->critical_section.Lock();//锁住
	CvPoint2D64f m_gps = app->GPS_Point;	
	app->critical_section.Unlock();//解锁
	app->critical_planningroad.Lock();
	cvClearSeq( planning_road );
	for(int i = 0; i<200; i++)
	{
		cvSeqPush( planning_road, &m_gps);
	}
	app->critical_planningroad.Unlock();
	app->stop = true;
	SetEvent(app->m_hEvent);//路径更新后，发送消息

	return 1;
}
double CVehicle::VertDist(CvPoint2D64f ori_gps,double ori_dir,CvPoint2D64f cur_gps)
{
		double north_dir = m_GpsData.GetAngle(cur_gps,ori_gps);
		double head_path_angle = (north_dir - ori_dir)*PI/180;


		//dist = 10-di;
		double dist = m_GpsData.GetDistance(cur_gps.x,cur_gps.y,ori_gps.x, ori_gps.y);
		dist = dist*cos( head_path_angle );
		//outn2<<"距离, "<<dist<<", ";
		return dist;
}
double CVehicle::LevelDist(CvPoint2D64f ori_gps,double ori_dir,CvPoint2D64f cur_gps)
{
		double north_dir = m_GpsData.GetAngle(cur_gps,ori_gps);
		double head_path_angle = (north_dir - ori_dir)*PI/180;


		//dist = 10-di;
		double dist = m_GpsData.GetDistance(cur_gps.x,cur_gps.y,ori_gps.x, ori_gps.y);
		dist = abs(dist*sin( head_path_angle ));
		//outn2<<"距离, "<<dist<<", ";
		return dist;
}
I_Map* CVehicle::getVelMap()
{
	view_vechile_vel_map = vel_Map;
	return view_vechile_vel_map;
}
I_Map* CVehicle::getLaneMidMap()
{
	view_vechile_mid_map = lane_mid_map;
	return view_vechile_mid_map;
}
CvSeq* CVehicle::getPlanRoad()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	app->critical_planningroad.Lock();
	view_vechile_planning_road = planning_road;
	app->critical_planningroad.Unlock();
	return view_vechile_planning_road;
}
int CVehicle::MoveLeft(CvPoint2D64f WayPoint[],CvPoint2D64f LeftWayPoint[],double length)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	CvPoint2D64f m_Gps_=app->GPS_Point;

	int n = 0;
	int n_next=199;
	double len = 0;
	double min_len = 99999;
	double ll=0;
	for(int i = 0;i<199;i++)
	{
		len = m_GpsData.GetDistance(m_Gps_.x,m_Gps_.y,WayPoint[i].x,WayPoint[i].y);
		if(len < min_len)
		{
			min_len = len;
			n = i;
		}
	}
	for(int i=n+1;i<200;i++)
	{
		ll=ll+m_GpsData.GetDistance(m_Gps_.x,m_Gps_.y,WayPoint[i].x,WayPoint[i].y);
		if(ll>15)
		{
			n_next=i;
			break;
		}
	}

	double dir1 = m_GpsData.GetAngle(WayPoint[n_next],WayPoint[n]);
	//double dir1 = m_GpsData.GetAngle(WayPoint[199],WayPoint[0]);
	double dir2 = dir1-90;
	double rad_dir2 = dir2*PI/180;

	double x = length * cos(rad_dir2);
	double y = length * sin(rad_dir2);

	double dx = (x*180)/(6378137*PI);
	//double dy = (y*180)/(6378137*PI*cos(x*PI/180));
	double dy = (y*180)/(6378137*PI*cos((WayPoint[n].x)*PI/180));

	
	//outn1<<"偏差"<<dx*100000<<", "<<dy*100000<<endl;
	for(int i=0;i<200;i++)
	{
		LeftWayPoint[i].x = WayPoint[i].x + dx;
		LeftWayPoint[i].y = WayPoint[i].y + dy;

	}
	return 1;
}
int CVehicle::MoveLeft(CvPoint2D64f WayPoint[],CvPoint2D64f LeftWayPoint[],double length,CvPoint2D64f &err)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	CvPoint2D64f m_Gps_=app->GPS_Point;

	int n = 0;
	int n_next=199;
	double len = 0;
	double min_len = 99999;
	double ll=0;
	for(int i = 0;i<199;i++)
	{
		len = m_GpsData.GetDistance(m_Gps_.x,m_Gps_.y,WayPoint[i].x,WayPoint[i].y);
		if(len < min_len)
		{
			min_len = len;
			n = i;
		}
	}
	for(int i=n+1;i<200;i++)
	{
		ll=ll+m_GpsData.GetDistance(m_Gps_.x,m_Gps_.y,WayPoint[i].x,WayPoint[i].y);
		if(ll>15)
		{
			n_next=i;
			break;
		}
	}

	double dir1 = m_GpsData.GetAngle(WayPoint[n_next],WayPoint[n]);
	//double dir1 = m_GpsData.GetAngle(WayPoint[199],WayPoint[0]);
	double dir2 = dir1-90;
	double rad_dir2 = dir2*PI/180;

	double x = length * cos(rad_dir2);
	double y = length * sin(rad_dir2);

	double dx = (x*180)/(6378137*PI);
	//double dy = (y*180)/(6378137*PI*cos(x*PI/180));
	double dy = (y*180)/(6378137*PI*cos((WayPoint[n].x)*PI/180));
	err.x -= dx;
	err.y -= dy;

	//outn1<<"偏差"<<dx*100000<<", "<<dy*100000<<endl;
	for(int i=0;i<200;i++)
	{
		LeftWayPoint[i].x = WayPoint[i].x + dx;
		LeftWayPoint[i].y = WayPoint[i].y + dy;

	}
	return 1;
}
int CVehicle::MoveRight(CvPoint2D64f WayPoint[],CvPoint2D64f RightWayPoint[],double length)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	CvPoint2D64f m_Gps_=app->GPS_Point;

	int n = 0;
	int n_next=199;
	double len = 0;
	double min_len = 99999;
	double ll=0;
	for(int i = 0;i<199;i++)
	{
		len = m_GpsData.GetDistance(m_Gps_.x,m_Gps_.y,WayPoint[i].x,WayPoint[i].y);
		if(len < min_len)
		{
			min_len = len;
			n = i;
		}
	}
	for(int i=n+1;i<200;i++)
	{
		ll=ll+m_GpsData.GetDistance(m_Gps_.x,m_Gps_.y,WayPoint[i].x,WayPoint[i].y);
		if(ll>15)
		{
			n_next=i;
			break;
		}
	}

	double dir1 = m_GpsData.GetAngle(WayPoint[n_next],WayPoint[n]);
	//double dir1 = m_GpsData.GetAngle(WayPoint[199],WayPoint[0]);
	double dir2 = dir1 + 90;
	double rad_dir2 = dir2*PI/180;

	double x = length * sin(rad_dir2);
	double y = length * cos(rad_dir2);

	double dx = (x*180)/(6378137*PI);
	//double dy = (y*180)/(6378137*PI*cos(x*PI/180));
	double dy = (y*180)/(6378137*PI*cos((WayPoint[n].x)*PI/180));
		
	//outn1<<"偏差"<<dx*100000<<", "<<dy*100000<<endl;
	for(int i=0;i<200;i++)
	{
		RightWayPoint[i].x = WayPoint[i].x + dx;
		RightWayPoint[i].y = WayPoint[i].y + dy;

	}
	return 1;
}

void CVehicle::seq_fuzhi(int object)
{
	seq_num = object;
}
void CVehicle::PathFuZhi(CvPoint2D64f goal,CvPoint2D64f *b)
{
	for(int i = 0;i<200;i++)
	{
		b[i].x = goal.x;
		b[i].y = goal.y;
	}
}

int CVehicle::MoveLeft1(CvPoint2D64f WayPoint[],CvPoint2D64f LeftWayPoint[],double length)
{

	double dir1 = m_GpsData.GetAngle(WayPoint[199],WayPoint[0]);
	double dir2 = dir1-90;
	double rad_dir2 = dir2*PI/180;

	double x = length ;
	double y = 0;

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
int CVehicle::MoveLeft2(CvPoint2D64f WayPoint[],CvPoint2D64f LeftWayPoint[],double length)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	CvPoint2D64f ori_map[200];
	CvPoint2D64f left_map[200];
	for(int i = 1;i<200;i++)
	{
		ori_map[i] = m_GpsData.APiontConverD(app->GPS_Point,WayPoint[i],app->GPS_Direction);
	}
	ori_map[0] = cvPoint2D64f(256,412);
	for(int i = 0;i<200;i++)
	{
		left_map[i].x = ori_map[i].x - length*5;
		left_map[i].y = ori_map[i].y;
	}
	for(int i = 1;i<200;i++)
	{
		LeftWayPoint[i] = m_GpsData.MaptoGPS(app->GPS_Point,app->GPS_Direction,left_map[i]);
	}
	LeftWayPoint[0] = cvPoint2D64f(app->GPS_Point.x,app->GPS_Point.y);
	return 1;
}
int CVehicle::MoveRight2(CvPoint2D64f WayPoint[],CvPoint2D64f LeftWayPoint[],double length)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	CvPoint2D64f ori_map[200];
	CvPoint2D64f left_map[200];
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_dir = app->GPS_Direction;
	for(int i = 1;i<200;i++)
	{
		ori_map[i] = m_GpsData.APiontConverD(m_gps,WayPoint[i],m_dir);
	}
	ori_map[0] = cvPoint2D64f(256,412);
	for(int i = 0;i<200;i++)
	{
		left_map[i].x = ori_map[i].x + length*5;
		left_map[i].y = ori_map[i].y;
	}
	for(int i = 0;i<200;i++)
	{
		LeftWayPoint[i] = m_GpsData.MaptoGPS(m_gps,m_dir,left_map[i]);
	}
	
	return 1;
}
int CVehicle::MoveWay (CvPoint2D64f m_gps, double m_gpsdir,CvPoint2D64f APoint[],int num,CvPoint2D64f WayPoint[])
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
int CVehicle::MoveWay(CvPoint2D64f m_gps, double m_gpsdir,CvPoint2D64f WayPoint[])
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
int CVehicle::MoveWay(CvPoint2D64f m_gps, double m_gpsdir,CvPoint2D64f WayPoint[],CvPoint2D64f &err)
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
	err.x += dx;
	err.y += dy;
	//outn1<<"偏差"<<dx*100000<<", "<<dy*100000<<endl;
	for(int i=0;i<200;i++)
	{
		WayPoint[i].x -= dx;
		WayPoint[i].y -= dy;

	}
	return 1;
}
double CVehicle::Countk(CvPoint2D64f *line,int l_num)
{
	double k = 0;
	double sumx = 0;
	double sumy = 0;
	double xmean , ymean = 0;
	double sumxx,sumxy = 0;
	for(int i = 0;i<l_num;i++)
	{
		sumx+=line[i].y;
		sumy+=line[i].x;
	}
	xmean = sumx/l_num;
	ymean = sumy/l_num;
	for(int i = 0;i<l_num;i++)
	{
		sumxx += (line[i].y - xmean) * (line[i].y - xmean);
		sumxy += ((line[i].y - xmean) * (line[i].x- ymean));
	}
	/*if(sumxx < 3*l_num)
		return 10000;*/
	//return sumxy/sumxx;

	return sumxx/sumxy;
}
int CVehicle::MoveWay(CvPoint2D64f center,CvPoint2D64f WayPoint[],int num)
{
	double mindis = 100000;
	int v_num = 0;
	double dis;
	for(int i=0;i<num;i++)
	{
		dis = sqrt((center.x-WayPoint[i].x)*(center.x-WayPoint[i].x)+(center.y-WayPoint[i].y)*(center.y-WayPoint[i].y));
		if(dis < mindis)
		{
			mindis= dis;
			v_num = i;
		}
	}

	double dx = WayPoint[v_num].x - center.x;
	double dy = WayPoint[v_num].y - center.y;
	
	//outn1<<"偏差"<<dx*100000<<", "<<dy*100000<<endl;
	for(int i=0;i<num;i++)
	{
		WayPoint[i].x -= dx;
		WayPoint[i].y -= dy;

	}
	return 1;
}
int CVehicle::MoveLeftDir(CvPoint2D64f WayPoint[],CvPoint2D64f LeftWayPoint[],double length,double dir1)
{
	//double dir1 = m_GpsData.GetAngle(WayPoint[199],WayPoint[0]);
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
int CVehicle::MoveLeftDir(CvPoint2D64f WayPoint[],CvPoint2D64f LeftWayPoint[],double length,double dir1,CvPoint2D64f &err)
{
	//double dir1 = m_GpsData.GetAngle(WayPoint[199],WayPoint[0]);
	double dir2 = dir1-90;
	double rad_dir2 = dir2*PI/180;

	double x = length * cos(rad_dir2);
	double y = length * sin(rad_dir2);

	double dx = (x*180)/(6378137*PI);
	double dy = (y*180)/(6378137*PI*cos(x*PI/180));
	err.x -= dx;
	err.y -= dy;

	//outn1<<"偏差"<<dx*100000<<", "<<dy*100000<<endl;
	for(int i=0;i<200;i++)
	{
		LeftWayPoint[i].x = WayPoint[i].x + dx;
		LeftWayPoint[i].y = WayPoint[i].y + dy;

	}
	return 1;
}
I_Map* CVehicle::getXinMap()
{
	view_vechile_ob_map = map_ts;
	return view_vechile_ob_map;
}
I_Map* CVehicle::getDangerMap()
{
	view_vechile_ob_map = map_dan;
	return view_vechile_ob_map;
}
I_Map* CVehicle::getVMap()
{
	view_vechile_ob_map = map_v;
	return view_vechile_ob_map;
}

I_Map* CVehicle::getDyMap()
{
	view_vechile_dy_map = dy_map;
	return view_vechile_dy_map;
}


double CVehicle::returnDist(LEAD a, CvPoint2D64f b,double dir)
{
	CvPoint2D64f temp;
	double dist = 0;
	temp.x = a.lat;
	temp.y = a.lng;
	dist = m_GpsData.GetDistance(b.x,b.y,temp.x,temp.y);
	//dist = VertDist(b,dir,temp);
	return dist;
}