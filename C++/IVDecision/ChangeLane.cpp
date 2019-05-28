#include "StdAfx.h"
#include "ChangeLane.h"
#include<fstream>
using namespace std;
ofstream out127("huandao.txt");
CChangeLane::CChangeLane(void)
{
	num  = 0;
	storage_road = cvCreateMemStorage(0);
	left_road = cvCreateSeq( CV_64FC2, sizeof(CvSeq), sizeof(CvPoint2D64f), storage_road );
	
	
}

CChangeLane::~CChangeLane(void)
{
	cvReleaseMemStorage(&storage_road);
}
void CChangeLane::ChangeLaneDirve()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	//app->raozhang = true;
	//app->left_right = -1;

	ChangeLeft_RightLane(MidGpsPoint,app->left_right);
	////quan ju bian liang buzuo can shu
	//app->left_right = -1;
	//ChangeLeftLanecos(vel_Map,realtime_Gps,realtime_Dir,vehicleob_x,vehicleob_y,app->left_right);

}

bool CChangeLane::ChangeLeft_RightLane(CvPoint2D64f cur_point[],int left_right)//1表示左边换道，-1表示右边换道
{
		
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->zuo_huandao = false;
	app->you_hundao = false;
	app->raozhang = true;
	CvPoint2D64f m_gps;
	double dist = 0;
	///on_obs = true;
	app->v_desire = 10;
	app->v_aim_length = 15;
	CvPoint2D64f temp_point[200] = {0};

	double huandao_dist = SearchChangeLaneDist(app->left_right,vehicleob_x,vehicleob_y);
	huandao_dist-=0.5;
	out127<<"换道距离 "<<huandao_dist<<endl;
	if(huandao_dist == 0)
	{	
		Stop();
		return 0;
	}

	if(left_right == 1)
	MoveLeft(cur_point,temp_point,3.5);
	if(left_right == -1)
	MoveLeft(cur_point,temp_point,-3.5);

	//MoveRight2(cur_point,temp_point,huandao_dist);

	double dir = m_GpsData.GetAngle(temp_point[199],temp_point[0]);
	//ArrayFuZhi(temp_point,MidGpsPoint);
///////////////////
	app->critical_planningroad.Lock();
	cvClearSeq( planning_road );
	for(int i = 0; i<200; i++)
	{
		cvSeqPush( planning_road, &temp_point[i]);
	}
	app->critical_planningroad.Unlock();
	SetEvent(app->m_hEvent);//路径更新后，发送消息
	
////////////////////
	int countdd = 0;
	//bool ob = false;
	double angle = m_GpsData.GetAngle(MidGpsPoint[199].x,MidGpsPoint[199].y,MidGpsPoint[190].x,MidGpsPoint[190].y);
	CvPoint2D64f ori_gps = app->GPS_Point;
	double ori_dir = app->GPS_Direction;
	out127<<" angle "<<angle<<endl;
	double level_dist = 0;
	double dir_err = 0;
	double last_dir_err = 0;
	double ob_dis = 0;
	double ob_ang = 0;
	double ob_firstdis = 0;
	Sleep(200);
	FindObDisAngle(temp_point,ori_gps,ob_firstdis,ob_ang);
	while(1)
	{   
	  
		app->critical_section.Lock();
		CvPoint2D64f m_gps = app->GPS_Point;
		double m_gpsdir = app->GPS_Direction;
		app->critical_section.Unlock();
		//level_dist = LevelDist(ori_gps,ori_dir,m_gps);
		
		last_dir_err = dir_err;
		out127<< " ob_firstdis "<<ob_firstdis<<" app->raozhang "<<app->raozhang<<endl;
		FindObDisAngle(temp_point,m_gps,ob_dis,ob_ang);
		out127<<" ob_dis "<<ob_dis<<endl;
		if((abs(m_gpsdir - ob_ang) < 7 || abs(360 - abs(m_gpsdir - ob_ang)) < 7) && abs(ob_dis )< 0.5)
		{	
			out127<<"quit"<<endl;
			break ;
		}
		if(abs(ob_dis )> 0.5*abs(ob_firstdis))
			app->raozhang = true;
		else
			app->raozhang = false;
		//level_dist = LevelDist(ori_gps,ori_dir,m_gps);
	
		/*if((abs(m_gpsdir - angle) < 7 || abs(360 - abs(m_gpsdir - angle)) < 7) && level_dist > 1.5)
		{	
			out127<<"quit"<<endl;
			break ;
		}*/
		/*if(dd < 5)
			break;*/
		if(countdd > 100)
		{
			out127<<"countdd tuichu"<<endl;
			break;
			
		}

		vel_Map = app->PercepMap;
		bool obb = SearchObstacleObstacle(temp_point,vel_Map,app->GPS_Speed*3+25,1);
		if(obb)
		{
			out127<<"obb tui chu"<<endl;
			Stop();
	
			break;
		}
		if(app->onroad == 5002)
			break;
		
		Sleep(100);
		countdd++;

	}//换道完成

	/*if(left_right == 1)
		app->zuo_huandao = false;
	if(left_right == -1)
		app->you_hundao = false;*/
	//on_obs = false;

	return 0;
	//aim_length = 18;
}
void CChangeLane::JudgeLeftOrRight()
{
	/*CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->secondDecison.Compare("左换道");
		left_right = 1;
	app->secondDecison.Compare("右换道");
		left_right = -1;*/
}
bool CChangeLane::ChangeLeftLanecos(I_Map *map,CvPoint2D64f rposition,double rdirection,int ob_x,int ob_y,int left_right)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
		app->zuo_huandao = false;
	cl_time = (int)((412 - ob_y)*0.2 - 15)/8;
	if(cl_time<1)cl_time=1;
	CvPoint2D64f temp = cvPoint2D64f(0,0);
	//app->critical_map.Lock();
	//for(int i = 0;i<512;i++)
	//{
	//	for(int j = 0;j<512;j++)
	//	{
	//		out127<<map->MapPoint[i][j];
	//	}
	//	out127<<endl;
	//}
	//app->critical_map.Unlock();
	int temp1 = 0;
	double *k_temp1,*k_temp2;

	temp1= GetCurLinenum(map);
	//确定路的方向
	
	int last_i = 0;

	for(int j = 350;j>150;j--)
		for(int i = 511;i>0;i--)
		{
			if(map->MapPoint[j][i] == temp1)
			{
				temp.x = i;
				
				temp.y = j;
				if(i!=last_i)
				cvSeqPush( left_road, &temp );
				last_i = i;
				break;
			}
		}
	num = left_road->total;
	if(left_road->total <= 0)
		return false;
	left_temp = (CvPoint2D64f *)malloc(num*sizeof(CvPoint2D64f) );

	int k = 0;
	while(k<num)
	{
		left_temp[k] = *(CvPoint2D64f*)cvGetSeqElem( left_road,0 );
		cvSeqPopFront( left_road,NULL );
		
		k++;
	}
	k = 0;
	double k_xielv = 0;
	double kmax1 = 0;
	double kmax2 = 0;
	int num2=num;
	/*if(num%2 == 0)*/
	{	
		k_temp1 = (double *)malloc((num - 1)*sizeof(double) );
		/*for(int i = 0;i<num-2;i+=2)*/
		for(int i = 0;i<num-1;i++)
		{
			
			//if(left_temp[i].x - left_temp[i+1].x == 0 )
			//{num2--;
			//continue;
			//}/*k_temp1[k] = 100000000000;*/
			//else
				k_temp1[k] = (left_temp[i].y - left_temp[i+1].y)/(double)(left_temp[i].x - left_temp[i+1].x);
			kmax1 += k_temp1[k];
			k++;
		}
		free(k_temp1);
		 k_xielv = (kmax1/(num2));
	}
	
	free(left_temp);

	double sita = PI/2 -atan(k_xielv);
	
	CvPoint2D64f *pos_zhijiao;
	CvPoint2D64f *pos;
	
	pos_zhijiao = (CvPoint2D64f *)malloc((/*2*10*/80+cl_time*VEL)*sizeof(CvPoint2D64f)) ;
	pos = (CvPoint2D64f *)malloc((/*2*10*/80+cl_time*VEL)*sizeof(CvPoint2D64f)) ;
	

	double wid_lane = 3.8;
	
	
	
	for(int i = 0;i<10;i++)
	{
		pos_zhijiao[i].x = 0;
		pos_zhijiao[i].y = i;
	}
	for(int i = 0+10;i<cl_time*VEL+10;i++)
	{
		pos_zhijiao[i].x = double(app->left_right)* wid_lane/2.0*(1 - cos(PI/(cl_time*VEL)*(i-10)));
		pos_zhijiao[i].y = i;
	}

	for(int i =cl_time*VEL +10;i<cl_time*VEL +80/*2*10*/;i++)
	{
		pos_zhijiao[i].x = pos_zhijiao[i-1].x;
		pos_zhijiao[i].y = i;
	}
	for(int i = 0;i<cl_time*VEL+/*2*10*/80;i++)
	{
		pos[i].x = -(pos_zhijiao[i].x/cos(sita)+(pos_zhijiao[i].y - pos_zhijiao[i].x*tan(sita))*sin(sita))*5 + 256;
		pos[i].y = 412 - (pos_zhijiao[i].y - pos_zhijiao[i].x*tan(sita))*cos(sita)*5;
	}
	/*for(int i = 0;i<20;i++)
	{
		out127<<pos[i].x<<","<<pos[i].y<<endl;
	}*/

	
	CvPoint2D64f *turn_pathgps;
	turn_pathgps = (CvPoint2D64f *)malloc((/*2*10*/80+cl_time*VEL)*sizeof(CvPoint2D64f)) ;
	for(int i = 1;i<cl_time*VEL+/*2*10*/80;i++)
	{
		turn_pathgps[i] = m_GpsData.MaptoGPS(rposition,rdirection,pos[i]);
	}
	turn_pathgps[0] = rposition;
	Bezier(turn_pathgps,cl_time*VEL+/*2*10*/80-1,MidGpsPoint);
////////////////////////
	bool ob = false;
	ob = SearchObstacleMoveWay(MidGpsPoint,vel_Map,50,1);
	if(ob)
	{
		for(int i = cl_time*VEL+10;i<cl_time*VEL +80/*2*10*/;i++)
		{
			pos[i].x = pos[i].x + app->left_right * 2;
		
		}
		for(int i = 1;i<cl_time*VEL+/*2*10*/80;i++)
		{
			turn_pathgps[i] = m_GpsData.MaptoGPS(rposition,rdirection,pos[i]);
		}
		turn_pathgps[0] = rposition;
		Bezier(turn_pathgps,cl_time*VEL+/*2*10*/80-1,MidGpsPoint);
	}
	app->critical_planningroad.Lock();
	cvClearSeq( planning_road );
	int i_youzhuan = 0;
	for(int i = 0; i<200; i++)
	{
		cvSeqPush( planning_road, &MidGpsPoint[i]);
	}
	app->critical_planningroad.Unlock();
	SetEvent(app->m_hEvent);//路径更新后，发送消息
	

	int countdd = 0;
	//bool ob = false;
	double angle = m_GpsData.GetAngle(MidGpsPoint[199].x,MidGpsPoint[199].y,MidGpsPoint[190].x,MidGpsPoint[190].y);
	CvPoint2D64f ori_gps = app->GPS_Point;
	double ori_dir = app->GPS_Direction;
	out127<<" angle "<<angle<<endl;
	double level_dist = 0;
	Sleep(200);
	while(1)
	{  
		app->critical_section.Lock();
		CvPoint2D64f m_gps = app->GPS_Point;
		double m_gpsdir = app->GPS_Direction;
		app->critical_section.Unlock();
		out127<<" realtime_gps "<<m_gpsdir<<endl; 
		level_dist = LevelDist(ori_gps,ori_dir,m_gps);
		out127 << "水平距离" <<level_dist<<endl;
		if((abs(m_gpsdir - angle) < 10 || abs(360 - abs(m_gpsdir - angle)) < 10) && level_dist > 1.2)
		{	
			out127<<"quit"<<endl;
			break ;
		}
		/*if(dd < 5)
			break;*/
		if(countdd > 200)
		{
			break;
		}
		bool obb = SearchObstacleObstacle(MidGpsPoint,vel_Map,30,1);
		if(obb)
		{
			this->Stop();
	
			break;
		}
		//if(!obb)
		//{
		//	cvClearSeq( planning_road );
		//	for(int i = 0; i<200; i++)
		//	{
		//		cvSeqPush( planning_road, &MidGpsPoint[i]);
		//	}

		//	SetEvent(app->m_hEvent);//路径更新后，发送消息

		//}
		Sleep(100);
		countdd++;
	}//换道完成
	free(pos);
	free(pos_zhijiao);
	Sleep(2000);
	//on_obs = false;
	return 0;
	//aim_length = 18;
}

int CChangeLane::GetCurLinenum(I_Map *map)
{
	int temp1;
	for(int j = 412;j>250;j--)
		for(int i = 511;i>0;i--)
		{
			if(map->MapPoint[j][i] > 10 &&map->MapPoint[j][i]<18)
			{
				temp1 = map->MapPoint[j][i];
				
				return temp1;
			}
		}

}

bool CChangeLane::SearchObstacleMoveWay(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down)
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
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
					continue;
		x = Rndf_MapPoint[i].x ;//2012.7.18xiugai -10
		y = Rndf_MapPoint[i].y;
		for(int m=-5;m<5;m++)
			for(int n=-10;n<10;n++)
			{
				if(y+m>380&&y+m<430)
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

bool CChangeLane::SearchObstacleObstacle(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down)
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
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
					continue;
		x = Rndf_MapPoint[i].x ;//2012.7.18xiugai -10
		y = Rndf_MapPoint[i].y;
		for(int m=-5;m<5;m++)
			for(int n=-3;n<3;n++)
			{
				if(y+m>380&&y+m<430)
					continue;
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
bool CChangeLane::SearchObstacleObstacle(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down,int &ob_x,int ob_y)
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
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511|Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
					continue;
		x = Rndf_MapPoint[i].x ;//2012.7.18xiugai -10
		y = Rndf_MapPoint[i].y;
		for(int m=-5;m<5;m++)
			for(int n=-3;n<3;n++)
			{
				if(y+m>380&&y+m<430)
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
double CChangeLane::SearchChangeLaneDist(int left_right,int ob_x,int ob_y)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();


	CvPoint2D64f map_midpoint[200];
	CvPoint2D64f map_pointzone[20];

	int k = 0;
	double dist_min = 0;
	double shuchuzhi = 0;


	for(int i = 1;i<200;i++)
	{
		map_midpoint[i] = m_GpsData.APiontConverD(app->GPS_Point,MidGpsPoint[i],app->GPS_Direction);
	}
	map_midpoint[0] = cvPoint2D64f(256,412);
	if(!Findedge(vel_Map,ob_x, ob_y,5,map_pointzone,k))//map_pointzone记录障碍点 k为障碍点的个数
	{
		int nn = 0;
		int x = FindXfromPath(map_midpoint, 200,ob_y, nn);
		
		double kp = (map_midpoint[nn+1].x - x)/(map_midpoint[nn+1].y - ob_y);
		double a = 3.7 * kp;
		double b = sqrt(3.7*3.7 + a*a);
		/////////没有障碍 换算垂直距离到水平
		return b;
	}

	//计算每个障碍点的距离
	double temp_dist[20] = {0};
	
	for(int i = 0; i<k; i++)
	{
		temp_dist[i] = DistancePointToPath(map_pointzone[i], map_midpoint, 200)*0.2;
	}
	
	////////最小距离及对应点
	int min_id = 0;
	double mindist = 0;
	FindMinPointID(temp_dist, k, min_id, mindist);//第min_id个点是最小距离
	
	/////查出这个距离对应于那个坐标点
	int bianyan_y = map_pointzone[min_id].y;
	int bianyan_x = FindXfromPath(map_midpoint, 200,bianyan_y,k);

	/*shuchuzhi =  (map_midpoint[pinxin_x].x - ob_x)*0.2*(mindist-1.9)/mindist;*/
	shuchuzhi = abs((bianyan_x - map_pointzone[min_id].x)*0.2*(mindist-1.9)/mindist);
	if(shuchuzhi > 3.7)
		shuchuzhi = 3.7;
	if((mindist-1.9)<1)
		return 0;
	return shuchuzhi;
}

int CChangeLane::FindXfromPath(CvPoint2D64f *map_path, int path_num,int y, int &id)
{
	int temp_d = 1000000;
	int temp_x = 0;
	int temp_id = 0; 
	for(int i = 0; i<path_num; i++)
	{
		double d = abs( map_path[i].y - y);
		if( d < temp_d )
		{
			temp_d = d;
			temp_x = map_path[i].x;
			temp_id = i;
		}
	}
	return temp_x;
}

int CChangeLane::Findedge(I_Map *map, int ob_x, int ob_y, int step, CvPoint2D64f *map_pointzone, int &n)//step上下扩展行数,n边缘障碍个数
{
	int k = 0;
	for(int i = ob_y+step;i>ob_y-step;i--)
	{
		for(int j = ob_x + 15;j<ob_x+35;j++)
		{
			if(vel_Map->MapPoint[i][j] == 8 ||vel_Map->MapPoint[i][j] == 18)
			{
				map_pointzone[k].x = j;
				map_pointzone[k].y = i;
				k++;
				break;
			}
		}
	}
	n = k;
	if(n>0)
		return 1;
	else 
		return 0;
}

double CChangeLane::DistancePointToPath(CvPoint2D64f point, CvPoint2D64f *path, int path_num)
{
	double dis = 100000;
	for( int i = 0; i<path_num; i++)
	{
		double dis1 = DistancePointToPoint(point, path[i]);
		if(dis1<dis)
			dis = dis1;
	}
	return dis;
}

double CChangeLane::DistancePointToPoint(CvPoint2D64f point1, CvPoint2D64f point2)
{
	double x = point1.x - point2.x;
	double y = point1.y - point2.y;
	return sqrt(x*x+y*y);
}

int CChangeLane::FindMinPointID(double *min_dist, int k,int &id, double &mindist)//从一组距离内找到最小的距离和它是第几个
{

	double temp_dist[20];
	for(int i=0;i<k;i++)
	{
		temp_dist[i] = min_dist[i];		
	}
	for(int i=0;i<k;i++)
	{
		if(temp_dist[i]<temp_dist[i+1])
		{
			temp_dist[i+1] = temp_dist[i];
		}
	}
	/////查出这个距离对应于那个坐标点
	int bianyan_y = 0;
	for(int i = 0;i<k;i++)
	{
		if(min_dist[i] == temp_dist[k-1])
		{
			bianyan_y = i;
		}
	}
	id = bianyan_y;
	mindist = temp_dist[k-1];

	return 1;
}
void CChangeLane::FindObDisAngle(CvPoint2D64f *midgps,CvPoint2D64f m_gps,double &obdis,double &ob_ang)
{
	double temp = 1000;
	double temp1= 1000;
	int k = 0;
	for(int i=0;i<200;i++)
	{
		temp1 = m_GpsData.GetDistance(m_gps.x,m_gps.y,midgps[i].x,midgps[i].y);
		if(temp > temp1)
		{
			temp = temp1;
			k = i;
		}
	}
	obdis = temp;
	ob_ang = m_GpsData.GetAngle(midgps[k+1].x,midgps[k+1].y,midgps[k].x,midgps[k].y);
}