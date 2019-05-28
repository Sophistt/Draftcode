#include "StdAfx.h"
#include "UTurn.h"
#include<fstream>

using namespace std;
ofstream out778("uturn.txt");

CUTurn::CUTurn(void)
{
}

CUTurn::~CUTurn(void)
{
}
void CUTurn::ReturnUTurnPath(CvSeq *gp,int seq,CvPoint2D64f (&upathyc)[200])
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->secondDecison = "UTurn";
	app->critical_section.Lock();
	CvPoint2D64f m_gps = app->GPS_Point;
	double dir = app->GPS_Direction;
	app->critical_section.Unlock();
	CvPoint2D64f waypoint[4] = {0};
	int xia_y = 0;
	CvPoint2D64f uturnpath[200];
	for(int i = 0;i<4;i++)
	{
		waypoint[i] = *(CvPoint2D64f*)cvGetSeqElem( gp, seq-2+i);

	
	}
	//waypoint[0].x = LeadPoint[seq-3].lat;
	//waypoint[0].y = LeadPoint[seq-3].lng;
	//
	//waypoint[1].x = LeadPoint[seq-2].lat;
	//waypoint[1].y = LeadPoint[seq-2].lng;	
	//
	//waypoint[2].x = LeadPoint[seq-1].lat;
	//waypoint[2].y = LeadPoint[seq-1].lng;

	//waypoint[3].x = LeadPoint[seq].lat;
	//waypoint[3].y = LeadPoint[seq].lng;
	

	

	CvPoint2D64f endpoint;                           //存储汽车地图中的中点
	

	CvPoint2D64f point[4] = {0};
	for(int i = 0;i<4;i++)
	{
		point[i] = m_GpsData.APiontConverD(m_gps,waypoint[i],dir);
	}

	///////////////////////

///////////////////////
/****************************************************************************************
至少在路点前搜索到20个道路边沿点
*****************************************************************************************/
	
   

	vel_Map = app->PercepMap; // getPerceptionMap();
	//for(int i = 0;i<512;i++)
	//{
	//	for(int j = 0;j<512;j++)
	//		//if(app->PercepMap->MapPoint[i][j] == 0)
	//			out778<<vel_Map->MapPoint[i][j];
	//		//else out333<<1;
	//	out778<<endl;
	//}
	int xia_x  = 256;
	//endpoint = SearchHuokou(vel_Map,xia_y,xia_x,);



	 CvPoint2D64f line[512];
	 int l_num=0;
	 int youbianjie = 150;
	 int zuobianjie = 332;
	 for (int i=xia_y + 15;i<412;i++)
	 {
		 for (int j=xia_x + 15;j>0;j--)
		 {
			 if (vel_Map->MapPoint[i][j]==8&&j>youbianjie&&j<zuobianjie)
			 {
				 line[l_num].y=i;
				 line[l_num].x=j;
				 youbianjie = j-20;
				 zuobianjie= j+20;
				 l_num++;	
				 break;
			 }

		 }

	 }
	 
	 double k = 9999999;
	 int count_kk = 0;
	 if(endpoint.x != 248 && endpoint.y !=350&&xia_y != 380)
	 k =  Countk(line,l_num);
	 for (int i = 0;i<l_num-1;i++)
	 {
		 if (line[i].x == line[i+1].x)
		 {
			 count_kk++;
		 }
	 }
	 if (count_kk > l_num - 3)
	 {
		k = 99999999;
	 }
	
	 //搜索豁口下端点


	
	
	CvPoint2D64f pedal1,pedal2;
	CvPoint2D64f ori_point1 = cvPoint2D64f(256,412);
	CvPoint2D64f ori_point2;
	//CvPoint2D64f ori_point2 = point[2];
	
	pedal1 = CountChuiZu(ori_point1,endpoint,k);
	double s = sqrt(pow((endpoint.x - pedal1.x),2)+pow((endpoint.y - pedal1.y),2))*0.2;
	pedal2.x = endpoint.x - (7/s)*(pedal1.x - endpoint.x);
	pedal2.y = endpoint.y - (7/s)*(pedal1.y - endpoint.y);
	//pedal2 = CountChuiZu(ori_point2,endpoint,k);

	ori_point2.x = 256-(pedal1.x - pedal2.x);
	ori_point2.y = 412-(pedal1.y - pedal2.y);

	
	CvPoint2D64f u_point[11] = {0};

	u_point[0].x = 256;
	u_point[0].y = 412;
	u_point[1].x = (256+2*pedal1.x)/3;
	u_point[1].y = (412+2*pedal1.y)/3;



	u_point[2]=pedal1;

	u_point[3].x=(endpoint.x+2*pedal1.x)/3;
	u_point[3].y=(endpoint.y+2*pedal1.y)/3;


	u_point[4].x=(2*endpoint.x+pedal1.x)/3;
	u_point[4].y=(2*endpoint.y+pedal1.y)/3;

	u_point[5]=endpoint;




	CvPoint2D64f a[5] = {0};
	CvPoint2D64f tempbezier[200] = {0};
	CvPoint2D64f temprightgps[5] = {0};
	CvPoint2D64f tempright[5] = {0};


	u_point[6].x=(2*endpoint.x+pedal2.x)/3;
	u_point[6].y=(2*endpoint.y+pedal2.y)/3;
	u_point[7].x=(endpoint.x+2*pedal2.x)/3;
	u_point[7].y=(endpoint.y+2*pedal2.y)/3;
	u_point[8]=pedal2;
	u_point[9].x = (ori_point2.x+2*pedal2.x)/3;
	u_point[9].y = (ori_point2.y+2*pedal2.y)/3;
	u_point[10] = ori_point2;

	CvPoint2D64f umappoint[200] = {0};
	int count = 0;
	while(1)
	{
	

		
		Bezier(u_point,10,umappoint);
		if(SearchUTurnObstacleMoveWay1(umappoint,vel_Map))
		{
			
			for(int i =6;i<11;i++)
			{
				u_point[i].x = u_point[i].x - 10;
			}
			count++;
			if(count>3)
				break;
		}
		else
			break;
	}
	
	for(int i = 1;i<200;i++)
	{
		uturnpath[i] = m_GpsData.MaptoGPS(m_gps,dir,umappoint[i]);
	}
	uturnpath[0] = m_gps;

	//////////4.24
	CvPoint2D64f aaa,bbb;
	aaa = m_GpsData.MaptoGPS(app->GPS_Point,app->GPS_Direction,u_point[9]);
	bbb = m_GpsData.MaptoGPS(app->GPS_Point,app->GPS_Direction,u_point[10]);
	double angel = m_GpsData.GetAngle(bbb,aaa);

	for (int i = 0;i<100;i++)
	{
		upathyc[i]=uturnpath[2*i];
	}
	for(int i=100;i<200;i++)
	{
		upathyc[i]=m_GpsData.MaptoGPS(uturnpath[199],angel,cvPoint2D64f(256,412-(i-100)*2));
	}
	//////////////4.24

	////for(int i = 0;i<200;i++)
	////{
	////	out778<<" ditudian "<<umappoint[i].x<<", "<<umappoint[i].y<<" gps "<<MidGpsPoint[i].x<<", "<<MidGpsPoint[i].y<<endl;
	////}
	//
	//app->critical_planningroad.Lock();
	//cvClearSeq( planning_road );
	////app->stop = false;
	//for(int i = 0; i<200; i++)
	//{
	//	cvSeqPush( planning_road, &MidGpsPoint[i] );
	//}
	//app->critical_planningroad.Unlock();
	//SetEvent(app->m_hEvent);//路径更新后，发送消息
	//int countdd = 0;
	//double angle = m_GpsData.GetAngle(u_point[9].x,u_point[9].y,u_point[10].x,u_point[10].y);
	//while(1)
 //   {   
	//	m_gps = app->GPS_Point;
	//	dir = app->GPS_Direction;

	//	if(abs(dir - angle) < 15 || abs(360 - abs(dir - angle)) < 15)
	//		break ;
	//	out778<<"dir-angle "<<abs(dir - angle)<<endl;
	//
	//	
	//	Sleep(100);

 //   }
	//	way_out = true;


	//
	//
	//

	////GetMidPoint();
	////GetPath();
}


bool CUTurn::SearchUTurnObstacleMoveWay(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down)
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

//double CUTurn::Countk(CvPoint2D64f *line,int l_num)
//{
//	 double road_k=0;
//	 double	road_b=0;
//	 int num_k;
//	 double sum_x,sum_y,sum_xy,sum_x2;
//	 sum_x=0;
//	 sum_x2=0;
//     sum_y=0;
//	 sum_xy=0;
//	 sum_x2=0;
//
//	 for(num_k=0; num_k<l_num; num_k++)
//	 {
//		 sum_x+=line[num_k].x;
//		 sum_x2+=(line[num_k].x*line[num_k].x);
//	 }
//
//	
//	 for(num_k=0; num_k<l_num; num_k++)
//	 {
//		 sum_y+=line[num_k].y;
//	 }
//
//	
//	 for(num_k=0; num_k<l_num; num_k++)
//	 {
//		 sum_xy+=(line[num_k].x*line[num_k].y);
//	 }
//
//
//	road_k=((l_num*sum_xy-sum_x*sum_y)/(l_num*sum_x2-sum_x*sum_x));
//    road_b=((sum_x2*sum_y-sum_x*sum_xy)/(l_num*sum_x2-sum_x*sum_x));
//	return road_k;
//}
//
//	double k_xielv = 0;
//	double kmax1 = 0;
//	double kmax2 = 0;
//	double *k_temp1;
//	int 	k = 0,k2=0;
//	/*if(num%2 == 0)*/
//	{	
//		k_temp1 = (double *)malloc((l_num - 1)*sizeof(double) );
//		/*for(int i = 0;i<num-2;i+=2)*/
//		for(int i = 0;i<l_num-1;i++)
//		{
//			
//			if(line[i].x - line[i+1].x == 0 )
//				k2++;
//			else
//			{
//				k2=0;
//				k_temp1[k] = (line[i-k2].y - line[i+1].y)/(double)(line[i-k2].x - line[i+1].x);
//				k++;
//				kmax1 += k_temp1[k];
//			}
//			
//			
//		}
//		free(k_temp1);
//		 k_xielv = (kmax1/(k));
//	}
//	return k_xielv;
//}
CvPoint2D64f CUTurn::CountChuiZu(CvPoint2D64f ori,CvPoint2D64f MidPoint,double k)
{
	//k = abs(k);
	CvPoint2D64f countp;
	countp.y =352; 
	countp.x = (countp.y - ori.y+ori.x*k)/k;


	CvPoint2D64f pedal;
	int x0=0;
	int y0=0;
	double x1=ori.x;
	double y1=ori.y;
	double x2=countp.x;
	double y2=countp.y;


	double A=y2-y1;
	double B=-(x2-x1);
	double C=x2*y1-x1*y2;

	pedal.x=(B*B*MidPoint.x-A*B*MidPoint.y-A*C)/(A*A+B*B);
	pedal.y=(-A*B*MidPoint.x+A*A*MidPoint.y-B*C)/(A*A+B*B);
	return pedal;
}


bool CUTurn::SearchUTurnObstacleMoveWay1(CvPoint2D64f Rndf_MapPoint[],I_Map *map)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	
	
	


	int x = 0;
	int y = 0;
	int obcount=0;
	bool flag=0;
	int ylast=0;

	int istart=100;
	int iend=200;
	int nleft=-9;
	if(app->bthirduturn)
	{
		istart=130;
		iend=180;
		nleft=-6;
	}

	for(int i = istart;i<iend;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > 472||Rndf_MapPoint[i].y <= 0 || Rndf_MapPoint[i].y<xia_y_pub)
					continue;
		x = Rndf_MapPoint[i].x ;
		y = Rndf_MapPoint[i].y;
		for(int m=-5;m<5;m++)
			for(int n=nleft;n<14;n++)//10//for(int n=-9;n<7;n++)
			{
				/*if(y+m>380&&y+m<430)
					continue;*/
				if(y+m>472||y+m<=0||x+n>511||x+n<0||y+m<xia_y_pub)
					continue;
				if(map->MapPoint[y+m][x+n] == 8 || map->MapPoint[y+m][x+n] == 18 || map->MapPoint[y+m][x+n] == 28)
				{
					if(ylast==0)
					{
						ylast=y+m;
						obcount=1;
					}
					else
					{
						
						if(ylast<y+m)
						{
							obcount++;
							ylast=y+m;
						}
					}
					if(obcount>=7)
						return true;
				}
			}
	}
	return false;
}


bool CUTurn::SearchUTurnObstacleMoveWay1_rightside(CvPoint2D64f Rndf_MapPoint[],I_Map *map)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	int x = 0;
	int y = 0;
	for(int i = 0;i<60;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > 472||Rndf_MapPoint[i].y < 0)
					continue;
		x = Rndf_MapPoint[i].x ;
		y = Rndf_MapPoint[i].y;
		for(int m=-5;m<5;m++)
			for(int n=4;n<8;n++)//10//12
			{
				/*if(y+m>380&&y+m<430)
					continue;*/
				if(y+m>472||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == 8 || map->MapPoint[y+m][x+n] == 18 || map->MapPoint[y+m][x+n] == 28)
				{
					return true;
			
				}
			}
	}
	return false;
}


bool CUTurn::SearchUTurnObstacleMoveWay1_rightsidethird(CvPoint2D64f Rndf_MapPoint[],I_Map *map)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	int x = 0;
	int y = 0;
	for(int i = 0;i<60;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > 472||Rndf_MapPoint[i].y < 392)
					continue;
		x = Rndf_MapPoint[i].x ;
		y = Rndf_MapPoint[i].y;
		for(int m=-5;m<5;m++)
			for(int n=6;n<12;n++)//10//12
			{
				/*if(y+m>380&&y+m<430)
					continue;*/
				if(y+m>472||y+m<392||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == 8 || map->MapPoint[y+m][x+n] == 18 || map->MapPoint[y+m][x+n] == 28)
				{
					return true;
			
				}
			}
	}
	return false;
}

//CvPoint2D64f CUTurn::SearchHuokou(I_Map *map,int&xia_y)
//{
//	int width=20;
//
//	CvPoint2D64f endpoint;                           //存储汽车地图中的中点
//	int x_free[512]={0};
//	int y_free[512]={0};
//
//	int x_down=0;
//	int y_down=0;
//	int x_up=0;
//	int y_up=0;
//
//	int obstacle=0;                                  //连续障碍物点个数
//	int non_ob1=0;                                    
//	int non_ob2=0;                                  //存储豁口长度
//	bool obstacle_flag=0;
//	int i;
//	int j;
//	int youbianjie = 100;
//	for ( i=412; i>0; i--)                       //搜索豁口,得到豁口大小
//	{
//		non_ob2=non_ob1;
//						 //将得到的豁口大小传到non_ob2中
//		for ( j=256; j>100; j--)
//		{
//			if(vel_Map->MapPoint[i][j]==18&&j<256&&j>youbianjie) //当遇到道路边沿时
//			{	
//				non_ob1=0;	
//				youbianjie = j - 50;
//				break;
//			}	
//		
//			y_up=i;
//			x_up=j;
//		}
//
//			
//
//		if (non_ob2>=width&&non_ob1==0)               //当豁口大小大于阈值且搜索到的点是道路边沿时跳出
//		{
//			
//			break;
//
//		}
//		if (non_ob2>=40&&non_ob1==1)                 //如果豁口大于8米则，跳出
//		{
//			
//			y_up=i;
//			break;
//		}
//		y_free[non_ob1]=i;    
//		x_free[non_ob1]=j;
//		non_ob1++;
//
//	 }
//	x_down = x_free[0];
//	y_down = y_free[0];
//	xia_y = y_down;
//
//	endpoint.x=(x_up+x_down)/2;
//	endpoint.y=(y_up+y_down)/2-20; 
//	return endpoint;
//}
bool CUTurn::SearchKongBai(I_Map *map)
{
	bool result = false;
	return result;
}

CvPoint2D64f CUTurn::SearchHuokou(I_Map *map,int&xia_y,int&xia_x,int lane_num,CvPoint2D64f *rndfpath)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	CvPoint2D64f ob_temp[512] = {0};
	int num_temp = 0;
	CvPoint2D64f endpoint;
	int i;
	int j;
	
	app->critical_section.Lock();//锁住
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;
	app->critical_section.Unlock();

	CvPoint2D64f Rndf_MapPoint[200]={0};
	for(int i=0;i<200;i++)
	{
		
		Rndf_MapPoint[i] = m_GpsData.APiontConverD(m_gps,rndfpath[i],m_gpsdir);
		if(m_gps.x==rndfpath[i].x&&m_gps.y==rndfpath[i].y)
		{
			Rndf_MapPoint[i].x = 256;
			Rndf_MapPoint[i].y = 411;
		}
	}

	int x1 = 0;
	int x2 = 0;
	int y = 0;
	for ( i=0; i<200; i++)                       //搜索豁口,得到豁口大小
	{
		
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > 397||Rndf_MapPoint[i].y < 0)
			continue;
		x1 = Rndf_MapPoint[i].x;
		/*
		if(app->bthirduturn)
			x2 = x1- 3.7*5*(1.5);
		else
			x2 = x1- 3.7*5*(2.5);
			*/

		x2 = x1-(3.7*1.5+xdisini)*5;

		if(x2 > x1-3.7*5)
			x2 = x1-3.7*5;

		y = Rndf_MapPoint[i].y;

		for ( j=x1; j>x2; j--)
		{
			if(vel_Map->MapPoint[y][j]==8||vel_Map->MapPoint[y][j]==18||vel_Map->MapPoint[y][j]==28)
			{	
				if( num_temp>=1)
				{
					if(y == ob_temp[num_temp-1].y )
					continue;
				}
				ob_temp[num_temp].x = j;
				ob_temp[num_temp].y = y;
				num_temp++;
				break;
			}	
			
		}
	}
	if(num_temp < 3 )
	{
		endpoint = cvPoint2D64f(242,350);
		xia_y = 380;
		xia_x = 242;
		return endpoint;
	}
	/*
	if(ob_temp[0].y <349)
	{
		endpoint = cvPoint2D64f(242,350);
		xia_y = 380;
		xia_x = 242;
		return endpoint;
	}
	*/
	/*
	if(ob_temp[0].y <310)//349
	{
		endpoint = cvPoint2D64f(242,320);//350
		xia_y = 320;//380
		xia_x = 242;
		return endpoint;
	}
	*/
	int obcnt=0;
	for(i = 0;i<num_temp-1;i++)
	{

		obcnt++;
	
		if(ob_temp[i].y-ob_temp[i+1].y>30)//20//35
		{
			if(obcnt>7)
			{
				//endpoint.x = (ob_temp[i].x+ob_temp[i+1].x)/2;
				/*
				if(app->bthirduturn)
					endpoint.y = (ob_temp[i].y+ob_temp[i+1].y)/2-25;//-4///6//21
				else
					endpoint.y = (ob_temp[i].y+ob_temp[i+1].y)/2-41;//-4///6//21
					*/
				//endpoint.y = (ob_temp[i].y+ob_temp[i+1].y)/2-8;
				endpoint.x=ob_temp[i].x;
				endpoint.y = ob_temp[i].y-25;
				xia_y = ob_temp[i].y;
				xia_x = ob_temp[i].x;
				for(int j=i-1;j>=0 && j>i-5;j--)
				{
					if(xia_x<ob_temp[j].x)
						xia_x=ob_temp[j].x;
					if(endpoint.x<ob_temp[j].x)
						endpoint.x=ob_temp[j].x;
				}
				return endpoint;
			}
			else
				obcnt=0;
		}
	}
	
	endpoint = cvPoint2D64f(ob_temp[num_temp-1].x,ob_temp[num_temp-1].y-25);//25//15

	xia_y = ob_temp[num_temp-1].y;
	xia_x = ob_temp[num_temp-1].x;
	for(int j=num_temp-2;j>=0 && j>num_temp-6;j--)
	{
		if(xia_x<ob_temp[j].x)
			xia_x=ob_temp[j].x;
		if(endpoint.x<ob_temp[j].x)
			endpoint.x=ob_temp[j].x;
	}

	return endpoint;
}
//double CUTurn::Countk(CvPoint2D64f *line,int l_num)
//{
//	double k = 0;
//	double sumx = 0;
//	double sumy = 0;
//	double xmean , ymean = 0;
//	double sumxx,sumxy = 0;
//	for(int i = 0;i<l_num;i++)
//	{
//		sumx+=line[i].y;
//		sumy+=line[i].x;
//	}
//	xmean = sumx/l_num;
//	ymean = sumy/l_num;
//	for(int i = 0;i<l_num;i++)
//	{
//		sumxx += (line[i].y - xmean) * (line[i].y - xmean);
//		sumxy += ((line[i].y - xmean) * (line[i].x- ymean));
//	}
//	/*if(sumxx < 3*l_num)
//		return 10000;*/
//	//return sumxy/sumxx;
//	return sumxx/sumxy;
//}
CvPoint2D64f CUTurn::SearchZuoKongBai(I_Map *map,double k)
{
	int youbianjie = 180;
	int zuobianjie = 332;
	CvPoint2D64f ob_temp[512] = {0};
	int num_temp = 0;
	CvPoint2D64f endpoint;
	int i;
	int j;
	for ( i=380; i>257; i--)                       //搜索豁口,得到豁口大小
	{
		
						 //将得到的豁口大小传到non_ob2中
		for ( j=256; j>0; j--)
		{
			if(vel_Map->MapPoint[i][j]==18) //当遇到道路边沿时
			{	
				ob_temp[num_temp].x = j;
				ob_temp[num_temp].y = i;
				youbianjie = j - 40;
				zuobianjie = j+40;
				num_temp++;
				break;
			}	
			
		}
	}
	if(num_temp < 3 )
	{
		endpoint = cvPoint2D64f(248,350);
		
		return endpoint;
	}
	if(ob_temp[0].y <349)
	{
		endpoint = cvPoint2D64f(248,350);
		
		return endpoint;
	}
	//k = Countk(ob_temp,num_temp);
	CvPoint2D64f car = cvPoint2D64f(256,412);
	double tempx = 0;
	int inum = 0;
	for(int i = 0;i<10;i++)
	{
		if(ob_temp[i].x > tempx )
		{
			tempx = ob_temp[i].x;
			inum = i;
		}
	}

	CvPoint2D64f chuizu = CountChuiZu(ob_temp[inum],car,k);
	endpoint.x = car.x - (car.x - chuizu.x)/3;
	endpoint.y =  car.y - 50;
	return endpoint;

}
void CUTurn::ReturnUTurnKongBaiPath()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->secondDecison = "UTurn";
	app->critical_section.Lock();
	CvPoint2D64f m_gps = app->GPS_Point;
	double dir = app->GPS_Direction;
	app->critical_section.Unlock();
	CvPoint2D64f waypoint[4] = {0};
	int xia_y = 0;
	if(LeadPoint[seq_num - 2].param2 == 4)
	{for(int i = 0;i<4;i++)
	{
		waypoint[i].x = LeadPoint[seq_num +i - 3].lat;
		waypoint[i].y = LeadPoint[seq_num +i - 3].lng;
	}
	}
	if(LeadPoint[seq_num - 3].param2 == 4)
	{for(int i = 0;i<4;i++)
	{
		waypoint[i].x = LeadPoint[seq_num +i - 4].lat;
		waypoint[i].y = LeadPoint[seq_num +i - 4].lng;
	}
	}
	CvPoint2D64f ru_uturn[2] = {0};
	for(int i = 0;i<2;i++)
	{
		ru_uturn[i] = m_GpsData.APiontConverD(app->GPS_Point,waypoint[i],app->GPS_Direction);
	}
	


	CvPoint2D64f endpoint;                           //存储汽车地图中的中点
	

	CvPoint2D64f point[4] = {0};
	for(int i = 0;i<4;i++)
	{
		point[i] = m_GpsData.APiontConverD(m_gps,waypoint[i],dir);
	}

	///////////////////////

///////////////////////
/****************************************************************************************
至少在路点前搜索到20个道路边沿点
*****************************************************************************************/
	
   

	vel_Map = getPerceptionMap();
	//for(int i = 0;i<512;i++)
	//{
	//	for(int j = 0;j<512;j++)
	//	{if(app->PercepMap->MapPoint[i][j] == 18)
	//			
	//	out778<<"1";
	//	else 
	//		out778<<"0";
	//	}
	//		//else out333<<1;
	//	out778<<endl;
	//}
	int xia_x  = 256;
	//endpoint = SearchHuokou(vel_Map,xia_y,xia_x);
	double k = 99999;
	if((ru_uturn[1].x - ru_uturn[0].x) == 0)
		k = 99999;
	else
		k = (ru_uturn[1].y - ru_uturn[0].y)/(ru_uturn[1].x - ru_uturn[0].x);
	
	endpoint = SearchZuoKongBai(vel_Map,k);

	
	
	CvPoint2D64f pedal1,pedal2;
	CvPoint2D64f ori_point1 = cvPoint2D64f(256,412);
	CvPoint2D64f ori_point2;
	//CvPoint2D64f ori_point2 = point[2];
	
	pedal1 = CountChuiZu(ori_point1,endpoint,k);
	double s = sqrt(pow((endpoint.x - pedal1.x),2)+pow((endpoint.y - pedal1.y),2))*0.2;
	pedal2.x = endpoint.x - (pedal1.x - endpoint.x);
	pedal2.y = endpoint.y - (pedal1.y - endpoint.y);
	//pedal2 = CountChuiZu(ori_point2,endpoint,k);

	ori_point2.x = 256-(pedal1.x - pedal2.x);
	ori_point2.y = 412-(pedal1.y - pedal2.y);

	
	CvPoint2D64f u_point[11] = {0};

	u_point[0].x = 256;
	u_point[0].y = 412;
	u_point[1].x = (256+2*pedal1.x)/3;
	u_point[1].y = (412+2*pedal1.y)/3;



	u_point[2]=pedal1;

	u_point[3].x=(endpoint.x+2*pedal1.x)/3;
	u_point[3].y=(endpoint.y+2*pedal1.y)/3;


	u_point[4].x=(2*endpoint.x+pedal1.x)/3;
	u_point[4].y=(2*endpoint.y+pedal1.y)/3;

	u_point[5]=endpoint;




	CvPoint2D64f a[5] = {0};
	CvPoint2D64f tempbezier[200] = {0};
	CvPoint2D64f temprightgps[5] = {0};
	CvPoint2D64f tempright[5] = {0};


	u_point[6].x=(2*endpoint.x+pedal2.x)/3;
	u_point[6].y=(2*endpoint.y+pedal2.y)/3;
	u_point[7].x=(endpoint.x+2*pedal2.x)/3;
	u_point[7].y=(endpoint.y+2*pedal2.y)/3;
	u_point[8]=pedal2;
	u_point[9].x = (ori_point2.x+2*pedal2.x)/3;
	u_point[9].y = (ori_point2.y+2*pedal2.y)/3;
	//u_point[10] = ori_point2;
	u_point[10].x = 3*(ori_point2.x-pedal2.x)+ori_point2.x;
	u_point[10].y = 3*(ori_point2.y-pedal2.y)+ori_point2.y;
	CvPoint2D64f umappoint[200] = {0};
	//Bezier(u_point,10,umappoint);
	int count = 0;
	while(1)
	{
	

		
		Bezier(u_point,10,umappoint);
		if(SearchUTurnObstacleMoveWay1(umappoint,vel_Map))
		{
			
			for(int i =6;i<11;i++)
			{
				u_point[i].x = u_point[i].x + 10;
			}
			count++;
			if(count>3)
				break;
		}
		else
			break;
	}
	
	for(int i = 1;i<200;i++)
	{
		MidGpsPoint[i] = m_GpsData.MaptoGPS(app->GPS_Point,app->GPS_Direction,umappoint[i]);
	}
	MidGpsPoint[0] = m_gps;
	//for(int i = 0;i<200;i++)
	//{
	//	out778<<" ditudian "<<umappoint[i].x<<", "<<umappoint[i].y<<" gps "<<MidGpsPoint[i].x<<", "<<MidGpsPoint[i].y<<endl;
	//}
	
	app->critical_planningroad.Lock();
	cvClearSeq( planning_road );
	//app->stop = false;
	for(int i = 0; i<200; i++)
	{
		cvSeqPush( planning_road, &MidGpsPoint[i] );
	}
	app->critical_planningroad.Unlock();
	SetEvent(app->m_hEvent);//路径更新后，发送消息
	int countdd = 0;
	double angle = m_GpsData.GetAngle(waypoint[3].x,waypoint[3].y,waypoint[2].x,waypoint[2].y);
	//double angle = m_GpsData.GetAngle(u_point[9].x,u_point[9].y,u_point[10].x,u_point[10].y);
	while(1)
    {   
		m_gps = app->GPS_Point;
		dir = app->GPS_Direction;

		if(abs(dir - angle) < 15 || abs(360 - abs(dir - angle)) < 15)
			break ;
		out778<<"dir "<<dir<<"  ANGLE   "<< angle;
		out778<<"  dir-angle "<<abs(dir - angle)<<endl;
	
		
		Sleep(100);

    }
		way_out = true;

}
void CUTurn::ReturnUTurnPathRNDF(int seq,CvPoint2D64f (&upathyc)[200],CvPoint2D64f (&upoint)[4],int lane_num)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->secondDecison = "UTurn";
	app->critical_section.Lock();
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;
	double dir = m_GpsData.GetAngle(upoint[1],upoint[0]);
	app->critical_section.Unlock();
	CvPoint2D64f waypoint[4] = {0};
	int xia_y = 0;
	CvPoint2D64f uturnpath[200];
	
	for(int i = 0;i<4;i++)
	{
		waypoint[i] = upoint[i];
		
	}

	CvPoint2D64f point[4] = {0};
	for(int i = 0;i<4;i++)
	{
		point[i] = m_GpsData.APiontConverD(m_gps,waypoint[i],dir);
	}
	CvPoint2D64f rndfpath[200],rndfpath1[200];
	for (int i=0;i<200;i++)
	{
		rndfpath1[i].y = 420-i;//rndfpath1[i].y = 500-2*i;
		rndfpath1[i].x = (point[0].x - point[1].x)*(rndfpath1[i].y - point[0].y)/(point[0].y - point[1].y)+point[0].x ;
		rndfpath[i] = m_GpsData.MaptoGPS(m_gps,dir,rndfpath1[i]);
	}
	MoveWay(m_gps,dir,rndfpath);
	CvPoint2D64f endpoint;                           //存储汽车地图中的中点
	
	double angleerr=m_GpsData.GetAngle(m_gps,upoint[0])-m_GpsData.GetAngle(upoint[1],upoint[0]);
	xdisini=(m_GpsData.GetDistance(m_gps.x,m_gps.y,upoint[0].x,upoint[0].y))*(sin(angleerr*PI/180));


	
	///////////////////////

///////////////////////
/****************************************************************************************
至少在路点前搜索到20个道路边沿点
*****************************************************************************************/
	
   //vel_Map = app->PercepMap; 
	app->critical_xinmap.Lock();
	CVehicle::vel_Map = getDyMap();
	app->critical_xinmap.Unlock();

	//vel_Map = getPerceptionMap();
	//for(int i = 0;i<512;i++)
	//{
	//	for(int j = 0;j<512;j++)
	//		//if(app->PercepMap->MapPoint[i][j] == 0)
	//			out778<<vel_Map->MapPoint[i][j];
	//		//else out333<<1;
	//	out778<<endl;
	//}
	int xia_x  = 256;
	double aim_dir = m_GpsData.GetAngle(waypoint[1],waypoint[0]);
	
	//YanChang(m_gps,dir,rndfpath1,aim_dir,rndfpath);
	endpoint = SearchHuokou(vel_Map,xia_y,xia_x,lane_num,rndfpath);//endpoint = SearchHuokou(vel_Map,xia_y,xia_x,lane_num,rndfpath);

	xia_y_pub=xia_y;

	out778<<" "<<endpoint.x<<","<<endpoint.y<<endl;
	int leftnum=0,rightnum=0;
	CvPoint2D64f rightmap[200]={0};
	CvPoint2D64f leftmap[200]={0};
	CvPoint2D64f leftrndf[200]={0};
	MoveWay(m_gps,dir,rndfpath);
	//CvPoint2D64f gps_endpoint = m_GpsData.MaptoGPS(m_gps,dir,endpoint);
	CvPoint2D64f gps_endpoint = m_GpsData.MaptoGPS(m_gps,m_gpsdir,endpoint);
	endpoint = m_GpsData.APiontConverD(m_gps,gps_endpoint,dir);
	out778<<"xiuzheng endpoint: "<<endpoint.x<<","<<endpoint.y<<endl;

	double dir_lane = m_GpsData.GetAngle(rndfpath[190],rndfpath[170]);
	double s_right = LevelDist(m_gps,dir_lane,gps_endpoint);
	double s_left = 2.5;//8//1
	MoveLeft(rndfpath,leftrndf,s_left+s_right);

	for (int i =0;i<200;i++)
	{
		CvPoint2D64f map = m_GpsData.APiontConverD(m_gps,rndfpath[i],dir);
		if(map.y>endpoint.y&&map.y<450)
		{
			rightmap[rightnum] = map;
			rightnum++;
		}
	}
	for (int i =0;i<200;i++)
	{
		CvPoint2D64f map = m_GpsData.APiontConverD(m_gps,leftrndf[i],dir);
		if(map.y>endpoint.y&&map.y<450)
		{
			leftmap[leftnum] = map;
			leftnum++;
		}
	}	
	CvPoint2D64f umappoint[200] = {0}; 
	CvPoint2D64f u_point[200] = {0};
	for (int i = 0;i<rightnum;i++)
	{
		u_point[i] = rightmap[i];
	}
	u_point[rightnum] = endpoint;
	for (int i = 0;i<leftnum;i++)
	{
		u_point[rightnum+1+i] = leftmap[leftnum-i-1];
	}
	CvPoint2D64f right_chuizu, left_chuizu,left_vehicle,right_vehicle;
	int x1 = u_point[0].x;
	int y1 = u_point[0].y;
	int x2 = u_point[rightnum -1].x;
	int y2 = u_point[rightnum -1].y;
	
	right_chuizu.y = endpoint.y;
	right_chuizu.x = ((x2-x1)/(y2-y1))*(right_chuizu.y-y1)+x1;


	right_vehicle.y = 412;
	right_vehicle.x = ((x2-x1)/(y2-y1))*(right_vehicle.y-y1)+x1;

	 x1 = u_point[rightnum+1].x;
	 y1 = u_point[rightnum+1].y;
	 x2 = u_point[leftnum+rightnum].x;
	 y2 = u_point[leftnum+rightnum].y;

	left_chuizu.y = endpoint.y;
	left_chuizu.x = ((x2-x1)/(y2-y1))*(left_chuizu.y-y1)+x1;


	left_vehicle.y = 412;
	left_vehicle.x = ((x2-x1)/(y2-y1))*(right_vehicle.y-y1)+x1;
	CvPoint2D64f uupoint[12];
	uupoint[0] = right_vehicle;
	uupoint[2] = right_chuizu;
	uupoint[1].x = (uupoint[0].x+uupoint[2].x)/2;
	uupoint[1].y = (uupoint[0].y+uupoint[2].y)/2;

	uupoint[3].x = (endpoint.x+2*right_chuizu.x)/3;
	uupoint[3].y = (endpoint.y+2*right_chuizu.y)/3;
	
	uupoint[4].x = (2*endpoint.x+right_chuizu.x)/3;
	uupoint[4].y = (2*endpoint.y+right_chuizu.y)/3;

	uupoint[5] = endpoint;
	
	uupoint[10] = left_vehicle;
	uupoint[8] = left_chuizu;
	uupoint[9].x = (uupoint[8].x+uupoint[10].x)/2;
	uupoint[9].y = (uupoint[8].y+uupoint[10].y)/2;

	uupoint[7].x = (endpoint.x+2*left_chuizu.x)/3;
	uupoint[7].y = (endpoint.y+2*left_chuizu.y)/3;

	uupoint[6].x = (2*endpoint.x+left_chuizu.x)/3;
	uupoint[6].y = (2*endpoint.y+left_chuizu.y)/3;
	double dist = sqrt(pow(uupoint[10].x-uupoint[9].x,2)+pow(uupoint[10].y- uupoint[9].y,2))/5;
	if(dist<50)
	{
		uupoint[11].x = ((dist-50)*uupoint[9].x + 50*uupoint[10].x)/dist;
		uupoint[11].y = ((dist-50)*uupoint[9].y + 50*uupoint[10].y)/dist;
	}
	else
	{
		/*
		uupoint[11].x = uupoint[10].x;
		uupoint[11].y = uupoint[10].y;
		*/

		uupoint[11].x = ((dist-(dist+1))*uupoint[9].x + (dist+1)*uupoint[10].x)/dist;
		uupoint[11].y = ((dist-(dist+1))*uupoint[9].y + (dist+1)*uupoint[10].y)/dist;
	}
	int count = 0;
	
	while(1)
	{
	

		
		Bezier(uupoint,11,umappoint);

		for (int i=0;i<200;i++)
		{
			umappoint[i] = m_GpsData.MaptoGPS(m_gps,dir,umappoint[i]);
			umappoint[i] = m_GpsData.APiontConverD(m_gps,umappoint[i],m_gpsdir);
		}

		if(SearchUTurnObstacleMoveWay1(umappoint,vel_Map))//if(SearchUTurnObstacleMoveWay1(umappoint,vel_Map))
		{
			
			for(int i = 6 ;i< 15;i++)//12
			{
				uupoint[i].x = uupoint[i].x - 2;
			}
			count++;
			if(count>6)
				break;
			/*
			if(app->bthirduturn)
			{
				if(count>6)
					break;
			}
			else
			{
				if(count>12)
					break;
			}
			*/
		}
		else
			break;
	}

	count = 0;

	/*
	if(app->bthirduturn)
	{
		while(1)
		{
			Bezier(uupoint,11,umappoint);

			for (int i=0;i<200;i++)
			{
				umappoint[i] = m_GpsData.MaptoGPS(m_gps,dir,umappoint[i]);
				umappoint[i] = m_GpsData.APiontConverD(m_gps,umappoint[i],m_gpsdir);
			}

			if(!(SearchUTurnObstacleMoveWay1_rightsidethird(umappoint,vel_Map)))
			{

				for(int i = 0 ;i< 5;i++)//6
				{
					uupoint[i].x = uupoint[i].x + 2;
				}
				count++;
				if(count>12)
					break;
			}
			else
				break;
		}
	}
	else
	{
		while(1)
		{
			Bezier(uupoint,11,umappoint);

			for (int i=0;i<200;i++)
			{
				umappoint[i] = m_GpsData.MaptoGPS(m_gps,dir,umappoint[i]);
				umappoint[i] = m_GpsData.APiontConverD(m_gps,umappoint[i],m_gpsdir);
			}

			if(!(SearchUTurnObstacleMoveWay1_rightside(umappoint,vel_Map)))
			{

				for(int i = 0 ;i< 5;i++)//6
				{
					uupoint[i].x = uupoint[i].x + 2;
				}
				count++;
				if(count>12)
					break;
			}
			else
				break;
		}
	}
	*/

	dist = sqrt(pow(uupoint[10].x-uupoint[9].x,2)+pow(uupoint[10].y- uupoint[9].y,2))/5;

	if(dist<50)
	{
		uupoint[11].x = ((dist-50)*uupoint[9].x + 50*uupoint[10].x)/dist;
		uupoint[11].y = ((dist-50)*uupoint[9].y + 50*uupoint[10].y)/dist;	
	}
	else
	{
		/*
		uupoint[11].x = uupoint[10].x;
		uupoint[11].y = uupoint[10].y;
		*/

		uupoint[11].x = ((dist-(dist+1))*uupoint[9].x + (dist+1)*uupoint[10].x)/dist;
		uupoint[11].y = ((dist-(dist+1))*uupoint[9].y + (dist+1)*uupoint[10].y)/dist;
	}

	Bezier(uupoint,11,umappoint);
	
	for(int i = 1;i<200;i++)
	{
		upathyc[i] = m_GpsData.MaptoGPS(m_gps,dir,umappoint[i]);
	}
	upathyc[0] = m_gps;
}
int CUTurn::YanChang(CvPoint2D64f m_gps, double m_dir,CvPoint2D64f *op, double dir,CvPoint2D64f *yanp)
{
	
	for (int i = 0;i<100;i++)
	{
		yanp[i]=op[2*i];
	}
	for(int i=100;i<200;i++)
	{
		yanp[i]=m_GpsData.MaptoGPS(op[199],dir,cvPoint2D64f(256,412-(i-100)*2));
	}
	/*for(int i = 0;i<200;i++)
	{
		yanp[i] = m_GpsData.APiontConverD(m_gps,yanp[i],m_dir);
	}*/
	return 1;
}
void CUTurn::ReturnUTurnPath1(CvSeq *gp,int seq,CvPoint2D64f (&upathyc)[200])
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->secondDecison = "UTurn";
	app->critical_section.Lock();
	CvPoint2D64f m_gps = app->GPS_Point;
	double dir = app->GPS_Direction;
	app->critical_section.Unlock();
	CvPoint2D64f waypoint[4] = {0};
	int xia_y = 0;
	CvPoint2D64f uturnpath[200];
	for(int i = 0;i<4;i++)
	{
		waypoint[i] = *(CvPoint2D64f*)cvGetSeqElem( gp, seq-2+i);


	}

	

	CvPoint2D64f endpoint;                           //存储汽车地图中的中点
	

	CvPoint2D64f point[4] = {0};
	for(int i = 0;i<4;i++)
	{
		point[i] = m_GpsData.APiontConverD(m_gps,waypoint[i],dir);
	}

	///////////////////////

///////////////////////
/****************************************************************************************
至少在路点前搜索到20个道路边沿点
*****************************************************************************************/
	
   

	vel_Map = getPerceptionMap();
	//for(int i = 0;i<512;i++)
	//{
	//	for(int j = 0;j<512;j++)
	//		//if(app->PercepMap->MapPoint[i][j] == 0)
	//			out778<<vel_Map->MapPoint[i][j];
	//		//else out333<<1;
	//	out778<<endl;
	//}
	int xia_x  = 256;
	//endpoint = SearchHuokou(vel_Map,xia_y,xia_x,);



	 CvPoint2D64f line[512];
	 int l_num=0;
	 int youbianjie = 150;
	 int zuobianjie = 332;
	 for (int i=xia_y + 15;i<412;i++)
	 {
		 for (int j=xia_x + 15;j>0;j--)
		 {
			 if (vel_Map->MapPoint[i][j]==8&&j>youbianjie&&j<zuobianjie)
			 {
				 line[l_num].y=i;
				 line[l_num].x=j;
				 youbianjie = j-20;
				 zuobianjie= j+20;
				 l_num++;	
				 break;
			 }

		 }

	 }
	 
	 double k = 9999999;
	 int count_kk = 0;
	 if(endpoint.x != 248 && endpoint.y !=350&&xia_y != 380)
	 k =  Countk(line,l_num);
	 for (int i = 0;i<l_num-1;i++)
	 {
		 if (line[i].x == line[i+1].x)
		 {
			 count_kk++;
		 }
	 }
	 if (count_kk > l_num - 3)
	 {
		k = 99999999;
	 }
	
	 //搜索豁口下端点


	
	
	CvPoint2D64f pedal1,pedal2;
	CvPoint2D64f ori_point1 = cvPoint2D64f(256,412);
	CvPoint2D64f ori_point2;
	//CvPoint2D64f ori_point2 = point[2];
	
	pedal1 = CountChuiZu(ori_point1,endpoint,k);
	double s = sqrt(pow((endpoint.x - pedal1.x),2)+pow((endpoint.y - pedal1.y),2))*0.2;
	pedal2.x = endpoint.x - (7/s)*(pedal1.x - endpoint.x);
	pedal2.y = endpoint.y - (7/s)*(pedal1.y - endpoint.y);
	//pedal2 = CountChuiZu(ori_point2,endpoint,k);

	ori_point2.x = 256-(pedal1.x - pedal2.x);
	ori_point2.y = 412-(pedal1.y - pedal2.y);

	
	CvPoint2D64f u_point[11] = {0};

	u_point[0].x = 256;
	u_point[0].y = 412;
	u_point[1].x = (256+2*pedal1.x)/3;
	u_point[1].y = (412+2*pedal1.y)/3;



	u_point[2]=pedal1;

	u_point[3].x=(endpoint.x+2*pedal1.x)/3;
	u_point[3].y=(endpoint.y+2*pedal1.y)/3;


	u_point[4].x=(2*endpoint.x+pedal1.x)/3;
	u_point[4].y=(2*endpoint.y+pedal1.y)/3;

	u_point[5]=endpoint;




	CvPoint2D64f a[5] = {0};
	CvPoint2D64f tempbezier[200] = {0};
	CvPoint2D64f temprightgps[5] = {0};
	CvPoint2D64f tempright[5] = {0};


	u_point[6].x=(2*endpoint.x+pedal2.x)/3;
	u_point[6].y=(2*endpoint.y+pedal2.y)/3;
	u_point[7].x=(endpoint.x+2*pedal2.x)/3;
	u_point[7].y=(endpoint.y+2*pedal2.y)/3;
	u_point[8]=pedal2;
	u_point[9].x = (ori_point2.x+2*pedal2.x)/3;
	u_point[9].y = (ori_point2.y+2*pedal2.y)/3;
	u_point[10] = ori_point2;

	CvPoint2D64f umappoint[200] = {0};
	int count = 0;
	while(1)
	{
	

		
		Bezier(u_point,10,umappoint);
		if(SearchUTurnObstacleMoveWay1(umappoint,vel_Map))
		{
			
			for(int i =6;i<11;i++)
			{
				u_point[i].x = u_point[i].x - 10;
			}
			count++;
			if(count>3)
				break;
		}
		else
			break;
	}
	
	for(int i = 1;i<200;i++)
	{
		uturnpath[i] = m_GpsData.MaptoGPS(m_gps,dir,umappoint[i]);
	}
	uturnpath[0] = m_gps;

	//////////4.24
	CvPoint2D64f aaa,bbb;
	aaa = m_GpsData.MaptoGPS(app->GPS_Point,app->GPS_Direction,u_point[9]);
	bbb = m_GpsData.MaptoGPS(app->GPS_Point,app->GPS_Direction,u_point[10]);
	double angel = m_GpsData.GetAngle(bbb,aaa);

	for (int i = 0;i<100;i++)
	{
		upathyc[i]=uturnpath[2*i];
	}
	for(int i=100;i<200;i++)
	{
		upathyc[i]=m_GpsData.MaptoGPS(uturnpath[199],angel,cvPoint2D64f(256,412-(i-100)*2));
	}
	//////////////4.24

	////for(int i = 0;i<200;i++)
	////{
	////	out778<<" ditudian "<<umappoint[i].x<<", "<<umappoint[i].y<<" gps "<<MidGpsPoint[i].x<<", "<<MidGpsPoint[i].y<<endl;
	////}
	//
	//app->critical_planningroad.Lock();
	//cvClearSeq( planning_road );
	////app->stop = false;
	//for(int i = 0; i<200; i++)
	//{
	//	cvSeqPush( planning_road, &MidGpsPoint[i] );
	//}
	//app->critical_planningroad.Unlock();
	//SetEvent(app->m_hEvent);//路径更新后，发送消息
	//int countdd = 0;
	//double angle = m_GpsData.GetAngle(u_point[9].x,u_point[9].y,u_point[10].x,u_point[10].y);
	//while(1)
 //   {   
	//	m_gps = app->GPS_Point;
	//	dir = app->GPS_Direction;

	//	if(abs(dir - angle) < 15 || abs(360 - abs(dir - angle)) < 15)
	//		break ;
	//	out778<<"dir-angle "<<abs(dir - angle)<<endl;
	//
	//	
	//	Sleep(100);

 //   }
	//	way_out = true;


	//
	//
	//

	////GetMidPoint();
	////GetPath();
}