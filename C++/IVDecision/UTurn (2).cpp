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
void CUTurn::ReturnUTurnPath()
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->secondDecison = "UTurn";
	app->critical_section.Lock();
	CvPoint2D64f m_gps = app->GPS_Point;
	double dir = app->GPS_Direction;
	app->critical_section.Unlock();
	CvPoint2D64f waypoint[4] = {0};
	for(int i = 0;i<4;i++)
	{
		waypoint[i].x = LeadPoint[seq_num +i - 3].lat;
		waypoint[i].y = LeadPoint[seq_num +i - 3].lng;
	}
	vel_Map = getPerceptionMap();
	

	const int width=20;

	CvPoint2D64f endpoint;                           //存储汽车地图中的中点
	int x_free[256]={0};
	int y_free[412]={0};
	int x_block[200]={0};
	int y_block[200]={0};
	int x_down=0;
	int y_down=0;
	int x_up=0;
	int y_up=0;

	int obstacle=0;                                  //连续障碍物点个数
	int non_ob1=0;                                    
	int non_ob2=0;                                  //存储豁口长度
	bool obstacle_flag=0;


	CvPoint2D64f point[4] = {0};
	for(int i = 0;i<4;i++)
	{
		point[i] = m_GpsData.APiontConverD(m_gps,waypoint[i],dir);
	}



/****************************************************************************************
至少在路点前搜索到20个道路边沿点
*****************************************************************************************/
	


	


		 for (int i=412; i>0; i--)                       //搜索豁口,得到豁口大小
		{
			non_ob2=non_ob1;
							 //将得到的豁口大小传到non_ob2中
			for (int j=256; j>100; j--)
			{
				if(vel_Map->MapPoint[i][j]==18&&j>waypoint[2].x) //当遇到道路边沿时
				{	
					non_ob1=0;					
					break;
				}	
			
				y_up=i;
				x_up=j;
			}

			

			if (non_ob2>=width&&non_ob1==0)               //当豁口大小大于阈值且搜索到的点是道路边沿时跳出
			{
				
				break;

			}
			if (non_ob2>=40&&non_ob1==1)                 //如果豁口大于8米则，跳出
			{
				y_up=i;
				break;
			}
			y_free[non_ob1]=i;                           //记录道路豁口点的纵坐标值
			non_ob1++;
		}
	


		 CvPoint2D64f line[30];
		 int l_num=0;
		 for (int i=10;i<40;i++)
		 {
			 for (int j=256;j>0;j--)
			 {
				 if (vel_Map->MapPoint[int(y_free[0]+i)][j]==18&&j>waypoint[2].x)
				 {
					 line[l_num].x=waypoint[0].y+i;
					 line[l_num].y=j;
					 l_num++;					
				 }

			 }

		 }


		 double road_k=0;
		 double	road_b=0;
		 int num_k;
		 double sum_x,sum_y,sum_xy,sum_x2;
		 sum_x=0;
		 sum_x2=0;
         sum_y=0;
		 sum_xy=0;
		 sum_x2=0;

		 for(num_k=0; num_k<l_num; num_k++)
		 {
			 sum_x+=line[num_k].x;
			 sum_x2+=(line[num_k].x*line[num_k].x);
		 }

		
		 for(num_k=0; num_k<l_num; num_k++)
		 {
			 sum_y+=line[num_k].y;
		 }

		
		 for(num_k=0; num_k<l_num; num_k++)
		 {
			 sum_xy+=(line[num_k].x*line[num_k].y);
		 }

   
		 road_k=((l_num*sum_xy-sum_x*sum_y)/(l_num*sum_x2-sum_x*sum_x));
		 road_b=((sum_x2*sum_y-sum_x*sum_xy)/(l_num*sum_x2-sum_x*sum_x));

	 //搜索豁口下端点
	
		for (int j=256; j>0; j--)
		{
            int i=y_free[0]+1;
			if (vel_Map->MapPoint[i][j]==18&&j>waypoint[2].x)
			{
				x_down=j;
				y_down=i;				
			}
		}

	
	if (non_ob2=40&&road_k<0)
	{
		x_up=x_down-40*cos(abs(atan(road_k)));
		y_up=y_down-40*sin(abs(atan(road_k)));		
	}

	if (non_ob2>40&&road_k>0)
	{
		x_up=x_down+40*cos(atan(road_k));
		y_up=y_down+40*sin(atan(road_k));
	}

	/*****************************************
	              计算中点
	******************************************/
	endpoint.x=(x_up+x_down)/2;
	endpoint.y=(y_up+y_down)/2-20;




     
     /***************************
      计算目标道路上道路边沿方程
	 ****************************/
	 double h=y_down-(road_k*x_down+road_b);
	 double k2=road_k;
	 double b2=road_b+2*h;
	


CvPoint2D64f MidPoint=m_GpsData.MaptoGPS(m_gps,dir,endpoint);




CvPoint2D64f u_point[10];

	CvPoint2D64f pedal1;
	CvPoint2D64f pedal2;
	int x0=0;
	int y0=0;

	double x1=waypoint[0].x;
	double y1=waypoint[0].y;
	double x2=waypoint[1].x;
	double y2=waypoint[1].y;

	double A=y2-y1;
	double B=-(x2-x1);
	double C=x2*y1-x1*y2;

	pedal1.x=(B*B*MidPoint.x-A*B*MidPoint.y-A*C)/(A*A+B*B);
	pedal1.y=(-A*B*MidPoint.x+A*A*MidPoint.y-B*C)/(A*A+B*B);

	x1=waypoint[2].x;
	y1=waypoint[2].y;
	x2=waypoint[3].x;
	y2=waypoint[3].y;

	A=y2-y1;
	B=-(x2-x1);
	C=x2*y1-x1*y2;

	pedal2.x=(B*B*MidPoint.x-A*B*MidPoint.y-A*C)/(A*A+B*B);
	pedal2.y=(-A*B*MidPoint.x+A*A*MidPoint.y-B*C)/(A*A+B*B);

	u_point[0].x=2*waypoint[1].x-pedal1.x;
	u_point[0].y=2*waypoint[1].y-pedal1.y;
	u_point[1]=waypoint[1];
	u_point[2]=pedal1;
	u_point[3].x=(MidPoint.x+2*pedal1.x)/3;
	u_point[3].y=(MidPoint.y+2*pedal1.y)/3;
	u_point[4].x=(2*MidPoint.x+pedal1.x)/3;
	u_point[4].y=(2*MidPoint.y+pedal1.y)/3;
	u_point[5]=MidPoint;
	u_point[6].x=(MidPoint.x+2*pedal2.x)/3;
	u_point[6].y=(MidPoint.y+2*pedal2.y)/3;
	u_point[7].x=(2*MidPoint.x+pedal2.x)/3;
	u_point[7].y=(2*MidPoint.y+pedal2.y)/3;
	u_point[8]=pedal2;
	u_point[9]=waypoint[2];
	u_point[10].x=2*waypoint[3].x-pedal2.x;
	u_point[10].y=2*waypoint[3].y-pedal2.y;

	Bezier(u_point, 10, MidGpsPoint);
    


	/*app->critical_section.Lock();
	CvPoint2D64f m_gps = app->GPS_Point;
	double dir = app->GPS_Direction;*/
	//app->critical_section.Unlock();
	//MoveWay(m_gps,dir,MidGpsPoint);
	MoveWay(m_gps,dir,waypoint,4,MidGpsPoint);

	cvClearSeq( planning_road );
	//app->stop = false;
	for(int i = 0; i<200; i++)
	{
		cvSeqPush( planning_road, &MidGpsPoint[i] );
	}

	SetEvent(app->m_hEvent);//路径更新后，发送消息
	int countdd = 0;
	double angle = m_GpsData.GetAngle(waypoint[3].x,waypoint[3].y,waypoint[2].x,waypoint[2].y);
	while(1)
    {   
		m_gps = app->GPS_Point;
		dir = app->GPS_Direction;
		//double dd = m_GpsData.GetDistance(m_gps.x,m_gps.y,WPoint[4].x,WPoint[4].y);
		if(abs(dir - angle) < 20 || abs(360 - abs(dir - angle)) < 20)
			break ;
		out778<<"dir-angle "<<abs(dir - angle)<<endl;
		/*if(dd < 5)
			break;*/
		if(countdd > 600)
		{
			break;
		}
		Sleep(100);
		countdd++;
    }
		way_out = true;
	cvClearSeq( planning_road );

	for(int i = 0; i<200; i++)
	{
		cvSeqPush( planning_road, &app->GPS_Point);
	}

	SetEvent(app->m_hEvent);//路径更新后，发送消息
	for(int i = 0;i<10;i++)
	{
		uturn_state.send_message("0");
		Sleep(10);
	}
	Sleep(3000);
}




