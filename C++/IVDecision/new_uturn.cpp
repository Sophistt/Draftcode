#include "StdAfx.h"
#include "new_uturn.h"
#include<fstream>
using namespace std;

ofstream out778("new_uturn");

new_uturn::new_uturn(void)
{
}

new_uturn::~new_uturn(void)
{
}

void new_uturn::UturnPath()
{
	CIVDecisionApp *app=(CIVDecisionApp*)AfxGetApp();
	app->secondDecison="UTurn";
	app->critical_section.Lock();
	CvPoint2D64f m_gps=app->GPS_Point;
	double dir=app->GPS_Direction;
	app->critical_section.Unlock();

	CvPoint2D64f waypoint[4]={0};
	for (int i=0; i<4; i++)
	{
		waypoint[i].x=LeadPoint[seq_num+i-2].lat;
		waypoint[i].y=LeadPoint[seq_num+i-2].lng;
	}

	
	CvPoint2D64f point[4]={0};
	for (int i=0; i<4; i++)
	{
		point[i]=m_GpsData.APiontConverD(m_gps,waypoint[i],dir);
	}

	CvPoint2D64f line[512];
	int l_num=0;
	double k_line;
	double b_line;
	
	CvPoint2D64f up_point;
	
	CvPoint2D64f down_point;
	CvPoint2D64f MidPoint;
    CvPoint2D64f Pedal;
	int jumppoint;

	CvPoint2D64f ori_point;
	ori_point.x=256;
	ori_point.y=412;
	
	                           //从第二个路点前方20米处向车辆所在位置搜索边界，存储在line中
	
	SearchLine(line,l_num);   
	
    
	for (int i=0; i<l_num; i++)
	{
		if (line[i].y-line[i+1].y>30&&line[i].x-line[i+1].x<20)
		{
			down_point=line[i];
			up_point=line[i+1];
			up_flag=1;
			down_flag=1;
			break;
		}
		
	}

	for (int i=1; i<l_num; i++)
	{
		if (up_flag==0&&down_flag==0&&line[0].y<380)
		{
			up_flag=1;
		}

		if (up_flag==0&&down_flag==0&&line[l_num].y<380)
		{
			down_flag=1;
		}
	}

	CvPoint2D64f line2[200];
	int l2_num=0;
	int youbianjie=150;


	if (up_flag==1&&down_flag==1)                               //标准uturn
	{
		MidPoint.y=(down_point.y+up_point.y)/2-20;
		MidPoint.x=(down_point.x+up_point.x)/2;

		for (int i=down_point.y+15; i<412; i++)
		{
			for (int j=256; j>0; j--)
			{
				if (vel_Map->MapPoint[i][j]==18&&j>youbianjie)
				{
					line2[l2_num].x=j;
					line2[l2_num].y=i;
					youbianjie=j-20;
					l2_num++;
					break;
				}
			}
		}
		 CountLine(line2,l2_num,k_line,b_line);
		 Pedal=CountPedal(ori_point,MidPoint,k_line);
	}


	 
	if (up_flag==1&&down_flag==0)                                        //只有上豁口
	{
		
		for (int i=up_point.y-15; i>0; i--)
		{
			for (int j=256; j>0; j--)
			{
				if (vel_Map->MapPoint[i][j]==18&&j>youbianjie)
				{
					line2[l2_num].x=j;
					line2[l2_num].y=i;
					youbianjie=j-20;
					l2_num++;
					break;
				}
			}
		}
     CountLine(line2,l2_num,k_line,b_line);
	 MidPoint.y=up_point.y+20;
	 MidPoint.x=(MidPoint.y-up_point.y+k_line*up_point.x)/k_line;
	  Pedal=CountPedal(ori_point,MidPoint,k_line);
	}

	if (up_flag==0&&down_flag==1)                                           //只有下豁口
	{
		for (int i=down_point.y+15; i<412; i++)
		{
			for (int j=256; j>0; j--)
			{
				if (vel_Map->MapPoint[i][j]==18&&j>youbianjie)
				{
					line2[l2_num].x=j;
					line2[l2_num].y=i;
					youbianjie=j-20;
					l2_num++;
					break;
				}
			}
		}

		CountLine(line2,l2_num,k_line,b_line);
		MidPoint.y=down_point.y-30;
		MidPoint.x=(MidPoint.y-down_point.y+k_line*down_point.x)/k_line;
		Pedal=CountPedal(ori_point,MidPoint,k_line);

	}

	

   if (up_flag==0&&down_flag==0)                                     //无隔离栏uturn掉头
   {
	   int daoluyoubianyan=350;
	   CvPoint2D64f line3[512];
	   int l3_num=0;
	   for (int i=412; i>0; i--)
	   {
		   for (int j=256; j<512; j++)
		   {
			   if (vel_Map->MapPoint[i][j]==18&&j<daoluyoubianyan)
			   {
				   line3[l3_num].x=j;
				   line3[l3_num].y=i;
				   l3_num++;
				   daoluyoubianyan=j+20;
				   break;
			   }
		   }
	   }

	   CvPoint2D64f you;
	   int sumx=0;
	   int sumy=0;
	   for (int i=0;i<l_num;i++)
	   {
		   sumx+=line3[i].x;
		   sumy+=line3[i].y;
	   }
	   you.x=sumx/l3_num;
	   you.y=sumy/l3_num;
	   
	   int b2=you.y-k_line*you.x;
	   //int roadwidth=0;
	   //roadwidth=abs(b_line-b2)/sqrt(k_line*k_line+1);

	   MidPoint.y=380;
	   MidPoint.x=(MidPoint.y-(b_line+b2)/2)/k_line;
	   Pedal=CountPedal(ori_point,MidPoint,k_line);



	   //MidPoint.x=245;
	   //MidPoint.y=300;
	   //Pedal.x=254;
	   //Pedal.y=301;

	   //point[1].y=412;
	   //point[1].x=256;

	   //point[2].x=241;
	   //point[2].y=412;
   }

 

  
   CvPoint2D64f upoint[11];
   ChaZhi(Pedal,point[1],point[2],MidPoint,upoint);


   CvPoint2D64f pathpoint[200];

   if (up_flag!=0||down_flag!=0)
   {
	   int move=0;
	   while(1)
	   {
		   Bezier(upoint,10,pathpoint);
		   if (MovePoint(pathpoint,vel_Map))
		   {
			   for (int i=6; i<11; i++)
			   {
				   upoint[i].x=upoint[i].x-10;
			   }
			   move++;
			   if (move>3)
			   {
				   break;
			   }
		   }

		   else 
			   break;
	   }
   }
   

  for (int i=0; i<200; i++)
  {
	  MidGpsPoint[i]=m_GpsData.MaptoGPS(m_gps,dir,pathpoint[i]);
  }

  MidGpsPoint[0]=m_gps;
  

  cvClearSeq(planning_road);

  for (int i=0; i<200; i++)
  {
	  cvSeqPush(planning_road,&MidGpsPoint[i]);
  }

  SetEvent(app->m_hEvent);             //路径更新后发送消息
  int countdd=0;
  double angle=m_GpsData.GetAngle(waypoint[3].x,waypoint[3].y,waypoint[2].x,waypoint[2].y);
  while(1)
  {
	  m_gps=app->GPS_Point;
	  dir=app->GPS_Direction;

	  if (abs(dir-angle)<20||abs(360-abs(dir-angle))<20)
	  {
		  break;
	  }

	  Sleep(100);
  }
   
    way_out=true;
}

void new_uturn::SearchLine(CvPoint2D64f (&line)[512],int &l_num)
{
	CIVDecisionApp *app=(CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
	CvPoint2D64f m_gps=app->GPS_Point;
	double dir=app->GPS_Direction;
	app->critical_section.Unlock();
	vel_Map=getPerceptionMap();

	int youbianjie=150;

	for (int i=412; i>0; i--)
	{
		for (int j=256; j>100; j--)
		{
			if (vel_Map->MapPoint[i][j]==18&&j>youbianjie)
			{
				line[l_num].x=j;
				line[l_num].y=i;
				youbianjie=j-40;                           //隔离栏右边界阈值向左偏移4米，可以帮助判定所得到的边界点不是逆向行驶路面的边界
				l_num++;
				break;
			}
		}
	}

}

void new_uturn::CountLine(CvPoint2D64f *line,int l_num,double &k,double &b)
{
	double sumx=0;
	double sumy=0;
	double xmean=0;
	double ymean=0;
	double sumxx=0;
	double sumxy=0;

	for (int i=0; i<l_num; i++)
	{
		sumx+=line[i].y;
		sumy+=line[i].x;
	}
	xmean=sumx/l_num;
	ymean=sumy/l_num;

	for (int i=0; i<l_num; i++)
	{
		sumxx+=(line[i].y-xmean)*(line[i].y-xmean);
		sumxy+=(line[i].y-xmean)*(line[i].x-ymean);
	}

	k=sumxx/sumxy;
	b=ymean-k*xmean;
}

void new_uturn::SearchMidPoint(I_Map *map,CvPoint2D64f up_point,CvPoint2D64f down_point,bool up_geli,bool down_geli,double k,CvPoint2D64f &MidPoint)
{
	if (up_geli==1&&down_geli==1)                                    //标准uturn
	{
		MidPoint.x=(up_point.x+down_point.x)/2;
		MidPoint.y=(up_point.y+down_point.y)/2-20;
	}

	if (up_geli==0&&down_geli==1)                                      //只有下半部分隔离栏的uturn
	{
		MidPoint.y=down_point.y+25;
		MidPoint.x=(MidPoint.y-down_point.y+k*down_point.x)/k;
	}

	if (up_geli==1&&down_geli==0)                                      //只有上半部分隔离栏的uturn
	{
		MidPoint.y=up_point.y-10;
        MidPoint.x=(MidPoint.y-up_point.y+k*up_point.x)/k;
	}

}

CvPoint2D64f new_uturn::CountPedal(CvPoint2D64f ori,CvPoint2D64f MidPoint,double k)
{
	CvPoint2D64f countp;
	countp.y=352;
	countp.x=(countp.y-ori.y+ori.x*k)/k;

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

void new_uturn::ChaZhi(CvPoint2D64f pedal,CvPoint2D64f pointt1,CvPoint2D64f point2,CvPoint2D64f MidPoint,CvPoint2D64f (&upoint)[11])
{
	if (up_flag==1&&down_flag==1)
	{
		CvPoint2D64f pedal1,pedal2;
	CvPoint2D64f ori_point1=cvPoint2D64f(256,412);


	pedal1=pedal;
	double s=sqrt(pow(MidPoint.x-pedal1.x,2)+pow(MidPoint.y-pedal1.y,2))*0.2;
	pedal2.x=MidPoint.x-(5/s)*(pedal1.x-MidPoint.x);
	pedal2.y=MidPoint.y-(5/s)*(pedal1.y-MidPoint.y);
	point2.x=256-(pedal1.x-pedal2.x);
	point2.y=412-(pedal1.y-pedal2.y);

	upoint[0].x=256;
	upoint[0].y=412;
	upoint[1].x=(256+pedal1.x*2)/3;
	upoint[1].y=(412+pedal1.y*2)/3;
	upoint[2]=pedal1;
	upoint[3].x=(MidPoint.x+2*pedal1.x)/3;
	upoint[3].y=(MidPoint.y+2*pedal1.y)/3;
	upoint[4].x=(MidPoint.x*2+pedal1.x)/3;
	upoint[4].y=(MidPoint.y*2+pedal1.y)/3;
	upoint[5]=MidPoint;
	upoint[6].x=(MidPoint.x*2+pedal2.x)/3;
	upoint[6].y=(MidPoint.y*2+pedal2.y)/3;
	upoint[7].x=(MidPoint.x+2*pedal2.x)/3;
	upoint[7].y=(MidPoint.y+2*pedal2.y)/3;
	upoint[8]=pedal2;
	upoint[9].x=(pedal2.x*2+point2.x)/3;
	upoint[9].y=(pedal2.y*2+point2.y)/3;
	upoint[10]=point2;
	}
	
}

bool new_uturn::MovePoint(CvPoint2D64f *pathpoint,I_Map *map)
{
	CIVDecisionApp *app=(CIVDecisionApp *)AfxGetApp();

	int x=0;
	int y=0;

	for (int i=100; i<200; i++)
	{
		if (pathpoint[i].x<0||pathpoint[i].x>511||pathpoint[i].y>511||pathpoint[i].y<0)
		{
			continue;
		}

		x=pathpoint[i].x;
		y=pathpoint[i].y;
		for (int m=-5; m<5; m++)
		{
			for (int n=-20; n<20; n++)
			{
				if (y+m>380&&y+m<430)
				{
					continue;
				}

				if (y+m>511||x+n>511||x+n<0)
				{
					continue;
				}

				if (map->MapPoint[y+m][x+n]==18)
				{
					return true;
				}
			}
		}
	}

	return false;
}