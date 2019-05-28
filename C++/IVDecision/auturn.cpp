#include "auturn.h"
#include "StdAfx.h"
#include "fstream.h"
using namespace  std;

auturn::auturn(void)
{
}

auturn::~auturn(void)
{
}
void auturn::ReturnUTurnPath()
{
	CIVDecisionApp *app=(CIVDecisionApp*)AfxGetApp();
	app->secondDecision="UTurn";
	app->critical_section.Lock();
	CvPoint2D64f m_gps=app->GPS_Point;
	double dir=app->GPS_Direction;
	app->critical_section.Unlock();
	CvPoint2D64f waypoint[4]={0};
	int xia_y=0;
    for (int i=0; i<4; i++)                                 //将路点复制存储
    {
		waypoint[i].x=LeadPoint[seq_num+i-3].lat;
		waypoint[i].y=LeadPoint[seq_num+i-3].lng;
    }

	CvPoint2D64f point[4]={0}；
    
	for (int i=0; i<4; i++)
	{
		point[i]=m_GpsData.APiontConverD(m_gps,waypoint[i],dir);
	}

	vel_Map=getPerceptionMap();
    
	CvPoint2D64f endpoint=SearchHuokou(vel_Map,xia_y);
	CvPoint2D64f line[512];
	int l_num=0;
	int youbianjie=150;
	for (int xia_y+15; i<412; i++)
	{
		for (int j=256; j>0; j--)
		{
			if (vel_Map->MapPoint[i][j]==18&&j>youbianjie)
			{
				line[l_num].y=i;
				line[l_num].x=j;
				youbianjie=j-20;
				l_num++;
				break;
			}
		}
	}

	double k=9999999;
	if (endpoint.x!=248&&endpoint.y!=350&&xia_y!=380)
	{
		k=Countk(line,l_num);
	}


	CvPoint2D64f pedal1,pedal2;
	CvPoint2D64f ori_point1=cvPoint2D54f(256,412)；
	CvPoint2D64f ori_point2;

	pedal1=CountChuiZu(ori_point1,endpoint,k);

	double s=sqrt(pow((endpoint.x-pedal1.x),2)+pow((endpoint.y-pedal1.y),2))*0.2;
	pedal2.x=endpoint.x-(5/s)*(pedal1.x-endpoint.x);                               //为什么除以s？5/s是一个比例，5是可以调节的。
	pedal2.y=endpoint.y-(5/s)*(pedal1.y-endpoint.y);

	ori_point2.x=256-(pedal1.x-pedal2.x);
	ori_point2.y=412-(pedal1.y-pedal2.y);

	CvPoint2D64f u_point[11]={0};
	u_point[0].x=256;
	u_point[0].y=412;
	u_point[1].x=(256+2*pedal1.x)/3;
	u_point[1].y=(412+2*pedal1.y)/3;


	u_point[2]=pedal1;

	u_point[3].x=(endpoint.x+2*pedal1.x)/3;
	u_point[3].y=(endpoint.y+2*pedal1.y)/3;

	u_point[4].x=(2*endpoint.x+pedal1.x)/3;
	u_point[4].y=(2*endpoint.x+pedal1.y)/3;

	u_point[5]=endpoint;

	u_point[6].x=(2*endpoint.x+pedal2.x)/3;
	u_point[6].y=(2*endpoint.y+pedal2.y)/3;

	u_point[7].x=(endpoint.x+pedal2.y*2)/3;
	u_point[7].y=(endpoint.y+pedal2.y*2)/3;

	u_point[8]=pedal2;

	u_point[9].x=(pedal2.x*2+ori_point2.x)/3;
	u_point[9].y=(pedal2.y*2+ori_point2.y)/3;

	u_point[10]=ori_point2;

	CvPoint2D64f umappoint[200]={0};
	int count=0;
	while (1)
	{
		Bezier(u_point,10,umappoint);
		if (SearchUTurnObstacleMoveWay1(umappoint,vel_Map))
		{
			for (int i=6; i<11; i++)
			{
				u_point[i].x=u_point[i].x-10;
			}

			count++;
			if (count>3)
			{
				break;
			}
		}
		else
			break;

	}

	for (int i; i<200; i++)
	{
		MidGpsPoint[i]=m_GpsData.MaptoGPS(m_gps,dir,umappoint);

	}
	MidGpsPoint[0]=m_gps;


	cvClearSeq(planning_road);

	for (int i=0; i<200; i++)
	{
		cvSeqPush(planning_road,&MidGpsPoint[i]);
	}

	SetEvent(app->m_hEvent);    //路径更新后发送信息

	int countdd=0;

	double angle=m_GpsData.GetAngle(waypoint[3].y,waypoint[2].x,waypoint[2].y);

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

bool auturn::SearchUturnObstacleMoveWay1(CvPoint Rndf_MapPoint[],I_Map *map)
{
	CIVDecisionApp *app=(CIVDecisionApp *)AfxGetApp();


	int x=0;
	int y=0;

	for (int i=100; i<200; i++)
	{
		if (Rndf_MapPoint[i].x<0||Rndf_MapPoint[i].x>511||Rndf_MapPoint[i].y>511||Rndf_MapPoint[i].y<0)
		
			continue;

			x=Rndf_MapPoint[i].x;
			y=Rndf_MapPoint[i].y;

			for (int m=-5; m<5; m++)
			{
				for (int n=-20; n<20; n++)
				{
					if (y+m>380&&y+m<430)
					{
						continue;
					}
					if (y+m>511||y+m<0||x+n>511||x+n<0)
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

CvPoint2D64f auturn::CountChuiZu(CvPoint2D64f ori,CvPoint2D64f MidPoint,double k)
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

double auturn::Countk(CvPoint2D64f *line,int l_num)
{
	double k=0;
	double sumx=0;
	double sumy=0;
	double xmean=0,ymean=0;
	double sumxx=0,sumyy=0;
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

	return sumxx/sumxy;
}
CvPoint2D64f auturn::SeachHuokou(I_Map *map,int&xia_y)
{
	int youbianjie=100;
	CvPoint2D64f ob_temp[512]={0};
	int num_temp=0;
	CvPoint2D64f endpoint;
	int i=0;
	int j=0;
	for (i=412; i>0; i--)
	{
		for (j=256; j>100; j--)
		{
			if (vel_Map->MapPoint[i][j]==18&&j>youbianjie)
			{
				ob_temp[num_temp].x=j;
				ob_temp[num_temp].y=i;
				youbianjie=j-40;                      //右边界是搜到的障碍物向左移动8米后得到的                  
				num_temp++;
				break;
			}
		}

	}

	if (num_temp<3)                                     //障碍物小于三个
	{
		endpoint=cvPoint2D64f(248,350);
		xia_y=380;
		return endpoint;
	}

	for (int i=0; i<num_temp; i++)
	{
		if (ob_temp[i].y-ob_temp[i+1].y>20)                 //大于六米
		{
			endpoint.x=(ob_temp[i].x+ob_temp[i+1].x)/2;
			endpoint.y=(ob_temp[i].y+ob_temp[i+1].y)/2-20;  //为了更符合实际
			xia_y=ob_temp[i].y;
			return endpoint;
		}
	}



}

