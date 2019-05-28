#include "U_Turn.h"

#include "dmctype.h"

U_Turn::U_Turn(void)
{
	

}

U_Turn::~U_Turn(void)
{
	
}

void U_Turn::SetVehicle( CvPoint2D64f pos, double dir, I_Map map, LEAD *leadpoint, CvPoint2D64f waypoint[])   //初始化
{
	m_pos=pos;
	m_dir=dir;
	m_map=map;
	m_leadpoint=leadpoint;
	for (int i=0; i<4; i++)
	{
         m_waypoint[i]=waypoint[i];
	}

	path[200]={0};
}

void U_Turn::Bezier(CvPoint2Df p[], int n, CvPoint2D64f (&pathpoint)[200])
{
	CvPoint2D64f *pc=new CvPoint2D64f[n+1];
	int i;
	int r;
	float u;
	int count=0;
    //u的步长决定了曲线点的精度
	for (int k=0; k<200; k++)
	{
		u=k/199.0;
		for (i=0; i<=n; i++)
		{
			pc[i]=p[i];
		}

		for (r=1; r<=n; r++)
		{
			for (i=0; i<=n-r; i++)
			{
				pc[i].x=(1-u)*pc[i].x+u*pc[i+1].x;
				pc[i].y=(1-u)*pc[i].y+u*pc[i+1].y;

			}
		}

		pathpoint[k]=pc[0];
	}

	delete[] pc;
}

CvPoint2D64f U_Turn::MaptoGPS(CvPoint2D64f v,double dir,CvPoint2D64f a)
{

	double xq1,yq1,x,y,sita,dire,xa,ya,x1,y1;
	double s1;	
	xq1 = a.x-256;
	yq1 = 412-a.y;	
	s1=sqrt(pow(xq1/5,2)+pow(yq1/5,2));
	sita=atan(xq1/yq1);
	if(yq1<0)
		yq1 += PI;
	dire=sita+antor(dir);
	ya=s1*sin(dire);
	xa=s1*cos(dire);
	x=v.x+(xa*180)/(6378137*3.1415926);
	y=v.y+(ya*180)/(6378137*3.1415926*cos(x*3.1415926/180));
	CvPoint2D64f c;
	c.x=x;
	c.y=y;
	return c;
}

CvPoint2D64f U_Turn::GetMidPoint()
{
	const int width=20;

	int x_free[width]={0};
	int y_free[width]={0};
	int x_block[width]={0};
	int y_block[width]={0};

	int obstacle=0;
    int non_ob=0;
	for (int i=412; i>0; i--)                       //搜索豁口
	{
		for (int j=256; i>0; i--)
		{
			if(m_map.MapPoint[i][j]=8)
			{	
				non_ob=0;
				y_free[width]={0};
				break;
			}	
			y_free[non_ob]=i;
            non_ob++;
		}
		
		if (non_ob>width)
		{
			break;
		}
	}

	for (int i=y_free[width]; i>y_free[width]-width; i--)  //搜索豁口前的一段道路边沿
	{
		for (int j=256; j>0; j--)
		{
			if (m_map.MapPoint[i][j]=8)
			{
				x_block[obstacle]=j;
				y_block[obstacle]=i;
				obstacle++;
			}
		}
	}

    int x_sum=0;
	int y_sum=0;

    for (int i=0; i<obstacle; i++)                         //计算道路边沿所在的直线
    {
       x_sum+=x_block[i];
	   y_sum+=y_block[i];
    }

	int ave_x=x_sum/obstacle;
	int ave_y=y_sum/obstacle;

	int k_nihe=tan((90-m_dir)*PI/180);
	int b_nihe=ave_y-k_nihe*ave_x;

	
	int endpoint_y=y_free[non_ob];
	int endpoint_x=int((endpoint_y-b_nihe)/k_nihe);
    CvPoint2D64f MidPoint=MaptoGPS(m_pos,m_dir,(endpoint_x,endpoint_y));
	
	return MidPoint;                                                     //返回中点
}

CvPoint2D64f *U_Turn::GetPath()
{
	
	CvPoint2D64f MidPoint=GetMidPoint();
	CvPoint2D64f u_point[10];

	CvPoint2D64f pedal1;
	CvPoint2D64f pedal2;
	int x0=0;
	int y0=0;

	double x1=m_waypoint[0].x;
	double y1=m_waypoint[0].y;
	double x2=m_waypoint[1].x;
	double y2=m_waypoint[1].y;

	double A=y2-y1;
	double B=-(x2-x1);
	double C=x2*y1-x1*y2;

	pedal1.x=(B*B*MidPoint.x-A*B*MidPoint.y-A*C)/(A*A+B*B);
	pedal1.y=(-A*B*MidPoint.x+A*A*MidPoint.y-B*C)/(A*A+B*B);

	x1=m_waypoint[2].x;
	y1=m_waypoint[2].y;
	x2=m_waypoint[3].x;
	y2=m_waypoint[3].y;
    
	A=y2-y1;
	B=-(x2-x1);
	C=x2*y1-x1*y2;
    
	pedal2.x=(B*B*MidPoint.x-A*B*MidPoint.y-A*C)/(A*A+B*B);
	pedal2.y=(-A*B*MidPoint.x+A*A*MidPoint.y-B*C)/(A*A+B*B);

	u_point[0].x=2*m_waypoint[1].x-pedal1.x;
	u_point[0].y=2*m_waypoint[1].y-pedal1.y;
	u_point[1]=m_waypoint[1];
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
	u_point[9]=m_waypoint[2];
	u_point[10].x=2*m_waypoint[3].x-pedal2.x;
	u_point[10].y=2*m_waypoint[3].y-pedal2.y;

  Bezier(u_point, 10, path);                       //如何返回数组？
}