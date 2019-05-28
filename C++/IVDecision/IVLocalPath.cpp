#include "StdAfx.h"
#include "IVLocalPath.h"
#include<fstream>
using namespace std;
	 ofstream outm1("map1.txt");
     ofstream outm2("map2.txt");
CIVLocalPath::CIVLocalPath(void)
{
	
	/*CvMemStorage* */storage_road = cvCreateMemStorage(0);
	seq_road = cvCreateSeq( CV_64FC2, sizeof(CvSeq), sizeof(CvPoint2D64f), storage_road );

}

CIVLocalPath::~CIVLocalPath(void)
{

}

void CIVLocalPath::Get10Sample(I_Map *map)
{
	double px[10],py[10];
	int lx,rx,ly,ry;

	px[0] = 256;
	py[0] = 240;
	for (int i = 1; i<10; i++)
	{
		//右边界
		for(int j = 0; j < 50; j++)
		{
			for(int k = 0; k<10; k++)
			{
				if( map->MapPoint[(240 - i*12 - k)][int(px[i-1]+j)] == 1)
				{
					rx = px[i-1]+j;
					break;
				}
			}
		}

		//左边界
		for(int j = 0; j < 50; j++)
		{
			for(int k = 0; k<10; k++)
			{
				if( map->MapPoint[(240 - i*12 - k)][int(px[i-1]-j)] == 1)
				{
					lx = px[i-1]-j;
					break;
				}
			}
		}
		px[i] = (rx+lx)/2.0;
		py[i] = 240 - i*12 - 6;

		//找不到左右某一边界时。
		if(abs(px[i]-px[i-1])>10)
		{
			px[i] = px[i-1]+0.1;
			py[i] = py[i-1]+1;
		}
		
	}
	Set10Point(px,py);
}

void CIVLocalPath::Set10Point(double px[],double py[])//输入10个坐标点
{
	double x[10],y[10];
	for (int i = 0; i<10; i++)
	{

		y[i] = px[i] - 256;
		x[i] = 256 - py[i];

	}
	
	int n = 10;
	//yy = (double *)malloc( m*sizeof(double) );
	spline( x,y,n);
	

}

//void chase(double a[],double b[],double c[],double d[],double s[],int n);
//void spline(double x[],double y[],double yy[],int n,int m);

void CIVLocalPath::spline(double x[],double y[],int n)
{
	double *s,*a,*b,*c,*d;
	double *a1,*b1,*c1,*d1;

	int i,j;
	double step;

	a = (double *)malloc( (n-2)*sizeof(double) );
	b = (double *)malloc( (n-2)*sizeof(double) );
	c = (double *)malloc( (n-2)*sizeof(double) );
	d = (double *)malloc( (n-2)*sizeof(double) );
	if(a==NULL)
		exit(1);
	if(b==NULL)
		exit(1);
	if(c==NULL)
		exit(1);
	if(d==NULL)
		exit(1);

	a1 = (double *)malloc( (n-1)*sizeof(double) );
	b1 = (double *)malloc( (n-1)*sizeof(double) );
	c1 = (double *)malloc( (n-1)*sizeof(double) );
	d1 = (double *)malloc( (n-1)*sizeof(double) );

	if(a1==NULL)
		exit(1);
	if(b1==NULL)
		exit(1);
	if(c1==NULL)
		exit(1);
	if(d1==NULL)
		exit(1);



//	yy = (double *)malloc( m*sizeof(double) );

	a[0]=0;
	for(i=1;i<n-2;i++)
	{
		a[i]=x[i]-x[i-1];
	}

	for(i=0;i<n-2;i++)
	{
		b[i]=2*(x[i+1]-x[i] +x[i+2]-x[i+1]);
	}

	for(i=0;i<n-3;i++)
	{
		c[i]=x[i+2]-x[i+1];
	}
	c[n-3]=0;

	for(i=0;i<n-2;i++)
	{
		d[i]= 6*(y[i+2]-y[i+1])/(x[i+2]-x[i+1])-6*(y[i+1]-y[i])/(x[i+1]-x[i]);
	}
/*
	// 验证 
	for(i=0;i<n-2;i++)
	{
		printf("a[%d]=%f b[%d]=%f c[%d]=%f d[%d]=%f\n",i,a[i],i,b[i],i,c[i],i,d[i]);
	}
	// 正确
*/

	s = (double *)malloc( n*sizeof(double) );
	
	// chase method
	chase(a,b,c,d,s,n-2);
	
/*
	// 验证 
	for(i=0;i<n;i++)
	{
		printf("s[%d]=%f\n",i,s[i]);
	}
	// 正确
*/	
	for(i=0;i<n-1;i++)
	{
		a1[i]=(s[i+1]-s[i])/6/(x[i+1]-x[i]);
		b1[i]=s[i]/2;
		c1[i]=(y[i+1]-y[i])/(x[i+1]-x[i])-(2*(x[i+1]-x[i])*s[i]+(x[i+1]-x[i])*s[i+1])/6;
		d1[i]=y[i];
		
	//	printf("a1[%d]=%f b1[%d]=%f c1[%d]=%f d1[%d]=%f\n",i,a1[i],i,b1[i],i,c1[i],i,d1[i]);
	}

	i=0;
	int k=0;
	double h;
	
	for (j = 0; j<200; j++)
	{
		rx[j] = x[0] + (x[9] - x[0])*j/199;
		if( rx[j] > x[i] )
		{
			k = i;
			i = i + 1;
		}
		h = rx[j] - x[k]; 
		ry[j] = a1[k]*pow(h,3)+b1[k]*pow(h,2)+c1[k]*h+d1[k];
 
	} 
	free(a);
	free(b);
	free(c);
	free(d);
	free(a1);
	free(b1);
	free(c1);
	free(d1);
	free(s);

}

// 追赶法
void CIVLocalPath::chase(double a[],double b[],double c[],double d[],double s[],int n)
{
	int i;
	
    double *r,*x,*y;

    r = (double *)malloc( n*sizeof(double) );
	x = (double *)malloc( n*sizeof(double) ); 
	y = (double *)malloc( n*sizeof(double) ); 
	if(r==NULL)
		exit(1);
	if(x==NULL)
		exit(1);
	if(y==NULL)
		exit(1);

	r[0]=c[0]/b[0];
	y[0]=d[0]/b[0];

	for(i=1;i<n;i++)
	{  
		r[i]=c[i]/( b[i]-a[i]*r[i-1] );
		y[i]=(d[i]-a[i]*y[i-1])/( b[i]-a[i]*r[i-1]);
	}

	x[n-1]=y[n-1];  

	for(i=n-1;i>=0;i--)
	{
		x[i]=y[i]-r[i]*x[i+1];
	}

	for(i=0;i<n;i++)
	{                                         
		s[i+1]=x[i];
	}
	s[0]=0;
	s[n+1]=0;

	free(r);
	free(x);
	free(y);
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
void CIVLocalPath::Get200Point(CvPoint2D64f (&MPoint)[200])
{
	for(int i = 0; i<200; i++)
	{
		MPoint[i].x = 256 + ry[i];
		MPoint[i].y = 256 - rx[i];
	}

} 

void CIVLocalPath::Straight(CvPoint2D64f p1,CvPoint2D64f p2,CvPoint2D64f (&GPoint)[200])
{
	for(int i = 0; i<200; i++)
	{
		GPoint[i].x = p1.x + (p2.x - p1.x)*i/199;
		GPoint[i].y = p1.y + (p2.y - p1.y)*i/199;
	}
}
void CIVLocalPath::Straight(double x[],double y[],int n,CvPoint2D64f (&MPoint)[200])
{
	spline(x,y,n);
	Get200Point(MPoint);
}

void CIVLocalPath::Cross(double x[],double y[],int n,CvPoint2D64f (&MPoint)[200])
{

}
void CIVLocalPath::Bezier(CvPoint2D64f p[],int n,CvPoint2D64f *MPoint,int m)
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
void CIVLocalPath::Bezier(CvPoint2D64f p[],int n,CvPoint2D64f (&MPoint)[150])
{

	CvPoint2D64f *pc=new CvPoint2D64f[n+1];   
	int i,r;   
	float u; 
	int count = 0;
	//u的步长决定了曲线点的精度    
	for(int k = 0;k<150;k++)
	{   
		u=k/149.0;
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
void CIVLocalPath::Bezier(CvPoint2D64f p[],int n,CvPoint2D64f (&MPoint)[200])
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
void CIVLocalPath::Bezier(CvPoint2D64f p[],int n,CvPoint2D64f (&MPoint)[512])
{

	CvPoint2D64f *pc=new CvPoint2D64f[n+1];   
	int i,r;   
	float u; 
	int count = 0;
	//u的步长决定了曲线点的精度    
	for(int k = 0;k<512;k++)
	{   
		u=k/511.0;
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

int CIVLocalPath::fac(int n)//阶乘
{
	int temp = 1;
	for(int i = 1;i<=n; i++)
		temp*=i;
	return temp;
}
int CIVLocalPath::GetMidPoint(I_Map *map,CvPoint2D64f (&MPoint)[300])
{

	int count = 0;
	for(int i = 255; i>0; i--)
	{
		for(int j = 0; j<256; j++)
		{
			if(map->MapPoint[i][256-j] == 9)
			{
				
			
				MPoint[count].x = 256-j;
				MPoint[count].y = i;
				if(i>240)
					MPoint[count].x = 256;
				count++;
		
				break;
					
				
			}
			else if(map->MapPoint[i][256+j] == 9)
			{
	
				MPoint[count].x = 256+j;
				MPoint[count].y = i;
				if(i>240)
					MPoint[count].x = 256;
				count++;

				break;
				
				
			}	
		}
	}

	return count;
}
void CIVLocalPath::ExtendMap(I_Map &map)
{
	I_Map *temp;
	temp = new I_Map;
	for(int i = 0; i<512; i++)
		for(int j = 0; j<512; j++)
		{
			temp->MapPoint[j][i] = ExtendPoint(map,i,j,5);
		}
	map = *temp;

	delete temp;
}

bool CIVLocalPath::ExtendPoint(I_Map &p,int x,int y,int n)
{

	for(int i = -n; i < n; i++)
	{
		if(i+x>511||i+x<0)continue;
		for(int j = -n; j < n; j++)	
		{
			if(y+j>511||y+j<0)continue;
			if(p.MapPoint[y+j][x+i] == 1)
			return 1;
		}
	}
	return 0;
				
}
int CIVLocalPath::threeSpline(CvPoint2D64f point[],int n,CvPoint2D64f WayPoint[])
{
	double *x,*y;

	x = (double *)malloc( n*sizeof(double) ); 
	y = (double *)malloc( n*sizeof(double) ); 

	for(int i=0;i<n;i++)
	{
		x[i] = point[i].x;
		y[i] = point[i].y;
	}
	spline(x,y,n);
	
	for(int i = 0; i<200; i++)
	{
		WayPoint[i].x = rx[i];
		WayPoint[i].y = ry[i];
	}
	return 0;
}
void CIVLocalPath::GetControlPoint(CvPoint2D64f oriPoint[], CvPoint2D64f ctlPoint[])
{
	double smooth_value = 0.8;
	double x0 = oriPoint[0].x;
	double y0 = oriPoint[0].y;
	double x1 = oriPoint[1].x;
	double y1 = oriPoint[1].y;
	double x2 = oriPoint[2].x;
	double y2 = oriPoint[2].y;
	double x3 = oriPoint[3].x;
	double y3 = oriPoint[3].y;
	double xc1 = (x0 + x1) / 2.0;
	double yc1 = (y0 + y1) / 2.0;
	double xc2 = (x1 + x2) / 2.0;
	double yc2 = (y1 + y2) / 2.0;
	double xc3 = (x2 + x3) / 2.0;
	double yc3 = (y2 + y3) / 2.0; 
	double len1 = sqrt((x1-x0) * (x1-x0) + (y1-y0) * (y1-y0));
	double len2 = sqrt((x2-x1) * (x2-x1) + (y2-y1) * (y2-y1));
	double len3 = sqrt((x3-x2) * (x3-x2) + (y3-y2) * (y3-y2)); 
	double k1 = len1 / (len1 + len2);
	double k2 = len2 / (len2 + len3); 
	double xm1 = xc1 + (xc2 - xc1) * k1;
	double ym1 = yc1 + (yc2 - yc1) * k1; 
	double xm2 = xc2 + (xc3 - xc2) * k2;
	double ym2 = yc2 + (yc3 - yc2) * k2; 
	double ctrl1_x = xm1 + (xc2 - xm1) * smooth_value + x1 - xm1;
	double ctrl1_y = ym1 + (yc2 - ym1) * smooth_value + y1 - ym1; 
	double ctrl2_x = xm2 + (xc2 - xm2) * smooth_value + x2 - xm2;
	double ctrl2_y = ym2 + (yc2 - ym2) * smooth_value + y2 - ym2; 
	ctlPoint[0] = oriPoint[1];
	ctlPoint[1] = cvPoint2D64f(ctrl1_x,ctrl1_y); 
	ctlPoint[2] = cvPoint2D64f(ctrl2_x,ctrl2_y); 
	ctlPoint[3] = oriPoint[2];
}

void CIVLocalPath::SplineBezier(CvPoint2D64f APoint[],int n,CvPoint2D64f (&WPoint)[200])
{
	CvPoint2D64f OriPoint[4];
	CvPoint2D64f CtrlPoint[4];
	CvPoint2D64f waypoint[200];
	cvClearSeq( seq_road );
	if (n<=2)
	{
		Bezier(APoint,n-1,WPoint);
		return;
	}
	for(int i = 0;i<n-1;i++)
	{
		if(i == 0)
		{
			OriPoint[0] = APoint[0];
			OriPoint[0].x -= 0.0000001 ;
			OriPoint[1] = APoint[0];
			OriPoint[2] = APoint[1];
			OriPoint[3] = APoint[2];
			GetControlPoint(OriPoint,CtrlPoint);
			Bezier(CtrlPoint,3,waypoint);
			for(int i = 0; i<200; i++)
				cvSeqPush( seq_road, &waypoint[i] );
			
		}
		else if(i == n-2)
		{
			OriPoint[0] = APoint[n-3];
			OriPoint[1] = APoint[n-2];
			OriPoint[2] = APoint[n-1];
			OriPoint[2].x -= 0.0000001;
			OriPoint[3] = APoint[n-1];
			GetControlPoint(OriPoint,CtrlPoint);
			Bezier(CtrlPoint,3,waypoint);
			for(int i = 0; i<200; i++)
				cvSeqPush( seq_road, &waypoint[i] );
		}
		else
		{
			OriPoint[0] = APoint[i-1];
			OriPoint[1] = APoint[i];
			OriPoint[2] = APoint[i+1];
			OriPoint[3] = APoint[i+2];
			GetControlPoint(OriPoint,CtrlPoint);
			Bezier(CtrlPoint,3,waypoint);
			for(int i = 0; i<200; i++)
				cvSeqPush( seq_road, &waypoint[i] );
		}
	}
	int m = seq_road->total;
	for(int i = 0;i<200;i++)
	{
		WPoint[i] =  *(CvPoint2D64f*)cvGetSeqElem( seq_road, i*(n-1) );
	}

}

double N04(double t)
{
	return (1-t)*(1-t)*(1-t)/6.0;
}

// double N04(double t)
//{
//	return (1-t)*(1-t)*(1-t)/6.0;
//}
 double N14(double t)
 {
	 return t*t*t/2.0-t*t+2.0/3.0;
 }
 double N24(double t)
 {
	 return -t*t*t/2.0+t*t/2+t/2+1.0/6.0;
 }
 double N34(double t)
 {
	 return t*t*t/6.0;
 } 
 double S(double t)
 {
	 return 1-(1-t*t*t)*(1-t*t*t)*(1-t*t*t);
 }
 void CIVLocalPath::BSpline(CvPoint2D64f wp[],int n,CvPoint2D64f (&WPoint)[200])
 {
	int m;
	CvPoint2D64f *MPoint;
	MPoint = (CvPoint2D64f *)malloc( (n-3)*20*sizeof(CvPoint2D64f) );
	CvPoint2D64f *p;
	p = (CvPoint2D64f *)malloc(n*sizeof(CvPoint2D64f) );
	cvClearSeq( seq_road );
	double alfa = 0.1;//样条的张度，取值为(0,1),越小张度越大，越大趋于多边形.
	double t;//参数，0~1之间
	double data = 0.05;//取样距离，[0,1]之间，取样个数 = 1/data
	//n 为路点的个数，记住要将首尾各重复一个点	//m为输出插值点的个数，目前只按data取样，取样点个数可能会超过m.	//建议将MPoint，设置成很大的一个数组，再写出 int &m,这样就可以返回m的值了
	CvPoint2D32f startpnt,endpnt;
	CvPoint2D32f SplinePnt,tmp;
	int j = 0;
	p[0] = wp[0];
	for(int i = 1;i< n-1;i++)
		p[i] = wp[i-1];
	p[n-1] = wp[n-3];
	for(int i=1;i<n-2;i++)	
	{
		startpnt.x = p[i-1].x*N04(0) + p[i].x*N14(0) + p[i+1].x*N24(0) + p[i+2].x*N34(0);
		startpnt.y = p[i-1].y*N04(0) + p[i].y*N14(0) + p[i+1].y*N24(0) + p[i+2].y*N34(0);
		endpnt.x = p[i-1].x*N04(1.0) + p[i].x*N14(1.0) + p[i+1].x*N24(1.0) + p[i+2].x*N34(1.0);
		endpnt.y = p[i-1].y*N04(1.0) + p[i].y*N14(1.0) + p[i+1].y*N24(1.0) + p[i+2].y*N34(1.0);
		for(float t=0;t<1;t+=data)
		{			
			float guiyihua = N04(t)+N14(t)+N24(t)+N34(t);
			tmp.x = p[i-1].x*N04(t) + p[i].x*N14(t) + p[i+1].x*N24(t) + p[i+2].x*N34(t);
			tmp.y = p[i-1].y*N04(t) + p[i].y*N14(t) + p[i+1].y*N24(t) + p[i+2].y*N34(t);
			SplinePnt.x = (1-alfa)*tmp.x + (1-S(t))*(p[i].x-(1-alfa)*startpnt.x) + S(t)*(p[i+1].x-(1-alfa)*endpnt.x);
			SplinePnt.y = (1-alfa)*tmp.y + (1-S(t))*(p[i].y-(1-alfa)*startpnt.y) + S(t)*(p[i+1].y-(1-alfa)*endpnt.y);
			MPoint[j].x = SplinePnt.x;
			MPoint[j].y = SplinePnt.y;
		
			cvSeqPush( seq_road, &MPoint[j] );
	
			j ++;
		}
	}
	Bezier(MPoint,j-1,WPoint);
	m = j;
	free(MPoint);
	free(p);
}