#include<iostream> 
#include<stdlib.h> 
#include<fstream> 
#include "VisionCK.h"
using namespace std;

ifstream f1("map0830.txt"); 
ofstream outl("ll.txt");
#define FOOTSTEP 24
#define SAMPLENUM 10
D_Point APoint[200]={0};
D_Point TPoint[200]={0};
void GetMap(I_Map &m) 
{ 


	//定义输入文件流，并打开相应文件，若打开失败则f1带回0值 

	if (!f1) { //当f1打开失败时进行错误处理 
		cerr<<"map.txt file not open!"<<endl; 
		exit(1); 
	}
	

	string temp;
	int i,j;
	for(i=0;i<513;i++)
	{
		f1>>temp;
		if(i==0)continue;
		for(j=0;j<512;j++)
		{
			m.MapPoint[i-1][j]=(int)temp[j]-48;

		}
	}




}
void Get200Point(double rx[],double ry[],D_Point (&MPoint)[200])
{
	for(int i = 0; i<200; i++)
	{
		MPoint[i].D_X =  256+ry[i];
		MPoint[i].D_Y =  256-rx[i];
		
	}

}
void chase(double a[],double b[],double c[],double d[],double s[],int n)
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
void spline(double x[],double y[],int n)
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
	double rx[200],ry[200];
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
		//APoint[j].D_X = rx[j];
		//APoint[j].D_Y = ry[j];
	} 

	Get200Point(rx,ry,APoint);
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
void Set10Point(double px[],double py[])//输入10个坐标点
{
	double x[SAMPLENUM],y[SAMPLENUM];
	for (int i = 0; i<SAMPLENUM; i++)
	{

		y[i] = px[i]-256 ;
		x[i] = 256-py[i];
		
	}
	
	int n = SAMPLENUM;
	//yy = (double *)malloc( m*sizeof(double) );
	spline( x,y,n);
	

}
void Get10Sample(I_Map *map/*,CvPoint2D64f (&MidPoint)[200]*/)
{
	double px[200],py[200];
	int lx,rx,ly,ry,temp;
	float n = 1;
	int error_num = 7;
	px[0] = 275;
	py[0] = 500;
	int count = 0;
	int count1 = 0;
	double ang = 0;
	for (int j = 1; j<SAMPLENUM; j++)
	{
		count1 = 0;
		rx = 0;//右边界
		lx = 0;
		ry = 0;
		ly = 0;
		for(int i = 0; i<100; i++)
		{
			temp = 0;
			for(int k = 0 + ang*180/3.14; k<180 + ang*180/3.14; k++)
			{
				double x = i * sin(k*3.14/180) + 0.5;
				double y = i * cos(k*3.14/180) + 0.5;
				if((py[j-1]-FOOTSTEP*cos(ang/n)-y)>511||(py[j-1]-FOOTSTEP*cos(ang/n)-y)<0||(px[j-1] +FOOTSTEP*sin(ang/n)+x)>511||(px[j-1] +FOOTSTEP*sin(ang/n)+x)<0)
					continue;
				temp += map->MapPoint[(int)(py[j-1]-FOOTSTEP*cos(ang/n)-y)][(int)(px[j-1] +FOOTSTEP*sin(ang/n)+x)];
				if(temp > error_num)
				{
					rx = px[j-1] + FOOTSTEP*sin(ang/n)+x;
					ry = py[j-1] - FOOTSTEP*cos(ang/n)-y;
					break;
				}
			}
			if(rx!=0)break;
		}
	

		//左边界
		
		for(int i = 0; i < 100; i++)
		{
			temp = 0;
			for(int k = 180 + ang*180/3.14; k<360 + ang*180/3.14; k++)
			{
				double x = i * sin(k*3.14/180) + 0.5;
				double y = i * cos(k*3.14/180) + 0.5;
				if((py[j-1]-FOOTSTEP*cos(ang/n)-y)>511||(py[j-1]-FOOTSTEP*cos(ang/n)-y)<0||(px[j-1] +FOOTSTEP*sin(ang/n)+x)>511||(px[j-1] +FOOTSTEP*sin(ang/n)+x)<0)
					continue;
				temp += map->MapPoint[(int)(py[j-1]-FOOTSTEP*cos(ang/n)-y)][(int)(px[j-1]+FOOTSTEP*sin(ang/n)+x)];
				if(temp > error_num)
				{
					lx = px[j-1]+FOOTSTEP*sin(ang/n)+x;
					ly = py[j-1]-FOOTSTEP*cos(ang/n)-y;
					break;
				}
			}
			if(lx!=0)break;
		}
		px[j] = (rx+lx)/2.0;
		
		py[j] = (ry+ly)/2.0;

		//找不到左右某一边界时。
		ang = atan((px[j]-px[j-1])/(py[j-1]-py[j]));
		if(((rx-lx>100||ry-ly>100)||(rx-lx<20&&ry-ly<20)||abs(px[j]-px[j-1])>16)||abs(py[j]-py[j-1])<5)
		{
			px[j] = px[j-1]+0.01;
			py[j] = py[j-1]+1;
			//count++;
			
		}

		//if(i<2)
		//	continue;
		TPoint[j].D_X = (px[j]+px[j-1])/2;
		TPoint[j].D_Y = (py[j]+py[j-1])/2;
	}
	double ppx[SAMPLENUM];
	double ppy[SAMPLENUM];
	ppx[0] = 275;
	ppy[0] = 500;

	for(int i = 1; i<SAMPLENUM; i++)
	{
		ppx[i] = (px[i]+px[i-1])/2;
		ppy[i] = (py[i]+py[i-1])/2;
	}
	//double length;
	//length = pow(((ppx[9]-256)*(ppx[9]-256) + (ppy[9]-256)*(ppy[9]-256)),0.5);
	//length = length/5;
	//outl<<length<<endl;

	Set10Point(ppx,ppy);

}



//void chase(double a[],double b[],double c[],double d[],double s[],int n);
//void spline(double x[],double y[],double yy[],int n,int m);



// 追赶法


