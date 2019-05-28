#include<iostream> 
#include<stdlib.h> 
#include<fstream>
#include<math.h>

using namespace std;


ifstream f1("tu.bmp"); 

//void GetMap(H_Map *map,D_Point p,double r,I_Map &m) 
//{ 
//	//I_Map *m;
//	//m = new I_Map;
//
//	double a,b,d;
//	int x,y;
//	
//	for(int i = 0; i<512; i++)
//		for(int j = 0; j<512; j++)
//		{
//			a = atan((double)(i - 256)/(256 - j));
//			if(j > 256) a = a + 3.1416;
//			b =  a + r;
//			d = pow((double)((i-256)*(i-256)+(256-j)*(256-j)),0.5);
//			x = (int)d*sin(b) + p.D_X;
//			y = (int)p.D_Y - d*cos(b);
//			
//			if(x>1024||x<0||y>1024||y<0)
//				m.MapPoint[j][i] = 1;
//			else
//				m.MapPoint[j][i] = map->MapPoint[y][x];
//	
//		}
//	////定义输入文件流，并打开相应文件，若打开失败则f1带回0值 
//
//	//if (!f1) { //当f1打开失败时进行错误处理 
//	//	cerr<<"map.txt file not open!"<<endl; 
//	//	exit(1); 
//	//}
//	//
//	////int xy[512][512];
//	//string temp;
//	//int i,j;
//	//for(i=0;i<512;i++)
//	//{
//	//	f1>>temp;
//	//	
//	//	for(j=0;j<512;j++)
//	//	{
//	//		m->MapPoint[i][j]=(int)temp[j]-48;
//	//	//	if(m->MapPoint[j][i] == 1)
//	//	//		pDC->SetPixel(j,i,RGB(255,0,0));
//	//	}
//	//}
//	////f1.close();
//
//	//return m;
//
//}
//
//H_Map *Get_hMap() 
//{
//	H_Map *m;
//	m = new H_Map;
//	for(int i=0;i<1024;i++)
//	{
//
//		for(int j=0;j<1024;j++)
//		{
//			if(j>480&&j<530)
//				m->MapPoint[i][j]=0;
//			else m->MapPoint[i][j]=1;
//
//		}
//	}
//	return m;
//}

void GetWayMap(I_Map *rMap,I_Map &map)
{

	for(int i = 0; i<256; i++)
	{	

		for(int j = 0; j<511;j++)
		{
			if((4*(j-256)+256)<0||(4*(j-256)+256)>512)continue;
			if(rMap->MapPoint[i][j+1] - rMap->MapPoint[i][j] > 0)
			{
				map.MapPoint[2*i][4*(j-256)+256] = 3;
				map.MapPoint[2*i+1][4*(j-256)+256] = 3;
			}
			else if((j<210&&rMap->MapPoint[i][j] - rMap->MapPoint[i+1][j] > 0)||(j>210&&rMap->MapPoint[i+1][j] - rMap->MapPoint[i][j] > 0))
			{
				map.MapPoint[2*i][4*(j-256)+256] = 3;
				map.MapPoint[2*i+1][4*(j-256)+256] = 3;
			}
			else
			{
				map.MapPoint[2*i][4*(j-256)+256] = 0;
				map.MapPoint[2*i+1][4*(j-256)+256] = 0;
			}

		}
	}
}
void rUpDate(double r,double s,D_Point &p,double &d)
{
	double a,b;//弧度
	double l;//弦长
	double x,y;//位移
	if(r == 1000000)
	{
		p.D_X = p.D_X + s*0.1*5*sin(d);
		p.D_Y = p.D_Y - s*0.1*5*cos(d);
	}
	else
	{
		a = s*0.1/r;
		b = (3.1416 - a)/2 - d;
		
		l =2 * r *sin(a/2);
		
		y = l*sin(b);
		x = l*cos(b);

		p.D_X += x*5;
		p.D_Y = p.D_Y - y*5;
		d = a + d;
	}

}