#include "StdAfx.h"
#include "IVKeepLane.h"
#include "map1.h"
#include<fstream>
using namespace std;
	ofstream out555("recv2.txt");
	ofstream out666("test.txt");

CIVKeepLane::CIVKeepLane(void)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	m = new I_Map;
	app->cuo_zhen = 0;
	
	
}

CIVKeepLane::~CIVKeepLane(void)
{
	delete m;
}

void CIVKeepLane::WaitMap(queue<I_Map> &Map,I_Map &map)
{

	int count = 0;
	int size;
	while(1)  //等待地图
	{	
		//if(写标记为假)
		
		if(Map.size() >= 1)
		{	
			//设读标记为真;
			size = Map.size();
			for(int i = size;i>1;i--)
			{		
				count = 0;
				Map.pop();
			}
			map = Map.front();
			//RT_MapQ.pop();
			//设读标记为假;
			break;
		}
		Sleep(5);
		count++;//记录没有收到新数据的时间。用于异常处理
		
	}


}

//目标点移动到距右车道2m
Map_Point CIVKeepLane::ReApoint(Map_Point Ap,I_Map *WayMap)
{
	//Map_Point A;
	Ap.x = (Ap.x-256)*0.2/0.05+276;
	Ap.y = 480-(256-Ap.y)*0.2/0.1;
	
	for(int i = 0; i<512; i++)
	{
		for(int k = 0; k<180; k++)
		{
			int x = i * sin(k*PI/180);
			int y = i * cos(k*PI/180);
			if(Ap.x+x>512||Ap.x+x<0||Ap.y-y>512||Ap.y-y<0)
				continue;
			if(WayMap->MapPoint[Ap.y-y][Ap.x+x]==3)
			{
				double d = pow((double)(x*x+y*y),0.5);
				int dx = x - x*40/d;
				int dy = y - y*20/d; 
				Ap.x+=dx;
				Ap.y+=dy;
				return Ap;
			}
		}
	}
}

//地图坐标转换为经纬度坐标
CvPoint2D64f CIVKeepLane::KeepWay(Map_Point Apoint1,I_Map *WayMap,double aa,double bb,double hx)
{


	Apoint1=ReApoint(Apoint1,WayMap);
	double xq1 = Apoint1.x-276;
	double yq1 = 480-Apoint1.y;
	double x,y,sita,dire,xa,ya;
	double s1;
	s1=sqrt(pow(xq1*0.05,2)+pow(yq1*0.1,2));
    sita=atan(xq1/yq1);
	dire=sita+rad(hx);
	xa=s1*sin(dire);
	ya=s1*cos(dire);
	x=aa+(ya*180)/(6378137*PI);
	y=bb+(xa*180)/(6378137*PI*cos(rad(x)));



	CvPoint2D64f t;
	t.x = x;
	t.y = y;
	return t;

}
int CIVKeepLane::SetXPoint(int y,int n,I_Map *map)
{
	int x1=0;

	while(y>0)
	{
		for(int i = 0; i<512; i++ )
		{
			if(map->MapPoint[y][i] == n)
			{
				x1 = i;
				return x1;
			}
				
		}
		y--;
	}
}


//计算车道正中间的x坐标
Map_Point CIVKeepLane::SetAPoint(int y,I_Map *map)
{
	Map_Point A;
	
	int x1 = SetXPoint(y,2,map);
	int x2 = SetXPoint(y,3,map);
	
	A.x = (x1+x2)/2;
	A.y = y;

	A.x = (A.x-276)*0.05/0.2+256;
	A.y = 256 - (480 - A.y)*0.1/0.2;
	return A;


}

double CIVKeepLane::SetADir(Map_Point A1,Map_Point A2,double rDir)
{
	double a = atan((double)(A2.x - A1.x)/(A1.y - A2.y));
	return a + rDir;
}
//在圆弧上5m处的gps点
Map_Point CIVKeepLane::CountYuan(Map_Point a,double r)
{
	Map_Point b;
	double af=0;
	double bt=0;//bt是车辆起始位置与第一个目标点所确定的圆心角
	double st=0;
	af=5/r;

	bt=asin((256-a.y)/r);
	if(abs((256-a.y)/r)>=1)
		bt = PI/2; 
	st=bt-af;
	b.x=256+r*(1-cos(st))*5;
	b.y=256-r*sin(st)*5;
	return b;
}

bool CIVKeepLane::GetLane(I_Map *map,Map_Point (&Ap)[200])
{
	int count = 0;
	for(int i = 240; i>0; i--)
	{
		for(int j = 0; j<512; j++)
			if(map->MapPoint[2*i][j] == 3)
			{
				Ap[count].x = (int)(j - 276)/4 + 246;
				Ap[count].y = (int)240 - (480 - 2*i)/2;
				count++;
				if(count == 200)return true;
				break;
			}	
	}
	for(int i = count; i < 200; i++)
	{
		Ap[i].x = 0;
		Ap[i].y = 0;
	}
	return false;
}
//2011
//void CIVKeepLane::SearchLane(I_Map *map,CvPoint2D64f MidPoint[])
//{
//	CvPoint2D64f leftlane = {0};
//	CvPoint2D64f rightlane = {0};
//	int i = 0;
//	int j = 0;
//	int k = 0;
//	int m = 0;
//		for( j = 246; j>0; j-=5)
//		{
//			for( i = 256; i<512; i++)
//				if(map->MapPoint[i][j] == 3)
//				{
//					break;
//				}
//			for( k = 256; k>0; k--)
//				if(map->MapPoint[i][j] == 3)
//				{	
//					
//					break;
//				}
//				MidPoint[m].x = (i+k)/2;
//				MidPoint[m].y = j;
//				m++;
//		}
//	
//}
//bool CIVKeepLane::SearchLane(I_Map *map,CvPoint2D64f *MidPoint)
//{
//	if(!Get10Sample(map))
//		return false;
//	for(int i=0;i<200;i++)
//	{
//		MidPoint[i].x = APoint[i].D_X;
//		MidPoint[i].y = APoint[i].D_Y;
//	}
//	return true;
//}
bool CIVKeepLane::SearchMidLane1(I_Map *map,CvPoint2D64f &StartPoint,int count1)
{
	for(int j = StartPoint.y-count1;j<StartPoint.y;j++)
	{	
		for(int i = 0;i<count1+1;i++)
		{
			if(j > 511 || j<0 || (int)StartPoint.x+i/2>511|| (int)StartPoint.x+i/2<0 || (int)StartPoint.x-i/2>511|| (int)StartPoint.x-i/2<0)
				continue;
			if(map->MapPoint[j][(int)StartPoint.x+i/2] == 1)
			{
				StartPoint.x = (int)StartPoint.x+i/2;
				StartPoint.y = j;
				return true;
			}

			if(map->MapPoint[j][(int)StartPoint.x-i/2] == 1)
			{
				StartPoint.x = (int)StartPoint.x-i/2;
				StartPoint.y = j;
				return true;
			}
		}
	}
	return false;
}
//void CIVKeepLane::SearchMidLaneSimple(I_Map *map,CvPoint2D64f *MidPoint)
//{
//
//}
//写成一个大循环，从511循环到300
//void CIVKeepLane::SearchMidLane(I_Map *map,CvPoint2D64f *MidPoint)
//{
//	CvPoint2D64f *Temp1Point=NULL;
//	CvPoint2D64f *Temp2Point=NULL;
//	CvPoint2D64f Temp3Point[500]={0};
//	CvPoint2D64f Temp4Point[200]={0};
//	CvPoint2D64f StartPoint1,StartPoint2,StartPoint3,StartPoint4;;
//	int count = 1;
//	int count1 = 0;
//	int err_num1 = 0;
//	int result1 = false;
//	double length = 0;
//	
//	
//	for(int j=511;j>-1;j--)
//	{
//		for(int i=0;i<512;i++)
//		{
//			if(map->MapPoint[j][i] == 1)
//			{
//				count1 = 0;
//				StartPoint1.x = i;
//				StartPoint1.y = j;
//				Temp3Point[count1].x = StartPoint1.x;
//				Temp3Point[count1].y = StartPoint1.y;
//					while(1)
//					{
//						result1 = SearchMidLane1(map,StartPoint1,count);
//						if(err_num1 == 16)
//							break;
//						if(result1)
//						{
//							count1++;
//							Temp3Point[count1].x = StartPoint1.x;
//							Temp3Point[count1].y = StartPoint1.y;
//							err_num1 = 0;
//							count = 1;
//						}
//						if(count1 > 1)
//						{
//							length = pow(((Temp3Point[count1-1].x-Temp3Point[0].x)*(Temp3Point[count1-1].x-Temp3Point[0].x) + (Temp3Point[count1-1].y-Temp3Point[0].y)*(Temp3Point[count1-1].y-Temp3Point[0].y)),0.5);
//							length = length/10;
//						}
//						if(length > 19 )
//							break;
//						if(!result1)
//						{
//							err_num1++;
//							count++;
//						}
//					}//tiao while
//					if(length > 19 )
//							break;
//				}//tiao i
//			
//		}//tiao j
//		if(length > 19 )
//						break;
//	}
//		int count2 = 0;
//		count = 1;
//		int err_num2 = 0;
//		bool result2 = false;
//		bool temp = true;
//		length = 0;
//		for(int j=511;j>-1;j--)
//		{
//			for(int i=0;i<512;i++)
//			{
//				temp = true;
//				if(map->MapPoint[j][i] == 1)
//				{
//					for(int k = 0;k<count1;k++)
//					{
//						if(abs(i - Temp3Point[k].x) < 7)
//						{
//							temp = false;
//							break;
//						}
//					}
//					if(!temp)
//					{
//						break;
//					}
//					count2 = 0;
//					StartPoint2.x = i;
//					StartPoint2.y = j;
//					Temp4Point[count2].x = StartPoint2.x;
//					Temp4Point[count2].y = StartPoint2.y;
//					length = 0;
//					while(1)
//					{
//						result2 = SearchMidLane1(map,StartPoint2,count);
//						if(err_num2 == 16)
//							break;
//						if(result2)
//						{
//							count2++;
//							Temp4Point[count2].x = StartPoint2.x;
//							Temp4Point[count2].y = StartPoint2.y;
//							err_num2 = 0;
//							count = 1;
//						}
//						if(count2 > 1)
//						{
//							length = pow(((Temp4Point[count2-1].x-Temp4Point[0].x)*(Temp4Point[count2-1].x-Temp4Point[0].x) + (Temp4Point[count2-1].y-Temp4Point[0].y)*(Temp4Point[count2-1].y-Temp4Point[0].y)),0.5);
//							length = length/10;
//						}
//						if(length > 18 )
//							break;
//						if(!result2)
//						{
//							err_num2++;
//							count++;
//						}
//					}//tiao while
//					if(length > 18 )
//						break;
//				}
//			}
//			if(length > 18 )
//				break;
//		}
//					
//	double xyTopX1 = 0 ;  
//    double xyTopY1 = 0 ;
//	double xyTopX2 = 0 ;  
//    double xyTopY2 = 0 ;
//	int *MidLane1X,*MidLane1Y,*MidLane2X,*MidLane2Y;
//	MidLane1X=(int *)malloc(count1*sizeof(int) );
//    MidLane1Y=(int *)malloc(count1*sizeof(int) );
//	MidLane2X=(int *)malloc(count2*sizeof(int) );
//	MidLane2Y=(int *)malloc(count2*sizeof(int) );
//	for(int i = 0;i<count1;i++)
//	{
//		MidLane1X[i] = (int)Temp3Point[i].x;
//		MidLane1Y[i] = (int)Temp3Point[i].y;
//	}
//	for(int i = 0;i<count2;i++)
//	{
//		MidLane2X[i] = (int)Temp4Point[i].x;
//		MidLane2Y[i] = (int)Temp4Point[i].y;
//	}
//	Min2Method(xyTopX1,xyTopY1,MidLane1X,MidLane1Y,count1);
//	Min2Method(xyTopX2,xyTopY2,MidLane2X,MidLane2Y,count2);
//	double temp1,temp2;
//	temp1 = -xyTopY1/xyTopX1;
//	temp2 = -xyTopY2/xyTopX2;
//	if(temp1 > temp2)
//	{
//		path.Bezier(Temp3Point,count1-1,MidPoint,200);
//	}
//	else
//	{
//		path.Bezier(Temp4Point,count2-1,MidPoint,200);
//	}
//}

//void CIVKeepLane::SearchLane(I_Map *map,CvPoint2D64f *MidPoint)
//{
//	//for(int i = 0;i<512;i++)
//	//{
//	//	for(int j = 0;j<512;j++)
//	//		//if(map->MapPoint[i][j] == 0)
//	//			out555<<map->MapPoint[i][j];
//	//		//else out555<<1;
//	//	out555<<endl;
//	//}
//	CvPoint2D64f leftlane = {0};
//	CvPoint2D64f rightlane = {0};
//	int count = 0;
//	int n = 0;
//	int j = 0;
//	int k = 0;
//	int m = 0;
//		for( j = 510; j>360; j--)
//		{
//			
//			for( n = 256; n<512; n++)
//			{
//				if(map->MapPoint[j][n] != 0)
//				{
//					break;
//				}
//			}
//			for( k = 256; k>0; k--)
//				if(map->MapPoint[j][k] != 0)
//				{	
//					
//					break;
//				}
//				
//				MidPoint[m].x = (n+k)/2;
//			
//				MidPoint[m].y = j;
//				n = 256;
//				k = 256;
//				m++;
//				
//			
//		}
//	
//}
//CvPoint2D64f CIVKeepLane::MaptoGps(CvPoint2D64f Apoint1,double aa,double bb,double hx)
//{
//	double xq1 = Apoint1.x-256;
//	double yq1 = 512-Apoint1.y;
//	double x,y,sita,dire,xa,ya;
//	double s1;
//	s1=sqrt(pow(xq1*0.1,2)+pow(yq1*0.1,2));
//    sita=atan(xq1/yq1);
//	dire=sita+rad(hx);
//	ya=s1*sin(dire);
//	xa=s1*cos(dire);
//	x=aa+(xa*180)/(6378137*PI);
//	y=bb+(ya*180)/(6378137*PI*cos(rad(x)));
//	CvPoint2D64f t;
//	t.x = x;
//	t.y = y;
//	return t;
//
//}
//CvPoint2D64f CGetGPSData::MaptoGPS(CvPoint2D64f v,double dir,Map_Point a)
//{
//
//	double xq1,yq1,x,y,sita,dire,xa,ya,x1,y1;
//	double s1;	
//	xq1 = a.x-256;
//	yq1 = 256-a.y;	
//	s1=sqrt(pow(xq1/5,2)+pow(yq1/5,2));
//    sita=atan(xq1/yq1);
//	dire=sita+rad(dir);
//	ya=s1*sin(dire);
//	xa=s1*cos(dire);
//	x=v.x+(xa*180)/(6378137*3.1415926);
//	y=v.y+(ya*180)/(6378137*3.1415926*cos(x*3.1415926/180));
//	CvPoint2D64f c;
//	c.x=x;
//	c.y=y;
//	return c;
//}

void CIVKeepLane:: Min2Method(double &xyTopX, double &xyTopY, double  X[], double Y[], int nCount)  
	/////////////////////////////////////////////////////////////////////////////////////  
	////////  功能描述：利用最小二乘法求斜率                                   //////////  
	////////            xyTopX -- 截距                                         //////////  
	////////            xyTopY -- 斜率  
     {  
	    int      i;  
	    double   SumX, SumY, SumXY, SumX2;  
	      
		 SumX = 0;  
	    SumX2 = 0;  
	    for( i=0; i<nCount; i++)  
	    {  
	        SumX += X[i];  
	        SumX2 += double(X[i]*X[i]);  
        }  	      
	    SumY = 0;  
		for( i=0; i<nCount; i++)  
	    {  
	        SumY += Y[i];  
	    }  	    SumXY = 0;  
	    for( i=0; i<nCount; i++)  
	    {  
        SumXY += double ( X[i]* Y[i]);  
	    }  
		xyTopX = ( (SumX2*SumY - SumX*SumXY) / double(nCount*SumX2 - SumX * SumX));  
		xyTopY = ( (nCount*SumXY - SumX*SumY) / double(nCount*SumX2 - SumX * SumX));  	
} 
int CIVKeepLane::SearchMidLane(I_Map *map,CvPoint2D64f MidPoint[])
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->lane_count = 0;//初始时将

	//MidLane_Res suc_search;

	int tempcount = 0;

	int searchnum = 10;

	int num = 0;
	//suc_search = SearchFromMidLane(map,TempPoint,num);
		
	if(!SearchFromMidLane(map,TempPoint,num))
		return 2;	
	else
	{	
		TempPoint_Vehicle[0].x = 256;
		TempPoint_Vehicle[0].y = 412;
		for(int i = 0;i<500;i++)
		{
			TempPoint_Vehicle[i+1] = TempPoint[i];
		}
		path.Bezier(TempPoint_Vehicle,num,MidPoint,200);	
		return 1;			
	}
	/*else
		return 2;*/
}
//gen x比一下 差不多的 就舍弃
int CIVKeepLane::SearchFromMidLane(I_Map *map,CvPoint2D64f *MidPoint,int &num)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	CvPoint2D64f TempPoint[500]={0};
	CvPoint2D64f StartPoint;
	int count = 1;
	int count1 = 0;
	int err_num = 0;
	int result = false;
	double length = 0;
	//MidLane_Res res;
	
	for(int j=350;j>200;j--)
	{
		for(int i=100;i<400;i++)
		{
			int a = map->MapPoint[j][i];
			/*if(map->MapPoint[j][i]!=1)
			{
				res.Mid_result = false;
				res.num = 0;
				return res;
			}*/
			if(map->MapPoint[j][i] == 1)
			{
				count1 = 0;
				StartPoint.x = i;
				StartPoint.y = j;
				TempPoint[count1].x = StartPoint.x;
				TempPoint[count1].y = StartPoint.y;
				int s_num=512;
				while(s_num--)
				{
					result = SearchMidLane1(map,StartPoint,count);
					/*if(TempPoint[count1-1].y < 10)
					{

					}*/
					if(err_num == 50 || TempPoint[count1-1].y < 50)
					{
					//pan duan lu jing chang du
						if(TempPoint[count1-1].y>350/*||count1<=1*/)
						{
							//res.Mid_result = false;
							num = 0;
							return false;
						}
						else
						{
							for(int i = 0;i<count1;i++)
							{
								MidPoint[i].x = (int)TempPoint[i].x;
								MidPoint[i].y = (int)TempPoint[i].y;
							}
							app->lane_count = 1;
							//res.Mid_result = true;
							num = count1;
							return true;	
						}
					}

					if(result)
					{
						count1++;
						TempPoint[count1].x = StartPoint.x;
						TempPoint[count1].y = StartPoint.y;
						err_num = 0;
						count = 1;
					}
						
					if(!result)
					{
						err_num++;
						count++;
					}
				}//tiao while
			
			}//tiao i
			
		}//tiao j
		
	}
	
	
	//res.Mid_result = false;
	num = 0;
	return false;
							
}
void CIVKeepLane::NiheLine(double &xyTopX1,double &xyTopY1,CvPoint2D64f *his_MidGpsPoint)
{
		double *MidLane1X,*MidLane1Y;
		double xyTopX = 0 ;  //斜率与截距
		double xyTopY = 0 ;
		MidLane1X=(double *)malloc(200*sizeof(double) );
		MidLane1Y=(double *)malloc(200*sizeof(double) );
		for(int i = 0;i<200;i++)
		{
			MidLane1X[i] = his_MidGpsPoint[i].x;
			MidLane1Y[i] = his_MidGpsPoint[i].y;
		}
		Min2Method(xyTopX,xyTopY,MidLane1X,MidLane1Y,200);
		xyTopX1 = xyTopX;
		xyTopY1 = xyTopY;
}
bool CIVKeepLane::JudgeProperLine(double b,double k,CvPoint2D64f *new_MidGpsPoint)
{
	double s[4] = {0};
	double c = sqrt(pow(k,2)+1);
	for(int i = 0;i<4;i++)
	{
		s[i] = abs(k*new_MidGpsPoint[40*i+10].x-new_MidGpsPoint[40*i+10].y+b)/c;
		out666<<i<<","<<s[i]<<endl;
		if(s[i] > 2)
			return false;
	}
	return true;
}
int CIVKeepLane::SearchOneMidLane(I_Map *map,CvPoint2D64f *MidPoint)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	//MidLane_Res suc_search;
	int tempcount = 0;
	int searchnum = 10;
	app->lane_count == 0;
	while(searchnum--)
	{

		/*hiscount = app->lane_count;*/
		int num = 0;
		//suc_search = SearchFromMidLane(map,TempPoint);
		if(SearchFromMidLane(map,TempPoint,num))
		{
			tempcount =num; 
		}
		if(!SearchFromMidLane(map,TempPoint,num))
		{
			break;
		}
	}
	
	if(app->lane_count == 0)
	{
		
		return 2;
	}


	else /*if(app->lane_count == 1)*/
	{
		
		TempPoint_Vehicle[0].x = 256;
		TempPoint_Vehicle[0].y = 412;
		for(int i = 0;i<500;i++)
		{
			TempPoint_Vehicle[i+1] = TempPoint[i];
		}

		{
			path.Bezier(TempPoint_Vehicle,tempcount,MidPoint,200);
			{	
				return 1;
			}
		}
	}
}
int CIVKeepLane::SearchOnlyOneMidLane(I_Map *map,CvPoint2D64f *MidPoint)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	CvPoint2D64f TempPoint[500]={0};
	CvPoint2D64f StartPoint;
	int count = 1;
	int count1 = 0;
	int err_num = 0;
	int result = false;
	double length = 0;
	MidLane_Res res;
	int k = 0;
	TempPoint[k].x = 256;
	TempPoint[k].y = 412;
	k++;
	for(int j=412;j>0;j--)
	{
		for(int i=100;i<400;i++)
		{
			if(map->MapPoint[j][i] == 1)
			{
				TempPoint[k].x = i;
			    TempPoint[k].y = j;
			    k++;
				break;
			}
		}
	}

	path.Bezier(TempPoint,k-1,MidPoint,200);
			{	
				return 1;
			}
}



//int CIVKeepLane::SearchMidLane(I_Map *map,CvPoint2D64f *MidPoint,int judge)
//{
//	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
//	app->lane_count = 0;
//	MidLane_Res suc_search;
//
//	int tempcount[5] = {0};
//	int hiscount = 0;
//	CvPoint2D64f a,b,c;
//
//	int searchnum = 10;
//
//	while(searchnum--)
//	{
//
//		/*hiscount = app->lane_count;*/
//		suc_search = SearchFromMidLane(map,TempPoint[app->lane_count]);
//		if(suc_search.Mid_result)
//		{
//			/*if(app->lane_count - 1 < 0)
//				break;*/
//			tempcount[app->lane_count - 1] = suc_search.num; 
//		}
//		if(!suc_search.Mid_result)
//		{
//			break;
//		}
//		/*if(app->lane_count == hiscount)
//			break;*/
//	}
//	
//	if(app->lane_count == 0)
//	{
//		
//		return 2;
//	}
//
//
//	else if(app->lane_count == 1)
//	{
//		
//		TempPoint_Vehicle[0][0].x = 256;
//		TempPoint_Vehicle[0][0].y = 412;
//		for(int i = 0;i<500;i++)
//		{
//			TempPoint_Vehicle[0][i+1] = TempPoint[0][i];
//		}
//		//path.Bezier(TempPoint[0],tempcount[0]-1,MidPoint,200);
//
//		//if(abs(his_startp.x-TempPoint[0][0].x) > 20&&app->cuo_zhen<10)//如果在左边
//		//{
//		//	app->cuo_zhen++;
//		//	return 2;
//		//}
//		//else
//		{
//			path.Bezier(TempPoint_Vehicle[0],tempcount[0],MidPoint,200);
//			{	
//				//app->cuo_zhen = 0;
//				//his_startp = TempPoint[0][0];
//				return 1;
//			}
//		}
//	}
//	else if(app->lane_count == 2)
//	{
//		TempPoint_Vehicle[0][0].x = 256;
//		TempPoint_Vehicle[0][0].y = 412;
//		TempPoint_Vehicle[1][0].x = 256;
//		TempPoint_Vehicle[1][0].y = 412;
//		for(int i = 0;i<500;i++)
//		{
//			TempPoint_Vehicle[0][i+1] = TempPoint[0][i];
//			TempPoint_Vehicle[1][i+1] = TempPoint[1][i];
//		}
//		if(judge == 2)
//		{
//			if(TempPoint[0][0].x > TempPoint[1][0].x)
//			{
//				//path.Bezier(TempPoint[0],tempcount[0]-1,MidPoint,200);
//				path.Bezier(TempPoint_Vehicle[1],tempcount[1],MidPoint,200);
//				{
//					//his_startp = TempPoint[1][0];
//					return 1;
//				}
//			}
//			else
//			{
//				//path.Bezier(TempPoint[1],tempcount[1]-1,MidPoint,200);
//				path.Bezier(TempPoint_Vehicle[0],tempcount[0],MidPoint,200);
//				{
//					//his_startp = TempPoint[0][0];
//					return 1;
//				}
//			}
//		}					
//		
//		else			//judge == 1	
//		{if(TempPoint[0][0].x > TempPoint[1][0].x)
//			{
//				//path.Bezier(TempPoint[0],tempcount[0]-1,MidPoint,200);
//				path.Bezier(TempPoint_Vehicle[0],tempcount[0],MidPoint,200);
//				{
//					//his_startp = TempPoint[0][0];
//					return 1;
//				}
//			}
//			else
//			{
//				//path.Bezier(TempPoint[1],tempcount[1]-1,MidPoint,200);
//				path.Bezier(TempPoint_Vehicle[1],tempcount[1],MidPoint,200);
//				{
//					//his_startp = TempPoint[1][0];
//					return 1;
//				}
//		}}
//		
//	}
//	else if(app->lane_count == 3)
//	{
//		//AfxMessageBox("感知出错了，哈哈");
//		double d,e,f,temp;
//		TempPoint_Vehicle[0][0].x = 256;
//		TempPoint_Vehicle[0][0].y = 412;
//		TempPoint_Vehicle[1][0].x = 256;
//		TempPoint_Vehicle[1][0].y = 412;
//		TempPoint_Vehicle[2][0].x = 256;
//		TempPoint_Vehicle[2][0].y = 412;
//		for(int i = 0;i<500;i++)
//		{
//			TempPoint_Vehicle[0][i+1] = TempPoint[0][i];
//			TempPoint_Vehicle[1][i+1] = TempPoint[1][i];
//			TempPoint_Vehicle[2][i+1] = TempPoint[2][i];
//		}
//		a = TempPoint[0][0];
//		b = TempPoint[1][0];
//		c = TempPoint[2][0];
//		d = a.x;
//		e = b.x;
//		f = c.x;
//		if(d < e )
//		{
//			temp = d;
//			d = e;
//			e =temp;
//		}
//		if(e<f)
//		{
//			temp = e;
//			e = f;
//			f =temp;
//		}
//		if(d < e )
//		{
//			temp = d;
//			d = e;
//			e =temp;
//		}
//		if(e == a.x)
//	
//		{path.Bezier(TempPoint_Vehicle[0],tempcount[0],MidPoint,200);
//		}
//		
//		if(e == b.x)
//		{path.Bezier(TempPoint_Vehicle[1],tempcount[1],MidPoint,200);}
//
//		if(e == c.x)
//		{path.Bezier(TempPoint_Vehicle[2],tempcount[2],MidPoint,200);}
//		return 1;
//
//	}
//	else
//		return 2;
//}