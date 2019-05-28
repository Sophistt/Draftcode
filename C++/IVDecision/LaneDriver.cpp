#include "StdAfx.h"
#include "LaneDriver.h"
#include<fstream>
using namespace std;
ofstream out125("lanemidmap.txt");
ofstream out126("lanevelmap.txt");
ofstream out129("ceshi.txt");
ofstream out130("ceshi1.txt");
ofstream haha("obstacle.txt");
CLaneDriver::CLaneDriver(void)
{
	/*lane_mid_map= new I_Map;
	for( int i=0; i<512; i++)
	{
		for( int j=0; j<512; j++)
		{
			lane_mid_map->MapPoint[i][j] = 0;
			
		}
	}*/
	
	temp_map = new I_Map;
	for(int i = 0 ;i<200;i++)
	{
		his_MidPoint[i] = cvPoint2D64f(0,0);
	}
	search_res = 0;
	midd_num = 0;
	b = 0;
	c = 0;
	nearmove_obcount = 0;
	count_ob1 = 0;
	count_ob2 = 0;
}

CLaneDriver::~CLaneDriver(void)
{
	delete temp_map;
}
int CLaneDriver::LaneDrive(int &num,CvPoint2D64f (&MPoint)[512])
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	int res;

//	app->secondDecison = "车道保持";
	midd_num = 0;

	GetLaneMap1(*CVehicle::vel_Map);//44
	if(midd_num < 10)		
	{
		GetLaneMap2(*CVehicle::vel_Map);//45
		if(midd_num < 10)	
			return 0;
		res = 2;//路沿结果
	}
	else
		res = 1;//车道线结果
	num = midd_num-5;

	app->critical_planningroad.Lock();
	for(int i = 0 ;i<num;i++)
	{
		MPoint[i] = MidPoint1[i];
	}
	//YanChang(MidPoint1,50,shift_MidPoint);



	app->critical_planningroad.Unlock();
	////////////7.2



	return res;
}

void CLaneDriver::ProcCenterLane(I_Map *map_src,I_Map *map_dst)
{
	
	midd_num = 0;
	if(!map_src)return;
	IplImage *src_tmp = cvCreateImage(cvSize(512, 512), 8, 1);
	for(int i = 0;i<512;i++)
	{
		MidPoint[i] = cvPoint2D64f(0,0);
	}
//	memcpy( src_tmp->imageData, (uchar *)map_src->MapPoint, 512*512*sizeof(uchar) );		
	//读取图像数据,i-列坐标----x，j行坐标-----y
	for(int j = 0;j<512;j++)
	{for(int i = 0;i<512;i++)
		{
			//a = ((uchar*)(frameImage->imageData + frameImage->widthStep*j))[i];
			//out4<<(int)a;
			if(map_src->MapPoint[j][i] == 1)
			{
				((uchar*)(src_tmp->imageData + src_tmp->widthStep*j))[i] = 255;
			}
			else
			{
				((uchar*)(src_tmp->imageData + src_tmp->widthStep*j))[i] = 0;
			}
		}

	}
	

	zFitLanesCenter(src_tmp, src_tmp);


	
	//cvThreshold(src_tmp,map_dst,22,1,CV_THRESH_BINARY);
//	读取图像数据,i-列坐标----x，j行坐标-----y
	for(int j = 507;j>=5;j--)
	{for(int i = 5;i<507;i++)
		{
			uchar a = ((uchar*)(src_tmp->imageData + src_tmp->widthStep*j))[i];
			//out4<<(int)a;
			if(a==255)
			{
				map_dst->MapPoint[j][i] = 1;
				
				MidPoint1[midd_num].x = i;
				MidPoint1[midd_num].y = j;
				midd_num++;
	
			}
			else
			{
				map_dst->MapPoint[j][i] = 0;
			}
		}
	}
	//MidPoint1 = (CvPoint2D64f *)malloc( (middp_num)*sizeof(CvPoint2D64f) );
	/*int k = 0;
	for(int j = 0;j<512;j++)
	{
		for(int i = 0;i<512;i++)
		{
			if(map_dst->MapPoint[j][i] == 1)
			{
				MidPoint1[k].x = i;
				MidPoint1[k].y = j;
				k++;
			}

		}
	}*/

	
//	free(MidPoint1);
	
	cvReleaseImage(&src_tmp);
}
void CLaneDriver::zFitLanesCenter(IplImage *src, IplImage *dst)
{
	if(!src) return;
	IplImage *tmpsrc = cvCloneImage(src);
	cvThreshold( tmpsrc, tmpsrc, 0, 255, CV_THRESH_BINARY_INV);
	cvErode(tmpsrc,tmpsrc);
	IplImage* tmpdist = cvCreateImage( cvGetSize(tmpsrc), IPL_DEPTH_32F,1);
	cvDistTransform(tmpsrc,tmpdist,CV_DIST_L2,3);
	IplImage* tmp = cvCreateImage( cvGetSize(tmpsrc), 8,1);
	IplImage* tmp1 = cvCreateImage( cvGetSize(tmpsrc), 8,1);
	IplImage* tmp_dist = cvCreateImage( cvGetSize(tmpsrc), 8,1);
	cvConvertScaleAbs( tmpdist, tmp_dist );
//	cvSaveImage("aa.jpg",tmp_dist);
//	cvSaveImage("bb.bmp",tmp_dist);
 	cvInRangeS(tmp_dist, cvScalarAll(0), cvScalarAll(30), tmp);
	cvLaplace(tmpdist,tmpdist,3); 
	cvZero(tmp1);
	cvConvertScaleAbs( tmpdist, tmp1 );
	cvThreshold(tmp1, tmp1, 5, 255, CV_THRESH_BINARY);//0.
	cvZero(dst);
	cvCopy( tmp1, dst, tmp );
	cvReleaseImage(&tmpdist);
	cvReleaseImage(&tmpsrc);
	cvReleaseImage(&tmp);
	cvReleaseImage(&tmp1);
	cvReleaseImage(&tmp_dist);
}

int CLaneDriver::KeepLane(CvPoint2D64f rPosition,double rDirection,CvPoint2D64f (&MidGpsPoint)[200])
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	
	//int iRet = WaitForSingleObject(app->m_MapEvent,100); 
	double s_old = 0;
	double s_new = 0;
	
	//search_res = SearchMidLane(lane_mid_map,MidPoint);//100ms
	double old_dist = m_GpsData.GetDistance(app->GPS_Point.x,app->GPS_Point.y,MidGpsPoint[0].x,MidGpsPoint[0].y);
	/////////////7.2
	/*if(search_res == 2)
	{
		return 0;
	}*/
	/////////////7.2
	
	if(midd_num < 20||abs(MidPoint1[0].x-256)>40)
	{
		/*if(old_dist > 30)
			ArrayFuZhi(app->GPS_Point,MidGpsPoint);*/
		return 0;
	}
	app->drive_curvature = 75/3.6;
	YanChang(MidPoint1,50,shift_MidPoint);

	int a = abs(vel_Map->MapPoint[0][0] );
	
	double curve = 0.3*a+0.7*b/*+0.4*c*/;
	b = curve;

	out130<<" a[]0[0] " <<a<<"  curve "<<curve<<endl;
	if(abs((shift_MidPoint[199].x - 256)/(shift_MidPoint[199].y - 412)) > 1||curve> 30)
	{
		app->drive_curvature = 15/3.6;
	}
	else if(curve > 5 )
		app->drive_curvature = 75/3.6;
	//if(search_res != 2)
	{
		GetSendGps(rPosition,rDirection,MidPoint1,ori_LaneMidGpsPoint,midd_num);
	
		s_new = m_GpsData.GetDistance(rPosition.x,rPosition.y,ori_LaneMidGpsPoint[midd_num-1].x,ori_LaneMidGpsPoint[midd_num-1].y);
		if(s_new > 10)
		{
			GetSendGps(rPosition,rDirection,shift_MidPoint,MidGpsPoint,200);
			
			//GetSendGps(rPosition,rDirection,MidPoint,OriGpsPoint);
					
			ArrayFuZhi(MidGpsPoint,his_MidPoint);
		
			//ArrayFuZhi(his_MidPoint,Lane_MidGpsPoint);
		}
	
		else 
		{
			/*if(old_dist > 30)
			ArrayFuZhi(app->GPS_Point,MidGpsPoint);*/
			return 0;
		}
	}

	//return 0;
	
}
int CLaneDriver::SearchMidLane(I_Map *map,CvPoint2D64f MidPoint[])
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
		this->Bezier(TempPoint_Vehicle,num,MidPoint,200);	
		return 1;			
	}
	/*else
		return 2;*/
}
//gen x比一下 差不多的 就舍弃
int CLaneDriver::SearchFromMidLane(I_Map *map,CvPoint2D64f *MidPoint,int &num)
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

int CLaneDriver::YanChang(CvPoint2D64f midpoint[],int length,CvPoint2D64f (&shift_point)[200])
{

 

	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	//int apply_zone = (midd_num-midd_num%10)/10;
	//int apply_zone_gewei = midd_num%100;

	//CvPoint2D64f *shift_midmap = (CvPoint2D64f *)malloc( (apply_zone+length)*sizeof(CvPoint2D64f) );
	int step = midd_num/20;

	//out125<<" 5 "<<fInterval/1000.0;

	CvPoint2D64f a1 = cvPoint2D64f(256,412); /*midpoint[midd_num/3]*/;
	CvPoint2D64f a2 = midpoint[midd_num - 1];
	for(int i = 0;i>-length;i--)
	{
		shift_midmap[-i+21].x = (i*1-a1.y)/(a2.y-a1.y)*(a2.x-a1.x)+a1.x;
		shift_midmap[-i+21].y = i*1;
	}

	
//	out125<<" 6 "<<fInterval/1000.0;
	shift_midmap[0] = cvPoint2D64f(256,412);
	for(int i = 1;i<21;i++)
	{
		
		shift_midmap[i] = midpoint[(i-1)*step];
	}
	

	//out125<<" 7 "<<fInterval/1000.0;
	
	Bezier(shift_midmap,20+length,shift_point);


	//out125<<" 8 "<<fInterval/1000.0<<endl;
	//free(shift_midmap);
	return 1;
}
bool CLaneDriver::SearchMidLane1(I_Map *map,CvPoint2D64f &StartPoint,int count1)
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
bool CLaneDriver::GetLaneMap(I_Map &ori_map,I_Map &nwmap)
{
	for( int i=0; i<512; i++)
	{
		for( int j=0; j<512; j++)
		{
			nwmap.MapPoint[j][i] = ori_map.MapPoint[j][i];
			if(nwmap.MapPoint[j][i] < 8 && nwmap.MapPoint[j][i] > 0 /*&&nwmap->MapPoint[j][i] < 8*/)
				nwmap.MapPoint[j][i] = 0;	
			if(nwmap.MapPoint[j][i] > 10 &&nwmap.MapPoint[j][i] < 18)
				nwmap.MapPoint[j][i] = 1;
			/*if(nwmap.MapPoint[j][i] ==11 &&nwmap.MapPoint[j][i] ==12)
				nwmap.MapPoint[j][i] = 1;*/
		}
	}
	return 1;
}
bool CLaneDriver::GetLaneMap1(I_Map &ori_map)
{
	for(int j = 507;j>=5;j--)
	{
		for(int i = 5;i<507;i++)
		{
 			if(ori_map.MapPoint[j][i] == 44)
			{
				//MidPoint1[midd_num].x = i+1;
				MidPoint1[midd_num].x = i-1;//MidPoint1[midd_num].x = i;
				MidPoint1[midd_num].y = j;
				midd_num++;
				break;
				
			}
		}
	}
	return 1;
}

bool CLaneDriver::GetLaneMap2(I_Map &ori_map)
{
	for(int j = 500;j>=5;j--)
	{
		for(int i = 5;i<500;i++)
		{
			if(ori_map.MapPoint[j][i] == 45)
			{
				//MidPoint1[midd_num].x = i;
				MidPoint1[midd_num].x = i-1;//MidPoint1[midd_num].x = i;
				MidPoint1[midd_num].y = j;
				midd_num++;
				break;

			}
		}
	}
	return 1;
}


/////////////////////////////////////////////////////////////////////////////////////////
//最小二乘法曲线拟合
//a(0)+a(1)*x+a(2)*x*x+a(3)*x*x*x
/////////////////////////////////////////////////////////////////////////////////////////

BOOL CLaneDriver::CalculateCurveParameter(CDoubleArray *X,CDoubleArray *Y,long M,long N,CDoubleArray *A)
{
	//X,Y --  X,Y两轴的坐标
	//M   --  结果变量组数
	//N   --  采样数目
	//A   --  结果参数

	//X,Y --  X,Y两轴的坐标
	//M   --  结果变量组数
	//N   --  采样数目
	//A   --  结果参数

	register long i,j,k;
	double Z,D1,D2,C,P,G,Q;
	CDoubleArray B,W,S;
	B.SetSize(N);
	W.SetSize(N);
	S.SetSize(N);
	if(M>N)M=N;
	for(i=0;i<M;i++)
		(*A)[i]=0;
	Z=0;
	B[0]=1;
	D1=N;
	P=0;
	C=0;
	for(i=0;i<N;i++)
	{
		P=P+(*X)[i]-Z;
		C=C+(*Y)[i];
	}
	C=C/D1;
	P=P/D1;
	(*A)[0]=C*B[0];
	if(M>1)
	{
		W[1]=1;
		W[0]=-P;
		D2=0;
		C=0;
		G=0;
		for(i=0;i<N;i++)
		{
			Q=(*X)[i]-Z-P;
			D2=D2+Q*Q;
			C=(*Y)[i]*Q+C;
			G=((*X)[i]-Z)*Q*Q+G;
		}
		ASSERT(D2);
		C=C/D2;
		P=G/D2;
		Q=D2/D1;
		D1=D2;
		(*A)[1]=C*W[1];
		(*A)[0]=C*W[0]+(*A)[0];
	}
	for(j=2;j<M;j++)
	{
		S[j]=W[j-1];
		S[j-1]=-P*W[j-1]+W[j-2];
		if(j>=3)
		{
			for(k=j-2;k>=1;k--)
				S[k]=-P*W[k]+W[k-1]-Q*B[k];
		}
		S[0]=-P*W[0]-Q*B[0];
		D2=0;
		C=0;
		G=0;
		for(i=0;i<N;i++)
		{
			Q=S[j];
			for(k=j-1;k>=0;k--)
				Q=Q*((*X)[i]-Z)+S[k];
			D2=D2+Q*Q;
			C=(*Y)[i]*Q+C;
			G=((*X)[i]-Z)*Q*Q+G;
		}
		C=C/D2;
		P=G/D2;
		Q=D2/D1;
		D1=D2;
		(*A)[j]=C*S[j];
		W[j]=S[j];
		for(k=j-1;k>=0;k--)
		{
			(*A)[k]=C*S[k]+(*A)[k];
			B[k]=W[k];
			W[k]=S[k];
		}
	}
	return TRUE;
}
int CLaneDriver::getRndfWjDir(I_Map &map, CvPoint2D64f m_gps,double dir,double &goal_dir,CvPoint2D64f *cpath,int &num)
{
	num = 0;
	CvPoint2D64f c_b,c_c;
	for (int j = 511;j>0;j--)
	{
		for(int i = 0;i<511;i++)
		{
		    if (map.MapPoint[j][i] == 88)
		    {
				cpath[num].x = i;
				cpath[num].y = j;
				num++;
				break;
		    }
		}
	}
	if (num == 0)
	{
		return 0;
	}
	GetPathDirMap(cpath,num,m_gps,dir,goal_dir);
	return 1;
}
int CLaneDriver::getRndfWjMapPath(I_Map &map,CvPoint2D64f (&cpath)[512],int &num)
{
	num = 0;
	for (int j = 511;j>0;j--)
	{
		for(int i = 0;i<511;i++)
		{
			if (map.MapPoint[j][i] == 88)
			{
				cpath[num].x = i;
				cpath[num].y = j;
				num++;
				break;
			}
		}
	}
	if (num == 0)
	{
		return 0;
	}
	return 1;
}

int CLaneDriver::Yanchangs(double lanedir,double dir,double s,CvPoint2D64f v,CvPoint2D64f &aim)
{
	double dd = -lanedir + dir +90;
	double s_vm = m_GpsData.GetDistance(v.x,v.y,aim.x,aim.y);
	double s_shen = s - s_vm;
	CvPoint2D64f aa;
	 aa= m_GpsData.APiontConverD(v,aim,dir);
	if(s_shen > 0)
	{
		
	 
		aa.x = aa.x+s_shen*cos(dd*3.1415926/180)*5;
		aa.y = aa.y-s_shen*sin(dd*3.1415926/180)*5;
		aim = m_GpsData.MaptoGPS(v,dir,aa);
		 return 1;
	}
	return 0;
}
int CLaneDriver::GetPathDirGPS(CvPoint2D64f *CrossPath,int num,CvPoint2D64f m_gps,double dir,double &goal_dir)
{
	int res1,res2;
	CvPoint2D64f goal1,goal2;
	res1 = GetPointfromNearGPS(CrossPath,num,m_gps,36,goal1);
	res2 = GetPointfromNearGPS(CrossPath,num,m_gps,42,goal2);
	if(res1&&res2)
		/*goal_dir = m_GpsData.GetAngle(CrossPath[n].x,CrossPath[n].y,CrossPath[n+5].x,CrossPath[n+5].y);*/
		goal_dir = m_GpsData.GetAngle(goal2,goal1);
	else
		goal_dir = m_GpsData.GetAngle(CrossPath[num-1].x,CrossPath[num-1].y,CrossPath[num-20].x,CrossPath[num-20].y);
	return 1;
}

int CLaneDriver::GetPathDirMap(CvPoint2D64f *CrossPath,int num,CvPoint2D64f m_gps,double dir,double &goal_dir)
{
	CvPoint2D64f gpspath[512];
	for (int i =0; i<num;i++)
	{
		gpspath[i]=m_GpsData.MaptoGPS(m_gps,dir,CrossPath[i]);
	}
	int res1,res2;
	CvPoint2D64f goal1,goal2;
	res1 = GetPointfromNearGPS(gpspath,num,m_gps,36,goal1);
	res2 = GetPointfromNearGPS(gpspath,num,m_gps,42,goal2);
	if(res1&&res2)
		/*goal_dir = m_GpsData.GetAngle(CrossPath[n].x,CrossPath[n].y,CrossPath[n+5].x,CrossPath[n+5].y);*/
		goal_dir = m_GpsData.GetAngle(goal2,goal1);
	else
		goal_dir = m_GpsData.GetAngle(gpspath[num-1].x,gpspath[num-1].y,gpspath[num-20].x,gpspath[num-20].y);
	return 1;
}

int CLaneDriver::GetPointfromNearGPS(CvPoint2D64f *path,int num,CvPoint2D64f m_gps,double s,CvPoint2D64f &res)
{
	double len;
	double min_len = 99999;
	int n = 0;
	for(int i = 0;i<num;i++)
	{
		len = m_GpsData.GetDistance(m_gps.x,m_gps.y,path[i].x,path[i].y);
		if(len < min_len)
		{
			min_len = len;
			n = i;
		}
	}
	int t=num-1;
	for(int i = n;i<num;i++)
	{
		len = m_GpsData.GetDistance(m_gps.x,m_gps.y,path[i].x,path[i].y);
		if(len > s)
		{
			t = i;
			res = path[i];
			return i;
		}

	}
	res = path[t];
	return 0;

}

int CLaneDriver::GetPointfromNearMap(CvPoint2D64f *path,int num,CvPoint2D64f cent,double s,	CvPoint2D64f &res)
{
	double len;
	double min_len = 99999;
	int n = 0;
	double x,y;
	for(int i = 0;i<num;i++)
	{
		x = (path[i].x - cent.x)/5;
		y = (path[i].y - cent.y)/5;
		len = sqrt(x*x+y*y);
		if(len < min_len)
		{
			min_len = len;
			n = i;
		}
	}
	int t=num-1;
	for(int i = n;i<num;i++)
	{
		x = (path[i].x - cent.x)/5;
		y = (path[i].y - cent.y)/5;
		len = sqrt(x*x+y*y);
		if(len > s)
		{
			t = i;
			res = path[i];
			return i;
		}

	}
	res = path[num-1];
	return 0;

}
int CLaneDriver::Get2Curve(CvPoint2D64f p1,CvPoint2D64f p2,CvPoint2D64f p3,CvPoint2D64f (&curve)[200])
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	CvPoint2D64f m_gps = app->GPS_Point;
	double dir = app->GPS_Direction;
	
	CvPoint2D64f pgps[3] = {p1,p2,p3};
	CvPoint2D64f pmap[3],p[3];
	
	double xy[3];

	for(int i = 0;i<3;i++)
	{
		pmap[i] = m_GpsData.APiontConverD(m_gps,pgps[i],dir);
		p[i].x = pmap[i].y;
		p[i].y = pmap[i].x;
	}
	double xishu[3] = {0};
	Calculate2cicurveparam(p[0],p[1],p[2],xy);
	for(int i = 0;i<3;i++)
	{
		xishu[i] = xy[i];
	}
	double dx = 512/199.0;
	for(int i = 0;i<200;i++)
	{
		curve[i].y = 511-dx*i;
		curve[i].x = xishu[0]*curve[i].y*curve[i].y+xishu[1]*curve[i].y+xishu[2];
	}
	for(int i = 0;i<200;i++)
	{
		curve[i] = m_GpsData.MaptoGPS(m_gps,dir,curve[i]);
		
	}
	return 1;
}

int CLaneDriver::Calculate2cicurveparam(CvPoint2D64f p1,CvPoint2D64f p2,CvPoint2D64f p3,double (&param)[3])
{
	double x1 = p1.x;
	double x2 = p2.x;
	double x3 = p3.x;
	double y1 = p1.y;
	double y2 = p2.y;
	double y3 = p3.y;
	param[0]=y1/((x1-x2)*(x1-x3))+y2/((x2-x1)*(x2-x3))+y3/((x3-x1)*(x3-x2));
	param[1]=-y1*(x2+x3)/((x1-x2)*(x1-x3))-y2*(x1+x3)/((x2-x1)*(x2-x3))-y3*(x1+x2)/((x3-x1)*(x3-x2));
	param[2] = y1*x2*x3/((x1-x2)*(x1-x3))+y2*x1*x3/((x2-x1)*(x2-x3))+y3*x1*x2/((x3-x1)*(x3-x2));
	return 1;
}
int CLaneDriver::GetPointNumfromNearGPS(CvPoint2D64f *path,int num,CvPoint2D64f m_gps,double s)
{
	double len;
	double min_len = 99999;
	int n = 0;
	for(int i = 0;i<num;i++)
	{
		len = m_GpsData.GetDistance(m_gps.x,m_gps.y,path[i].x,path[i].y);
		if(len < min_len)
		{
			min_len = len;
			n = i;
		}
	}
	int t=num-1;
	len = 0;
	for(int i = n;i<num-1;i++)
	{
		len +=  m_GpsData.GetDistance(path[i+1].x,path[i+1].y,path[i].x,path[i].y);
		
		if(len > s)
		{
			t = i;
			return i;
		}

	}
	return 0;

}

double  CLaneDriver::GetMinDis(CvPoint2D64f *path,int num,CvPoint2D64f m_gps)
{
	double len;
	double min_len = 99999;
	int n = 0;
	for(int i = 0;i<num;i++)
	{
		len = m_GpsData.GetDistance(m_gps.x,m_gps.y,path[i].x,path[i].y);
		if(len < min_len)
		{
			min_len = len;
			n = i;
		}
	}
	return min_len;
}
int  CLaneDriver::GetMinNum(CvPoint2D64f *path,int num,CvPoint2D64f m_gps)
{
	double len;
	double min_len = 99999;
	int n = 0;
	for(int i = 0;i<num;i++)
	{
		len = m_GpsData.GetDistance(m_gps.x,m_gps.y,path[i].x,path[i].y);
		if(len < min_len)
		{
			min_len = len;
			n = i;
		}
	}
	return n;
}
int CLaneDriver::SearchMoveStaticObstacleGps(CvPoint2D64f *waypoint)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	bool ob = false;
	bool ob_left=false;
	bool left_yellow = false;
	bool ob_right=false;
	int up,down;
	double left_dis = 10000;
	double middle_dis = 10000;
	double right_dis = 10000;
	CvPoint2D64f yellow[200];
	up=50;
	if(app->GPS_Speed*3.6>30)
		up=45;
	else
		up = 40;
	if(app->GPS_Speed*3.6>25)
		down = 0/*-10*/;
	else
		down = 0;
	ob = SearchObstacle(waypoint,map_ts,28,28,28,15,0);
	if(ob)
		nearmove_obcount++;
	else
		nearmove_obcount = 0;
	if(nearmove_obcount > 2)
		return 3;/////20m内距离有移动障碍物并且统计次数大于2,返回2
	
	ob = SearchObstacle1(waypoint,vel_Map,8,18,38,up,0,middle_dis);
	int obsick = SearchObstacle(waypoint,map_ts,19,19,19,up,0);
	if(ob||obsick)
		stillmove_obcount++;
	else
		stillmove_obcount = 0;
	if(stillmove_obcount > 1)
	{
		MoveLeft(waypoint,exe_leftpoint,3.9);
		//for(int i = 0;i<512;i++)
		//{
		//	for(int j = 0;j<512;j++)
		//		//if(app->PercepMap->MapPoint[i][j] == 0)
		//			out130<<map_ts->MapPoint[i][j];
		//		//else out333<<1;
		//	out130<<endl;
		//}
		//for(int i = 0;i<512;i++)
		//{
		//	for(int j = 0;j<512;j++)
		//		//if(app->PercepMap->MapPoint[i][j] == 0)
		//		out129<<vel_Map->MapPoint[i][j];
		//	//else out333<<1;
		//	out129<<endl;
		//}
			//ob_left = SearchObstacleLuyan(exe_leftpoint,map_ts,8,28,38,up,-10);
		ob_left = SearchObstacle1(exe_leftpoint,map_ts,8,28,38,up,down,right_dis);
		
		if(!ob_left)
		{
			
			{
				app->secondDecison = "右换道";
				app->left_right = -1;
					return -1;
			}
			//////有些小问题
		}
		else
		{
			MoveLeft(waypoint,exe_rightpoint,-3.9);
			ob_right = SearchObstacle1(exe_rightpoint,map_ts,8,28,38,up,down,left_dis);
			MoveLeft(waypoint,yellow,3.7/2);
			left_yellow = SearchObstacle(yellow,map_ts,46,46,46,up,0,3.7/2);
			if(ob_right/*||left_yellow*/)
			{	
				if (middle_dis>40&&left_dis>40&&right_dis>40)////给25的速度
				{
					return 6;
				}
				else if (middle_dis>20&&left_dis>10&&right_dis>10)/////给10的速度
				{
					return 5;
				}
				/*else if (middle_dis<5)///停车
				{	
					return 4;
				}*/
				else ////给出的速度为5
					return 4;
			
			}
			else
			{
				app->secondDecison = "左换道";
				app->left_right = 1;
				return 1;
			}
		}
	}
	ob = SearchObstacle(waypoint,map_ts,38,38,38,80,0);
	if(ob)
		return 3;	
	return 0;
}
int CLaneDriver::SearchMoveStaticObstacleGps1(CvPoint2D64f *waypoint)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	int ob = false;
	bool ob_moving = false;
	bool ob_left=false;
	bool left_yellow = false;
	bool ob_right=false;
	bool obstop_left = false;
	bool obstop_right = false;
	int up,down;
	int x_dis,y_dis = 0;
	double left_dis = 10000;
	double middle_dis = 10000;
	double right_dis = 10000;
	CvPoint2D64f yellow[200];
	bool open_light = false;
	up=app->GPS_Speed*3.6*1.2+23;//up=app->GPS_Speed*3.6+23;//33
	if(up<35)
		up=35;
	else if(up>80)
		up=80;
	if (app->range_flag)
	{
		up=30;
	}

	down = 0;

	/*
	ob = SearchObstacleMoveStill(waypoint,vel_Map,8,18,28,up,down,x_dis,y_dis);
	if (ob)
	{
		stillmove_obcount++;
	}
	else
		stillmove_obcount = 0;
	if (stillmove_obcount < 2)
	{
		count_ob1 = 0;
		count_ob2 = 0;
		return 0;
	}
	*/
	double lanemovedist;
	/*
	if(app->GPS_Speed*3.6<=30)
		lanemovedist=1;
	else
		lanemovedist=1+(app->GPS_Speed*3.6-30)/20;
		*/

	if(app->GPS_Speed*3.6<=25)
	{
		lanemovedist=0.5;
	}
	else if(app->GPS_Speed*3.6<=45)
	{
		lanemovedist=1;
	}
	else if(app->GPS_Speed*3.6<=62)
	{
		lanemovedist=1+(app->GPS_Speed*3.6-40)/10;
	}
	else
	{
		lanemovedist=3.2;
	}

	bool bturnrightreq = 0;
	bool bturnleftreq = 0;

	/*
	ob_left=SearchObstacle_leftedge(waypoint,vel_Map,8,18,28,up,-25,middle_dis);
	ob_right=SearchObstacle_rightedge(waypoint,vel_Map,8,18,28,up,-25,middle_dis);
	if(ob_left&&(!ob_right))
		bturnrightreq=1;
		*/

	/*
	ob_left=SearchObstacle_leftedge_rightside(waypoint,vel_Map,8,18,28,up,-25,middle_dis);
	ob_right=SearchObstacle_rightedge_rightside(waypoint,vel_Map,8,18,28,up,-25,middle_dis);
	if((!ob_left)&&ob_right)
		bturnleftreq=1;
		*/

	double lanenochangedist;
	if(app->GPS_Speed*3.6<13)
		lanenochangedist=7;
	else if(app->GPS_Speed*3.6>60)
		lanenochangedist=15;
	else
		lanenochangedist=7+(app->GPS_Speed*3.6-13)*8/47;

	ob_left=0;
	ob_right=0;

	int lanechangenum_left = 6;//6
	int lanechangenum_right = 6;//6

	int up_sidelane=up+lanemovedist*6+10;
	up_sidelaneleft=up_sidelane;
	up_sidelaneright=up_sidelane;

	{
		ob = SearchObstacle1_lanechange(waypoint,vel_Map,8,18,28,up,down,middle_dis);

		if(ob==2 || (ob>0 && app->bGpsGanrao))
		{
			stillmove_obcount++;
			if (stillmove_obcount < 3)
				ob=0;
			else
				ob=1;
		}
		else
			stillmove_obcount = 0;

		if(ob&&y_middle_front[0]>=0)
		{
			if((y_middle_front[0]>=(app->GPS_Speed)*(app->GPS_Speed)/3+6)&&(spdrelmidfront>0 || spdrelmidfront<-1000))//if((y_middle_front[0]>=(app->GPS_Speed)*(app->GPS_Speed)/7+6)&&(spdrelmidfront>5/3.6 || spdrelmidfront<-1000))
				ob=0;
			/*
			if(spdrelmidfront>-1000)
			{
				double comfortdist=20;
				if(spdrelmidfront<0)
					comfortdist=spdrelmidfront*spdrelmidfront/2/0.4+6;
				if(comfortdist<20)
					comfortdist=20;
				if(spdrelmidfront>=0)
					comfortdist=20;//15
				if(y_middle_front[0]>=comfortdist)
					ob=0;
			}
			*/
		}

		if(app->laneturndir==1)
		{
			up_sidelaneleft=app->GPS_Speed*3.6*1.2+23;

			if(app->GPS_Speed<12/3.6 && app->lukoulanechangereq)
				up_sidelaneleft=app->GPS_Speed*3.6*1.2+13;

		}

		if(app->laneturndir==2)
		{
			up_sidelaneright=app->GPS_Speed*3.6*1.2+23;

			if(app->GPS_Speed<12/3.6 && app->lukoulanechangereq)
				up_sidelaneright=app->GPS_Speed*3.6*1.2+13;

		}

		if(ob==1 && app->GPS_Speed<12/3.6)
		{
			if(app->laneturndir==1)
			{
				if(up_sidelaneleft>middle_dis+10)
					up_sidelaneleft=middle_dis+10;
			}
			else if(app->laneturndir==2)
			{
				if(up_sidelaneright>middle_dis+10)
					up_sidelaneright=middle_dis+10;
			}
			else
			{
				if(up_sidelaneleft>middle_dis+15)
					up_sidelaneleft=middle_dis+15;
				if(up_sidelaneright>middle_dis+15)
					up_sidelaneright=middle_dis+15;
			}
		}

		double threshold;
		if(app->GPS_Speed<15/3.6)
			threshold=20;
		else if(app->GPS_Speed>45/3.6)
			threshold=40;
		else
			threshold=20+(app->GPS_Speed*3.6-15)/(45-15)*(40-20);

		int threshold_left=threshold;
		int threshold_right=threshold;

		if(threshold_left>up_sidelaneleft)
			threshold_left=up_sidelaneleft;
		if(threshold_right>up_sidelaneright)
			threshold_right=up_sidelaneright;

		//MoveLeft(waypoint,exe_leftpoint,3.7);
		for(int i=0;i<200;i++)
		{
			exe_leftpoint[i].x=app->rndfbezier_left[i].x;
			exe_leftpoint[i].y=app->rndfbezier_left[i].y;
		}

		if(app->lukoulanechangereq && app->laneturndir==1)
		{
			if((y_left_front[0]>-1000)&&((y_left_front[0]<(app->GPS_Speed)*(app->GPS_Speed)/2+6)||(y_left_front[0]<=up_sidelaneleft && (spdrelleftfront<-5/3.6 || app->GPS_Speed<10/3.6))||((y_left_front[0]<=up_sidelaneleft-10 || y_left_front[0]<=13) && (spdrelleftfront<5/3.6))))
			{
				ob_left=true;
				haha<<"1"<<endl;
			}
		}
		else
		{
			if((y_left_front[0]>-1000)&&((y_left_front[0]<middle_dis+10 && (!((y_left_front[0]>=(app->GPS_Speed)*(app->GPS_Speed)/2+16)&&(spdrelleftfront>10/3.6))))||(y_left_front[0]<(app->GPS_Speed)*(app->GPS_Speed)/7+6)||(y_left_front[0]<=up_sidelaneleft && (spdrelleftfront<-5/3.6 || app->GPS_Speed<10/3.6))||((y_left_front[0]<=up_sidelaneleft-10 || y_left_front[0]<=13) && (spdrelleftfront<5/3.6))))
			{
				ob_left=true;
				haha<<"2"<<endl;
			}
		}

		if((y_left_front[0]>-1000)&& spdrelleftfront>-1000 &&(spdrelleftfront+app->GPS_Speed<-10/3.6))
		{
			ob_left=true;
			haha<<"3"<<endl;
		}

		if((y_left_rear[0]>-1000)&&((y_left_rear[0]<8)||(y_left_rear[0]<=50 && (spdrelleftrear>5/3.6))||(y_left_rear[0]<=20 && (spdrelleftrear>0))||(y_left_rear[0]<=15 && app->GPS_Speed>10/3.6 && (spdrelleftrear>-5/3.6))))
		{
			ob_left=true;
			haha<<"4"<<endl;
		}
		if((y_left_rear[0]>-1000)&&(y_left_rear[0]<=50 && (spdrelleftrear<-1000)))
		{
			ob_left=true;
			haha<<"5"<<endl;
		}

		int nobleft=SearchObstacle1_leftlaneobdist(waypoint,vel_Map,8,18,28,10,-1,left_dis);
		if(nobleft<0)
		{
			if(lanechangenum_left>(int)((-nobleft*0.2-2)/0.53))
				lanechangenum_left=(int)((-nobleft*0.2-2)/0.53);
			if(lanechangenum_left<4)
				lanechangenum_left=4;
		}

		//ob_left = SearchObstacle1_lanechange_left(exe_leftpoint,vel_Map,8,18,28,up_sidelane,-25,left_dis);//-25
		bool ob_left_nextlane = SearchObstacle1_leftnextlane(exe_leftpoint,vel_Map,8,18,28,up+lanemovedist*6+10,3,left_dis);
		if(ob_left_nextlane && lanechangenum_left>5)
			lanechangenum_left=5;
		/*
		if(!ob_left)
		{
			MoveLeft(waypoint,exe_leftpoint,3.2);
			ob_left = SearchObstacle1_lanechange_left(exe_leftpoint,vel_Map,8,18,28,up_sidelane,-25,left_dis);//-25
		}
		*/

		if(!ob_left)
		{
			ob_left = SearchObstacle1_leftnear(waypoint,vel_Map,8,18,28,lanenochangedist,-1,left_dis);//10
			if(ob_left)
				haha<<"6"<<endl;
		}

		/*
		if(!ob_left)
		{
			ob_left = SearchObstacle1_leftnear_new(waypoint,vel_Map,8,18,28,threshold_left,lanenochangedist,-1,left_dis);//10
			if(ob_left)
				haha<<"6"<<endl;
		}
		*/

		//MoveLeft(waypoint,exe_rightpoint,-3.7);
		for(int i=0;i<200;i++)
		{
			exe_rightpoint[i].x=app->rndfbezier_right[i].x;
			exe_rightpoint[i].y=app->rndfbezier_right[i].y;
		}

		if(app->lukoulanechangereq && app->laneturndir==2)
		{
			if((y_right_front[0]>-1000)&&((y_right_front[0]<(app->GPS_Speed)*(app->GPS_Speed)/2+6)||(y_right_front[0]<=up_sidelaneright && (spdrelrightfront<-5/3.6 || app->GPS_Speed<10/3.6))||((y_right_front[0]<=up_sidelaneright-10 || y_right_front[0]<=13) && (spdrelrightfront<5/3.6))))
				ob_right=true;
		}
		else
		{
			if((y_right_front[0]>-1000)&&((y_right_front[0]<middle_dis+10 && (!((y_right_front[0]>=(app->GPS_Speed)*(app->GPS_Speed)/2+16)&&(spdrelrightfront>10/3.6))))||(y_right_front[0]<(app->GPS_Speed)*(app->GPS_Speed)/7+6)||(y_right_front[0]<=up_sidelaneright && (spdrelrightfront<-5/3.6 || app->GPS_Speed<10/3.6))||((y_right_front[0]<=up_sidelaneright-10 || y_right_front[0]<=13) && (spdrelrightfront<5/3.6))))
				ob_right=true;
		}

		if((y_right_front[0]>-1000) && spdrelrightfront>-1000 &&(spdrelrightfront+app->GPS_Speed<-10/3.6))
			ob_right=true;

		if((y_right_rear[0]>-1000)&&((y_right_rear[0]<8)||(y_right_rear[0]<=50 && (spdrelrightrear>5/3.6))||(y_right_rear[0]<=20 && (spdrelrightrear>0))||(y_right_rear[0]<=15 && app->GPS_Speed>10/3.6 && (spdrelrightrear>-5/3.6))))
			ob_right=true;
		if((y_right_rear[0]>-1000)&&(y_right_rear[0]<=50 && (spdrelrightrear<-1000)))
			ob_right=true;

		int noright=SearchObstacle1_rightlaneobdist(waypoint,vel_Map,8,18,28,10,-1,right_dis);
		if(noright>0)
		{
			if(lanechangenum_right>(int)((noright*0.2-2)/0.53))
				lanechangenum_right=(int)((noright*0.2-2)/0.53);
			if(lanechangenum_right<4)
				lanechangenum_right=4;
		}

		//ob_right = SearchObstacle1_lanechange_right(exe_rightpoint,vel_Map,8,18,28,up_sidelane,-25,right_dis);//-25
		bool ob_right_nextlane = SearchObstacle1_rightnextlane(exe_rightpoint,vel_Map,8,18,28,up+lanemovedist*6+10,3,right_dis);
		if(ob_right_nextlane && lanechangenum_right>5)
			lanechangenum_right=5;
		/*
		if(!ob_right)
		{
			MoveLeft(waypoint,exe_rightpoint,-3.2);
			ob_right = SearchObstacle1_lanechange_right(exe_rightpoint,vel_Map,8,18,28,up_sidelane,-25,right_dis);//-25
		}
		*/

		if(!ob_right)
			ob_right = SearchObstacle1_rightnear(waypoint,vel_Map,8,18,28,lanenochangedist,-1,right_dis);//10

		/*
		if(!ob_right)
			ob_right = SearchObstacle1_rightnear_new(waypoint,vel_Map,8,18,28,threshold_right,lanenochangedist,-1,right_dis);//10
			*/

		/*
		if(app->PercepMap->MapPoint[256][511]==55)
			ob_left = true;
			*/
	}
	if((app->bjiedaochaoche!=1)&&(app->PercepMap->MapPoint[412][0]==255 || app->PercepMap->MapPoint[411][0]==255))
	{
		ob_left = true;
		haha<<"7"<<endl;
	}
	if((app->bjiedaochaoche!=1)&&(app->PercepMap->MapPoint[412][511]==255 || app->PercepMap->MapPoint[411][511]==255))
		ob_right = true;

	if(app->GPS_Speed<12/3.6)
	{
		boblefthis=false;
		bobrighthis=false;
		boblefthispre=false;
		bobrighthispre=false;
	}

	bool boblefttmp=boblefthispre;
	bool bobrighttmp=bobrighthispre;
	boblefthispre=boblefthis;
	bobrighthispre=bobrighthis;
	boblefthis=ob_left;
	bobrighthis=ob_right;

	if(boblefttmp||boblefthispre||boblefthis)
		ob_left=true;
	if(bobrighttmp||bobrighthispre||bobrighthis)
		ob_right=true;

	if(app->bleftturnban && (app->bjiedaochaoche!=1))
	{
		ob_left = true;
		haha<<"9"<<endl;
	}
	if(app->brightturnban)
		ob_right = true;

	bool leftturnfirst=0;
	bool rightturnfirst=0;

	if(app->laneturndir==1)
	{
		//ob_right = true;
		leftturnfirst = true;
	}
	else if(app->laneturndir==2)
	{
		//ob_left = true;
		rightturnfirst = true;
	}

	/*
	if(ob==2 || (ob>0 && app->bGpsGanrao))
	{
		stillmove_obcount++;
		if (stillmove_obcount < 3)
			ob=0;
		else
			ob=1;
	}
	else
		stillmove_obcount = 0;
		*/

	bobfrontlchgreqdisp=ob;

	/*
	if(app->GPS_Speed*3.6<10)
		lanenochangedist=6;
	else if(app->GPS_Speed*3.6>60)
		lanenochangedist=27;
	else
		lanenochangedist=6+(app->GPS_Speed*3.6-10)*21/50;

	if(SearchObstacle1_lanechangenot(waypoint,vel_Map,8,18,28,lanenochangedist,0)>0)
		return 5;
		*/

	if(app->leftturnreq_proc)
	{
		bturnleftreq=true;
		bturnrightreq=false;
		ob_right=true;
	}
	else if(app->rightturnreq_proc)
	{
		bturnrightreq=true;
		bturnleftreq=false;
		ob_left=true;
		haha<<"10"<<endl;
	}

	if(app->lukoulanechangereq && (app->bjiedaochaoche!=1))
	{
		if(app->laneturndir==1)
			ob_right=true;
		else if(app->laneturndir==2)
		{
			ob_left=true;
			haha<<"11"<<endl;
		}
	}

	if(app->bbiandaoban && (app->bjiedaochaoche!=1))
	{
		ob_right=true;
		ob_left=true;
		haha<<"12"<<endl;
	}
	if(app->bnobiandaoraozhang && (app->bjiedaochaoche!=1))
	{
		ob_right=true;
		ob_left=true;
		haha<<"13"<<endl;
	}

	if(app->PercepMap->MapPoint[410][0]!=0 && (app->bjiedaochaoche!=1))
	{
		ob_left = true;
		haha<<"14"<<endl;
	}

	bobleftlchgnotalooweddisp=(int)ob_left;
	bobrightlchgnotalooweddisp=(int)ob_right;

	if(SearchFrontOb(vel_Map,lanenochangedist)>0)
	{
		bobfrontlchgnotalooweddisp=1;
		return 5;
	}
	else
		bobfrontlchgnotalooweddisp=0;

	/*
	ob_left=true;
	ob_right=true;
	*/

	if(bturnrightreq)
	{
		if(!ob_right)
		{
			app->secondDecison = "右换道";
			app->left_right = 1;
			count_ob1 = 0;
			count_ob2 = 0;
			if(app->bGpsGanrao)
				app->obb_left = false;
			else
				app->obb_left = true;
			app->lanechangenum = lanechangenum_right;
			return 1;
		}
	}
	else if(bturnleftreq)
	{
		if(!ob_left)
		{
			app->secondDecison = "左换道";
			app->left_right = -1;
			count_ob1 = 0;
			count_ob2 = 0;
			if(app->bGpsGanrao)
				app->obb_right = false;
			else
				app->obb_right = true;
			app->lanechangenum = lanechangenum_left;
			return -1;
		}
	}

	if(app->laneturndir==2)
	{
		if(!ob_right)
		{
			app->secondDecison = "右换道";
			app->left_right = 1;
			count_ob1 = 0;
			count_ob2 = 0;
			app->lanechangenum = lanechangenum_right;
			return 1;
		}
		else if(app->lukoulanechangereq && (app->bjiedaochaoche!=1))
			bwaitlchg=true;
	}
	else if(app->laneturndir==1)
	{
		if(!ob_left)
		{
			app->secondDecison = "左换道";
			app->left_right = -1;
			count_ob1 = 0;
			count_ob2 = 0;
			app->lanechangenum = lanechangenum_left;
			return -1;
		}
		else if(app->lukoulanechangereq && (app->bjiedaochaoche!=1))
			bwaitlchg=true;
	}

	/*
	if(app->lukoulanechangereq==2)
	{
		if(!ob_right)
		{
			app->secondDecison = "右换道";
			app->left_right = 1;
			count_ob1 = 0;
			count_ob2 = 0;
			app->obb_left = true;
			app->lanechangenum = lanechangenum_right;
			return 1;
		}
	}
	else if(app->lukoulanechangereq==1)
	{
		if(!ob_left)
		{
			app->secondDecison = "左换道";
			app->left_right = -1;
			count_ob1 = 0;
			count_ob2 = 0;
			app->obb_right = true;
			app->lanechangenum = lanechangenum_left;
			return -1;
		}
	}
	*/

	if (ob&&!ob_left&&!ob_right)
	{
		if(y_left_front[0]>-1000 && y_right_front[0]>-1000)
		{
			if(y_left_front[0]>y_right_front[0])
			{
				app->secondDecison = "左换道";
				app->left_right = -1;
				count_ob1 = 0;
				count_ob2 = 0;
				if(app->bGpsGanrao)
					app->obb_right = false;
				else
					app->obb_right = true;
				app->lanechangenum = lanechangenum_left;
				app->braohui=1;
				return -1;
			}
			else
			{
				app->secondDecison = "右换道";
				app->left_right = 1;
				count_ob1 = 0;
				count_ob2 = 0;
				if(app->bGpsGanrao)
					app->obb_left = false;
				else
					app->obb_left = true;
				app->lanechangenum = lanechangenum_right;
				app->braohui=0;
				return 1;
			}
		}
		else if(y_right_front[0]>-1000)
		{
			app->secondDecison = "左换道";
			app->left_right = -1;
			count_ob1 = 0;
			count_ob2 = 0;
			if(app->bGpsGanrao)
				app->obb_right = false;
			else
				app->obb_right = true;
			app->lanechangenum = lanechangenum_left;
			app->braohui=1;
			return -1;
		}
		else
		{
			app->secondDecison = "右换道";
			app->left_right = 1;
			count_ob1 = 0;
			count_ob2 = 0;
			if(app->bGpsGanrao)
				app->obb_left = false;
			else
				app->obb_left = true;
			app->lanechangenum = lanechangenum_right;
			app->braohui=0;
			return 1;
		}
	}
	else if (ob&&ob_left&&!ob_right)
	{
		app->secondDecison = "右换道";
		app->left_right = 1;
		count_ob1 = 0;
		count_ob2 = 0;
		if(app->bGpsGanrao)
			app->obb_left = false;
		else
			app->obb_left = true;
		app->lanechangenum = lanechangenum_right;
		app->braohui=0;
		return 1;
	}
	else if (ob&&!ob_left)
	{
		app->secondDecison = "左换道";
		app->left_right = -1;
		count_ob1 = 0;
		count_ob2 = 0;
		if(app->bGpsGanrao)
			app->obb_right = false;
		else
			app->obb_right = true;
		app->lanechangenum = lanechangenum_left;
		app->braohui=1;
		return -1;
	}
	else if (ob&&ob_left&&ob_right)
	{
		/*
		if(!bturnrightreq)
		{
			{
				for(int i=6;i>=6;i--)
				{
					MoveLeft(waypoint,exe_leftpoint,0.53*i);
					ob_left = SearchObstacle1_lanechange(exe_leftpoint,vel_Map,8,18,28,up+lanemovedist*6,-25,left_dis);
					if(!ob_left)
					{
						app->secondDecison = "左换道";
						app->left_right = -1;
						count_ob1 = 0;
						count_ob2 = 0;
						app->obb_right = true;
						app->obb_left = false;
						app->lanechangenum = i;
						return -1;
					}
				}
			}
			{
				for(int i=6;i>=6;i--)
				{
					MoveLeft(waypoint,exe_rightpoint,-0.53*i);
					ob_right = SearchObstacle1_lanechange(exe_rightpoint,vel_Map,8,18,28,up+lanemovedist*6,-25,right_dis);
					if(!ob_right)
					{
						app->secondDecison = "右换道";
						app->left_right = 1;
						count_ob1 = 0;
						count_ob2 = 0;
						app->obb_left = true;
						app->obb_right = false;
						app->lanechangenum = i;
						return 1;
					}
				}
			}
		}
		else
		{
			{
				for(int i=6;i>=6;i--)
				{
					MoveLeft(waypoint,exe_rightpoint,-0.53*i);
					ob_right = SearchObstacle1_lanechange(exe_rightpoint,vel_Map,8,18,28,up+lanemovedist*6,-25,right_dis);
					if(!ob_right)
					{
						app->secondDecison = "右换道";
						app->left_right = 1;
						count_ob1 = 0;
						count_ob2 = 0;
						app->obb_left = true;
						app->obb_right = false;
						app->lanechangenum = i;
						return 1;
					}
				}
			}
			{
				for(int i=6;i>=6;i--)
				{
					MoveLeft(waypoint,exe_leftpoint,0.53*i);
					ob_left = SearchObstacle1_lanechange(exe_leftpoint,vel_Map,8,18,28,up+lanemovedist*6,-25,left_dis);
					if(!ob_left)
					{
						app->secondDecison = "左换道";
						app->left_right = -1;
						count_ob1 = 0;
						count_ob2 = 0;
						app->obb_right = true;
						app->obb_left = false;
						app->lanechangenum = i;
						return -1;
					}
				}
			}
		}
		*/
		if (middle_dis<20)
		{
			if(app->bjiedaochaoche!=1)
				return 4;
			else
				return 10;
		}
		return 2;
	}
	else
	{
		count_ob1 = 0;
		count_ob2 = 0;
		return 0;
	}
}
double CLaneDriver::SearchFrontOb(I_Map *map,double range)//正前方range米内有无障碍
{
	int x = 256;
	int y = 411;
	for(int m=-1;m>-range*5;m--)
		for(int n=-4/*6*/;n<=4/*6*/;n++)
		{	
			if(y+m>511||y+m<0||x+n>511||x+n<0)
				continue;
			if(map->MapPoint[y+m][x+n] == 8 ||map->MapPoint[y+m][x+n] == 18  ||map->MapPoint[y+m][x+n] == 28)
			{
				return abs(m/5.0);

			}
		}

		return 0;

}
double CLaneDriver::SearchRearOb(I_Map *map,double range)//正hou方range米内有无障碍
{
	int x = 256;
	int y = 411;
	for(int m=1;m<range*2;m++)
		for(int n=-4/*6*/;n<=4/*6*/;n++)
		{	
			if(y+m>511||y+m<0||x+n>511||x+n<0)
				continue;
			if(map->MapPoint[y+m][x+n] == 8 ||map->MapPoint[y+m][x+n] == 18  ||map->MapPoint[y+m][x+n] == 28)
			{
				return abs(m/2.0);

			}
		}

		return 0;

}
double CLaneDriver::SearchRearObForApa(I_Map *map,double range)//正hou方range米内有无障碍
{
	int x = 256;
	int y = 411;
	for(int m=1;m<range*10;m++)
		for(int n=-3/*6*/;n<=3/*6*/;n++)
		{	
			if(y+m>511||y+m<0||x+n>511||x+n<0)
				continue;
			if(map->MapPoint[y+m][x+n] == 8 ||map->MapPoint[y+m][x+n] == 18  ||map->MapPoint[y+m][x+n] == 28)
			{
				return abs(m/10.0);

			}
		}

		return 0;

}
double CLaneDriver::SearchrightFrontOb(I_Map *map,double range)//you前方range米内有无障碍
{
	int x = 256;
	int y = 411;
	for(int m=-1;m>-3*5;m--)//for(int m=-1;m>-range*5;m--)
		for(int n=-4;n<=19;n++)
		{	
			if(y+m>511||y+m<0||x+n>511||x+n<0)
				continue;
			if(map->MapPoint[y+m][x+n] == 8 ||map->MapPoint[y+m][x+n] == 18  ||map->MapPoint[y+m][x+n] == 28)
			{
				return abs(m/5.0);
			}
		}

	x = 256;
	y = 396;
	int path_width = 4;
	double radius=5.3256;

	for(int m=-1;m>-(range-3)*5;m--)
	{
		for(int n=-path_width;n<=path_width;n++)
		{	
			double radiustmp=radius;
			if(0)
				radiustmp=radiustmp+n*0.2;
			else
				radiustmp=radiustmp-n*0.2;
			double angle=abs(m*0.2)/radiustmp;
			int xtmp=x+n+radiustmp*(1-cos(angle))*5;
			if(0)
				xtmp=x+n-radiustmp*(1-cos(angle))*5;
			int ytmp=y-radiustmp*sin(angle)*5;

			if(ytmp>511||ytmp<0||xtmp>511||xtmp<0)
				continue;
			if(map->MapPoint[ytmp][xtmp] == 8 || map->MapPoint[ytmp][xtmp] == 18 || map->MapPoint[ytmp][xtmp] == 28)
			{
				return abs((m-15)/5.0);
			}
		}
	}
	
	return 0;

}
double CLaneDriver::SearchleftFrontOb(I_Map *map,double range)//you前方range米内有无障碍
{
	int x = 256;
	int y = 411;
	for(int m=-1;m>-3*5;m--)//for(int m=-1;m>-range*5;m--)
		for(int n=-19;n<=4;n++)
		{	
			if(y+m>511||y+m<0||x+n>511||x+n<0)
				continue;
			if(map->MapPoint[y+m][x+n] == 8 ||map->MapPoint[y+m][x+n] == 18  ||map->MapPoint[y+m][x+n] == 28)
			{
				return abs(m/5.0);
			}
		}

	x = 256;
	y = 396;
	int path_width = 4;
	double radius=5.3256;

	for(int m=-1;m>-(range-3)*5;m--)
	{
		for(int n=-path_width;n<=path_width;n++)
		{	
			double radiustmp=radius;
			if(1)
				radiustmp=radiustmp+n*0.2;
			else
				radiustmp=radiustmp-n*0.2;
			double angle=abs(m*0.2)/radiustmp;
			int xtmp=x+n+radiustmp*(1-cos(angle))*5;
			if(1)
				xtmp=x+n-radiustmp*(1-cos(angle))*5;
			int ytmp=y-radiustmp*sin(angle)*5;

			if(ytmp>511||ytmp<0||xtmp>511||xtmp<0)
				continue;
			if(map->MapPoint[ytmp][xtmp] == 8 || map->MapPoint[ytmp][xtmp] == 18 || map->MapPoint[ytmp][xtmp] == 28)
			{
				return abs((m-15)/5.0);
			}
		}
	}
	
	return 0;

}
int CLaneDriver::SearchkongbaiFrontOb(I_Map *map,double range)//you前方range米内有无障碍
{
	int x = 256;
	int y = 411;

	bool obn[39];
	for(int i=0;i<39;i++)
		obn[i]=0;

	int mini=0;

	for(int m=-1;m>-range*5;m--)
	{
		for(int n=-19;n<=19;n++)
		{
			if(y+m>511||y+m<0||x+n>511||x+n<0)
				continue;
			if(map->MapPoint[y+m][x+n] == 8 || map->MapPoint[y+m][x+n] == 18  || map->MapPoint[y+m][x+n] == 28)
			{
				obn[n+19]=1;
				if(mini==0)
					mini=m;
			}
		}
		if(mini-m>=3)
			break;
	}

	int obfoundflag=0;
	int noblast=-1000;
	int nleftgap=-1000;
	int nrightgap=-1000;
	int nleftstrt=-1000;
	int nleftend=-1000;
	int nrightstrt=-1000;
	int nrightend=-1000;

	for(int n=-19;n<=19;n++)
	{
		if(obn[n+19])
		{
			if(obfoundflag==0)
			{
				if(n+19>15)
				{
					nleftgap=n+19;
					obfoundflag=2;
					nleftstrt=-19;
					nleftend=n;
				}
				else
				{
					obfoundflag=1;
				}
			}
			else if(obfoundflag==1)
			{
				if(n-noblast>15)
				{
					nleftgap=n-noblast;
					obfoundflag=2;
					nleftstrt=noblast;
					nleftend=n;
				}
			}
			else if(obfoundflag==2)
			{
				if(n-noblast>15)
				{
					nrightgap=n-noblast;
					obfoundflag=3;
					nrightstrt=noblast;
					nrightend=n;
				}
			}
			noblast=n;
		}
	}

	if(noblast>-1000 && nrightgap==-1000)
	{
		if(19-noblast>15)
		{
			nrightgap=19-noblast;
			nrightstrt=noblast;
			nrightend=19;
		}
	}

	bool bobrightnear=(SearchrightFrontOb(map,5)>0);
	bool bobleftnear=(SearchleftFrontOb(map,5)>0);

	int retvalue=0;
	
	if(nleftgap>-1000 && nrightgap>-1000)
	{
		if(bobrightnear && bobleftnear)
			retvalue = 0;
		else if((!bobleftnear)&&bobrightnear)
		{
			if((nleftstrt+nleftend)/2<=-4)
				retvalue = 1;
			else if((nleftstrt+nleftend)/2>=4)
				retvalue = 2;
			else
				retvalue = 0;
		}
		else if((!bobrightnear)&&bobleftnear)
		{
			if((nrightstrt+nrightend)/2<=-4)
				retvalue = 1;
			else if((nrightstrt+nrightend)/2>=4)
				retvalue = 2;
			else
				retvalue = 0;
		}
		else
		{
			if(nleftgap>=nrightgap)
			{
				if((nleftstrt+nleftend)/2<=-4)
					retvalue = 1;
				else if((nleftstrt+nleftend)/2>=4)
					retvalue = 2;
				else
					retvalue = 0;
			}
			else
			{
				if((nrightstrt+nrightend)/2<=-4)
					retvalue = 1;
				else if((nrightstrt+nrightend)/2>=4)
					retvalue = 2;
				else
					retvalue = 0;
			}
		}
	}
	else if(nleftgap>-1000)
	{
		if((nleftstrt+nleftend)/2<=-4)
			retvalue = 1;
		else if((nleftstrt+nleftend)/2>=4)
			retvalue = 2;
		else
			retvalue = 0;
	}
	else if(nrightgap>-1000)
	{
		if((nrightstrt+nrightend)/2<=-4)
			retvalue = 1;
		else if((nrightstrt+nrightend)/2>=4)
			retvalue = 2;
		else
			retvalue = 0;
	}
	else
		retvalue = 0;

	if(retvalue==1 && bobleftnear)
		retvalue = 0;
	if(retvalue==2 && bobrightnear)
		retvalue = 0;

	return retvalue;

}


int CLaneDriver::SearchkongbaiFrontObnew(CvPoint2D64f GPSpoint[],I_Map *map,double range)//you前方range米内有无障碍
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	app->critical_section.Lock();
	realtime_Gps = app->GPS_Point;
	realtime_Dir = app->GPS_Direction;
	app->critical_section.Unlock();

	double len = 0;
	double min_len = 99999;
	CvPoint2D64f strt;
	CvPoint2D64f goal;
	int nn = 0;
	for(int i = 0;i<200;i++)
	{
		len = m_GpsData.GetDistance(realtime_Gps.x,realtime_Gps.y,GPSpoint[i].x,GPSpoint[i].y);
		if(len < min_len)
		{
			min_len = len;
			nn = i;
		}
	}
	int t=199;
	len = 0;
	for(int i = nn;i<199;i++)
	{
		len += m_GpsData.GetDistance(GPSpoint[i].x,GPSpoint[i].y,GPSpoint[i+1].x,GPSpoint[i+1].y);

		if(len > 13)
		{
			t = i+1;
			break;
		}
	}
	strt = GPSpoint[nn];
	goal = GPSpoint[t];

	double angleroad=m_GpsData.GetAngle(goal,strt);

	int x = 256;
	int y = 412;

	bool obn[39];
	for(int i=0;i<39;i++)
		obn[i]=0;

	int mini=0;

	for(int m=-1;m>-range*5;m--)
	{
		for(int n=-19;n<=19;n++)
		{
			CvPoint2D64f pointgps=m_GpsData.MaptoGPS(realtime_Gps,angleroad,cvPoint2D64f(256+n,412+m));
			CvPoint2D64f pointmap=m_GpsData.APiontConverD(realtime_Gps,pointgps,realtime_Dir);

			int xmap=(int)(pointmap.x);
			int ymap=(int)(pointmap.y);

			if(ymap>511||ymap<0||xmap>511||xmap<0)
				continue;
			if(map->MapPoint[ymap][xmap] == 8 || map->MapPoint[ymap][xmap] == 18  || map->MapPoint[ymap][xmap] == 28)
			{
				obn[n+19]=1;
				if(mini==0)
					mini=m;
			}
		}
		if(mini-m>=3)
			break;
	}

	int obfoundflag=0;
	int noblast=-1000;
	int nleftgap=-1000;
	int nrightgap=-1000;
	int nleftstrt=-1000;
	int nleftend=-1000;
	int nrightstrt=-1000;
	int nrightend=-1000;

	for(int n=-19;n<=19;n++)
	{
		if(obn[n+19])
		{
			if(obfoundflag==0)
			{
				if(n+19>15)
				{
					nleftgap=n+19;
					obfoundflag=2;
					nleftstrt=-19;
					nleftend=n;
				}
				else
				{
					obfoundflag=1;
				}
			}
			else if(obfoundflag==1)
			{
				if(n-noblast>15)
				{
					nleftgap=n-noblast;
					obfoundflag=2;
					nleftstrt=noblast;
					nleftend=n;
				}
			}
			else if(obfoundflag==2)
			{
				if(n-noblast>15)
				{
					nrightgap=n-noblast;
					obfoundflag=3;
					nrightstrt=noblast;
					nrightend=n;
				}
			}
			noblast=n;
		}
	}

	if(noblast>-1000 && nrightgap==-1000)
	{
		if(19-noblast>15)
		{
			nrightgap=19-noblast;
			nrightstrt=noblast;
			nrightend=19;
		}
	}

	bool bobrightnear=(SearchrightFrontOb(map,5)>0);
	bool bobleftnear=(SearchleftFrontOb(map,5)>0);

	int retvalue=0;
	
	if(nleftgap>-1000 && nrightgap>-1000)
	{
		if(bobrightnear && bobleftnear)
			retvalue = 0;
		else if((!bobleftnear)&&bobrightnear)
		{
			if((nleftstrt+nleftend)/2<=-4)
				retvalue = 1;
			else if((nleftstrt+nleftend)/2>=4)
				retvalue = 2;
			else
				retvalue = 0;
		}
		else if((!bobrightnear)&&bobleftnear)
		{
			if((nrightstrt+nrightend)/2<=-4)
				retvalue = 1;
			else if((nrightstrt+nrightend)/2>=4)
				retvalue = 2;
			else
				retvalue = 0;
		}
		else
		{
			if(nleftgap>=nrightgap)
			{
				if((nleftstrt+nleftend)/2<=-4)
					retvalue = 1;
				else if((nleftstrt+nleftend)/2>=4)
					retvalue = 2;
				else
					retvalue = 0;
			}
			else
			{
				if((nrightstrt+nrightend)/2<=-4)
					retvalue = 1;
				else if((nrightstrt+nrightend)/2>=4)
					retvalue = 2;
				else
					retvalue = 0;
			}
		}
	}
	else if(nleftgap>-1000)
	{
		if((nleftstrt+nleftend)/2<=-4)
			retvalue = 1;
		else if((nleftstrt+nleftend)/2>=4)
			retvalue = 2;
		else
			retvalue = 0;
	}
	else if(nrightgap>-1000)
	{
		if((nrightstrt+nrightend)/2<=-4)
			retvalue = 1;
		else if((nrightstrt+nrightend)/2>=4)
			retvalue = 2;
		else
			retvalue = 0;
	}
	else
		retvalue = 0;

	if(retvalue==1 && bobleftnear)
		retvalue = 0;
	if(retvalue==2 && bobrightnear)
		retvalue = 0;

	return retvalue;

}

double CLaneDriver::SearchFrontTurnVehOb(I_Map *map,double range)//正前方range米内有无障碍
{
	int x = 256;
	int y = 411;

	int nleft=0;
	int nright=0;

	for(int m=-1;m>-range*5;m--)
		for(int n=-4;n<=4;n++)
		{	
			if(y+m>511||y+m<0||x+n>511||x+n<0)
				continue;
			if(map->MapPoint[y+m][x+n] == 8 ||map->MapPoint[y+m][x+n] == 18  ||map->MapPoint[y+m][x+n] == 28)
			{
				for(int nn=0;nn<=50;nn++)
				{
					for(int mm=-1;mm>=-5;mm--)
					{
						if((!(y+m+mm>511||y+m+mm<0||x+n+nn>511||x+n+nn<0))&&(nn-nright<8))
						{
							if(map->MapPoint[y+m+mm][x+n+nn] == 8 ||map->MapPoint[y+m+mm][x+n+nn] == 18  ||map->MapPoint[y+m+mm][x+n+nn] == 28)
							{
								nright=nn;
							}
						}
						if((!(y+m+mm>511||y+m+mm<0||x+n-nn>511||x+n-nn<0))&&(nn+nleft<8))
						{
							if(map->MapPoint[y+m+mm][x+n-nn] == 8 ||map->MapPoint[y+m+mm][x+n-nn] == 18  ||map->MapPoint[y+m+mm][x+n-nn] == 28)
							{
								nleft=-nn;
							}
						}
					}
				}
				if(nright-nleft<32)
					return abs(m/5.0);
				else
					return 0;
			}
		}

		return 0;

}
int CLaneDriver::SearchNiXing(CvPoint2D64f *waypoint)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	int ob = 0;
	bool ob_left=false;
	bool left_yellow = false;
	bool ob_right=false;
	CString str;
	//for(int i = 0;i<512;i++)
	//	for(int j = 0;j<512;j++)
	//	{
	//		if(vel_Map->MapPoint[i][j] == 10)
	//		{
	//			str.Format("%d,%d",i,j);
	//			AfxMessageBox(str);
	//			return 0;
	//		}

	//	}
	ob = vel_Map->MapPoint[254][0];
	out129<<ob<<"  "<<vel_Map->MapPoint[254][0]<<endl;
	if(ob==10)
	{
		MoveLeft(waypoint,exe_leftpoint,-3.7);
		ob_right = SearchObstacle(exe_leftpoint,map_ts,8,28,38,50,-10);
		if(!ob_right)
		{
				app->secondDecison = "右换道";
				app->left_right = -1;
				return 1;
		}
		else
		{
			return 0;
		}	
	}
	else if(ob==1)
	{
		MoveLeft(waypoint,exe_leftpoint,3.7);
		ob_right = SearchObstacle(exe_leftpoint,map_ts,8,28,38,50,-10);
		if(!ob_right)
		{
				app->secondDecison = "左换道";
				app->left_right = 1;
				return -1;
		}
		else
		{
			return 0;
		}	
	}
	return 0;
}
int CLaneDriver::SearchObstacle(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down)
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
		for(int m=-10;m<10;m++)
			for(int n=-6;n<7;n++)
			{
				/*if(y+m>380&&y+m<430)
					continue;*/
				if((down==0)&&(y+m>=412))
					continue;
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
				{
					if (a==b && b==c && c==28)
					{
						if (map_v->MapPoint[y+m][x+n] < app->GPS_Speed+1)
							return 1;
					}
					else
						return 1;
					
				}
			}
	}
	
	return false;
}
int CLaneDriver::SearchObstacle_laneyichang(CvPoint2D64f GPSpoint[],I_Map *map)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_up = 412-35*5;
	int m_down = 411;
	
	int x = 0;
	int y = 0;
	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x ;
		y = Rndf_MapPoint[i].y;
		for(int m=-5;m<6;m++)
			for(int n=-5;n<6;n++)
			{
				if(y+m>m_down||y+m<m_up||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == 8 ||map->MapPoint[y+m][x+n] == 18 ||map->MapPoint[y+m][x+n] == 28)
				{
					return 1;
				}
			}
	}
	
	return false;
}

int CLaneDriver::SearchObstacle_lanenotusedcheck(CvPoint2D64f GPSpoint[],I_Map *map,double lengtherr)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_up = 412-80*5;
	int m_down = 411+10*2;

	int nleft=-7;
	int nright=7;
	if(lengtherr>0)
	{
		if(nright<lengtherr*5+5)
			nright=lengtherr*5+5;
	}
	if(lengtherr<0)
	{
		if(nleft>lengtherr*5-5)
			nleft=lengtherr*5-5;
	}
	
	int x = 0;
	int y = 0;
	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x ;
		y = Rndf_MapPoint[i].y;
		for(int m=-5;m<6;m++)
			for(int n=nleft;n<=nright;n++)
			{
				if(y+m>m_down||y+m<m_up||x+n>511||x+n<0)
					continue;
				if((n<-7||n>7)&&(y+m>412 || y+m<312))
					continue;
				if(map->MapPoint[y+m][x+n] == 8 ||map->MapPoint[y+m][x+n] == 18 ||map->MapPoint[y+m][x+n] == 28)
				{
					return 1;
				}
			}
	}
	
	return false;
}

int CLaneDriver::SearchObstacle1(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	int width2 = 4;
	if (app->range_flag)
	{
		width2 = 4;
	}
	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(int m=-9;m<10;m++)
			for(int n=-width2;n<=width2;n++)
			{
				if((down==0)&&(y+m>=412))
					continue;
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if((map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b)&&(y+m>462))
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b)
				{
					ob_y = abs(412-(y+m))*0.2;
					return 1;
				}
				if(map->MapPoint[y+m][x+n] == c)
				{
					if(y+m<412)
					{
						if(dynamicmap_v_x->MapPoint[y+m][x+n]<10/3.6 || y+m>237)
						{
							ob_y = abs(412-(y+m))*0.2;
							return 1;
						}
					}
					else
					{
						if(dynamicmap_v_x->MapPoint[y+m][x+n]>-10/3.6 || y+m<462)
						{
							ob_y = abs(412-(y+m))*0.2;
							return 1;
						}
					}
				}
			}
	}
	
	return false;
}
int CLaneDriver::SearchObstacle1_frontnear(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	int width2 = 4;
	int m;
	int n;

	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(m=4;m>=-4;m--)
			for(n=-1;n<=1;n++)
			{
				y = Rndf_MapPoint[i].y;

				if(y+m>=412||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
				{
					y=y+m;
					int ob_count=0;
					for(n=-6;n<=6;n++)
					{
						for(m=6;m>=-6;m--)
						{
							if(y+m>=412||y+m<0||x+n>511||x+n<0)
								continue;
							if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
							{
								ob_count++;
								if(ob_count>=6)
									return 1;
								else
									break;
							}
						}
					}
					return false;
				}
			}
	}
	
	return false;
}
int CLaneDriver::SearchObstacle1_leftnear(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	int width2 = 6;
	if (app->range_flag)
	{
		width2 = 4;
	}
	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(int m=-5;m<6;m++)
			for(int n=-22;n<=0;n++)
			{
				if(y+m>511||y+m<0||x+n>511||x+n<0||(n>-7 && y+m<397))
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
				{
					ob_y = abs(412-(y+m))*0.2;
					return 1;
				}
			}
	}
	
	return false;
}

int CLaneDriver::SearchObstacle1_leftnear_new(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c, int upnew,int up,int down,double &ob_y)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_down = 412 -down*5;

	int m_upnew = 412-upnew*5;
	
	int x = 0;
	int y = 0;

	int width2 = 6;
	if (app->range_flag)
	{
		width2 = 4;
	}
	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(int m=-5;m<6;m++)
			for(int n=-22;n<=0;n++)
			{
				if(y+m>511||y+m<0||x+n>511||x+n<0||(n>-7 && y+m<397))
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
				{
					ob_y = abs(412-(y+m))*0.2;
					return 1;
				}
			}
	}

	int ytop=0;
	int ybottom=0;
	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_upnew)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		int nleft=-1000;
		int nright=-1000;
		for(int m=-5;m<6;m++)
		{
			for(int n=-22;n<=0;n++)
			{
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
				{
					if(nleft==-1000)
					{
						nleft=n;
						nright=n;
					}
					if(n<nleft)
						nleft=n;
					if(n>nright)
						nright=n;
				}
			}
		}
		if(nleft>-14 && nright<-4)
		{
			if(ybottom==0)
			{
				ybottom=Rndf_MapPoint[i].y;
				ytop=Rndf_MapPoint[i].y;
			}
			else
				ytop=Rndf_MapPoint[i].y;
			if(abs(ytop-ybottom)*0.2>3)
			{
				ob_y = abs(412-ybottom)*0.2;
				return 1;
			}
		}
		else if(nleft>-1000 || abs(ytop-Rndf_MapPoint[i].y)*0.2>5 || nright>=-4)
		{
			ybottom=0;
			ytop=0;
		}
	}
	
	return false;
}

int CLaneDriver::SearchObstacle1_leftlaneobdist(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
		return 0;
	}
	int m_up = 412-up*5;
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	int width2 = 6;
	if (app->range_flag)
	{
		width2 = 4;
	}

	for(int n=-18;n>=-28;n--)
	{
		for(int i = 0;i<200;i++)
		{
			if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
				continue;
			x = Rndf_MapPoint[i].x;
			y = Rndf_MapPoint[i].y;
			for(int m=-5;m<6;m++)
			{
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
				{
					ob_y = abs(412-(y+m))*0.2;
					return n;
				}
			}
		}
	}
	
	return 0;
}

int CLaneDriver::SearchObstacle1_rightlaneobdist(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
		return 0;
	}
	int m_up = 412-up*5;
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	int width2 = 6;
	if (app->range_flag)
	{
		width2 = 4;
	}
	for(int n=18;n<=28;n++)
	{
		for(int i = 0;i<200;i++)
		{
			if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
				continue;
			x = Rndf_MapPoint[i].x;
			y = Rndf_MapPoint[i].y;
			for(int m=-5;m<6;m++)
			{
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
				{
					ob_y = abs(412-(y+m))*0.2;
					return n;
				}
			}
		}
	}
	
	return 0;
}

int CLaneDriver::SearchObstacle1_rightnear(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	int width2 = 6;
	if (app->range_flag)
	{
		width2 = 4;
	}
	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(int m=-5;m<6;m++)
			for(int n=0;n<=22;n++)
			{
				if(y+m>511||y+m<0||x+n>511||x+n<0||(n<7 && y+m<397))
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
				{
					ob_y = abs(412-(y+m))*0.2;
					return 1;
				}
			}
	}
	
	return false;
}

int CLaneDriver::SearchObstacle1_rightnear_new(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int upnew,int up,int down/*,int &ob_x*/,double &ob_y)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_down = 412 -down*5;

	int m_upnew = 412-upnew*5;
	
	int x = 0;
	int y = 0;

	int width2 = 6;
	if (app->range_flag)
	{
		width2 = 4;
	}
	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(int m=-5;m<6;m++)
			for(int n=0;n<=22;n++)
			{
				if(y+m>511||y+m<0||x+n>511||x+n<0||(n<7 && y+m<397))
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
				{
					ob_y = abs(412-(y+m))*0.2;
					return 1;
				}
			}
	}

	int ytop=0;
	int ybottom=0;
	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_upnew)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		int nleft=-1000;
		int nright=-1000;
		for(int m=-5;m<6;m++)
		{
			for(int n=0;n<=22;n++)
			{
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
				{
					if(nleft==-1000)
					{
						nleft=n;
						nright=n;
					}
					if(n<nleft)
						nleft=n;
					if(n>nright)
						nright=n;
				}
			}
		}
		if(nleft>4 && nright<14)
		{
			if(ybottom==0)
			{
				ybottom=Rndf_MapPoint[i].y;
				ytop=Rndf_MapPoint[i].y;
			}
			else
				ytop=Rndf_MapPoint[i].y;
			if(abs(ytop-ybottom)*0.2>3)
			{
				ob_y = abs(412-ybottom)*0.2;
				return 1;
			}
		}
		else if(nleft>-1000 || abs(ytop-Rndf_MapPoint[i].y)*0.2>5 || nleft<=4)
		{
			ybottom=0;
			ytop=0;
		}
	}
	
	return false;
}
int CLaneDriver::SearchObstacle1_leftnextlane(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down,double &ob_y)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	int width2 = 6;
	if (app->range_flag)
	{
		width2 = 4;
	}
	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(int m=-5;m<6;m++)
			for(int n=-13;n<=0;n++)
			{
				if((down>=0)&&(y+m>=412))
					continue;
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
				{
					ob_y = abs(n)*0.2;
					return 1;
				}
			}
	}
	
	return false;
}
int CLaneDriver::SearchObstacle1_rightnextlane(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down,double &ob_y)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	int width2 = 6;
	if (app->range_flag)
	{
		width2 = 4;
	}
	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(int m=-5;m<6;m++)
			for(int n=0;n<=13;n++)
			{
				if((down>=0)&&(y+m>=412))
					continue;
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
				{
					ob_y = abs(n)*0.2;
					return 1;
				}
			}
	}
	
	return false;
}

void CLaneDriver::SearchObstacle1_lchgconcheck(CvPoint2D64f GPSpoint[],I_Map *map,double &ob_y_front,double &ob_y_rear)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;
	app->critical_section.Unlock();

	int n = 0;
	double len = 0;
	double min_len = 99999;

	CvPoint2D64f rndfbezierchadian[1991];
	CvPoint2D64f Rndf_MapPointchadian[1991];

	ob_y_front=-10000;
	ob_y_rear=-10000;

	for(int i=0;i<200;i++)
	{
		len = m_GpsData.GetDistance(m_gps.x,m_gps.y,GPSpoint[i].x,GPSpoint[i].y);
		if(len < min_len)
		{
			min_len = len;
			n = i;
		}
	}
	if(n==0)
		n=1;

	int chadiannum=0;
	for(int i=0;i<199;i++)
	{
		for(int kk=0;kk<10;kk++)
		{
			rndfbezierchadian[chadiannum].x=(GPSpoint[i].x*(10-kk)+GPSpoint[i+1].x*kk)*0.1;
			rndfbezierchadian[chadiannum].y=(GPSpoint[i].y*(10-kk)+GPSpoint[i+1].y*kk)*0.1;
			chadiannum++;
		}
	}
	rndfbezierchadian[chadiannum]=GPSpoint[199];

	n=(n-1)*10;

	//ob_dis_prepre=ob_dis_pre;
	//ob_dis_pre=ob_dis_now;

	for(int i=0;i<1991;i++)
	{
		Rndf_MapPointchadian[i] = m_GpsData.APiontConverD(m_gps,rndfbezierchadian[i],m_gpsdir);
		if(m_gps.x==rndfbezierchadian[i].x&&m_gps.y==rndfbezierchadian[i].y)
		{
			Rndf_MapPointchadian[i].x = 256;
			Rndf_MapPointchadian[i].y = 411;
		}
	}
	
	int x = 0;
	int y = 0;

	bool breakflag=0;

	for(int i = n;i<1991;i++)
	{
		if(Rndf_MapPointchadian[i].x < 0||Rndf_MapPointchadian[i].x > 511||Rndf_MapPointchadian[i].y > 511||Rndf_MapPointchadian[i].y < 0)
			continue;
		x = Rndf_MapPointchadian[i].x;
		y = Rndf_MapPointchadian[i].y;
		for(int m=5;m>=-5;m--)
		{
			for(int n=-5;n<=5;n++)
			{
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == 8 || map->MapPoint[y+m][x+n] == 18 || map->MapPoint[y+m][x+n] == 28)
				{
					if(y+m<=412)
						ob_y_front = abs(412-(y+m))*0.2;
					else
						ob_y_front=sqrt(double((x+n-256)*(x+n-256)+(y+m-412)*(y+m-412)))*0.2;
					breakflag = 1;
					break;
				}
			}
			if(breakflag)
				break;
		}
		if(breakflag)
			break;
	}

	x = 0;
	y = 0;
	bool breakflag2=0;

	for(int i = n;i>=0;i--)
	{
		if(Rndf_MapPointchadian[i].x < 0||Rndf_MapPointchadian[i].x > 511||Rndf_MapPointchadian[i].y > 511||Rndf_MapPointchadian[i].y < 0)
			continue;
		x = Rndf_MapPointchadian[i].x;
		y = Rndf_MapPointchadian[i].y;
		for(int m=-4;m<=4;m++)
		{
			for(int n=-5;n<=5;n++)
			{
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == 8 || map->MapPoint[y+m][x+n] == 18 || map->MapPoint[y+m][x+n] == 28)
				{
					if(y+m>=412)
						ob_y_rear = abs(412-(y+m))*0.5;
					else
						ob_y_rear=sqrt(double((x+n-256)*(x+n-256)+(y+m-412)*(y+m-412)))*0.2;
					breakflag2 = 1;
					break;
				}
			}
			if(breakflag2)
				break;
		}
		if(breakflag2)
			break;
	}
}

int CLaneDriver::SearchObstacle1_lanechange(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	int width2 = 5;//4

	int ob=0;
	int obcount=0;

	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		obcount=0;
		for(int m=6;m>=-6;m--)
			for(int n=-width2;n<=5;n++)
			{
				if(y+m>=412||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||((map->MapPoint[y+m][x+n] == c)&&(dynamicmap_v_x->MapPoint[y+m][x+n]<10/3.6 || y+m>312)))
				{
					obcount++;
					if(ob==0)
					{
						ob_y = abs(412-(y+m))*0.2;
						ob=2;
					}
					if(obcount>10)
						return 1;
				}
			}
	}
	
	return ob;
}
int CLaneDriver::SearchObstacle1_vehlanechange(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	int width2 = 4;

	int ob=0;
	int obcount=0;

	int i_max=0;
	int i_min=0;

	bool b_obfind=0;
	bool bcheckflag=0;
	bool bwitdhenough=0;

	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		obcount=0;
		bool btempflag=0;
		for(int n=-width2;n<=width2;n++)
		{
			bcheckflag=0;
			for(int m=6;m>=-6;m--)
			{
				if(y+m>=412||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||((map->MapPoint[y+m][x+n] == c)&&(dynamicmap_v_x->MapPoint[y+m][x+n]<10/3.6 || y+m>312)))
				{
					btempflag=1;
					if(b_obfind==0)
					{
						b_obfind=1;
						i_max=y+m;
						i_min=y+m;
					}
					else
					{
						if(y+m<i_min)
							i_min=y+m;
					}
					if(bcheckflag==0)
					{
						bcheckflag=1;
						obcount++;
					}
					if(obcount>6)//人为4-5，此阈值最多设7
						bwitdhenough=1;
					if(bwitdhenough)
					{
						//app->ob_width_111=obcount;
						//obwidth_out<<obcount<<endl;
						return 1;
					}
					/*
					if((i_max-i_min>7)&&bwitdhenough)
						return 1;
						*/
					/*
					obcount++;
					if(ob==0)
					{
						ob_y = abs(412-(y+m))*0.2;
						ob=2;
					}
					if(obcount>10)
						return 1;
						*/
				}
				/*
				if(b_obfind)
				{
					if(i_max-(y+m)>30)
						return 5;
				}
				*/
			}
		}
		if(btempflag)
		{
			//obwidth_out<<obcount<<endl;
			//app->ob_width_111=obcount;
		}
	}
	
	if(b_obfind)
	{
		//app->ob_width_111=obcount;
		//obwidth_out<<obcount<<endl;

		width2 = 6;

		ob=0;
		obcount=0;

		i_max=0;
		i_min=0;

		b_obfind=0;
		bcheckflag=0;
		bwitdhenough=0;

		for(int i = 0;i<200;i++)
		{
			if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
				continue;
			x = Rndf_MapPoint[i].x;
			y = Rndf_MapPoint[i].y;
			obcount=0;
			for(int n=-width2;n<=width2;n++)
			{
				bcheckflag=0;
				for(int m=6;m>=-6;m--)
				{
					if(y+m>=412||y+m<0||x+n>511||x+n<0)
						continue;
					if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||((map->MapPoint[y+m][x+n] == c)&&(dynamicmap_v_x->MapPoint[y+m][x+n]<10/3.6 || y+m>312)))
					{
						if(bcheckflag==0)
						{
							bcheckflag=1;
							obcount++;
						}
						if(obcount>6)//人为4-5，此阈值最多设7
							return 1;
					}
				}
			}
		}

		return 5;
	}
	else
		return 0;
}
int CLaneDriver::SearchObstacle1_vehlanechange111(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	int width2 = 4;

	int ob=0;
	int obcount=0;

	int i_max=0;
	int i_min=0;

	bool b_obfind=0;
	bool bcheckflag=0;
	bool bwitdhenough=0;
	bool bmiddledis=0;

	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(int n=-width2;n<=width2;n++)
		{
			for(int m=6;m>=-6;m--)
			{
				if(y+m>=412||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||((map->MapPoint[y+m][x+n] == c)&&(dynamicmap_v_x->MapPoint[y+m][x+n]<10/3.6 || y+m>312)))
				{
					b_obfind=1;
					if(bmiddledis==0)
					{
						bmiddledis=1;
						ob_y=abs((412-y+m)*0.2);
					}
				}
				if(map->MapPoint[y+m][x+n] == 38)
				{
					if(bmiddledis==0)
					{
						bmiddledis=1;
						ob_y=abs((412-y+m)*0.2);
					}
					return 5;
				}
			}
		}
	}
	
	if(b_obfind)
	{
		return 1;
	}
	else
		return 0;
}
int CLaneDriver::SearchObstacle1_lanechangenot(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	int width2 = 4;

	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(int m=4;m>=-4;m--)
			for(int n=-1;n<=1;n++)
			{
				if(y+m>=412||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||((map->MapPoint[y+m][x+n] == c)&&((y+m>412-15*5)||(dynamicmap_v_x->MapPoint[y+m][x+n]<-3/3.6))))
				{
					return 1;
				}
			}
	}
	
	return 0;
}
int CLaneDriver::SearchObstacle1_lanechange_left(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	int width2 = 5;

	int n_left=-17;
	if(app->GPS_Speed*3.6<=45)
		n_left=-5;//-7//-6

	if(app->laneturndir==1)
	{
		if(app->GPS_Speed*3.6<=45)
			n_left=-6;//7
	}
	n_left=-5;

	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(int m=-6;m<=6;m++)
			for(int n=n_left;n<=width2;n++)
			{
				if((down==0)&&(y+m>=412))
					continue;
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				/*
				if((map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b)&&(y+m>472))
					continue;
					*/
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
				{
					ob_y = abs(412-(y+m))*0.2;
					return 1;
				}
				/*
				if(map->MapPoint[y+m][x+n] == c)
				{
					if(y+m<412)
					{
						return 1;
					}
					else
					{
						if(dynamicmap_v_x->MapPoint[y+m][x+n]>-10/3.6 || y+m<472)
						{
							ob_y = abs(412-(y+m))*0.2;
							return 1;
						}
					}
				}
				*/
			}
	}
	
	return false;
}
int CLaneDriver::SearchObstacle1_lanechange_mid(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(int m=-5;m<=5;m++)
			for(int n=-5;n<=5;n++)
			{
				if((down==0)&&(y+m>=412))
					continue;
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
				{
					ob_y = abs(412-(y+m))*0.2;
					return 1;
				}
			}
	}
	
	return false;
}
int CLaneDriver::SearchObstacle1_VelMapPath_left(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down,int &ob_hx_left)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	int width2 = 6;

	int ob_hx_left_min = 20;

	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(int m=-6;m<=6;m++)
			for(int n=-1;n>=-14;n--)//-13
			{
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
				{
					if((y+m>372 && y+m<422)||(y+m<372 && ((spdrelleftfront<2/3.6)||(abs(y+m-412)*0.2<y_left_front[0]-5)||(y_left_front[0]<-1000)))||(y+m>422 && spdrelleftrear>5/3.6))
					{
						int ob_hx_tmp = -n;
						if(ob_hx_tmp<ob_hx_left_min)
							ob_hx_left_min=ob_hx_tmp;
					}
				}
			}
	}

	if(ob_hx_left_min<20)
	{
		ob_hx_left=ob_hx_left_min;
		return true;
	}
	else
		return false;
}
int CLaneDriver::SearchObstacle1_VelMapPath_right(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down,int &ob_hx_left)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	int width2 = 6;

	int ob_hx_left_min = 20;

	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(int m=-6;m<=6;m++)
			for(int n=1;n<=14;n++)//13
			{
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
				{
					if((y+m>372 && y+m<422)||(y+m<372 && ((spdrelrightfront<2/3.6)||(abs(y+m-412)*0.2<y_right_front[0]-5)||(y_right_front[0]<-1000)))||(y+m>422 && spdrelrightrear>5/3.6))
					{
						int ob_hx_tmp = n;
						if(ob_hx_tmp<ob_hx_left_min)
							ob_hx_left_min=ob_hx_tmp;
					}
				}
			}
	}

	if(ob_hx_left_min<20)
	{
		ob_hx_left=ob_hx_left_min;
		return true;
	}
	else
		return false;
}
int CLaneDriver::SearchObstacle1_VelMapPath_leftini(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down,int &ob_hx_left)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	int width2 = 6;

	int ob_hx_left_min = 20;

	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(int m=-6;m<=6;m++)
			for(int n=-1;n>=-13;n--)
			{
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
				{
					int ob_hx_tmp = -n;
					if(ob_hx_tmp<ob_hx_left_min)
						ob_hx_left_min=ob_hx_tmp;
				}
			}
	}

	if(ob_hx_left_min<20)
	{
		ob_hx_left=ob_hx_left_min;
		return true;
	}
	else
		return false;
}
int CLaneDriver::SearchObstacle1_VelMapPath_rightini(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down,int &ob_hx_left)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	int width2 = 6;

	int ob_hx_left_min = 20;

	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(int m=-6;m<=6;m++)
			for(int n=1;n<=13;n++)
			{
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
				{
					int ob_hx_tmp = n;
					if(ob_hx_tmp<ob_hx_left_min)
						ob_hx_left_min=ob_hx_tmp;
				}
			}
	}

	if(ob_hx_left_min<20)
	{
		ob_hx_left=ob_hx_left_min;
		return true;
	}
	else
		return false;
}
int CLaneDriver::SearchObstacle1_lanechange_right(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	int width2 = 5;

	int n_right=9;//17
	if(app->GPS_Speed*3.6<=45)
		n_right=6;//7
	if(app->laneturndir==2)
	{
		if(app->GPS_Speed*3.6<=45)
			n_right=6;//7
	}

	n_right=5;

	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(int m=-6;m<=6;m++)
			for(int n=-width2;n<=n_right;n++)
			{
				if((down==0)&&(y+m>=412))
					continue;
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				/*
				if((map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b)&&(y+m>472))
					continue;
					*/
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
				{
					ob_y = abs(412-(y+m))*0.2;
					return 1;
				}
				/*
				if(map->MapPoint[y+m][x+n] == c)
				{
					if(y+m<412)
					{
						return 1;
					}
					else
					{
						if(dynamicmap_v_x->MapPoint[y+m][x+n]>-10/3.6 || y+m<472)
						{
							ob_y = abs(412-(y+m))*0.2;
							return 1;
						}
					}
				}
				*/
			}
	}
	
	return false;
}
int CLaneDriver::SearchObstacle_leftedge(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	int ob_num=0;
	bool flag=0;

	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		flag=0;
		for(int m=-4;m<5;m++)
		{
			for(int n=-18;n<=-10;n++)
			{
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
				{
					ob_num++;
					flag=1;
					if(ob_num>100)
					{
						ob_y = abs(412-(y+m))*0.2;
						return 1;
					}
				}
				if(flag)
					break;
			}
			if(flag)
				break;
		}
	}
	
	return false;
}
int CLaneDriver::SearchObstacle_rightedge(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(int m=-4;m<5;m++)
			for(int n=10;n<=42;n++)
			{
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
				{
					ob_y = abs(412-(y+m))*0.2;
					return 1;
				}
			}
	}
	
	return false;
}
int CLaneDriver::SearchObstacle_leftedge_rightside(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(int m=-4;m<5;m++)
			for(int n=-42;n<=-10;n++)
			{
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
				{
					ob_y = abs(412-(y+m))*0.2;
					return 1;
				}
			}
	}
	
	return false;
}
int CLaneDriver::SearchObstacle_rightedge_rightside(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	int ob_num=0;
	bool flag=0;

	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		flag=0;
		for(int m=-4;m<5;m++)
		{
			for(int n=10;n<=18;n++)
			{
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
				{
					ob_num++;
					flag=1;
					if(ob_num>100)
					{
						ob_y = abs(412-(y+m))*0.2;
						return 1;
					}
				}
				if(flag)
					break;
			}
			if(flag)
				break;
		}
	}
	
	return false;
}

int CLaneDriver::SearchObstacle111(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y)
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
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	int width2 = 4;
	if (app->range_flag)
	{
		width2 = 4;
	}
	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x ;//2012.7.18xiugai -10
		y = Rndf_MapPoint[i].y;
		for(int m=-4;m<5;m++)
			for(int n=-width2;n<=width2;n++)
			{
				if((down==0)&&(y+m>=412))
					continue;
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(y+m<412)
					continue;
				if((map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b)&&(y+m>362)&&(y+m<424))
				{
					ob_y = abs(412-(y+m))*0.2;
					return 1;
				}
				//if((map->MapPoint[y+m][x+n] == c)&&(y+m>337)&&(dynamicmap_v_x->MapPoint[y+m][x+n]>-20/3.6 || y+m<472))
				if((map->MapPoint[y+m][x+n] == c)&&(y+m>362)&&(dynamicmap_v_x->MapPoint[y+m][x+n]>5/3.6 || y+m<424))//2/3.6
				{
					ob_y = abs(412-(y+m))*0.2;
					return 1;
				}
			}
	}
	
	return false;
}

int CLaneDriver::SearchObstacle222(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down/*,int &ob_x*/,double &ob_y)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	int width2 = 4;
	if (app->range_flag)
	{
		width2 = 4;
	}
	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(int m=-4;m<5;m++)
			for(int n=-width2;n<=width2;n++)
			{
				if((down==0)&&(y+m>=412))
					continue;
				if(y+m<412)
					continue;
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if((map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)&&(y+m<424)&&(y+m>362))
				{
					ob_y = abs(412-(y+m))*0.2;
					return 1;
				}
			}
	}
	
	return false;
}

int CLaneDriver::SearchObstacle2(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down,double &ob_y, int dir)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();
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
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;

	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		for(int m=-9;m<10;m++)
			for(int n=-5;n<6;n++)
			{
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if (dir==1)
				{
					if (y+m > 387)
					{
						if(x+n<250)
						{
							if(map->MapPoint[y+m][x+n] == c && dynamicmap_v_x->MapPoint[y+m][x+n]>5/3.6)
							{
								return 2;
							}
						}
						if(y+m<422 && n>=-4 && n<=4)
						{
							if(map->MapPoint[y+m][x+n] == a || map->MapPoint[y+m][x+n] == b || map->MapPoint[y+m][x+n] == c)
							{
								return 2;
							}
						}
					}
				}

				if (dir==2)
				{
					if (y+m > 387)
					{
						if(x+n>262)
						{
							if(map->MapPoint[y+m][x+n] == c && dynamicmap_v_x->MapPoint[y+m][x+n]>5/3.6)
							{
								return 2;
							}
						}
						if(y+m<422 && n>=-4 && n<=4)
						{
							if(map->MapPoint[y+m][x+n] == a || map->MapPoint[y+m][x+n] == b || map->MapPoint[y+m][x+n] == c)
							{
								return 2;
							}
						}
					}
				}
			}
	}
	
	return false;
}

int CLaneDriver::SearchObstacle(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down,double w)
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
	int width = w*5;
	int x = 0;
	int y = 0;
	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x ;//2012.7.18xiugai -10
		y = Rndf_MapPoint[i].y;
		for(int m=-10;m<10;m++)
			for(int n=-width;n<width;n++)
			{
				/*if(y+m>380&&y+m<430)
					continue;*/
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
				{
					if (a==b && b==c && c==28)
					{
						if (map_v->MapPoint[y+m][x+n] < app->GPS_Speed+1)
							return 1;
					}
					else
						return 1;
					
				}
			}
	}
	
	return false;
}
int CLaneDriver::SearchObstacleLuyan(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down)
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
		for(int m=-10;m<10;m++)
			for(int n=-5;n<6;n++)
			{
				/*if(y+m>380&&y+m<430)
				continue;*/
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b||map->MapPoint[y+m][x+n] == c)
				{
					return 1;
				}
			}
	}

	return false;
}
int CLaneDriver::SearchObstacleMoveStill(CvPoint2D64f GPSpoint[],I_Map *map,int a,int b,int c,int up,int down,int &ob_x,int &ob_y)
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
	int m_down = 412 -down*5;
	
	int x = 0;
	int y = 0;
	int width_ob = 6;
	if(app->range_flag)
	{
		width_ob = 4;
	}
	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x ;//2012.7.18xiugai -10
		y = Rndf_MapPoint[i].y;
		for(int m=-9;m<10;m++)
			for(int n=-width_ob;n<=width_ob;n++)
			{
				/*if(y+m>380&&y+m<430)
					continue;*/
				if((down==0)&&(y+m>=412))
					continue;
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == c)
				{
					ob_y = y+m;
					ob_x = x+n;
					//return 2;
					return 1;
				}
				if(map->MapPoint[y+m][x+n] == a||map->MapPoint[y+m][x+n] == b)
				{
				
					{
						ob_y = abs(412-(y+m))*0.2;
						return 1;
					}
				}
			}
	}
	
	return 0;
}
int CLaneDriver::GetMinDisPointGPS(CvPoint2D64f *path,int num,CvPoint2D64f m_gps)
{
	double len;
	double min_len = 99999;
	int n = 0;
	for(int i = 0;i<num;i++)
	{
		len = m_GpsData.GetDistance(m_gps.x,m_gps.y,path[i].x,path[i].y);
		if(len < min_len)
		{
			min_len = len;
			n = i;
		}
	}
	return n;
}
bool CLaneDriver::SearchObstacleLukou(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	app->critical_section.Lock();//锁住
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;

	app->critical_section.Unlock();
	int ob_num = 0;
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
		for(int m=-10;m<10;m++)
			for(int n=-6;n<6;n++)
			{
				if(y+m>380&&y+m<430)
					continue;
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == 8||map->MapPoint[y+m][x+n] == 18)
				{
					
					ob_num++;
					if(ob_num > 2)
						return true;

				}
			}
	}
	return false;
}

bool CLaneDriver::SearchPeople(CvPoint2D64f GPSpoint[],I_Map *map,int width,int up,int &dis)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	dis = 200;
	/*for(int y=411;y>411-up*5;y--)
	{
		for(int x=255-width*5;x<255+width;x++)
		{
			if(map->MapPoint[y][x] == 58)	
			{
				dis = sqrt(double(x*x+y*y))/5;
				return 1;
			}


		}
		
	}
	*/
	

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

	int m_up = 412-up*5;
	int m_down = 411 ;

	int x = 0;
	int y = 0;
	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > m_down||Rndf_MapPoint[i].y < m_up)
			continue;
		x = Rndf_MapPoint[i].x ;//2012.7.18xiugai -10
		y = Rndf_MapPoint[i].y;
		for(int m=-7;m<7;m++)
			for(int n=-width*5;n<width*5;n++)
			{

				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if((map->MapPoint[y+m][x+n] == 58)||(map->MapPoint[y+m][x+n] == 68))
				{
					dis = sqrt(double((x-256)*(x-256)+(y-411)*(y-411)))/5;
					return 1;

				}
			}
	}
	return false;

}
void CLaneDriver::GetBSpline1(LEAD *m_vgpspoints,int gpssize, CvPoint2D64f *curve)
{
	double *Road_Point_lat = new double[gpssize+2];
	double *Road_Point_lan = new double[gpssize+2];
	for(int k=0;k<gpssize;k++)
	{
		Road_Point_lat[k+1] = m_vgpspoints[k].lat;
		Road_Point_lan[k+1] = m_vgpspoints[k].lng;
	}
	Road_Point_lat[0] = m_vgpspoints[0].lat;
	Road_Point_lan[0] = m_vgpspoints[0].lng;
	Road_Point_lat[gpssize+1] = m_vgpspoints[gpssize-1].lat;
	Road_Point_lan[gpssize+1] = m_vgpspoints[gpssize-1].lng;

	//初始化cagd类参数
	CCAGD_NURBS cagd;
	cagd.Get_Control_Point(gpssize+2,Road_Point_lat,Road_Point_lan);//输入多少个控制顶点，全部输入
	delete[] Road_Point_lat;
	delete[] Road_Point_lan;

	cagd.BSpline_k = 3;//三次样条曲线
	cagd.BSpline_Type = 3;//非均匀B样条

	int outputsize = 50;//每段取点个数
	int All_Point = (cagd.Control_Point_Num - cagd.BSpline_k) * outputsize + 1;//比如15个点，三次样条的话，每段50个点，共有(15-3)*50+1
	//用于输出所有的插值点
	double *output_x = new double[All_Point];
	double *output_y = new double[All_Point];
	double arfa = 0.5;//曲线松弛因子
	//得到所有的插值点
	cagd.Get_Insert_Point(arfa,outputsize,All_Point,output_x,output_y);
	for (int j =0;j<All_Point-1;j++)
	{
		curve[j].x = output_x[j];
		curve[j].y = output_y[j];
	}
	delete[] output_x;
	delete[] output_y;
}