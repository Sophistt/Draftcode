#include "StdAfx.h"
#include "DynamicObFuse.h"

DynamicObFuse::DynamicObFuse(void)
{
	current_map=new I_Map;
	current_map_v_x=new v_Map;
	current_map_v_y=new v_Map;
}

DynamicObFuse::~DynamicObFuse(void)
{
}

bool DynamicObFuse::IbeoExpansion2( CvPoint2D64f box_points[4],CvPoint2D64f point,int length,int width )
{
	double dis1,dis2,dis3,dis4,discha;
	double a,b,c;
	
	a=(box_points[0].y-box_points[1].y)/(box_points[0].x-box_points[1].x);
	b=-1;
	c=box_points[0].y-(box_points[0].x*(box_points[0].y-box_points[1].y)/(box_points[0].x-box_points[1].x));
	dis1 = (abs(a*point.x+b*point.y+c))/(sqrt(a*a+b*b));

	a=(box_points[1].y-box_points[2].y)/(box_points[1].x-box_points[2].x);
	b=-1;
	c=box_points[1].y-(box_points[1].x*(box_points[1].y-box_points[2].y)/(box_points[1].x-box_points[2].x));
	dis2 = (abs(a*point.x+b*point.y+c))/(sqrt(a*a+b*b));

	a=(box_points[2].y-box_points[3].y)/(box_points[2].x-box_points[3].x);
	b=-1;
	c=box_points[2].y-(box_points[2].x*(box_points[2].y-box_points[3].y)/(box_points[2].x-box_points[3].x));
	dis3 = (abs(a*point.x+b*point.y+c))/(sqrt(a*a+b*b));

	a=(box_points[3].y-box_points[0].y)/(box_points[3].x-box_points[0].x);
	b=-1;
	c=box_points[3].y-(box_points[3].x*(box_points[3].y-box_points[0].y)/(box_points[3].x-box_points[0].x));
	dis4 = (abs(a*point.x+b*point.y+c))/(sqrt(a*a+b*b));

	discha = abs(dis1+dis2+dis3+dis4-length-width);

	if (discha<=DELTA)
	{
		return 1;
	}
	else
	{
		return 0;
	}

}

void DynamicObFuse::SetVehicle( double current_dir,CvPoint2D64f current_gps,I_Map *map)
{
	vehicle_dir=current_dir;
	vehicle_gps=current_gps;
	current_map=map;

}

void DynamicObFuse::UpDatelist( I_Map *map,ObjectWT *ibobject,int ibnum)
{
	cluster.Init(map,ibobject,ibnum);
	cluster.Clear();
	vector<DyObstacle>templist;
	if (dylist.size()==0)
	{
		for (int i=0; i<cluster.obclusters.size(); i++)
		{
			CvPoint2D64f p[2000]={0};
			CvPoint2D64f p_gps[2000];
			DyObstacle dy_ob;
			CvPoint2D64f obcenter;
			CvPoint2D64f obcenter_gps;
			int obsize=0;

			cluster.obclusters[i].GetPoint(p);//获取栅格图上的障碍物块坐标点
			obsize=cluster.obclusters[i].GetSize();//获取栅格地图上障碍物块大小
			dy_ob.SetMapPos(p);
			dy_ob.SetArea(obsize);//设置障碍物大小
			for (int k=0; k<obsize; k++)
			{
				p_gps[k]=Map2Gps(vehicle_gps,vehicle_dir,p[k]);
			}
			obcenter=cluster.obclusters[i].GetCenter();
			obcenter_gps=Map2Gps(vehicle_gps,vehicle_dir,obcenter);
			
			dy_ob.SetPos(p_gps);//设置位置
			//dy_ob.SetPos(p);
		
			dy_ob.SetCenter(obcenter_gps);//设置中心点位置
			dy_ob.centerlist.push(obcenter_gps);
			dy_ob.SetV(0);//速度
			dy_ob.SetMovingDir(0);//方向
			dy_ob.Setdirchange(0);//方向改变次数
			dy_ob.SetExistConfidence(1);//存在置信度
			dy_ob.SetObNum(dy_obstalce_num);
			dy_ob.xmin=cluster.obclusters[i].xmin;
			dy_ob.xmax=cluster.obclusters[i].xmax;
			dy_ob.ymin=cluster.obclusters[i].ymin;
			dy_ob.ymax=cluster.obclusters[i].ymax;
			dylist.push_back(dy_ob);
			dy_obstalce_num++;
		}
	}
	else
	{
		NDiff diff[DIFFX][DIFFY];                                  //用来存储列表障碍物与图中障碍物中的距离
		for (int i=0; i<DIFFX; i++)
		{
			for (int j=0; j<DIFFY; j++)
			{
				diff[i][j].d=100;
				diff[i][j].visited=0;
			}
		}

		for (int i=0; i<cluster.obclusters.size() && i<DIFFX; i++)
		{
			CvPoint2D64f obcenter;
			CvPoint2D64f obcenter_gps;
			DyObstacle newob;
			CvPoint2D64f obpoint[2000]={0};
			CvPoint2D64f obpoint_gps[2000]={0};
			cluster.obclusters[i].GetPoint(obpoint);

			for (int j=0; j<cluster.obclusters[i].GetSize(); j++)
			{
				obpoint_gps[j]=Map2Gps(vehicle_gps,vehicle_dir,obpoint[j]);
			}

			obcenter=cluster.obclusters[i].GetCenter();
			obcenter_gps=Map2Gps(vehicle_gps,vehicle_dir,obcenter);
			newob.SetPos(obpoint_gps);
			//newob.SetPos(obpoint);
			newob.SetMapPos(obpoint);
			newob.SetCenter(obcenter_gps);
			newob.SetArea(cluster.obclusters[i].GetSize());
			int pos=0;
			while(pos<dylist.size() && pos<DIFFY)                             //与障碍物列表中的每一个障碍物对比，存储差异值
			{	

				DyObstacle listob=dylist[pos];                 //获取第pos个障碍物
				diff[i][pos].d=MatchDyobstacle(newob,listob);                    //返回的是两个物体中心点的距离
				pos++;		
			}
		}
		/************************************************************************/
		/* 进行障碍物匹配                                                                     */
		/************************************************************************/
		
		DyObstacle tempobstacle;
		double smallest;
		int small_cluster=0;
		int small_list=0;
		int list_match[200];                                        //存储链表中已经匹配上物体
		for (int l=0; l<200; l++)
		{
			list_match[l]=-1;
		}
		int ob_match[200];                                          //存储已经匹配上的障碍物序号
		for (int m=0; m<200; m++)
		{
			ob_match[m]=-1;
		}
		int lmatch_num=0;                                          //存储已经匹配上的列表障碍物数目
		int omatch_num=0;                                          //存储已经匹配上的障碍物数目

		while(1)
		{
			smallest=100;
			for (int i=0; i<cluster.obclusters.size() && i<DIFFX; i++)
			{
				for (int j=0; j<dylist.size() && j<DIFFY; j++)
				{
					if (diff[i][j].visited==0&&smallest>diff[i][j].d)
					{
						smallest=diff[i][j].d;
						small_cluster=i;
						small_list=j;
					}
				}
			}

			if (smallest>10)                            //如果最小距离大于设定值，停止匹配
			{
				break;
			}
			for (int mk=0; mk<cluster.cluster_num; mk++)
			{
				diff[mk][small_list].visited=1;
			}
			for (int nk=0; nk<dylist.size(); nk++)
			{
				diff[small_cluster][nk].visited=1;
			}

			CvPoint2D64f obcenter=cluster.obclusters[small_cluster].GetCenter();
			CvPoint2D64f listobcenter=dylist[small_list].GetCenter();
			obcenter=Map2Gps(vehicle_gps,vehicle_dir,obcenter);
			tempobstacle.SetCenter(obcenter);//更新中心位置
			tempobstacle.centerlist=dylist[small_list].centerlist;
		
			
			
			CvPoint2D64f point[2000]={0};
			cluster.obclusters[small_cluster].GetPoint(point);
			tempobstacle.SetMapPos(point);

			for (int j=0; j<cluster.obclusters[small_cluster].GetSize(); j++)
			{
				point[j]=Map2Gps(vehicle_gps,vehicle_dir,point[j]);
			}

			tempobstacle.SetPos(point);//更新位置
			CvPoint2D64f tcenterpoint=tempobstacle.centerlist.front();
			double dist=GetDistance(tcenterpoint.x,tcenterpoint.y,obcenter.x,obcenter.y);
			double v=dylist[small_list].GetV();

			double longerdis=0;

			if (tempobstacle.centerlist.size()<10)
			{
				longerdis=dist/(tempobstacle.centerlist.size());
			}
			if (tempobstacle.centerlist.size()==10)
			{
				longerdis=dist/10;
			}

			//ASSERT(v<30);
			double see_v=/*GetDistance(listobcenter.x,listobcenter.y,obcenter.x,obcenter.y)*/longerdis*3;
			//ASSERT(see_v<30);

			if (v==0)
			{
				v=see_v;
			}
			else
			{
				if (v-see_v>0.2||v-see_v<-0.2)
				{
					v=v+0.1*(see_v-v);
				}
				else
				{
					v=v*0.7+see_v*0.3;
				}

			}
			//ASSERT(v<30);
			
			
			
			
			tempobstacle.centerlist.push(obcenter);
			if (tempobstacle.centerlist.size()>10)
			{
				tempobstacle.centerlist.pop();
			}
			
			double dir=GetAngle(obcenter,tcenterpoint);
			double predir=dylist[small_list].GetMovingDir();
			tempobstacle.SetMovingDir(dir);//更新运动方向
			double dirchange=dir-predir;
			if (dirchange>360)
			{
				dirchange=dirchange-360;
			}
			if (dirchange<-360)
			{
				dirchange=dirchange+360;
			}
			tempobstacle.dir_change=dirchange;
			if (dirchange>45||dirchange<-45)
			{
				tempobstacle.Setdirchange(1);//增加方向改变次数
			}
			tempobstacle.SetArea(cluster.obclusters[small_cluster].GetSize());
			double past_v=tempobstacle.GetV();
			//ASSERT(v<30);
			tempobstacle.SetV(v);//更新速度
			tempobstacle.v_change=v-past_v;
			int moving=tempobstacle.GetMovingConfidence();
			if (abs(tempobstacle.v_change)<1.5)
			{
				tempobstacle.SetMovingConfidence(moving+1);
			}
			else
			{
				tempobstacle.SetMovingConfidence(0);
			}
			tempobstacle.SetObNum(dylist[small_list].GetObNum());
			tempobstacle.xmin=cluster.obclusters[small_cluster].xmin;
			tempobstacle.xmax=cluster.obclusters[small_cluster].xmax;
			tempobstacle.ymin=cluster.obclusters[small_cluster].ymin;
			tempobstacle.ymax=cluster.obclusters[small_cluster].ymax;
			int exist=dylist[small_list].GetExistConfidence();
			tempobstacle.SetExistConfidence(exist+1);//更新存在时间
/************************************************************************/
/* 初始化与更新滤波器                                                                     */
/************************************************************************/
			double v_x=(obcenter.x-listobcenter.x)/0.25;
			double v_y=(obcenter.y-listobcenter.y)/0.25;
			/*if (tempobstacle.fileter.m_count==0)
			{
				tempobstacle.fileter.Filter(obcenter.x,obcenter.y,v_x, v_y);
			}
			else
			{
				tempobstacle.fileter.Filter(obcenter.x,obcenter.y);
			}*/

			tempobstacle.disapear=0;
			tempobstacle.showp=0;
			templist.push_back(tempobstacle);  //将障碍物添加到列表中
			list_match[lmatch_num]=small_list;
			lmatch_num++;
			ob_match[omatch_num]=small_cluster;
			omatch_num++;
		}

		/************************************************************************/
		/* 将新发现的动态障碍物加入列表                                                                     */
		/************************************************************************/
		for (int i=0; i<cluster.obclusters.size(); i++)
		{
			bool pipei=0;
			for (int j=0; j<omatch_num; j++)
			{
				if (ob_match[j]==i)
				{
					pipei=1;
					break;
				}
			}

			if (pipei==0)
			{	
				DyObstacle newobstacle;
				CvPoint2D64f temp[2000]={0};
				CvPoint2D64f temp_gps[2000]={0};
				cluster.obclusters[i].GetPoint(temp);
				newobstacle.SetMapPos(temp);

				for (int mm=0; mm<cluster.obclusters[i].GetSize(); mm++)
				{
					temp_gps[mm]=Map2Gps(vehicle_gps,vehicle_dir,temp[mm]);
				}

				CvPoint2D64f center=cluster.obclusters[i].GetCenter();
				center=Map2Gps(vehicle_gps,vehicle_dir,center);
				newobstacle.SetPos(temp_gps);
				//newobstacle.SetPos(temp);
				//newobstacle.SetArea(cluster.obclusters[i].GetSize());
				newobstacle.SetCenter(center);
				newobstacle.centerlist.push(center);
				newobstacle.SetArea(cluster.obclusters[i].GetSize());
				newobstacle.SetObNum(dy_obstalce_num);
				newobstacle.SetExistConfidence(1);
				newobstacle.showp=0;//匹配上将showp置为0
				newobstacle.disapear=0;
				newobstacle.SetV(0);
				newobstacle.SetMovingDir(0);
				newobstacle.Setdirchange(0);
				newobstacle.xmin=cluster.obclusters[i].xmin;
				newobstacle.xmax=cluster.obclusters[i].xmax;
				newobstacle.ymin=cluster.obclusters[i].ymin;
				newobstacle.ymax=cluster.obclusters[i].ymax;
				templist.push_back(newobstacle);
				dy_obstalce_num++;
			}
		} 

		/************************************************************************/
		/* 对链表上未匹配上的障碍物，添加这一帧的信息                                                                    
		/************************************************************************/
		for (int i=0; i<dylist.size(); i++)
		{
			dylist[i].showp=0;
			int pipei=0; 
			for (int j=0; j<lmatch_num; j++)
			{
				if (list_match[j]==i)
				{
					pipei=1;
					break;
				}
			}
			if (pipei==0)
			{
				dylist[i].disapear=dylist[i].disapear+1;
				dylist[i].showp=1;
				if (dylist[i].disapear<4)
				{
					CvPoint2D64f ppp[2000]={0};
					dylist[i].SetMapPos(ppp);//如果当前帧没有匹配上将该障碍物块点清零，若消失次数小于4将该障碍物加入列表
					dylist[i].SetPos(ppp);
					//templist.push_back(dylist[i]);
				}
			}
		}

		dylist.clear();
		for (int kkk=0; kkk<templist.size();kkk++)
		{
			//if (dylist[kkk].disapear<4)
			{
				dylist.push_back(templist[kkk]);
			}
			
		}

	}//else结束
	

}


CvPoint2D64f DynamicObFuse::APiontConverD(CvPoint2D64f v,CvPoint2D64f a,double direction)
{
	double t = GetAngle(a,v);
	double m = direction - t +90;
	m = rad(m);
	double s = GetDistance(v.x,v.y,a.x,a.y);
	CvPoint2D64f p;
	p.x = 256 + (s * cos(m) * 5);  //*5换成20cm单位
	p.y = 412 - (s * sin(m) * 5);

	return p;

}

CvPoint2D64f DynamicObFuse::Map2Gps( CvPoint2D64f vehicle_gps,double vehicle_dir,CvPoint2D64f point )
{
	if (point.x == 256&&point.y == 412)
	{
		return vehicle_gps;
	}
	double xq1,yq1,x,y,sita,dire,xa,ya,x1,y1;
	double s1;	
	xq1 = point.x-256;
	yq1 = 412-point.y;	
	s1=sqrt(pow(xq1/5,2)+pow(yq1/5,2));
	sita=atan(xq1/yq1);
	if(yq1<0)
		sita += 3.1415926535897932;
	dire=sita+rad(vehicle_dir);
	ya=s1*sin(dire);
	xa=s1*cos(dire);
	x=vehicle_gps.x+(xa*180)/(6378137*3.1415926);
	y=vehicle_gps.y+(ya*180)/(6378137*3.1415926*cos(x*3.1415926/180));
	CvPoint2D64f c;
	c.x=x;
	c.y=y;
	return c;
}

double DynamicObFuse::MatchDyobstacle( DyObstacle mapob,DyObstacle listob )//如果方向不对，则
{
	CvPoint2D64f mapobcenter=mapob.GetCenter();
	CvPoint2D64f listobcenter=listob.GetCenter();
	double distance=GetDistance(mapobcenter.x,mapobcenter.y,listobcenter.x,listobcenter.y);
	double dir= GetAngle(mapobcenter,listobcenter);
	double moving_dir=listob.GetMovingDir();
	double dir_change=moving_dir-dir;
	if (dir_change<-360)
	{
		dir_change=360+dir_change;
	}
	if (dir_change>360)
	{
		dir_change=dir_change-360;
	}
	
	if (dir_change>90||dir_change<-90)
	{
		//distance+=8;
	}

	int list_size=listob.GetArea();
	int map_size=mapob.GetArea();
	if (abs(list_size-map_size)>15)
	{
		//distance+=1;
	}
	return distance;
}

double DynamicObFuse::GetDistance( double lat1, double lng1, double lat2, double lng2 )
{
	if(lat1 == lat2 && lng1 == lng2)
		return 0;
	double radLat1 = rad(lat1);
	double radLat2 = rad(lat2);
	double a = radLat1 - radLat2;
	double b = rad(lng1) - rad(lng2);
	double s = 2 * asin(sqrt(pow(sin(a/2),2) + 
		cos(radLat1)*cos(radLat2)*pow(sin(b/2),2)));
	s = s * 6378.137*1000;
	//s = round(s * 10000) / 10000;
	return s;
}

double DynamicObFuse::GetAngle( CvPoint2D64f apoint, CvPoint2D64f bpoint)
{
	double lat1 = apoint.x;
	double lng1 = apoint.y;
	double lat2 = bpoint.x;
	double lng2 = bpoint.y;
	double radLat1 = rad(lat1);
	double radLat2 = rad(lat2);
	double a = radLat1 - radLat2;
	double b = (rad(lng1) - rad(lng2))*cos(radLat1);
	double t=atan(b/a);
	if (a<0)
		t=t+3.1415926535897932;
	//t=PI/2-t;

	t = t * 180 / 3.1415926535897932;
	return t;

}

void DynamicObFuse::UpDateUselist()
{

	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	double uv=app->GPS_Speed;
	
	usefullist.clear();
	for (int mmm=0; mmm<dylist.size();mmm++)
	{
		if (dylist[mmm].GetV()*1.6>1.8&&dylist[mmm].showp==0)
		{
			usefullist.push_back(dylist[mmm]);
		}
	}
	int i=usefullist.size();
	CvPoint2D64f p[2000]={0};

	

	for(int k=0; k<i; k++)
	{
		DyObstacle obstacle=usefullist[k];
		obstacle.GetMapPos(p);
		int nn=obstacle.GetArea();
		CvPoint2D64f center=obstacle.GetCenter();
		center=APiontConverD(app->GPS_Point,center,app->GPS_Direction);

		if(obstacle.GetV()*1.6>=1.5)
		{
			for(int qq=0; qq<nn; qq++)
			{
				int x=p[qq].x;
				int y=p[qq].y;
				current_map->MapPoint[y][x]=28;
				current_map_v_x->MapPoint[y][x]=obstacle.GetV()*1.6-uv;
			}
		}
		else
		{
			for(int qq=0; qq<nn; qq++)
			{
				int x=p[qq].x;
				int y=p[qq].y;
				current_map->MapPoint[y][x]=8;
			}
		}
	}
}

void DynamicObFuse::SearchRLine( CvPoint2D64f (&line)[512],int &l_num )
{
	for (int i=450; i>0; i--)
	{
		for (int j=400; j>256; j--)
		{
			if (current_map->MapPoint[i][j]==18)
			{
				line[l_num].x=j;
				line[l_num].y=i;
				l_num++;
				break;
			}
		}
	}


}

void DynamicObFuse::SearchLLine( CvPoint2D64f (&line)[512],int &l_num )
{
	for (int i=450; i>0; i--)
	{
		for (int j=256; j>100; j--)
		{
			if (current_map->MapPoint[i][j]==18)
			{
				line[l_num].x=j;
				line[l_num].y=i;
				l_num++;
				break;
			}
		}
	}

}

void DynamicObFuse::CountLine( CvPoint2D64f *line,int l_num,double &k,double &b )
{
	double sumx=0;
	double sumy=0;
	double xmean=0;
	double ymean=0;
	double sumxx=0;
	double sumxy=0;

	for (int i=0; i<l_num; i++)
	{
		sumx+=line[i].x;
		sumy+=line[i].y;
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

void DynamicObFuse::ObInRoad()
{
	inroadlist.clear();
	bool r_IsVertical=0;
	bool l_IsVertical=0;
	/************************************************************************/
	/* 右侧路沿拟合                                                                     */
	/************************************************************************/
	CvPoint2D64f r_line[512]={0};
	int r_num=0;
	SearchRLine(r_line,r_num);
	if (r_num>30)
	{
		r_variancex=0;
		r_variancey=0;

		CountLine(r_line,r_num,m_rk,m_rb);
		for (int i=0; i<r_num; i++)
		{
			r_meanx+=r_line[i].x;
			r_meany+=r_line[i].y;
		}
		r_meanx=r_meanx/r_num;
		r_meany=r_meany/r_num;

		for (int i=0; i<r_num; i++)
		{
			r_variancex=(r_line[i].x-r_meanx)*(r_line[i].x-r_meanx);
			r_variancey=(r_line[i].y-r_meany)*(r_line[i].y-r_meany);
		}
		r_variancex=r_variancex/r_num;
		r_variancey=r_variancey/r_num;
	}

	double r_angle=atan(m_rk)*180/3.1415926535897932;
	if (r_angle<0)
	{
		r_angle=r_angle+180;
	}

	/************************************************************************/
	/* 左侧路沿拟合                                                               */
	/************************************************************************/
	CvPoint2D64f l_line[512]={0};
	int l_num=0;
	SearchLLine(l_line,l_num);
	ASSERT(l_num>=0&&l_num<=512);
	if (l_num>30)
	{
		l_variancex=0;
		l_variancey=0;
		CountLine(l_line,l_num,m_lk,m_lb);
		for (int i=0; i<l_num; i++)
		{
			l_meanx+=l_line[i].x;
			l_meany+=l_line[i].y;
		}
		l_meanx=l_meanx/l_num;
		l_meany=l_meany/l_num;

		for (int i=0; i<l_num; i++)
		{
			l_variancex=(l_line[i].x-l_meanx)*(l_line[i].x-l_meanx);
			l_variancey=(l_line[i].y-l_meany)*(l_line[i].y-l_meany);
		}
		l_variancex=l_variancex/l_num;
		l_variancey=l_variancey/l_num;
	}
	double l_angle=atan(m_lk)*180/3.1415926535897932;
	if (l_angle<0)
	{
		l_angle=l_angle+180;
	}

	for (int obnum=0; obnum<usefullist.size(); obnum++)
	{
		DyObstacle obstacle=usefullist[obnum];
		CvPoint2D64f center=obstacle.GetCenter();
		center=APiontConverD(vehicle_gps,center,vehicle_dir);
		int center_x=(center.x*10+5)/10;
		int center_y=(center.y*10+5)/10;

		if (r_variancex<3/*&&m_rk!=0*//*&&r_num>10*/)
		{
			r_IsVertical=1;
		}
		if (l_variancex<3/*&&m_lk!=0*//*&&l_num>10*/)
		{
			l_IsVertical=1;
		}

		if (r_IsVertical==1&&l_IsVertical==1)                //都垂直
		{
			if (center_x-l_meanx>10&&r_meanx-center_x>10)
			{
				inroadlist.push_back(obstacle);
			}
		}
		if (r_IsVertical==1&&l_IsVertical==0&&m_lk!=0)     // 右边垂直，左边不垂直，斜率不为零
		{
			if (r_meanx-center_x>10&&center_x-(center_y-m_lb)/m_lk>10)
			{
				inroadlist.push_back(obstacle);
			}
		}

		if (r_IsVertical==1&&l_IsVertical==0&&m_lk==0)    //右侧垂直，左边不垂直，斜率为零
		{
			if (r_meanx-center_x>10)
			{
				inroadlist.push_back(obstacle);
			}
		}


		if (r_IsVertical==0&&l_IsVertical==1&&m_rk!=0) //右边不垂直，斜率不为零，左边垂直
		{
			if ((center_y-m_rb)/m_rk-center_x>10&&center_x-l_meanx>10)
			{
				inroadlist.push_back(obstacle);
			}
		}

		if (r_IsVertical==0&&l_IsVertical==0&&m_rk!=0&&m_lk!=0)   //左右都不垂直，斜率都不为零
		{
			if ((center_y-m_rb)/m_rk-center_x>10&&center_x-(center_y-m_lb)/m_lk>10)
			{
				inroadlist.push_back(obstacle);
			}
		}
		if (r_IsVertical==0&&l_IsVertical==0&&m_rk==0&&m_lk!=0)   //左右都不垂直，右侧斜率为零，左侧不为零
		{
			if (center_x-(center_y-m_lb)/m_lk>10)
			{
				inroadlist.push_back(obstacle);
			}
		}

		if (r_IsVertical==0&&l_IsVertical==0&&m_rk!=0&&m_lk==0)  //左右都不垂直，右侧斜率不为零，左侧为零
		{
			if ((center_y-m_rb)/m_rk-center_x>10)
			{
				inroadlist.push_back(obstacle);
			}
		}

		if (r_IsVertical==0&&m_rk==0&&l_IsVertical==1)          //左边垂直，右边没有道路边沿
		{
			if (center_x-l_meanx>10)
			{
				inroadlist.push_back(obstacle);
			}
		}
	}

	

}


