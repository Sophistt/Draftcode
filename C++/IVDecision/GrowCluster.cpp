#include "StdAfx.h"
#include "GrowCluster.h"
#include <time.h>
#include <fstream>


int mycount=0;
GrowCluster::GrowCluster(void)
{
	m_bf=50;
	m_bh=450;
	m_lt=150;
	m_rt=320;
	m_width=-6;
	m_lengh=10;
}

GrowCluster::~GrowCluster(void)
{
}

void GrowCluster::Setdetect(double before,double behind,double left,double right)
{
	m_bf=412-(int)before/0.2;
	m_bh=412+(int)behind/0.2;
	m_lt=256-(int)left/0.2;
	m_rt=256+(int)right/0.2;
}

void GrowCluster::Setgrow(double width, double length)
{
	m_width=width;
	m_lengh=length;
}
void GrowCluster::Init(I_Map *map)
{
	//grow_time.Start();
	ObPoints.clear();
	obclusters.clear();
	obmap=new ObMap;                               //自定义障碍物地图
	for (int i=0; i<512; i++)                             //读入障碍物地图
	{
		for (int j=0; j<512; j++)
		{	
			obmap->obstacle[i][j].isob=map->MapPoint[i][j]; 
			obmap->obstacle[i][j].isreached=0;
			if (obmap->obstacle[i][j].isob==8/*||obmap->obstacle[i][j].isob==18*/)//将道路边沿和障碍物居委一类2013 6 15 10:28
			{
				obmap->obstacle[i][j].clusterId=-1;                     //把每一个障碍物点的类属性设定为-1
			}
			else obmap->obstacle[i][j].clusterId=-2;                    //非障碍物的类别标志设为-2
			obmap->obstacle[i][j].isreached=0;                          //把是否分类标定为0

		}
	}

	CList<Pos,Pos> open;                                                 //存储一类点坐标
	Pos mPos;
	
	int disA=m_width;
	int disB=m_lengh;
	cluster_num=0;
	for (int i=m_bf; i<m_bh; i++)                                           //搜索障碍点
	{
		for (int j=m_lt; j<m_rt; j++)
		{
			if (obmap->obstacle[i][j].clusterId==-1)                      //如果障碍物点未分类
			{
				obmap->obstacle[i][j].clusterId=cluster_num;              //设定类别序数
				for (int a=disA-10;a<disB+10;a++)                               //搜索范围可以设定
				{
					for (int b=disA; b<disB; b++)
					{
						if (i+a>=0&&i+a<512&&j+b>=0&&j+b<512&&obmap->obstacle[i+a][j+b].clusterId==-1)  //如果坐标点在图中，未分类
						{
							obmap->obstacle[i+a][j+b].clusterId=obmap->obstacle[i][j].clusterId;
                            mPos.x=j+b;
							mPos.y=i+a;
							open.AddTail(mPos);
						}
					}
				}
				cluster_num++;                                            //从0开始加
			}

			int x;
			int y;
			while(!open.IsEmpty())
			{
				Pos temp=open.GetTail();
				open.RemoveTail();
				x=temp.x;
				y=temp.y;
				
				for (int a=disA-10;a<disB+10;a++)                             //搜索范围
				{
					for (int b=disA; b<disB; b++)
					{
						if (y+a>=0&&y+a<512&&x+b>=0&&x+b<512&&obmap->obstacle[y+a][x+b].clusterId==-1)
						{
							obmap->obstacle[y+a][x+b].clusterId=obmap->obstacle[y][x].clusterId;
							mPos.x=x+b;
							mPos.y=y+a;
							open.AddTail(mPos);
						}
					}
				}

			}
		}
	}

	

	
	int num=0;
	for (int i=0; i<511; i++)
	{
		for (int j=0; j<511; j++)
		{
			if (obmap->obstacle[i][j].isob==8&&obmap->obstacle[i][j].clusterId>=0)             //将类别号大于等于0的都存储
			{
				ObPoint temp;
				temp.InPut(j,i,obmap->obstacle[i][j].clusterId);
				ObPoints.push_back(temp);
				num++;
			}
		}
	}
	
		ob_count=num;
		mycount++;


		
		for (int temp_index = 0; temp_index < cluster_num; temp_index++)
		{
			ObCluster temp_cluster;
			CvPoint2D64f temp_points[20000] = {0};
			CvPoint2D64f insert_points[2000] = {0};
			CvPoint2D64f sum_points;
			sum_points.x = 0;
			sum_points.y = 0;
			CvPoint2D64f temp_center;
			temp_center.x = 0;
			temp_center.y = 0;
			int temp_size = 0;
			int temp_flag = 0;
			for (int point_index = 0; point_index < ob_count; point_index++)
			{
				if (ObPoints[point_index].GetClusterId() == temp_index)
				{
					temp_points[temp_size].x = ObPoints[point_index].Getx();
					temp_points[temp_size].y = ObPoints[point_index].Gety();
					sum_points.x += temp_points[temp_size].x;
					sum_points.y += temp_points[temp_size].y;
					temp_size++;
					ASSERT(temp_size!=20000);
				}
			}

			xmin=511;
			xmax=0;
			ymin=511;
			ymax=0;
			for (int ym=0; ym<temp_size;ym++)
			{
				if (temp_points[ym].x<=xmin)
				{
					xmin=temp_points[ym].x;
				}
				if (temp_points[ym].x>=xmax)
				{
					xmax=temp_points[ym].x;
				}
				if (temp_points[ym].y <= ymin)
					ymin = temp_points[ym].y;
				if(temp_points[ym].y >= ymax)
					ymax = temp_points[ym].y;
			}

			temp_cluster.xmin=xmin;
			temp_cluster.xmax=xmax;
			temp_cluster.ymin=ymin;
			temp_cluster.ymax=ymax;


			if ( ymax-ymin <50&&xmax-xmin<50)
			{
				temp_center.x = sum_points.x/temp_size;
				temp_center.y = sum_points.y/temp_size;
				temp_cluster.SetCenter(temp_center);

				for (int temp_j = 0; temp_j < temp_size; temp_j++)
				{
					insert_points[temp_j] = temp_points[temp_j];
				}
				temp_cluster.SetPoint(insert_points);

				temp_cluster.SetSize(temp_size);
				if (temp_size>5)
				{
					obclusters.push_back(temp_cluster);
				}
				
			}			
			
			

			

		}

	
}

void GrowCluster::Init(I_Map *map,ObjectWT *ibcpWT, int ib_num)
{
	CvPoint2D64f ibcp[2000];
	int kk=0;

	//obmap=new ObMap;
	for (int i=0; i<512; i++)
	{
		for (int j=0; j<512; j++)
		{	
			obmap->obstacle[i][j].isob=map->MapPoint[i][j]; 
			obmap->obstacle[i][j].isreached=0;
			if (obmap->obstacle[i][j].isob==8)
			{
				obmap->obstacle[i][j].clusterId=-1;
			}
			else obmap->obstacle[i][j].clusterId=-2;
		}
	}
	CList<Pos,Pos> open;
	Pos mPos;
	cluster_num=0;
	
	int w=3;
	int l=20;

	for(int i=0; i<ib_num; i++)
	{
		if(ibcpWT[i].ObjectBox_CenterY<512&&ibcpWT[i].ObjectBox_CenterY>12&&ibcpWT[i].ObjectBox_CenterX<=284&&ibcpWT[i].ObjectBox_CenterX>=228)
		{
			ObPoints.clear();
			int ix=ibcpWT[i].ObjectBox_CenterX;
			int iy=ibcpWT[i].ObjectBox_CenterY;
			obmap->obstacle[iy][ix].clusterId=cluster_num;
			v_x_clusterId[cluster_num]=ibcpWT[i].Relative_VelocityX;
			v_y_clusterId[cluster_num]=ibcpWT[i].Relative_VelocityY;
			mPos.x=ix;
			mPos.y=iy;
			open.AddTail(mPos);
			ObPoint t1;
			t1.InPut(ix,iy,obmap->obstacle[iy][ix].clusterId);
			ObPoints.push_back(t1);
			for(int j=0;j<ibcpWT[i].iContour_Points;j++)
			{
				if((ibcpWT[i].iContour_Y[j]>12)&&(ibcpWT[i].iContour_Y[j]<512)&&(ibcpWT[i].iContour_X[j]>=228)&&(ibcpWT[i].iContour_X[j]<=284))
				{
					ix=ibcpWT[i].iContour_X[j];
					iy=ibcpWT[i].iContour_Y[j];
					obmap->obstacle[iy][ix].clusterId=cluster_num;
					mPos.x=ix;
					mPos.y=iy;
					open.AddTail(mPos);
					ObPoint t;
					t.InPut(ix,iy,obmap->obstacle[iy][ix].clusterId);
					ObPoints.push_back(t);
				}
			}

			cluster_num++;
			while(!open.IsEmpty())
			{
				Pos temp=open.GetTail();
				open.RemoveTail();

				for (int m=-w; m<=w; m++)
				{
					for (int n=-l; n<=l; n++)
					{
						int x=temp.x+m;
						int y=temp.y+n;

						if (x<=284&&x>=228&&y<512&&y>12&&obmap->obstacle[y][x].clusterId==-1)
						{
							obmap->obstacle[y][x].clusterId=obmap->obstacle[temp.y][temp.x].clusterId;
							mPos.x=x;
							mPos.y=y;
							open.AddTail(mPos);
							ObPoint t;
							t.InPut(x,y,obmap->obstacle[y][x].clusterId);
							ObPoints.push_back(t);
						}

					}
				}
			}
			if(ObPoints.size()==0)
				continue;
			ASSERT(ObPoints.size()!=0);
			
			xmin=511;
			xmax=0;
			ymin=511;
			ymax=0;
			int temp_size=ObPoints.size();
			for (int ym=0; ym<temp_size;ym++)
			{
				if (ObPoints[ym].Getx()<=xmin)
				{
					xmin=ObPoints[ym].Getx();
				}
				if (ObPoints[ym].Getx()>=xmax)
				{
					xmax=ObPoints[ym].Getx();
				}
				if (ObPoints[ym].Gety() <= ymin)
					ymin = ObPoints[ym].Gety();
				if(ObPoints[ym].Gety() >= ymax)
					ymax = ObPoints[ym].Gety();
			}

			int max_x=xmax-xmin;
			int max_y=ymax-ymin;
			int max=0;
			int smallest=0;
			if(max_x<max_y)
			{
				max=max_y;
				smallest=max_x;
			}
			else
			{
				max=max_x;
				smallest=max_y;
			}
				
			if (!(max<125&&smallest<30))
			{
				v_x_clusterId[cluster_num-1]=-10000;
				v_y_clusterId[cluster_num-1]=-10000;
			}
		}
	}
}

void GrowCluster::Init_history(I_Map *map,ObjectWT *ibcpWT, int ib_num)
{
	CvPoint2D64f ibcp[2000];
	int kk=0;

	//obmap=new ObMap;
	for (int i=0; i<512; i++)
	{
		for (int j=0; j<512; j++)
		{	
			obmap->obstacle[i][j].isob=map->MapPoint[i][j]; 
			obmap->obstacle[i][j].isreached=0;
			if (obmap->obstacle[i][j].isob==8)
			{
				obmap->obstacle[i][j].clusterId=-1;
			}
			else obmap->obstacle[i][j].clusterId=-2;
		}
	}
	CList<Pos,Pos> open;
	Pos mPos;
	cluster_num=0;
	
	int w=3;
	int l=20;

	for(int i=0; i<ib_num; i++)
	{
		if(ibcpWT[i].ObjectBox_CenterY<512&&ibcpWT[i].ObjectBox_CenterY>12&&ibcpWT[i].ObjectBox_CenterX<=286&&ibcpWT[i].ObjectBox_CenterX>=226)
		{
			ObPoints.clear();
			int ix=ibcpWT[i].ObjectBox_CenterX;
			int iy=ibcpWT[i].ObjectBox_CenterY;
			//obmap->obstacle[iy][ix].clusterId=cluster_num;
			v_x_clusterId[cluster_num]=ibcpWT[i].Relative_VelocityX;
			v_y_clusterId[cluster_num]=ibcpWT[i].Relative_VelocityY;
			for (int m=-w; m<=w; m++)
				{
					for (int n=-l; n<=l; n++)
					{
						int x=ix+m;
						int y=iy+n;

						if (x<=286&&x>=226&&y<512&&y>12&&obmap->obstacle[y][x].clusterId==-1)
						{
							obmap->obstacle[y][x].clusterId=cluster_num;
							mPos.x=x;
							mPos.y=y;
							open.AddTail(mPos);
							ObPoint t;
							t.InPut(x,y,obmap->obstacle[y][x].clusterId);
							ObPoints.push_back(t);
						}
					}
				}
			for(int j=0;j<ibcpWT[i].iContour_Points;j++)
			{
				if((ibcpWT[i].iContour_Y[j]>12)&&(ibcpWT[i].iContour_Y[j]<512)&&(ibcpWT[i].iContour_X[j]>=226)&&(ibcpWT[i].iContour_X[j]<=286))
				{
					ix=ibcpWT[i].iContour_X[j];
					iy=ibcpWT[i].iContour_Y[j];
					for (int m=-w; m<=w; m++)
						{
							for (int n=-l; n<=l; n++)
							{
								int x=ix+m;
								int y=iy+n;

								if (x<=286&&x>=226&&y<512&&y>12&&obmap->obstacle[y][x].clusterId==-1)
								{
									obmap->obstacle[y][x].clusterId=cluster_num;
									mPos.x=x;
									mPos.y=y;
									open.AddTail(mPos);
									ObPoint t;
									t.InPut(x,y,obmap->obstacle[y][x].clusterId);
									ObPoints.push_back(t);
								}

							}
						}
				}
			}

			cluster_num++;
			while(!open.IsEmpty())
			{
				Pos temp=open.GetTail();
				open.RemoveTail();

				for (int m=-w; m<=w; m++)
				{
					for (int n=-l; n<=l; n++)
					{
						int x=temp.x+m;
						int y=temp.y+n;

						if (x<=286&&x>=226&&y<512&&y>12&&obmap->obstacle[y][x].clusterId==-1)
						{
							obmap->obstacle[y][x].clusterId=obmap->obstacle[temp.y][temp.x].clusterId;
							mPos.x=x;
							mPos.y=y;
							open.AddTail(mPos);
							ObPoint t;
							t.InPut(x,y,obmap->obstacle[y][x].clusterId);
							ObPoints.push_back(t);
						}

					}
				}
			}
			if(ObPoints.size()==0)
				continue;
			ASSERT(ObPoints.size()!=0);
			
			xmin=511;
			xmax=0;
			ymin=511;
			ymax=0;
			int temp_size=ObPoints.size();
			for (int ym=0; ym<temp_size;ym++)
			{
				if (ObPoints[ym].Getx()<=xmin)
				{
					xmin=ObPoints[ym].Getx();
				}
				if (ObPoints[ym].Getx()>=xmax)
				{
					xmax=ObPoints[ym].Getx();
				}
				if (ObPoints[ym].Gety() <= ymin)
					ymin = ObPoints[ym].Gety();
				if(ObPoints[ym].Gety() >= ymax)
					ymax = ObPoints[ym].Gety();
			}

			int max_x=xmax-xmin;
			int max_y=ymax-ymin;
			int max=0;
			int smallest=0;
			if(max_x<max_y)
			{
				max=max_y;
				smallest=max_x;
			}
			else
			{
				max=max_x;
				smallest=max_y;
			}
				
			if (!(max<125&&smallest<30))
			{
				v_x_clusterId[cluster_num-1]=-10000;
				v_y_clusterId[cluster_num-1]=-10000;
			}
		}
	}
}



void GrowCluster::Clear()
{
	delete obmap;
	
}
