#include "StdAfx.h"
#include "NearCluster.h"
#include <fstream>
NearCluster::NearCluster(void)
{
}

NearCluster::~NearCluster(void)
{
}

void NearCluster::Ini(I_Map *map)
{
	CList<Relate,Relate> relation;                        //存储关联类别序数
	Relate tepRelate;
	Relate ptr;
	ObMap *obmap=new ObMap;                               //自定义障碍物地图
	for (int i=0; i<512; i++)                             //读入障碍物地图
	{
		for (int j=0; j<512; j++)
		{	
			obmap->obstacle[i][j].isob=map->MapPoint[i][j]; 
			obmap->obstacle[i][j].isreached=0;
			if (obmap->obstacle[i][j].isob==18)
			{
			obmap->obstacle[i][j].clusterId=-1;          //把每一个障碍物点的类属性设定为-1
			}
			obmap->obstacle[i][j].isreached=0;           //把是否分类标定为0
		
		}
	}

	int cluster_num=0;
	ObPoint temp;

	for (int i=0; i<512; i++)                          
	{
		for (int j=0; j<512; j++)                      //从左到右，从上到下扫描点
		{
			if (obmap->obstacle[i][j].isob==18)         //如果地图点是障碍物点
			{

				if (obmap->obstacle[i][j].clusterId==-1) //如果障碍物点还没有分类
				{	
					if (j+1<512&&obmap->obstacle[i][j+1].isob==18&&obmap->obstacle[i][j+1].clusterId!=-1)    //如果障碍物未分类且右边点已分类，将此点归类到右边点               
					{
						obmap->obstacle[i][j].clusterId=obmap->obstacle[i][j+1].clusterId;
						continue;
					}
					if (j+2<512&&obmap->obstacle[i][j+2].isob==181&&obmap->obstacle[i][j+2].clusterId!=-1)    //如果障碍物未分类且右边隔一个点已分类，将点归类到此点
					{
						obmap->obstacle[i][j].clusterId=obmap->obstacle[i][j+2].clusterId;
						continue;
					}
					if (i-1>=0&&j+1<512&&obmap->obstacle[i-1][j+1].isob==18&&obmap->obstacle[i-1][j+1].clusterId!=-1)    //如果点未分类且右上方点已分类，归类到此
					{
						obmap->obstacle[i][j].clusterId=obmap->obstacle[i-1][j+1].clusterId;
						continue;
					}


					cluster_num=cluster_num+1;           //类别数加1
					obmap->obstacle[i][j].clusterId=cluster_num; //障碍物点类别设定
					if (i+1<512&&obmap->obstacle[i+1][j].isob==18) //如果障碍物点下面的点为障碍物点
					{	
						obmap->obstacle[i+1][j].clusterId=obmap->obstacle[i][j].clusterId;
					}
					if (j+1<512&&obmap->obstacle[i][j+1].isob==18)  //如果障碍物点右面的点为障碍物点
					{
						obmap->obstacle[i][j+1].clusterId=obmap->obstacle[i][j].clusterId;
					}
					if (i+2<512&&obmap->obstacle[i+2][j].isob==18)  //如果障碍物点下方距离1格为障碍物点
					{
						obmap->obstacle[i+2][j].clusterId=obmap->obstacle[i][j].clusterId;
					}
					if (j+2<512&&obmap->obstacle[i][j+2].isob==18)   //如果障碍物点右方距离1格为障碍物点
					{
						obmap->obstacle[i][j+2].clusterId=obmap->obstacle[i][j].clusterId;
					}
					if (i+1<512&&j+1<512&&obmap->obstacle[i+1][j+1].isob==18)   //如果障碍物点右下方为障碍物点
					{
						obmap->obstacle[i+1][j+1].clusterId=obmap->obstacle[i][j].clusterId;
					}
				}
				
				if (obmap->obstacle[i][j].clusterId!=-1)   //障碍物点已分类                                       
				{
					if (i+1<512&&obmap->obstacle[i][j+1].clusterId==-1&&obmap->obstacle[i+1][j].isob==18)    //如果障碍物点下方点为障碍物点，与此点归为一类
					{	
						obmap->obstacle[i+1][j].clusterId=obmap->obstacle[i][j].clusterId;
					}
					if (j+1<512&&obmap->obstacle[i][j+1].clusterId==-1&&obmap->obstacle[i][j+1].isob==18)    //如果障碍物点右方为障碍物点，与此点归为一类
					{
						obmap->obstacle[i][j+1].clusterId=obmap->obstacle[i][j].clusterId;
					}
					if (i+2<512&&obmap->obstacle[i+2][j].clusterId==-1&&obmap->obstacle[i+2][j].isob==18)   //如果障碍物点下方第二个点为障碍物点，与此点归为一类
					{
						obmap->obstacle[i+2][j].clusterId=obmap->obstacle[i+2][j].clusterId;
					}
					if (j+2<512&&obmap->obstacle[i][j+2].clusterId==-1&&obmap->obstacle[i][j+2].isob==18)   //如果障碍物点右方第二个点为障碍物点，与此点归为一类
					{
						obmap->obstacle[i][j+2].clusterId=obmap->obstacle[i][j].clusterId;
					}
					if (i+1<512&&j+1<512&&obmap->obstacle[i+1][j+1].clusterId==-1&&obmap->obstacle[i+1][j+1].isob==18) //如果障碍物点右下方有未分类障碍物点，则归为一类
					{
						obmap->obstacle[i+1][j+1].clusterId=obmap->obstacle[i][j].clusterId;
					}

					/*已分类障碍物点附近搜索到不同类障碍物点，将两类关联*/

					if (obmap->obstacle[i][j+1].isob==18&&obmap->obstacle[i][j+1].clusterId!=-1&&obmap->obstacle[i][j+1].clusterId!=obmap->obstacle[i][j].clusterId)
					{
						tepRelate.x=obmap->obstacle[i][j+1].clusterId;
						tepRelate.y=obmap->obstacle[i][j].clusterId;

						POSITION pos = relation.GetHeadPosition();
						int flag=0;
							while(pos != NULL)
							{
								ptr =relation.GetNext(pos);
								if ((ptr.x==tepRelate.x&&ptr.y==tepRelate.y)||(ptr.x==tepRelate.y&&ptr.y==tepRelate.x))
								{
									flag=1;
									break;
								}
							}

							if (flag==0)
							{
								relation.AddTail(tepRelate);
							}
						
					}
				}
				
			}
		}
	}

	/*Relate rl;*/
	/*rl=relation.GetHead();*/
	/*int x;
	int y;
	POSITION position1=relation.GetHeadPosition();
	POSITION position2=relation.GetHeadPosition();
	while(!relation.IsEmpty())
	{
        Relate rl=relation.GetHead();
		x=rl.x;
		y=rl.y;
		while(position1!=NULL)
		{
			Relate rlate=relation.GetNext(position1);
			if (rlate.x=rl.x)
			{
			}
		}
	}*/
	
		
	/*CList<int, int > open;
	CList<int ,int > closed;
    Relate pr;
	POSITION position;
	POSITION position2;
	open.AddTail(relation.GetHead().x);*/
	/*while(position!=NULL){	*/
		/*pr=relation.GetNext(position);
		if (open.Find(pr.x)==NULL&&open.Find(pr.y)==NULL)
		{
			open.AddTail(pr);
			open.AddTail()
		}
		if (pr.x==open.GetTail()&&closed.Find(pr.y)==NULL)
		{
			open.AddTail(pr.y);
		}
		if (pr.y==open.GetTail())
		{
			open.AddTail(pr.x);
		}*/
		//if (FindOpen(open,pr.x)==0&&FindClose(closed,pr.x))                      //x不在Open表和closed里
		//{
		//	open.AddTail(pr.x);
		//}
		//if (FindOpen(open,pr.y)==0&&FindClose(closed,pr.y))                      //y不在Open和closed表里
		//{
		//	open.AddTail(pr.y);
		//}
	/*}*/


int num=0;
	for (int i=0; i<511; i++)
	{
		for (int j=0; j<511; j++)
		{
			if (obmap->obstacle[i][j].isob==18)
			{
				temp.InPut(j,i,obmap->obstacle[i][j].clusterId);
				ObPoints.push_back(temp);
				num++;
			}
		}
	}
	ofstream outob("D\:cluster.txt");
	for (int i=0; i<num; i++)
	{
		outob<<ObPoints[i].Getx()<<" "<<ObPoints[i].Gety()<<" "<<ObPoints[i].GetClusterId()<<"\n";
	}
}

