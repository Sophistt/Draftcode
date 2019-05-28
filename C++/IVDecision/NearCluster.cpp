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
	CList<Relate,Relate> relation;                        //�洢�����������
	Relate tepRelate;
	Relate ptr;
	ObMap *obmap=new ObMap;                               //�Զ����ϰ����ͼ
	for (int i=0; i<512; i++)                             //�����ϰ����ͼ
	{
		for (int j=0; j<512; j++)
		{	
			obmap->obstacle[i][j].isob=map->MapPoint[i][j]; 
			obmap->obstacle[i][j].isreached=0;
			if (obmap->obstacle[i][j].isob==18)
			{
			obmap->obstacle[i][j].clusterId=-1;          //��ÿһ���ϰ������������趨Ϊ-1
			}
			obmap->obstacle[i][j].isreached=0;           //���Ƿ����궨Ϊ0
		
		}
	}

	int cluster_num=0;
	ObPoint temp;

	for (int i=0; i<512; i++)                          
	{
		for (int j=0; j<512; j++)                      //�����ң����ϵ���ɨ���
		{
			if (obmap->obstacle[i][j].isob==18)         //�����ͼ�����ϰ����
			{

				if (obmap->obstacle[i][j].clusterId==-1) //����ϰ���㻹û�з���
				{	
					if (j+1<512&&obmap->obstacle[i][j+1].isob==18&&obmap->obstacle[i][j+1].clusterId!=-1)    //����ϰ���δ�������ұߵ��ѷ��࣬���˵���ൽ�ұߵ�               
					{
						obmap->obstacle[i][j].clusterId=obmap->obstacle[i][j+1].clusterId;
						continue;
					}
					if (j+2<512&&obmap->obstacle[i][j+2].isob==181&&obmap->obstacle[i][j+2].clusterId!=-1)    //����ϰ���δ�������ұ߸�һ�����ѷ��࣬������ൽ�˵�
					{
						obmap->obstacle[i][j].clusterId=obmap->obstacle[i][j+2].clusterId;
						continue;
					}
					if (i-1>=0&&j+1<512&&obmap->obstacle[i-1][j+1].isob==18&&obmap->obstacle[i-1][j+1].clusterId!=-1)    //�����δ���������Ϸ����ѷ��࣬���ൽ��
					{
						obmap->obstacle[i][j].clusterId=obmap->obstacle[i-1][j+1].clusterId;
						continue;
					}


					cluster_num=cluster_num+1;           //�������1
					obmap->obstacle[i][j].clusterId=cluster_num; //�ϰ��������趨
					if (i+1<512&&obmap->obstacle[i+1][j].isob==18) //����ϰ��������ĵ�Ϊ�ϰ����
					{	
						obmap->obstacle[i+1][j].clusterId=obmap->obstacle[i][j].clusterId;
					}
					if (j+1<512&&obmap->obstacle[i][j+1].isob==18)  //����ϰ��������ĵ�Ϊ�ϰ����
					{
						obmap->obstacle[i][j+1].clusterId=obmap->obstacle[i][j].clusterId;
					}
					if (i+2<512&&obmap->obstacle[i+2][j].isob==18)  //����ϰ�����·�����1��Ϊ�ϰ����
					{
						obmap->obstacle[i+2][j].clusterId=obmap->obstacle[i][j].clusterId;
					}
					if (j+2<512&&obmap->obstacle[i][j+2].isob==18)   //����ϰ�����ҷ�����1��Ϊ�ϰ����
					{
						obmap->obstacle[i][j+2].clusterId=obmap->obstacle[i][j].clusterId;
					}
					if (i+1<512&&j+1<512&&obmap->obstacle[i+1][j+1].isob==18)   //����ϰ�������·�Ϊ�ϰ����
					{
						obmap->obstacle[i+1][j+1].clusterId=obmap->obstacle[i][j].clusterId;
					}
				}
				
				if (obmap->obstacle[i][j].clusterId!=-1)   //�ϰ�����ѷ���                                       
				{
					if (i+1<512&&obmap->obstacle[i][j+1].clusterId==-1&&obmap->obstacle[i+1][j].isob==18)    //����ϰ�����·���Ϊ�ϰ���㣬��˵��Ϊһ��
					{	
						obmap->obstacle[i+1][j].clusterId=obmap->obstacle[i][j].clusterId;
					}
					if (j+1<512&&obmap->obstacle[i][j+1].clusterId==-1&&obmap->obstacle[i][j+1].isob==18)    //����ϰ�����ҷ�Ϊ�ϰ���㣬��˵��Ϊһ��
					{
						obmap->obstacle[i][j+1].clusterId=obmap->obstacle[i][j].clusterId;
					}
					if (i+2<512&&obmap->obstacle[i+2][j].clusterId==-1&&obmap->obstacle[i+2][j].isob==18)   //����ϰ�����·��ڶ�����Ϊ�ϰ���㣬��˵��Ϊһ��
					{
						obmap->obstacle[i+2][j].clusterId=obmap->obstacle[i+2][j].clusterId;
					}
					if (j+2<512&&obmap->obstacle[i][j+2].clusterId==-1&&obmap->obstacle[i][j+2].isob==18)   //����ϰ�����ҷ��ڶ�����Ϊ�ϰ���㣬��˵��Ϊһ��
					{
						obmap->obstacle[i][j+2].clusterId=obmap->obstacle[i][j].clusterId;
					}
					if (i+1<512&&j+1<512&&obmap->obstacle[i+1][j+1].clusterId==-1&&obmap->obstacle[i+1][j+1].isob==18) //����ϰ�������·���δ�����ϰ���㣬���Ϊһ��
					{
						obmap->obstacle[i+1][j+1].clusterId=obmap->obstacle[i][j].clusterId;
					}

					/*�ѷ����ϰ���㸽����������ͬ���ϰ���㣬���������*/

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
		//if (FindOpen(open,pr.x)==0&&FindClose(closed,pr.x))                      //x����Open���closed��
		//{
		//	open.AddTail(pr.x);
		//}
		//if (FindOpen(open,pr.y)==0&&FindClose(closed,pr.y))                      //y����Open��closed����
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

