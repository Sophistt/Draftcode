#include "StdAfx.h"
#include "ObDetct.h"
#include <fstream>
using namespace std;
ofstream outdata("D:\\obdetect");
int mycount=0;
extern const int ob_flag;
/************************************************************************/
/* 定义检测范围                                                                     */
/************************************************************************/
extern const int o_left;
extern const int o_right;
extern const int o_front;
extern const int o_rear;
/************************************************************************/
/* 定义单个点的生长范围                                                                     */
/************************************************************************/
extern const int o_widt;
extern const int o_lenghth;

ObDetct::ObDetct(void)
{
}

ObDetct::~ObDetct(void)
{
}

void ObDetct::Init(I_Map *map)
{
	ObMap *obmap=new ObMap;
	for (int index_i=0; index_i<512; index_i++)
	{
		for (int index_j=0; index_j<512; index_j++)
		{
			obmap->obstacle[index_i][index_j].isob=map->MapPoint[index_i][index_j];
			if (obmap->obstacle[index_i][index_j].isob==ob_flag)
			{
				obmap->obstacle[index_i][index_j].clusterId=-1;
			}
			else
				obmap->obstacle[index_i][index_j].clusterId=-2;
		}
	}

	CList<Pos,Pos> open;
	Pos mPos;
	mPos.x=-1;
	mPos.y=-1;
	int cluster_num=0;
	for (int i=o_front; i<o_rear; i++)
	{
		for (int j=o_left; j<o_right; j++)
		{
			if (obmap->obstacle[i][j].clusterId==-1)
			{
				obmap->obstacle[i][j].clusterId=cluster_num;
				for (int a=o_width; a<o_lenghth; a++)
				{
					for (int b=o_width; b<o_lenghth; b++)
					{
						if (i+a>=0&&i+a<=512&&obmap->obstacle[i+a][j+b].clusterId==-1)
						{
							obmap->obstacle[i+a][j+b].clusterId=obmap->obstacle[i][j].clusterId;
							mPos.x=j+b;
							mPos.y=i+a;
							open.AddTail(mPos);
						}
					}
				}
				cluster_num++;
			}
			int x=0;
			int y=0;
			while(!open.IsEmpty())
			{
				Pos tempos=open.GetTail();
				open.RemoveTail();
				x=tempos.x;
				y=tempos.y;
				for (int a=o_width; a<o_lenghth; a++)
				{
					for (int b=o_width; b<o_lenghth; b++)
					{
						if (y+a>=0&&y+b<512&&obmap->obstacle[y+a][x+b].clusterId==-1)
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
	for (int i=0; i<512; i++)
	{
		for (int j=0; j<512; j++)
		{
			if (obmap->obstacle[i][j].clusterId>=0)
			{
				ObPoint temp;
				temp.InPut(j,i,obmap->obstacle[i][j].clusterId);
				ObPoints.push_back(temp);
				num++;
			}
		}
	}
	point_count=num;
	outdata<<mycount<<"\t"<<point_count<<"\t"<<ob_count<<"\n";
	delete obmap;
}

