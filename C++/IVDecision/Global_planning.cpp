#include "stdafx.h"
#include "Global_planning.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif
# define I 999999
#define  T 30000

struct Point
{
   double x,y;
}point[100];

void Global_planning::solvewaypointnum()//求rndf文件中所有路点数
{
	waypointnum = 0;
	astar.rndf=RNDF;
	
	for(int segnum=0;segnum<astar.rndf.m_mapInfo.segment_num;segnum++)
	{
		for(int lanenum=0;lanenum<astar.rndf.m_mapInfo.pSegment[segnum].lane_num;lanenum++)
		{
			waypointnum+=astar.rndf.m_mapInfo.pSegment[segnum].pLane[lanenum].waypoints_num;
		}
	}
}
MAPPOINT* Global_planning::solvepoint(double x,double y)//根据点的GPS坐标求离其最近的路点
{
	astar.rndf=RNDF;
	double distance;
	double mindis = I;
	MAPPOINT *nearpoint = NULL;
	for(int i=0;i<astar.rndf.m_mapInfo.segment_num;i++)
	{
		for(int j=0;j<astar.rndf.m_mapInfo.pSegment[i].lane_num;j++)
		{
			for(int k=0;k<astar.rndf.m_mapInfo.pSegment[i].pLane[j].waypoints_num;k++)
			{
				distance = sqrt(pow((x - (astar.rndf.m_mapInfo.pSegment[i].pLane[j].pPoint[k].x)),2) + pow((y - (astar.rndf.m_mapInfo.pSegment[i].pLane[j].pPoint[k].y)),2));
				if(distance<mindis)
				{
					mindis = distance;
					nearpoint = &astar.rndf.m_mapInfo.pSegment[i].pLane[j].pPoint[k];
				}
			}
		}
	}
	return nearpoint;
}


void  Global_planning::point2point()//生成所有点的连通性
{
	astar.rndf=RNDF;
	
	for(int num=0;num<astar.rndf.m_mapInfo.segment_num;num++)
	{
		int waypointNUM;
		int idid,mm,nn,pp;
		if(astar.rndf.m_mapInfo.pSegment[num].lane_num==1)
		{
			waypointNUM=astar.rndf.m_mapInfo.pSegment[num].pLane[0].waypoints_num;
			for (int t=0;t<waypointNUM-1;t++)
			{
				astar.rndf.m_mapInfo.pSegment[num].pLane[0].pPoint[t].nearnode[0]=&astar.rndf.m_mapInfo.pSegment[num].pLane[0].pPoint[t+1];
			}

			int j=astar.rndf.m_mapInfo.pSegment[num].pLane[0].exit_num;

			if(j!=0)
			{  
				for(int k=0;k<j;k++)
				{
					idid = astar.rndf.m_mapInfo.pSegment[num].pLane[0].pExit[k].exit_Id;
					mm = astar.rndf.m_mapInfo.pSegment[num].pLane[0].pExit[k].m;
					nn = astar.rndf.m_mapInfo.pSegment[num].pLane[0].pExit[k].n;
					pp = astar.rndf.m_mapInfo.pSegment[num].pLane[0].pExit[k].p;

					astar.rndf.m_mapInfo.pSegment[num].pLane[0].pPoint[idid-1].nearnode[k]=&astar.rndf.m_mapInfo.pSegment[mm-1].pLane[nn-1].pPoint[pp-1];
				}
			}
		}

		else if(astar.rndf.m_mapInfo.pSegment[num].lane_num==2)
		{
			waypointNUM=astar.rndf.m_mapInfo.pSegment[num].pLane[0].waypoints_num;
			for (int t=0;t<waypointNUM-1;t++)
			{
				astar.rndf.m_mapInfo.pSegment[num].pLane[0].pPoint[t].nearnode[0]=&astar.rndf.m_mapInfo.pSegment[num].pLane[0].pPoint[t+1];
			}

			int j=astar.rndf.m_mapInfo.pSegment[num].pLane[0].exit_num;

			if(j!=0)
			{  
				for(int k=0;k<j;k++)
				{
					idid = astar.rndf.m_mapInfo.pSegment[num].pLane[0].pExit[k].exit_Id;
					mm = astar.rndf.m_mapInfo.pSegment[num].pLane[0].pExit[k].m;
					nn = astar.rndf.m_mapInfo.pSegment[num].pLane[0].pExit[k].n;
					pp = astar.rndf.m_mapInfo.pSegment[num].pLane[0].pExit[k].p;

					astar.rndf.m_mapInfo.pSegment[num].pLane[0].pPoint[idid-1].nearnode[k]=&astar.rndf.m_mapInfo.pSegment[mm-1].pLane[nn-1].pPoint[pp-1];
				}
			}
			astar.rndf.m_mapInfo.pSegment[num].pLane[0].pPoint[waypointNUM-1].nearnode[3]=&astar.rndf.m_mapInfo.pSegment[num].pLane[1].pPoint[0];

			waypointNUM=astar.rndf.m_mapInfo.pSegment[num].pLane[1].waypoints_num;
			for (int t=0;t<waypointNUM-1;t++)
			{
				astar.rndf.m_mapInfo.pSegment[num].pLane[1].pPoint[t].nearnode[0]=&astar.rndf.m_mapInfo.pSegment[num].pLane[1].pPoint[t+1];
			}

			j=astar.rndf.m_mapInfo.pSegment[num].pLane[1].exit_num;

			if(j!=0)
			{  
				for(int k=0;k<j;k++)
				{
					idid = astar.rndf.m_mapInfo.pSegment[num].pLane[1].pExit[k].exit_Id;
					mm = astar.rndf.m_mapInfo.pSegment[num].pLane[1].pExit[k].m;
					nn = astar.rndf.m_mapInfo.pSegment[num].pLane[1].pExit[k].n;
					pp = astar.rndf.m_mapInfo.pSegment[num].pLane[1].pExit[k].p;

					astar.rndf.m_mapInfo.pSegment[num].pLane[1].pPoint[idid-1].nearnode[k]=&astar.rndf.m_mapInfo.pSegment[mm-1].pLane[nn-1].pPoint[pp-1];
				}
			}
			astar.rndf.m_mapInfo.pSegment[num].pLane[1].pPoint[waypointNUM-1].nearnode[3]=&astar.rndf.m_mapInfo.pSegment[num].pLane[0].pPoint[0];
		}
	}
}


void Global_planning::planning(int m1,int n1,int p1,int m2,int n2,int p2)
{
     astar.rndf=RNDF;

	 for(int i=0;i<astar.rndf.m_mapInfo.segment_num;i++)//初始化所有路点
	 {
		 for(int j=0;j<astar.rndf.m_mapInfo.pSegment[i].lane_num;j++)
		 {
			 for(int k=0;k<astar.rndf.m_mapInfo.pSegment[i].pLane[j].waypoints_num;k++)
			 {
				 astar.rndf.m_mapInfo.pSegment[i].pLane[j].pPoint[k].g = I;
				 astar.rndf.m_mapInfo.pSegment[i].pLane[j].pPoint[k].nearnode[0] = NULL;
				 astar.rndf.m_mapInfo.pSegment[i].pLane[j].pPoint[k].nearnode[1] = NULL;
				 astar.rndf.m_mapInfo.pSegment[i].pLane[j].pPoint[k].nearnode[2] = NULL;
				 astar.rndf.m_mapInfo.pSegment[i].pLane[j].pPoint[k].nearnode[3] = NULL;
				 astar.rndf.m_mapInfo.pSegment[i].pLane[j].pPoint[k].next = NULL;
				 astar.rndf.m_mapInfo.pSegment[i].pLane[j].pPoint[k].par = NULL;
			 }
		 }
	 }

	 struct MAPPOINT *node,*best;
	 float g_value;
	 astar.open=(struct MAPPOINT*)calloc(1,sizeof( struct MAPPOINT ));  	
	 astar.close=(struct MAPPOINT*)calloc(1,sizeof( struct MAPPOINT )); 
	 int existop0,existop1,existop2,existop3,existcl0,existcl1,existcl2,existcl3;
	
	start_point =&astar.rndf.m_mapInfo.pSegment[m1-1].pLane[n1-1].pPoint[p1-1];
	end_point = &astar.rndf.m_mapInfo.pSegment[m2-1].pLane[n2-1].pPoint[p2-1];
	
	node = start_point;
	astar.open->next = node;
	node->g = 0;
	

	while((node->x !=end_point->x) || (node->y != end_point->y))
	{
		existop0 = astar.existopen(astar.open,node->nearnode[0]);
		existop1 = astar.existopen(astar.open,node->nearnode[1]);
		existop2 = astar.existopen(astar.open,node->nearnode[2]);
		existop3 = astar.existopen(astar.open,node->nearnode[3]);
		existcl0 = astar.existclose(astar.close,node->nearnode[0]);
		existcl1 = astar.existclose(astar.close,node->nearnode[1]);
		existcl2 = astar.existclose(astar.close,node->nearnode[2]);
		existcl3 = astar.existclose(astar.close,node->nearnode[3]);

		if((node->nearnode[0]!=NULL)&&(existop0 == 0)&&(existcl0 == 0))
		{astar.addopen(node->nearnode[0]);}
		if((node->nearnode[1]!=NULL)&&(existop1 == 0)&&(existcl1 == 0))
		{astar.addopen(node->nearnode[1]);}
		if((node->nearnode[2]!=NULL)&&(existop2 == 0)&&(existcl2 == 0))
		{astar.addopen(node->nearnode[2]);}
		if((node->nearnode[3]!=NULL)&&(existop3 == 0)&&(existcl3 == 0))
		{astar.addopen(node->nearnode[3]);}
		
		astar.deleteopen(node);
		astar.addclose(node);

		if((astar.open->next!=NULL))
		{
			//无法找到最短路径
			CString str="无法找到最短路径";
			AfxMessageBox(str);
			break;
		}
		node->d0 = astar.solvedistance(*node,node->nearnode[0]);
		node->d1 = astar.solvedistance(*node,node->nearnode[1]);
		node->d2 = astar.solvedistance(*node,node->nearnode[2]);
		node->d3 = astar.solvedistance(*node,node->nearnode[3]);

		if((node->nearnode[0]!=NULL) && (!existcl0))
		{
			g_value = node->g + node->d0;
			if ((g_value) < (node->nearnode[0]->g))
			{
				node->nearnode[0]->g = g_value;
				node->nearnode[0]->par = node;
			}
		}
		if((node->nearnode[1]!=NULL) && (!existcl1))
		{
			g_value = node->g + node->d1;
			if ((g_value) < (node->nearnode[1]->g))
			{
				node->nearnode[1]->g = g_value;
				node->nearnode[1]->par = node;
			}
		}
		if((node->nearnode[2]!=NULL) && (!existcl2))
		{
			g_value = node->g + node->d2;
			if ((g_value) < (node->nearnode[2]->g))
			{
				node->nearnode[2]->g = g_value;
				node->nearnode[2]->par = node;
			}
		}
		if((node->nearnode[3]!=NULL) && (!existcl3))
		{
			g_value = node->g + node->d3;
			if ((g_value) < (node->nearnode[3]->g))
			{
				node->nearnode[3]->g = g_value;
				node->nearnode[3]->par = node;
			}
		}
		

		if((node->nearnode[0]!=NULL) && (!existcl0))
		{
			node->nearnode[0]->h = astar.solveheuristic(node->nearnode[0],*end_point);
		}
		if((node->nearnode[1]!=NULL) && (!existcl1))
		{
			node->nearnode[1]->h = astar.solveheuristic(node->nearnode[1],*end_point);
		}
		if((node->nearnode[2]!=NULL) && (!existcl2))
		{
			node->nearnode[2]->h = astar.solveheuristic(node->nearnode[2],*end_point);
		}
		if((node->nearnode[3]!=NULL) && (!existcl3))
		{
			node->nearnode[3]->h = astar.solveheuristic(node->nearnode[3],*end_point);
		}
		if((node->nearnode[0]!=NULL) && (!existcl0))
		{
			node->nearnode[0]->f = node->nearnode[0]->g + node->nearnode[0]->h;
		}
		if((node->nearnode[1]!=NULL) && (!existcl1))
		{
			node->nearnode[1]->f = node->nearnode[1]->g + node->nearnode[1]->h;
		}
		if((node->nearnode[2]!=NULL) && (!existcl2))
		{
			node->nearnode[2]->f = node->nearnode[2]->g + node->nearnode[2]->h;
		}
		if((node->nearnode[3]!=NULL) && (!existcl3))
		{
			node->nearnode[3]->f = node->nearnode[3]->g + node->nearnode[3]->h;
		}
		astar.Sort();//根据f值大小对open表中所有的点进行排序
		best = astar.SearchBest();//取f值最小的点
		node = best;
	}

	struct MAPPOINT *pathpoint =end_point;
	path =(struct MAPPOINT*)calloc(1,sizeof( struct MAPPOINT ));

	while(pathpoint != start_point)
	{
		pathpoint->next = path->next;
		path->next = pathpoint;
		pathpoint = pathpoint->par;
	}
	pathpoint->next = path->next;
	path->next = pathpoint;

}