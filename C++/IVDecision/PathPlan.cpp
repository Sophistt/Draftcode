#include "StdAfx.h"
#include "PathPlan.h"

# define I 999999

PathPlan::PathPlan(void)
{
	storage_road180 = cvCreateMemStorage(0);
	plan_road = cvCreateSeq( CV_64FC2, sizeof(CvSeq), sizeof(CvPoint2D64f), storage_road180 );

	his_ap.x = 0;
	his_ap.y = 0;
	his_ap_tmp.x=0;
	his_ap_tmp.y=0;
	for (int i=0;i<23;i++)
	{
		his_hiscost[i]=0;

	}
	changedis = 0;
	
}

PathPlan::~PathPlan(void)
{
	cvReleaseMemStorage(&storage_road180);
}

bool PathPlan::existopen(struct MAPPOINT_star *Eo,struct MAPPOINT_star *noop)
{
	while(Eo->next!=NULL)
	{
		if(Eo->next == noop)
			return true;
		Eo = Eo->next;
	}
	return false;
}

bool PathPlan::existclose(struct MAPPOINT_star *Ec,struct MAPPOINT_star *nocl)
{
	while(Ec->next!=NULL)
	{
		if(Ec->next == nocl)
			return true;
		Ec = Ec->next;
	}
	return false;
}


void PathPlan::addopen(struct MAPPOINT_star *pno)
{
	if (pno!=NULL)
	{
	pno->next = open->next;
	open->next = pno;
	}
}

int PathPlan::deleteopen(struct MAPPOINT_star *pyes)
{
	struct MAPPOINT_star *Delo;
	Delo = open->next;
	if(Delo == pyes)
	{
		open->next = Delo->next;
		return 1;
	}
	else
	{
		while(Delo!=NULL)
		{
			if (Delo->next == pyes)
			{
				Delo->next = pyes->next;
				pyes->next = NULL;
				return 1;
			}
			Delo = Delo->next;
		}
	}
}

void PathPlan::addclose(struct MAPPOINT_star *cno)
{
	if (cno!=NULL)
	cno->next = close->next;
	close->next = cno;
}

struct MAPPOINT_star* PathPlan::SearchBest() 
{
	struct MAPPOINT_star *tp;
	if(open->next!=NULL)  	
	{  		
		tp=open->next;  		
		open->next=tp->next;  		
		return tp;  	
	}  	
	else  		
		exit(0);  
} 

void PathPlan::Sort()  
{  	
	struct MAPPOINT_star *temp1,*temp2,*temp3,*temp4;    	
	struct MAPPOINT_star tempv;  	
	struct MAPPOINT_star *temp= &tempv;
	for(temp1=open->next;temp1!=NULL && temp1->next!=NULL;temp1=temp1->next)	
	{  
		for(temp2=open;temp2->next!=NULL && temp2->next->next!=NULL;temp2=temp2->next)
		{
			temp3=temp2->next;
			temp4=temp3->next;
			if((temp3->f)>(temp4->f))  
			{
				temp=temp4->next;
				temp2->next = temp4;
				temp4->next = temp3;
				temp3->next = temp;
			}
		}
	}    
} 

int PathPlan::AstarPathPlan(CvPoint2D64f m_gps,double m_dir,CvPoint2D64f ap_gps,double ap_dir,CvPoint2D64f (&MPoint)[200])//保持车道
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();	
	Map_Point ap_map = m_GpsData.APiontConver(m_gps,ap_gps,m_dir);
	CvPoint2D64f tp_map[23];
	
	int range = 0;
/*	if(app->GPS_Speed*3.6<5)range = 2;
	else if(app->GPS_Speed*3.6<10)range = 5;
	else */
	//range = 15;

	if(app->GPS_Speed*3.6<10 || app->left_right!=0)
		range = 8;
	else
		range = 8;
	if(app->left_right!=0)
		range = 6;

	int pianyinum;
	if(app->blanechanging)
		pianyinum=3;
	else
		pianyinum=2;

	bool bstopret=false;

	//if(app->bzheng || (app->left_right==0) || app->GPS_Speed<22/3.6)
	if((app->left_right==0) || app->GPS_Speed<22/3.6)
	{
		if(app->bnearstop || (app->kuaisulufulu==1))
		{
			if(SearchFrontOb(vel_Map,5))
				bstopret=true;
				//return -1;
		}
		else
		{
			if(SearchFrontOb(vel_Map,6))
				bstopret=true;
				//return -1;
		}
	}

//////计算轨迹端点，轨迹代价
	/*
	for (int i = 0; i<23;i++)
	{
		tp_map[i].x = ap_map.x + (-36 + 3*i)*cos((ap_dir-m_dir)*3.14159265/180);
		tp_map[i].y = ap_map.y + (-36 + 3*i)*sin((ap_dir-m_dir)*3.14159265/180);
	}
	*/

	bool bob7dist=true;

	if((app->bbanmaxiannoraozhang || app->bspecialraozhangcanshu)&&(app->bGpsGanrao==0))
	{
		app->obb_left=true;
		app->obb_right=true;
	}

	for(int i=0;i<11;i++)
	{
		{
			tp_map[12+i].x = ap_map.x + (-36 + 3*(12+i))*cos((ap_dir-m_dir)*3.14159265/180);
			tp_map[12+i].y = ap_map.y + (-36 + 3*(12+i))*sin((ap_dir-m_dir)*3.14159265/180);

			int xmin;
			int ymin;
			int xmax;
			int ymax;
			if(tp_map[12+i].y<412)
			{
				xmin=tp_map[12+i].x;
				ymin=tp_map[12+i].y;
				xmax=256;
				ymax=412;
			}
			else
			{
				xmin=256;
				ymin=412;
				xmax=tp_map[12+i].x;
				ymax=tp_map[12+i].y;
			}

			bob7dist=false;

			if(((app->obb_left)&&(12+i<11))||((app->left_right==0) && app->bleftturnban && (12+i<12-pianyinum))||((app->obb_right)&&(12+i>13))||((app->left_right==0) && app->brightturnban && (12+i>12+pianyinum)))
				bob7dist=true;

			int xdst=tp_map[12+i].x;
			int ydst=tp_map[12+i].y;

			if(bob7dist==false)
			{
				for(int m=-7;m<=7;m++)
				{
					for(int n=-7;n<=7;n++)
					{
						if(ydst+m>511||ydst+m<0||xdst+n>511||xdst+n<0)
							continue;
						if(vel_Map->MapPoint[ydst+m][xdst+n] == 8 || vel_Map->MapPoint[ydst+m][xdst+n] == 18 || vel_Map->MapPoint[ydst+m][xdst+n] == 28)
						{
							bob7dist=true;
						}
					}
				}
			}

			if(bob7dist==false)
			{
				int y_bottom=412+15;
				if(y_bottom<ydst+15)
					y_bottom=ydst+15;
				int y_up=ydst-15;
				if(y_up>412-15)
					y_up=412-15;
				if(ydst>412-100)
					y_up=y_up-85;
				int x_left=106;
				int x_right=406;
				if(y_bottom>511)
					y_bottom=511;
				if(y_up<0)
					y_up=0;
				if(xdst>256)
				{
					x_left=106;
					x_right=xdst+150;
					if(x_right>511)
						x_right=511;
				}
				else
				{
					x_left=xdst-150;
					x_right=406;
					if(x_left<0)
						x_left=0;
				}
				
				struct MAPPOINT_star *vel_Map_Astar[512][512];
				//vel_Map = app->PercepMap;

				for(int x=x_left;x<=x_right;x++)
				{
					for(int y=y_bottom;y>=y_up;y--)
					{
						vel_Map_Astar[y][x]=(struct MAPPOINT_star*)calloc(1,sizeof( struct MAPPOINT_star));
						vel_Map_Astar[y][x]->obflag=0;
						vel_Map_Astar[y][x]->g = I;
						vel_Map_Astar[y][x]->next = NULL;
						vel_Map_Astar[y][x]->par = NULL;
						vel_Map_Astar[y][x]->x = x;
						vel_Map_Astar[y][x]->y = y;
						bool breakflag=0;
						if(app->obb_left)
						{
							if(y<=ymin)
							{
								if(x<xmin-15)
									vel_Map_Astar[y][x]->obflag=1;
							}
							else if(y>=ymax)
							{
								if(x<xmax-15)
									vel_Map_Astar[y][x]->obflag=1;
							}
							else if(ymax!=ymin)
							{
								if(x<(double)((y-ymin)*(xmax-xmin))/(double)(ymax-ymin)+xmin-15)
									vel_Map_Astar[y][x]->obflag=1;
							}
						}
						if(app->obb_right)
						{
							if(y<=ymin)
							{
								if(x>xmin+15)
									vel_Map_Astar[y][x]->obflag=1;
							}
							else if(y>=ymax)
							{
								if(x>xmax+15)
									vel_Map_Astar[y][x]->obflag=1;
							}
							else if(ymax!=ymin)
							{
								if(x>(double)((y-ymin)*(xmax-xmin))/(double)(ymax-ymin)+xmin+15)
									vel_Map_Astar[y][x]->obflag=1;
							}
						}
						if(vel_Map_Astar[y][x]->obflag==0)
						{
							for(int i=-7;i<=7;i++)
							{
								for(int j=-7;j<=7;j++)
								{
									if(vel_Map->MapPoint[y+j][x+i]==8 || vel_Map->MapPoint[y+j][x+i]==18 || vel_Map->MapPoint[y+j][x+i]==28)
									{
										vel_Map_Astar[y][x]->obflag=1;
										breakflag=1;
										break;
									}
								}
								if(breakflag)
									break;
							}
						}
					}
				}

				for(int x=x_left;x<=x_right;x++)
				{
					for(int y=y_bottom;y>=y_up;y--)
					{
						int k=0;
						for(int i=-1;i<=1;i++)
						{
							for(int j=-1;j<=1;j++)
							{
								if((i!=0) || (j!=0))
								{
									if(y+j>=y_up && y+j<=y_bottom && x+i>=x_left && x+i<=x_right)
										vel_Map_Astar[y][x]->nearnode[k]=vel_Map_Astar[y+j][x+i];
									else
										vel_Map_Astar[y][x]->nearnode[k]=NULL;
									k++;
								}
							}
						}
					}
				}

				struct MAPPOINT_star *node,*best;
				float g_value;
				
				open=(struct MAPPOINT_star*)calloc(1,sizeof( struct MAPPOINT_star));  	
				close=(struct MAPPOINT_star*)calloc(1,sizeof( struct MAPPOINT_star)); 

				struct MAPPOINT_star *start_point,*end_point;
				start_point =vel_Map_Astar[412][256];
				end_point = vel_Map_Astar[(int)(ydst)][(int)(xdst)];

				node = start_point;
				open->next = node;
				node->g = 0;

				int existop[8],existcl[8];

				while((node->x !=end_point->x) || (node->y != end_point->y))
				{
					for(int k=0;k<8;k++)
					{
						existop[k] = existopen(open,node->nearnode[k]);
						existcl[k] = existclose(close,node->nearnode[k]);
						if((node->nearnode[k]!=NULL) && (existop[k] == 0)&&(existcl[k] == 0) && (vel_Map_Astar[node->nearnode[k]->y][node->nearnode[k]->x]->obflag==0))
							addopen(node->nearnode[k]);
					}

					deleteopen(node);
					addclose(node);

					if(open->next==NULL)
					{
						bob7dist=true;
						break;
					}

					for(int k=0;k<8;k++)
					{
						if((node->nearnode[k]!=NULL) && (vel_Map_Astar[node->nearnode[k]->y][node->nearnode[k]->x]->obflag==0))
						{
							node->d[k] = sqrt((double)(pow((double)(node->x - (node->nearnode[k]->x)),2) + pow((double)(node->y - (node->nearnode[k]->y)),2)));
							if(node->par==NULL)
							{
								if(!(k==0 || k==3 || k==5))
									node->d[k] = I;
							}
							else
							{
								if(node->x-node->par->x==-1 && node->y-node->par->y==-1)
								{
									if(!(k==0 || k==1 || k==3))
										node->d[k] = I;
								}
								else if(node->x-node->par->x==-1 && node->y-node->par->y==0)
								{
									if(!(k==0 || k==1 || k==2))
										node->d[k] = I;
								}
								else if(node->x-node->par->x==-1 && node->y-node->par->y==1)
								{
									if(!(k==1 || k==2 || k==4))
										node->d[k] = I;
								}
								else if(node->x-node->par->x==0 && node->y-node->par->y==-1)
								{
									if(!(k==0 || k==3 || k==5))
										node->d[k] = I;
								}
								else if(node->x-node->par->x==0 && node->y-node->par->y==1)
								{
									if(!(k==2 || k==4 || k==7))
										node->d[k] = I;
								}
								else if(node->x-node->par->x==1 && node->y-node->par->y==-1)
								{
									if(!(k==3 || k==5 || k==6))
										node->d[k] = I;
								}
								else if(node->x-node->par->x==1 && node->y-node->par->y==0)
								{
									if(!(k==5 || k==6 || k==7))
										node->d[k] = I;
								}
								else if(node->x-node->par->x==1 && node->y-node->par->y==1)
								{
									if(!(k==4 || k==6 || k==7))
										node->d[k] = I;
								}
							}
							if(!existcl[k])
							{
								g_value = node->g + node->d[k];
								if ((g_value) < (node->nearnode[k]->g))
								{
									node->nearnode[k]->g = g_value;
									node->nearnode[k]->par = node;
								}
								node->nearnode[k]->h = sqrt((double)(pow((double)(end_point->x - (node->nearnode[k]->x)),2) + pow((double)(end_point->y - (node->nearnode[k]->y)),2)));
								node->nearnode[k]->f = node->nearnode[k]->g + node->nearnode[k]->h;
							}
						}
						else
						{
							node->d[k] = I;
						}
					}
					Sort();
					best = SearchBest();
					node = best;
				}

				if(bob7dist==false)
				{
					struct MAPPOINT_star *pathpoint =end_point;
					struct MAPPOINT_star *path =(struct MAPPOINT_star*)calloc(1,sizeof( struct MAPPOINT_star ));

					while(pathpoint != start_point)
					{
						pathpoint->next = path->next;
						path->next = pathpoint;
						pathpoint = pathpoint->par;
					}
					pathpoint->next = path->next;
					path->next = pathpoint;

					int pathpointnum = 0;

					struct MAPPOINT_star *copypath_todelete=(struct MAPPOINT_star*)calloc(1,sizeof( struct MAPPOINT_star ));

					struct MAPPOINT_star *copypath=copypath_todelete;
					copypath->next = path->next;
					for(;copypath->next!=NULL;)
					{
						pathpointnum++;
						copypath = copypath->next;
					}

					copypath->next = path->next;
					CvPoint2D64f *pc = (CvPoint2D64f *)malloc( pathpointnum*sizeof(CvPoint2D64f) );
					int chadiannum=0;
					for(int i=0;i<pathpointnum;i++)
					{
						pc[i].x=copypath->next->x;
						pc[i].y=copypath->next->y;
						copypath = copypath->next;
					}

					Bezier(pc,pathpointnum-1,MPoint);

					free(pc);

					free(path);
					free(copypath_todelete);
				}

				for(int x=x_left;x<=x_right;x++)
				{
					for(int y=y_bottom;y>=y_up;y--)
					{
						if(vel_Map_Astar[y][x]!=NULL)
							free(vel_Map_Astar[y][x]);
					}
				}
				free(open);
				free(close);

				if(bob7dist==false)
				{
					his_ap_tmp=m_GpsData.MaptoGPS(m_gps,m_dir,tp_map[12+i]);
					break;
				}
			}
		}

		if(i>0 && bob7dist==true)
		{
			tp_map[12-i].x = ap_map.x + (-36 + 3*(12-i))*cos((ap_dir-m_dir)*3.14159265/180);
			tp_map[12-i].y = ap_map.y + (-36 + 3*(12-i))*sin((ap_dir-m_dir)*3.14159265/180);

			int xmin;
			int ymin;
			int xmax;
			int ymax;
			if(tp_map[12-i].y<412)
			{
				xmin=tp_map[12-i].x;
				ymin=tp_map[12-i].y;
				xmax=256;
				ymax=412;
			}
			else
			{
				xmin=256;
				ymin=412;
				xmax=tp_map[12-i].x;
				ymax=tp_map[12-i].y;
			}

			bob7dist=false;

			if(((app->obb_left)&&(12-i<11))||((app->left_right==0) && app->bleftturnban && (12-i<12-pianyinum))||((app->obb_right)&&(12-i>13))||((app->left_right==0) && app->brightturnban && (12-i>12+pianyinum)))
				bob7dist=true;

			int xdst=tp_map[12-i].x;
			int ydst=tp_map[12-i].y;

			if(bob7dist==false)
			{
				for(int m=-7;m<=7;m++)
				{
					for(int n=-7;n<=7;n++)
					{
						if(ydst+m>511||ydst+m<0||xdst+n>511||xdst+n<0)
							continue;
						if(vel_Map->MapPoint[ydst+m][xdst+n] == 8 || vel_Map->MapPoint[ydst+m][xdst+n] == 18 || vel_Map->MapPoint[ydst+m][xdst+n] == 28)
						{
							bob7dist=true;
						}
					}
				}
			}

			if(bob7dist==false)
			{
				int y_bottom=412+15;
				if(y_bottom<ydst+15)
					y_bottom=ydst+15;
				int y_up=ydst-15;
				if(y_up>412-15)
					y_up=412-15;
				if(ydst>412-100)
					y_up=y_up-85;
				int x_left=106;
				int x_right=406;
				if(y_bottom>511)
					y_bottom=511;
				if(y_up<0)
					y_up=0;
				if(xdst>256)
				{
					x_left=106;
					x_right=xdst+150;
					if(x_right>511)
						x_right=511;
				}
				else
				{
					x_left=xdst-150;
					x_right=406;
					if(x_left<0)
						x_left=0;
				}
				
				struct MAPPOINT_star *vel_Map_Astar[512][512];
				//vel_Map = app->PercepMap;

				for(int x=x_left;x<=x_right;x++)
				{
					for(int y=y_bottom;y>=y_up;y--)
					{
						vel_Map_Astar[y][x]=(struct MAPPOINT_star*)calloc(1,sizeof( struct MAPPOINT_star));
						vel_Map_Astar[y][x]->obflag=0;
						vel_Map_Astar[y][x]->g = I;
						vel_Map_Astar[y][x]->next = NULL;
						vel_Map_Astar[y][x]->par = NULL;
						vel_Map_Astar[y][x]->x = x;
						vel_Map_Astar[y][x]->y = y;
						bool breakflag=0;
						if(app->obb_left)
						{
							if(y<=ymin)
							{
								if(x<xmin-15)
									vel_Map_Astar[y][x]->obflag=1;
							}
							else if(y>=ymax)
							{
								if(x<xmax-15)
									vel_Map_Astar[y][x]->obflag=1;
							}
							else if(ymax!=ymin)
							{
								if(x<(double)((y-ymin)*(xmax-xmin))/(double)(ymax-ymin)+xmin-15)
									vel_Map_Astar[y][x]->obflag=1;
							}
						}
						if(app->obb_right)
						{
							if(y<=ymin)
							{
								if(x>xmin+15)
									vel_Map_Astar[y][x]->obflag=1;
							}
							else if(y>=ymax)
							{
								if(x>xmax+15)
									vel_Map_Astar[y][x]->obflag=1;
							}
							else if(ymax!=ymin)
							{
								if(x>(double)((y-ymin)*(xmax-xmin))/(double)(ymax-ymin)+xmin+15)
									vel_Map_Astar[y][x]->obflag=1;
							}
						}
						if(vel_Map_Astar[y][x]->obflag==0)
						{
							for(int i=-7;i<=7;i++)
							{
								for(int j=-7;j<=7;j++)
								{
									if(vel_Map->MapPoint[y+j][x+i]==8 || vel_Map->MapPoint[y+j][x+i]==18 || vel_Map->MapPoint[y+j][x+i]==28)
									{
										vel_Map_Astar[y][x]->obflag=1;
										breakflag=1;
										break;
									}
								}
								if(breakflag)
									break;
							}
						}
					}
				}

				for(int x=x_left;x<=x_right;x++)
				{
					for(int y=y_bottom;y>=y_up;y--)
					{
						int k=0;
						for(int i=-1;i<=1;i++)
						{
							for(int j=-1;j<=1;j++)
							{
								if((i!=0) || (j!=0))
								{
									if(y+j>=y_up && y+j<=y_bottom && x+i>=x_left && x+i<=x_right)
										vel_Map_Astar[y][x]->nearnode[k]=vel_Map_Astar[y+j][x+i];
									else
										vel_Map_Astar[y][x]->nearnode[k]=NULL;
									k++;
								}
							}
						}
					}
				}

				struct MAPPOINT_star *node,*best;
				float g_value;
				
				open=(struct MAPPOINT_star*)calloc(1,sizeof( struct MAPPOINT_star));  	
				close=(struct MAPPOINT_star*)calloc(1,sizeof( struct MAPPOINT_star)); 

				struct MAPPOINT_star *start_point,*end_point;
				start_point =vel_Map_Astar[412][256];
				end_point = vel_Map_Astar[(int)(ydst)][(int)(xdst)];

				node = start_point;
				open->next = node;
				node->g = 0;

				int existop[8],existcl[8];

				while((node->x !=end_point->x) || (node->y != end_point->y))
				{
					for(int k=0;k<8;k++)
					{
						existop[k] = existopen(open,node->nearnode[k]);
						existcl[k] = existclose(close,node->nearnode[k]);
						if((node->nearnode[k]!=NULL) && (existop[k] == 0)&&(existcl[k] == 0) && (vel_Map_Astar[node->nearnode[k]->y][node->nearnode[k]->x]->obflag==0))
							addopen(node->nearnode[k]);
					}

					deleteopen(node);
					addclose(node);

					if(open->next==NULL)
					{
						bob7dist=true;
						break;
					}

					for(int k=0;k<8;k++)
					{
						if((node->nearnode[k]!=NULL) && (vel_Map_Astar[node->nearnode[k]->y][node->nearnode[k]->x]->obflag==0))
						{
							node->d[k] = sqrt((double)(pow((double)(node->x - (node->nearnode[k]->x)),2) + pow((double)(node->y - (node->nearnode[k]->y)),2)));
							if(node->par==NULL)
							{
								if(!(k==0 || k==3 || k==5))
									node->d[k] = I;
							}
							else
							{
								if(node->x-node->par->x==-1 && node->y-node->par->y==-1)
								{
									if(!(k==0 || k==1 || k==3))
										node->d[k] = I;
								}
								else if(node->x-node->par->x==-1 && node->y-node->par->y==0)
								{
									if(!(k==0 || k==1 || k==2))
										node->d[k] = I;
								}
								else if(node->x-node->par->x==-1 && node->y-node->par->y==1)
								{
									if(!(k==1 || k==2 || k==4))
										node->d[k] = I;
								}
								else if(node->x-node->par->x==0 && node->y-node->par->y==-1)
								{
									if(!(k==0 || k==3 || k==5))
										node->d[k] = I;
								}
								else if(node->x-node->par->x==0 && node->y-node->par->y==1)
								{
									if(!(k==2 || k==4 || k==7))
										node->d[k] = I;
								}
								else if(node->x-node->par->x==1 && node->y-node->par->y==-1)
								{
									if(!(k==3 || k==5 || k==6))
										node->d[k] = I;
								}
								else if(node->x-node->par->x==1 && node->y-node->par->y==0)
								{
									if(!(k==5 || k==6 || k==7))
										node->d[k] = I;
								}
								else if(node->x-node->par->x==1 && node->y-node->par->y==1)
								{
									if(!(k==4 || k==6 || k==7))
										node->d[k] = I;
								}
							}
							if(!existcl[k])
							{
								g_value = node->g + node->d[k];
								if ((g_value) < (node->nearnode[k]->g))
								{
									node->nearnode[k]->g = g_value;
									node->nearnode[k]->par = node;
								}
								node->nearnode[k]->h = sqrt((double)(pow((double)(end_point->x - (node->nearnode[k]->x)),2) + pow((double)(end_point->y - (node->nearnode[k]->y)),2)));
								node->nearnode[k]->f = node->nearnode[k]->g + node->nearnode[k]->h;
							}
						}
						else
						{
							node->d[k] = I;
						}
					}
					Sort();
					best = SearchBest();
					node = best;
				}

				if(bob7dist==false)
				{
					struct MAPPOINT_star *pathpoint =end_point;
					struct MAPPOINT_star *path =(struct MAPPOINT_star*)calloc(1,sizeof( struct MAPPOINT_star ));

					while(pathpoint != start_point)
					{
						pathpoint->next = path->next;
						path->next = pathpoint;
						pathpoint = pathpoint->par;
					}
					pathpoint->next = path->next;
					path->next = pathpoint;

					int pathpointnum = 0;

					struct MAPPOINT_star *copypath_todelete=(struct MAPPOINT_star*)calloc(1,sizeof( struct MAPPOINT_star ));

					struct MAPPOINT_star *copypath=copypath_todelete;
					copypath->next = path->next;
					for(;copypath->next!=NULL;)
					{
						pathpointnum++;
						copypath = copypath->next;
					}

					copypath->next = path->next;
					CvPoint2D64f *pc = (CvPoint2D64f *)malloc( pathpointnum*sizeof(CvPoint2D64f) );
					int chadiannum=0;
					for(int i=0;i<pathpointnum;i++)
					{
						pc[i].x=copypath->next->x;
						pc[i].y=copypath->next->y;
						copypath = copypath->next;
					}

					Bezier(pc,pathpointnum-1,MPoint);

					free(pc);

					free(path);
					free(copypath_todelete);
				}

				for(int x=x_left;x<=x_right;x++)
				{
					for(int y=y_bottom;y>=y_up;y--)
					{
						if(vel_Map_Astar[y][x]!=NULL)
							free(vel_Map_Astar[y][x]);
					}
				}
				free(open);
				free(close);

				if(bob7dist==false)
				{
					his_ap_tmp=m_GpsData.MaptoGPS(m_gps,m_dir,tp_map[12-i]);
					break;
				}
			}
		}
	}

	if(bstopret)
		return -1;
	else if(bob7dist==false)
		return 0;
	else
		return -1;
}


//保持车道的函数
int PathPlan::KeepLane(CvPoint2D64f m_gps,double m_dir,CvPoint2D64f ap_gps,double ap_dir,CvPoint2D64f (&MPoint)[200],bool byulukouzhiflag)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();	
	Map_Point ap_map = m_GpsData.APiontConver(m_gps,ap_gps,m_dir);
	CvPoint2D64f tp_map[23];//计算待选点
	CvPoint2D64f tp_gps[23];
	CvPoint2D64f hpoint[23][200];
	CvPoint2D64f ypoint[23][200];

	double apcost[23];
	double obcost[23];
	double hiscost[23];//代价

	int res;
	int range = 0;

	if(app->GPS_Speed*3.6<10 || app->left_right!=0)
		range = 8;
	else
		range = 8;
	if(app->left_right!=0)
		range = 6;

	int pianyinum;
	if(app->blanechanging)
		pianyinum=3;
	else
		pianyinum=2;

	bool bbrakeflag=0;

	bool bstopret=false;

	
	if((app->left_right==0) || app->GPS_Speed<22/3.6)
	{
		// 比 keeplanewan 多了一重判断
		if(app->bnearstop || (app->kuaisulufulu==1))
		{
			if(SearchFrontOb(vel_Map,5))
				bstopret=true;
		}
		else
		{
			if(SearchFrontOb(vel_Map,6))
				bstopret=true;
		}

		if(SearchFrontOb(vel_Map,range))
			bbrakeflag=1;
	}

//////计算轨迹端点，轨迹代价
	for (int i = 0; i<23;i++)
	{
		// -(36 + 3*i) 是什么含义？
		tp_map[i].x = ap_map.x + (-36 + 3*i) * cos((ap_dir-m_dir) * 3.14159265/180);
		tp_map[i].y = ap_map.y + (-36 + 3*i) * sin((ap_dir-m_dir) * 3.14159265/180);
		apcost[i] = abs(36 - 3*i);
		tp_gps[i] = m_GpsData.MaptoGPS(m_gps,m_dir,tp_map[i]);

		double ln = app->GPS_Speed*2;
		if(ln>35)ln = 35;
		if(ln<5)ln = 5;

		// 计算 hermite 曲线，存放到 hpoint[i][200] 中
		if(app->range_flag)
			Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],0.2,5);//Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],2,5);
		else if(app->GPS_Speed > 22/3.6)
			Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],0.2,0.5);
		else
			Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],0.2,5);

		// 下采样 hermite 曲线, 存放到 ypoint[i][200]中
		YanChang(m_gps,m_dir,hpoint[i],ap_dir,ypoint[i]);

		// keeplanewan用的是 ObstacleCost_lukouwan, range 从 2.2 增加到2.5
		obcost[i] = ObstacleCost(hpoint[i],vel_Map_path, 8, 2.2);

		// his_ap 的含义？
		if(his_ap.x==0)
			hiscost[i]=0;
		else
			hiscost[i] = m_GpsData.GetDistance(hpoint[i][199].x,hpoint[i][199].y,his_ap.x,his_ap.y);
	}

	if((app->bbanmaxiannoraozhang || app->bspecialraozhangcanshu)&&(app->bGpsGanrao==0))
	{
		app->obb_left=true;
		app->obb_right=true;
	}

	int num = 24;
	double cost,tempcost;
	cost = 40000;
	tempcost = 40000;
	
	for(int i = 0;i<12;i++)
	{
		//if(!(app->bdirectrunreq&&(11+i<11||11+i>13)))
		{
			tempcost = 0.4 * apcost[11+i] * apcost[11+i] + 12 * obcost[11+i] * obcost[11+i] + hiscost[11+i] * hiscost[11+i];
			if (cost > tempcost && (!(((app->obb_left) && (11+i<11)) || (byulukouzhiflag && (11+i<10||11+i>14)) 
				|| ((app->GPS_Speed > 22/3.6 || ((app->left_right==0) && app->bleftturnban))&&(11+i<12-pianyinum))||(app->obb_left&&app->left_right!=0&&app->GPS_Speed>22/3.6&&11+i<11)||(app->obb_right&&app->left_right!=0&&app->GPS_Speed>22/3.6&&11+i>13)||((app->obb_right)&&(11+i>13))||((app->GPS_Speed>22/3.6|| ((app->left_right==0) && app->brightturnban))&&(11+i>12+pianyinum)))))
			{
				cost = tempcost;
				num = 11+i;
			}
		}

		// 无论上面是否满足，只要下面满足, num = 11 - i; 
		//if(!(app->bdirectrunreq&&(11-i<11||11-i>13)))
		{
			tempcost = 0.4 * apcost[11-i] * apcost[11-i] + 12 * obcost[11-i] * obcost[11-i] + hiscost[11-i] * hiscost[11-i];
			if (cost > tempcost && (!(((app->obb_left) && (11-i<11)) || (byulukouzhiflag && (11-i<10||11-i>14))
				||((app->GPS_Speed > 22/3.6|| ((app->left_right==0) && app->bleftturnban))&&(11-i<12-pianyinum))||(app->obb_left&&app->left_right!=0&&app->GPS_Speed>22/3.6&&11-i<11)||(app->obb_right&&app->left_right!=0&&app->GPS_Speed>22/3.6&&11-i>13)||((app->obb_right)&&(11-i>13))||((app->GPS_Speed>22/3.6|| ((app->left_right==0) && app->brightturnban))&&(11-i>12+pianyinum)))))
			{
				cost = tempcost;
				num = 11-i;
			}
		}
	}

	app->critical_pathplanroad.Lock();
	cvClearSeq( plan_road );
	for(int j = 0; j<23; j++)
	{
		for(int i = 0; i<200; i++)
		{
			cvSeqPush(plan_road, &hpoint[j][i] );
		}
	}	
	app->critical_pathplanroad.Unlock();

/////平滑路径
	if(app->kuaisulufulu==1)
		num=12;

	if (num<24)
	{
		for(int i = 0;i<200;i++)
		{
			MPoint[i]=hpoint[num][i];
		}

		res = 1;
		his_ap_tmp = hpoint[num][199];

		bpathfound=true;

		if(bstopret)
			return -1;
		else if(bbrakeflag)
			return 0;
		else
			return res;
	}

////尖锐路径
	if (num == 24)
	{	
		int num2 = 24;
		cost = 160000000;
		tempcost = 160000000;

		double ln = app->GPS_Speed * 2;
		if(ln>35) ln = 35;

		for(int i = 0; i<23;i++)
		{
			if(app->GPS_Speed > 22/3.6)
				Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],0.2,0.5);
			else
				Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],0.2,5);

			obcost[i] = ObstacleCost(hpoint[i],vel_Map_path,8,2.2);
			YanChang(m_gps,m_dir,hpoint[i],ap_dir,ypoint[i]);
		}

		for(int i = 0;i<12;i++)
		{
			//if(!(app->bdirectrunreq&&(11+i<11||11+i>13)))
			{
				tempcost = 0.4 * apcost[11+i]*apcost[11+i]+12*obcost[11+i]*obcost[11+i]+hiscost[11+i]*hiscost[11+i];
				if (cost>tempcost && (!(((app->obb_left)&&(11+i<11))||(byulukouzhiflag&&(11+i<10||11+i>14))||((app->GPS_Speed>22/3.6|| ((app->left_right==0) && app->bleftturnban))&&(11+i<12-pianyinum))||(app->obb_left&&app->left_right!=0&&app->GPS_Speed>22/3.6&&11+i<11)||(app->obb_right&&app->left_right!=0&&app->GPS_Speed>22/3.6&&11+i>13)||((app->obb_right)&&(11+i>13))||((app->GPS_Speed>22/3.6|| ((app->left_right==0) && app->brightturnban))&&(11+i>12+pianyinum)))))
				{
					cost = tempcost;
					num2 = 11+i;
				}
			}
			//if(!(app->bdirectrunreq&&(11-i<11||11-i>13)))
			{
				tempcost = 0.4*apcost[11-i]*apcost[11-i]+12*obcost[11-i]*obcost[11-i]+hiscost[11-i]*hiscost[11-i];
				if (cost>tempcost && (!(((app->obb_left)&&(11-i<11))||(byulukouzhiflag&&(11-i<10||11-i>14))||((app->GPS_Speed>22/3.6|| ((app->left_right==0) && app->bleftturnban))&&(11-i<12-pianyinum))||(app->obb_left&&app->left_right!=0&&app->GPS_Speed>22/3.6&&11-i<11)||(app->obb_right&&app->left_right!=0&&app->GPS_Speed>22/3.6&&11-i>13)||((app->obb_right)&&(11-i>13))||((app->GPS_Speed>22/3.6|| ((app->left_right==0) && app->brightturnban))&&(11-i>12+pianyinum)))))
				{
					cost = tempcost;
					num2 = 11-i;
				}
			}
		}
		
		/////平滑路径
		if (num2 < 24)
		{

			for(int i = 0;i<200;i++)
			{
				MPoint[i]=hpoint[num2][i];
			}

			res = 3;
			his_ap_tmp = hpoint[num2][199];

			if(cost<40000)
				bpathfound=true;

			if(bstopret)
				return -1;
			else if(bbrakeflag)
				return 0;
			else
				return res;
		}
		else
		{
			if(bstopret)
				return -1;
			else
				return 0;
		}
	}
	
}

// Goal: 该函数根据当前GPS坐标与朝向，目标GPS坐标与朝向，byulukouzhiflag的值，得到从当前GPS坐标到目标GPS坐标的存储于 MPPoint[200]中
// Method: 计算 cost function，cost function 与障碍物距离相关
// 返回值的含义？ 0， -1， 3，4
int PathPlan::KeepLanewan(CvPoint2D64f m_gps,double m_dir,CvPoint2D64f ap_gps,double ap_dir,CvPoint2D64f (&MPoint)[200],bool byulukouzhiflag)  
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();	
	Map_Point ap_map = m_GpsData.APiontConver(m_gps, ap_gps, m_dir);  // GPS坐标转到地图坐标
	
	CvPoint2D64f tp_map[23];//计算待选点
	CvPoint2D64f tp_gps[23];
	CvPoint2D64f hpoint[23][200];
	CvPoint2D64f ypoint[23][200];

	double apcost[23];
	double obcost[23];
	double hiscost[23];//代价

	int res;
	
	int range = 0;

	if(app->GPS_Speed*3.6<10 || app->left_right!=0)
		range = 8;
	else
		range = 8;
	if(app->left_right!=0)
		range = 6;

	int pianyinum;
	if(app->blanechanging)
		pianyinum=3;
	else
		pianyinum=2;

	bool bbrakeflag=0;
	bool bstopret=false;

	// 如果速度小于时速22，搜索5米内前面的障碍物，如果有，则设置标志位为1
	if((app->left_right==0) || app->GPS_Speed<22/3.6)
	{
		if(SearchFrontOb(vel_Map,range))  // range范围
			bbrakeflag=1;

		if(SearchFrontOb(vel_Map,5))  // 5米
			bstopret=true;
	}
	
	//计算轨迹端点，轨迹代价
	// 在目标 GPS 坐标点处，结合当前朝向与目标点朝向确认23个点的展开方向，最终得到23个点的坐标，存储在 tp_map[i]中
	for (int i = 0; i<23;i++)
	{
		// 
		tp_map[i].x = ap_map.x + (-36 + 3*i)* cos((ap_dir-m_dir) * 3.14159265/180);
		tp_map[i].y = ap_map.y + (-36 + 3*i)* sin((ap_dir-m_dir) * 3.14159265/180);
		
		// 该cost偏向于选择中间点，越靠近两边的点cost越大
		apcost[i] = abs(36 - 3*i);
		
		// 将tp_map[23]坐标重新装换为GPS坐标，存储在tp_gps[23]中
		tp_gps[i] = m_GpsData.MaptoGPS(m_gps,m_dir,tp_map[i]);

		// 车速的限制，具体目的？
		double ln = app->GPS_Speed*2;
		if(ln>35)ln = 35;
		if(ln<5)ln = 5;

		// Note： 得到的 hpoint[i][200]是GPS坐标
		if(app->range_flag)
			Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],0.2,5);
		else if(app->GPS_Speed>22/3.6)
			Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],0.2,0.5);
		else
			Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],0.2,5);

		YanChang(m_gps,m_dir,hpoint[i],ap_dir,ypoint[i]);
		
		// 计算障碍物的cost， vel_Map_path = MapPoint[512][512]
		obcost[i] = ObstacleCost_lukouwan(hpoint[i], vel_Map_path, 8, 2.5); // 该处的8并没有用到，2.5是设置的最小距离障碍物的距离
		
		// his_ap 是什么含义？
		if(his_ap.x==0)
			hiscost[i]=0;
		else
			hiscost[i] = m_GpsData.GetDistance(hpoint[i][199].x,hpoint[i][199].y,his_ap.x,his_ap.y);
	}

	app->obb_left=0;
	app->obb_right=0;

	// num 和 num2 是什么意思？
	int num=24;
	double cost,tempcost;
	cost = 40000;
	tempcost = 40000;

	// 这里是选择最优路径的代码？ num的编号就是最优路径吗？
	for(int i = 0;i<12;i++)
	{
		//if(!(app->bdirectrunreq&&(11+i<11||11+i>13)))
		{
			tempcost = 0.4 * apcost[11+i] * apcost[11+i] + 12 * obcost[11+i] * obcost[11+i] + hiscost[11+i] * hiscost[11+i];
			// obb_left 左后来车
			// app->left_right == 0,绕障幅度小
			if (cost>tempcost && (!(((app->obb_left)&&(11+i<11))||(byulukouzhiflag&&(11+i<10||11+i>14))||((app->GPS_Speed>22/3.6|| ((app->left_right==0) && app->bleftturnban))&&(11+i<12-pianyinum))||(app->obb_left&&app->left_right!=0&&app->GPS_Speed>22/3.6&&11+i<11)||(app->obb_right&&app->left_right!=0&&app->GPS_Speed>22/3.6&&11+i>13)||((app->obb_right)&&(11+i>13))||((app->GPS_Speed>22/3.6|| ((app->left_right==0) && app->brightturnban))&&(11+i>12+pianyinum)))))
			{
				cost = tempcost;
				num = 11+i;
			}
		}
		//if(!(app->bdirectrunreq&&(11-i<11||11-i>13)))
		{
			tempcost = 0.4*apcost[11-i]*apcost[11-i]+12*obcost[11-i]*obcost[11-i]+hiscost[11-i]*hiscost[11-i];
			if (cost>tempcost && (!(((app->obb_left)&&(11-i<11))||(byulukouzhiflag&&(11-i<10||11-i>14))||((app->GPS_Speed>22/3.6|| ((app->left_right==0) && app->bleftturnban))&&(11-i<12-pianyinum))||(app->obb_left&&app->left_right!=0&&app->GPS_Speed>22/3.6&&11-i<11)||(app->obb_right&&app->left_right!=0&&app->GPS_Speed>22/3.6&&11-i>13)||((app->obb_right)&&(11-i>13))||((app->GPS_Speed>22/3.6|| ((app->left_right==0) && app->brightturnban))&&(11-i>12+pianyinum)))))
			{
				cost = tempcost;
				num = 11-i;
			}
		}
	}

	app->critical_pathplanroad.Lock();
	cvClearSeq( plan_road );
	for(int j = 0; j<23; j++)
	{
		for(int i = 0; i<200; i++)
		{
			cvSeqPush(plan_road, &hpoint[j][i] );
		}
	}	
	app->critical_pathplanroad.Unlock();

/////平滑路径
	if(app->kuaisulufulu==1)
		num=12;

	if (num<24)
	{
		
		for(int i = 0;i<200;i++)
		{
			MPoint[i]=hpoint[num][i];
		}

		res = 1;
		his_ap_tmp = hpoint[num][199];

		bpathfound=true;

		if(bstopret)
			return -1;  
		else if(bbrakeflag)
			return 0;
		else
			return res;
	}

////尖锐路径
	if (num==24)
	{	
		int num2=24;
		cost = 160000000000;
		tempcost = 160000000000;

		double ln = app->GPS_Speed*2;
		if(ln>35)ln = 35;

		for(int i = 0; i<23;i++)
		{
			if(app->GPS_Speed>22/3.6)
				Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],0.2,0.5);
			else
				Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],0.2,5);
			obcost[i] = ObstacleCost_lukouwan(hpoint[i],vel_Map_path,8,2.5);
			YanChang(m_gps,m_dir,hpoint[i],ap_dir,ypoint[i]);
		}

		for(int i = 0;i<12;i++)
		{
			//if(!(app->bdirectrunreq&&(11+i<11||11+i>13)))
			{
				tempcost = 0.4*apcost[11+i]*apcost[11+i]+12*obcost[11+i]*obcost[11+i]+hiscost[11+i]*hiscost[11+i];
				if (cost>tempcost && (!(((app->obb_left)&&(11+i<11))||(byulukouzhiflag&&(11+i<10||11+i>14))
					||((app->GPS_Speed>22/3.6 || ((app->left_right==0) && app->bleftturnban))&&(11+i<12-pianyinum))
					||(app->obb_left&&app->left_right!=0&&app->GPS_Speed>22/3.6&&11+i<11)
					||(app->obb_right&&app->left_right!=0&&app->GPS_Speed>22/3.6&&11+i>13)
					||((app->obb_right)&&(11+i>13))||((app->GPS_Speed>22/3.6|| ((app->left_right==0) && app->brightturnban))&&(11+i>12+pianyinum)))))
				{
					cost = tempcost;
					num2 = 11+i;
				}
			}
			//if(!(app->bdirectrunreq&&(11-i<11||11-i>13)))
			{
				tempcost = 0.4*apcost[11-i]*apcost[11-i]+12*obcost[11-i]*obcost[11-i]+hiscost[11-i]*hiscost[11-i];
				if (cost>tempcost && (!(((app->obb_left)&&(11-i<11))||(byulukouzhiflag&&(11-i<10||11-i>14))||((app->GPS_Speed>22/3.6|| ((app->left_right==0) && app->bleftturnban))&&(11-i<12-pianyinum))||(app->obb_left&&app->left_right!=0&&app->GPS_Speed>22/3.6&&11-i<11)||(app->obb_right&&app->left_right!=0&&app->GPS_Speed>22/3.6&&11-i>13)||((app->obb_right)&&(11-i>13))||((app->GPS_Speed>22/3.6|| ((app->left_right==0) && app->brightturnban))&&(11-i>12+pianyinum)))))
				{
					cost = tempcost;
					num2 = 11-i;
				}
			}
		}
		
		/////平滑路径
		if (num2<24)
		{

			for(int i = 0;i<200;i++)
			{
				MPoint[i]=hpoint[num2][i];
			}

			res = 3;
			his_ap_tmp = hpoint[num2][199];

			if(cost<8000000)
				bpathfound=true;

			if(bstopret)
				return -1;
			else if(bbrakeflag)
				return 0;
			else
				return res;
		}
		else
		{	
			if(bstopret)
				return -1;
			else
				return 0;
		}
	}
	
}

int PathPlan::KeepLane_lukou(CvPoint2D64f m_gps,double m_dir,CvPoint2D64f ap_gps,double ap_dir,CvPoint2D64f (&MPoint)[200],int lukoufx)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();	
	Map_Point ap_map = m_GpsData.APiontConver(m_gps,ap_gps,m_dir);
	CvPoint2D64f tp_map[23];
	CvPoint2D64f tp_gps[23];
	CvPoint2D64f hpoint[23][200];
	CvPoint2D64f ypoint[23][200];

	double apcost[23];
	double obcost[23];
	double hiscost[23];

	int res;
	
	int range = 5;
	if(lukoufx==1)
		range = 6;
	if(app->bnearstop)
		range = 5;

	int pianyinum=3;

	bool bstopret=false;
	
	if(SearchFrontOb(vel_Map,range))
		bstopret=true;
		//return -1;

	for (int i = 0; i<23;i++)
	{
		tp_map[i].x = ap_map.x + (-36 + 3*i)*cos((ap_dir-m_dir)*3.14159265/180);
		tp_map[i].y = ap_map.y + (-36 + 3*i)*sin((ap_dir-m_dir)*3.14159265/180);
		apcost[i] = abs(36 - 3*i);
		tp_gps[i] = m_GpsData.MaptoGPS(m_gps,m_dir,tp_map[i]);

		double ln = app->GPS_Speed*2;
		if(ln>35)ln = 35;
		if(ln<5)ln = 5;
		
		Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],0.2,3);//Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],2,3);
		YanChang(m_gps,m_dir,hpoint[i],ap_dir,ypoint[i]);
		obcost[i] = ObstacleCost_lukou(hpoint[i],vel_Map,8,2.2);
		if(his_ap.x==0)
			hiscost[i]=0;
		else
			hiscost[i] = m_GpsData.GetDistance(hpoint[i][199].x,hpoint[i][199].y,his_ap.x,his_ap.y);
	}

	int num=24;
	double cost,tempcost;
	cost = 40000;
	tempcost = 40000;
	if((app->bbanmaxiannoraozhang)&&(app->bGpsGanrao==0))
	{
		app->obb_left=true;
		app->obb_right=true;
	}
	{
		for(int i = 0;i<12;i++)
		{
			//if(!(app->bdirectrunreq&&(11+i<11||11+i>13)))
			{
				tempcost = 0.4*apcost[11+i]*apcost[11+i]+12*obcost[11+i]*obcost[11+i]+hiscost[11+i]*hiscost[11+i];
				if (cost>tempcost && (!(((app->obb_left)&&(11+i<10))||((app->obb_right)&&(11+i>14)))))
				{
					cost = tempcost;
					num = 11+i;
				}
			}
			//if(!(app->bdirectrunreq&&(11-i<11||11-i>13)))
			{
				tempcost = 0.4*apcost[11-i]*apcost[11-i]+12*obcost[11-i]*obcost[11-i]+hiscost[11-i]*hiscost[11-i];
				if (cost>tempcost && (!(((app->obb_left)&&(11-i<10))||((app->obb_right)&&(11-i>14)))))
				{
					cost = tempcost;
					num = 11-i;
				}
			}
		}
	}
	app->critical_pathplanroad.Lock();
	cvClearSeq( plan_road );
	for(int j = 0; j<23; j++)
	{
		for(int i = 0; i<200; i++)
		{
			cvSeqPush(plan_road, &hpoint[j][i] );
		}
	}	
	app->critical_pathplanroad.Unlock();
/////平滑路径
	if (num<24)
	{
		
		for(int i = 0;i<200;i++)
		{
			MPoint[i]=hpoint[num][i];
		}
		/*
		if (abs(num-12)<4)
			res = 1;
		else
		{
			res = 2;
			changedis = (num-12)*0.5;

		}
		*/
		res = 1;
		his_ap_tmp = hpoint[num][199];

		bpathfound=true;

		if(bstopret)
			return -1;
		else
			return res;
	}

////尖锐路径
	if (num==24)
	{	
		int num2=24;
		cost = 160000000;
		tempcost = 160000000;

		double ln = app->GPS_Speed*2;
		if(ln>35)ln = 35;

		for(int i = 0; i<23;i++)
		{
			Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],0.2,0.5);
			obcost[i] = ObstacleCost_lukou(hpoint[i],vel_Map,8,2.2);
			YanChang(m_gps,m_dir,hpoint[i],ap_dir,ypoint[i]);
		}

		if((app->bbanmaxiannoraozhang)&&(app->bGpsGanrao==0))
		{
			app->obb_left=true;
			app->obb_right=true;
		}
		{
			for(int i = 0;i<12;i++)
			{
				//if(!(app->bdirectrunreq&&(11+i<11||11+i>13)))
				{
					tempcost = 0.4*apcost[11+i]*apcost[11+i]+12*obcost[11+i]*obcost[11+i]+hiscost[11+i]*hiscost[11+i];
					if (cost>tempcost && (!(((app->obb_left)&&(11+i<10))||((app->obb_right)&&(11+i>14)))))
					{
						cost = tempcost;
						num2 = 11+i;
					}
				}
				//if(!(app->bdirectrunreq&&(11-i<11||11-i>13)))
				{
					tempcost = 0.4*apcost[11-i]*apcost[11-i]+12*obcost[11-i]*obcost[11-i]+hiscost[11-i]*hiscost[11-i];
					if (cost>tempcost && (!(((app->obb_left)&&(11-i<10))||((app->obb_right)&&(11-i>14)))))
					{
						cost = tempcost;
						num2 = 11-i;
					}
				}
			}
		}

		if (num2<24)
		{
			for(int i = 0;i<200;i++)
			{
				MPoint[i]=hpoint[num2][i];
			}

			res = 3;
			his_ap_tmp = hpoint[num2][199];

			if(cost<40000)
				bpathfound=true;

			if(bstopret)
				return -1;
			else 
				return res;
		}
		else
		{
			//his_ap = ap_gps;
			if(bstopret)
				return -1;
			else 
				return 0;
		}
	}
	

}

int PathPlan::KeepLane_lukouwan(CvPoint2D64f m_gps,double m_dir,CvPoint2D64f ap_gps,double ap_dir,CvPoint2D64f (&MPoint)[200],int lukoufx)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();	
	Map_Point ap_map = m_GpsData.APiontConver(m_gps,ap_gps,m_dir);
	CvPoint2D64f tp_map[23];
	CvPoint2D64f tp_gps[23];
	CvPoint2D64f hpoint[23][200];
	CvPoint2D64f ypoint[23][200];

	double apcost[23];
	double obcost[23];
	double hiscost[23];

	int res;
	
	int range = 5;
	/*
	if(app->inter_UTURN)
		range = 4;
		*/
	if(lukoufx==1)
		range = 6;

	int pianyinum=3;

	bool bstopret=false;
	
	if(SearchFrontOb(vel_Map,range))
		bstopret=true;
		//return -1;

	for (int i = 0; i<23;i++)
	{
		tp_map[i].x = ap_map.x + (-36 + 3*i)*cos((ap_dir-m_dir)*3.14159265/180);
		tp_map[i].y = ap_map.y + (-36 + 3*i)*sin((ap_dir-m_dir)*3.14159265/180);
		apcost[i] = abs(36 - 3*i);
		tp_gps[i] = m_GpsData.MaptoGPS(m_gps,m_dir,tp_map[i]);

		double ln = app->GPS_Speed*2;
		if(ln>35)ln = 35;
		if(ln<5)ln = 5;
		
		Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],0.2,3);//Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],2,3);
		YanChang(m_gps,m_dir,hpoint[i],ap_dir,ypoint[i]);
		obcost[i] = ObstacleCost_lukouwan(hpoint[i],vel_Map,8,2.5);//4//3
		if(his_ap.x==0)
			hiscost[i]=0;
		else
			hiscost[i] = m_GpsData.GetDistance(hpoint[i][199].x,hpoint[i][199].y,his_ap.x,his_ap.y);
	}

	int num=24;
	double cost,tempcost;
	cost = 40000;
	tempcost = 40000;
	if((app->bbanmaxiannoraozhang)&&(app->bGpsGanrao==0))
	{
		app->obb_left=true;
		app->obb_right=true;
	}
	{
		for(int i = 0;i<12;i++)
		{
			//if(!(app->bdirectrunreq&&(11+i<11||11+i>13)))
			{
				tempcost = 0.4*apcost[11+i]*apcost[11+i]+12*obcost[11+i]*obcost[11+i]+0.6*hiscost[11+i]*hiscost[11+i];//tempcost = 0.4*apcost[11+i]*apcost[11+i]+12*obcost[11+i]*obcost[11+i]+hiscost[11+i]*hiscost[11+i];//0.5
				if (cost>tempcost && (!(((app->obb_left)&&(11+i<10))||((app->obb_right)&&(11+i>14)))))
				{
					cost = tempcost;
					num = 11+i;
				}
			}
			//if(!(app->bdirectrunreq&&(11-i<11||11-i>13)))
			{
				tempcost = 0.4*apcost[11-i]*apcost[11-i]+12*obcost[11-i]*obcost[11-i]+0.6*hiscost[11-i]*hiscost[11-i];//tempcost = 0.4*apcost[11-i]*apcost[11-i]+12*obcost[11-i]*obcost[11-i]+hiscost[11-i]*hiscost[11-i];//0.5
				if (cost>tempcost && (!(((app->obb_left)&&(11-i<10))||((app->obb_right)&&(11-i>14)))))
				{
					cost = tempcost;
					num = 11-i;
				}
			}
		}
	}
	app->critical_pathplanroad.Lock();
	cvClearSeq( plan_road );
	for(int j = 0; j<23; j++)
	{
		for(int i = 0; i<200; i++)
		{
			cvSeqPush(plan_road, &hpoint[j][i] );
		}
	}	
	app->critical_pathplanroad.Unlock();
/////平滑路径
	if (num<24)
	{
		
		for(int i = 0;i<200;i++)
		{
			MPoint[i]=hpoint[num][i];
		}
		/*
		if (abs(num-12)<4)
			res = 1;
		else
		{
			res = 2;
			changedis = (num-12)*0.5;

		}
		*/
		res = 1;
		his_ap_tmp = hpoint[num][199];

		bpathfound=true;

		if(bstopret)
			return -1;
		else
			return res;
	}

////尖锐路径
	if (num==24)
	{	
		int num2=24;
		cost = 160000000000;
		tempcost = 160000000000;

		double ln = app->GPS_Speed*2;
		if(ln>35)ln = 35;

		for(int i = 0; i<23;i++)
		{
			Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],0.2,0.5);
			obcost[i] = ObstacleCost_lukouwan(hpoint[i],vel_Map,8,2.5);//4//3
			YanChang(m_gps,m_dir,hpoint[i],ap_dir,ypoint[i]);
		}

		if((app->bbanmaxiannoraozhang)&&(app->bGpsGanrao==0))
		{
			app->obb_left=true;
			app->obb_right=true;
		}
		{
			for(int i = 0;i<12;i++)
			{
				//if(!(app->bdirectrunreq&&(11+i<11||11+i>13)))
				{
					tempcost = 0.4*apcost[11+i]*apcost[11+i]+12*obcost[11+i]*obcost[11+i]+0.6*hiscost[11+i]*hiscost[11+i];//tempcost = 0.4*apcost[11+i]*apcost[11+i]+12*obcost[11+i]*obcost[11+i]+hiscost[11+i]*hiscost[11+i];//0.5
					if (cost>tempcost && (!(((app->obb_left)&&(11+i<10))||((app->obb_right)&&(11+i>14)))))
					{
						cost = tempcost;
						num2 = 11+i;
					}
				}
				//if(!(app->bdirectrunreq&&(11-i<11||11-i>13)))
				{
					tempcost = 0.4*apcost[11-i]*apcost[11-i]+12*obcost[11-i]*obcost[11-i]+0.6*hiscost[11-i]*hiscost[11-i];//tempcost = 0.4*apcost[11-i]*apcost[11-i]+12*obcost[11-i]*obcost[11-i]+hiscost[11-i]*hiscost[11-i];//0.5
					if (cost>tempcost && (!(((app->obb_left)&&(11-i<10))||((app->obb_right)&&(11-i>14)))))
					{
						cost = tempcost;
						num2 = 11-i;
					}
				}
			}
		}

		if (num2<24)
		{
			for(int i = 0;i<200;i++)
			{
				MPoint[i]=hpoint[num2][i];
			}

			res = 3;
			his_ap_tmp = hpoint[num2][199];

			if(cost<8000000)
				bpathfound=true;

			if(bstopret)
				return -1;
			else
				return res;
		}
		else
		{
			//his_ap = ap_gps;
			if(bstopret)
				return -1;
			else
				return 0;
		}
	}
	

}
int PathPlan::KeepLane_lukou1(CvPoint2D64f m_gps,double m_dir,CvPoint2D64f ap_gps,double ap_dir,CvPoint2D64f (&MPoint)[200],int lukoufx)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();	
	Map_Point ap_map = m_GpsData.APiontConver(m_gps,ap_gps,m_dir);
	CvPoint2D64f tp_map[23];
	CvPoint2D64f tp_gps[23];
	CvPoint2D64f hpoint[23][200];
	CvPoint2D64f ypoint[23][200];

	double apcost[23];
	double obcost[23];
	double hiscost[23];

	int res;
	
	int range = 5;
	if(lukoufx==1)
		range = 6;

	int pianyinum=3;
	
	if(SearchFrontOb(vel_Map,range))
		return -1;

	for (int i = 0; i<23;i++)
	{
		tp_map[i].x = ap_map.x + (-48 + 4*i)*cos((ap_dir-m_dir)*3.14159265/180);
		tp_map[i].y = ap_map.y + (-48 + 4*i)*sin((ap_dir-m_dir)*3.14159265/180);
		apcost[i] = abs(48 - 4*i);
		tp_gps[i] = m_GpsData.MaptoGPS(m_gps,m_dir,tp_map[i]);

		double ln = app->GPS_Speed*2;
		if(ln>35)ln = 35;
		if(ln<5)ln = 5;
		
		Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],2,3);
		YanChang(m_gps,m_dir,hpoint[i],ap_dir,ypoint[i]);
		obcost[i] = ObstacleCost_lukouwan(hpoint[i],vel_Map,8,3);//4
		if(his_ap.x==0)
			hiscost[i]=0;
		else
			hiscost[i] = m_GpsData.GetDistance(hpoint[i][199].x,hpoint[i][199].y,his_ap.x,his_ap.y);
	}

	int num=24;
	double cost,tempcost;
	cost = 40000;
	tempcost = 40000;
	for(int i = 0;i<12;i++)
	{
		//if(!(app->bdirectrunreq&&(11+i<11||11+i>13)))
		{
			tempcost = 0.4*apcost[11+i]*apcost[11+i]+12*obcost[11+i]*obcost[11+i]+hiscost[11+i]*hiscost[11+i];
			if (cost>tempcost && (!(((app->obb_left)&&(11+i<11))||((app->obb_right)&&(11+i>13)))))
			{
				cost = tempcost;
				num = 11+i;
			}
		}
		//if(!(app->bdirectrunreq&&(11-i<11||11-i>13)))
		{
			tempcost = 0.4*apcost[11-i]*apcost[11-i]+12*obcost[11-i]*obcost[11-i]+hiscost[11-i]*hiscost[11-i];
			if (cost>tempcost && (!(((app->obb_left)&&(11-i<11))||((app->obb_right)&&(11-i>13)))))
			{
				cost = tempcost;
				num = 11-i;
			}
		}
	}
	app->critical_pathplanroad.Lock();
	cvClearSeq( plan_road );
	for(int j = 0; j<23; j++)
	{
		for(int i = 0; i<200; i++)
		{
			cvSeqPush(plan_road, &hpoint[j][i] );
		}
	}	
	app->critical_pathplanroad.Unlock();
/////平滑路径
	if (num<24)
	{
		
		for(int i = 0;i<200;i++)
		{
			MPoint[i]=hpoint[num][i];
		}
		/*
		if (abs(num-12)<4)
			res = 1;
		else
		{
			res = 2;
			changedis = (num-12)*0.5;

		}
		*/
		res = 1;
		his_ap = hpoint[num][199];

		return res;
	}

////尖锐路径
	if (num==24)
	{	
		int num2=24;
		cost = 160000000;
		tempcost = 160000000;

		double ln = app->GPS_Speed*2;
		if(ln>35)ln = 35;

		for(int i = 0; i<23;i++)
		{
			Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],0.2,0.5);
			obcost[i] = ObstacleCost_lukouwan(hpoint[i],vel_Map,8,3);//4
			YanChang(m_gps,m_dir,hpoint[i],ap_dir,ypoint[i]);
		}

		for(int i = 0;i<12;i++)
		{
			//if(!(app->bdirectrunreq&&(11+i<11||11+i>13)))
			{
				tempcost = 0.4*apcost[11+i]*apcost[11+i]+12*obcost[11+i]*obcost[11+i]+hiscost[11+i]*hiscost[11+i];
				if (cost>tempcost && (!(((app->obb_left)&&(11+i<11))||((app->obb_right)&&(11+i>13)))))
				{
					cost = tempcost;
					num2 = 11+i;
				}
			}
			//if(!(app->bdirectrunreq&&(11-i<11||11-i>13)))
			{
				tempcost = 0.4*apcost[11-i]*apcost[11-i]+12*obcost[11-i]*obcost[11-i]+hiscost[11-i]*hiscost[11-i];
				if (cost>tempcost && (!(((app->obb_left)&&(11-i<11))||((app->obb_right)&&(11-i>13)))))
				{
					cost = tempcost;
					num2 = 11-i;
				}
			}
		}

		if (num2<24)
		{
			for(int i = 0;i<200;i++)
			{
				MPoint[i]=hpoint[num2][i];
			}

			res = 3;
			his_ap = hpoint[num2][199];
			return res;
		}
		else
		{
			//his_ap = ap_gps;
			return 0;
		}
	}
	

}
void PathPlan::Hermite(CvPoint2D64f m_gps,double m_dir,CvPoint2D64f ap_gps,double ap_dir,CvPoint2D64f (&MPoint)[200],int l1,int l2)
{
	CvPoint2D64f m_map1,ap_map1,m_gps1,ap_gps1;

	// 
	double distinter = m_GpsData.GetDistance(m_gps.x,m_gps.y,ap_gps.x,ap_gps.y);
	
	if(l1>distinter/2)
		l1=distinter/2;
	if(l2>distinter/2)
		l2=distinter/2;

	double x1 = 256;
	double y1 = 412 - l1*5;
	m_map1.x = x1;
	m_map1.y = y1;
	m_gps1 = m_GpsData.MaptoGPS(m_gps,m_dir,m_map1);
	
	double aim = 90-(-ap_dir + m_dir);
	double x2 = 256 + l2 * cos(-aim*3.14159265/180)*5;
	double y2 = 412 - l2 * sin(-aim*3.14159265/180)*5;
	ap_map1.x = x2;
	ap_map1.y = y2;        //////////////////////////////////////////////
	ap_gps1 = m_GpsData.MaptoGPS(ap_gps,m_dir,ap_map1);
 
	CvPoint2D64f pt[4];
	pt[0]=m_gps;
	pt[1]=m_gps1;
	pt[2]=ap_gps1;
	pt[3]=ap_gps;

	Bezier(pt,3,MPoint);
}

double PathPlan::ObstacleCost(CvPoint2D64f GPSpoint[],I_Map *map,int ob_n,double range)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	
	// 获取当前车辆的GPS坐标以及朝向
	app->critical_section.Lock();//锁住
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;
	app->critical_section.Unlock();
	
	int ob_x,ob_y;
	int ob_num = 0;
	
	double cost,tempcost;
	cost = range;
	tempcost = range;
	
	CvPoint2D64f Rndf_MapPoint[200]={0};
	for(int i=0;i<200;i++)
	{
		// 将 GPSpoint[200] 转化为地图坐标
		Rndf_MapPoint[i] = m_GpsData.APiontConverD(m_gps,GPSpoint[i],m_gpsdir);  
		if(m_gps.x==GPSpoint[i].x && m_gps.y==GPSpoint[i].y)
		{
			Rndf_MapPoint[i].x = 256;
			Rndf_MapPoint[i].y = 411;
		}
	}
	if(Rndf_MapPoint[199].x == 256 && Rndf_MapPoint[199].y == 411)
	{
		//return false;
	}

	int x = 0;
	int y = 0;
	int r = 1.5;  //1.5
	int up = 30;
	
	// 该代码块设置 up
	if(app->range_flag)
	{
		up=30;
		if (app->left_right!=0)
		{
			up=20;
		}
	}
	else if (app->road_ut)
	{
		up=30;
		if (app->left_right!=0)
		{
			up=25;
		}
	}
	else
	{
		up=45;
		if (app->left_right!=0)
		{
			up=30;
		}
	}

	// 该代码块设置 speedreducedist_
	double speedreducedist_;
	// app->left_right==0: 即不变道
	if(app->left_right==0) 
		speedreducedist_ = app->GPS_Speed * 3.6 + 17;
	else
		speedreducedist_ = (app->GPS_Speed) * (app->GPS_Speed) / 8+11;
	
	if(speedreducedist_<15)
		speedreducedist_=15;
	else if(speedreducedist_>80)
		speedreducedist_=80;

	if(up>speedreducedist_)
		up=speedreducedist_;

	double distob=-1;

	// lstrt 是指离原点的距离
	double lstrt=0;

	for(int i = 0;i<200;i++)
	{
		// MapPoint的点
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511 || Rndf_MapPoint[i].y > (411-r*5) || Rndf_MapPoint[i].y < 411- up * 5)
			continue;

		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		
		// lstrt += 这个距离是为了什么？ 得到[199]的点的离原点距离？
		if(i>0)
			lstrt = lstrt + m_GpsData.GetDistance(GPSpoint[i-1].x,GPSpoint[i-1].y,GPSpoint[i].x,GPSpoint[i].y);
		
		for(int m=-13;m<13;m++)
		{	
			// 一格代表0.2m m=±13代表 ±2.6m？
			for(int n=-13;n<13 ; n++)
			{	
				if(y+m>412||y+m<0||x+n>511||x+n<0)
					continue;

				if(!(m>-5 && m<5 && n>-5 && n<5))
				{
					if(y+m>411-r*5)
						continue;
				}

				// 此处 if 判断的内容为？ 比 lukouwan 的要多了下面两行，为什么？
				if(map->MapPoint[y+m][x+n] == 8 || map->MapPoint[y+m][x+n] == 18 || ((map->MapPoint[y+m][x+n] == 28) 
					&& (!(((dynamicmap_v_x->MapPoint[y+m][x+n]>=10/3.6)&&(y+m<=372))
					||((dynamicmap_v_x->MapPoint[y+m][x+n]>=3/3.6)&&(y+m<=337))))))
				{
					// 重新转回GPS点
					CvPoint2D64f obpathpoint = m_GpsData.MaptoGPS(m_gps, m_gpsdir, cvPoint2D64f(x+n,y+m));
					if(!(m>-5 && m<5 && n>-5 && n<5))
					{
						if(m_GpsData.GetDistance(m_gps.x,m_gps.y,obpathpoint.x,obpathpoint.y) < r)
							continue;
					}

					CvPoint2D64f pianyipoint1;
					CvPoint2D64f pianyipoint2;
					double dis_tmp;
					double angleerr;
					double angleerr2;
					bool bok1;
					bool bok2;

					if(i==199)
					{
						pianyipoint1=GPSpoint[198];
						pianyipoint2=GPSpoint[199];
					}
					else if(i==0)
					{
						pianyipoint1=GPSpoint[0];
						pianyipoint2=GPSpoint[1];
					}
					else
					{
						angleerr=m_GpsData.GetAngle(obpathpoint, GPSpoint[i-1]) - m_GpsData.GetAngle(GPSpoint[i], GPSpoint[i-1]);
						angleerr2=m_GpsData.GetAngle(obpathpoint, GPSpoint[i]) - m_GpsData.GetAngle(GPSpoint[i], GPSpoint[i-1]);
						
						if(angleerr>360)
							angleerr=angleerr-360;
						else if(angleerr<-360)
							angleerr=angleerr+360;
						
						if(angleerr>180)
							angleerr=angleerr-360;
						else if(angleerr<-180)
							angleerr=angleerr+360;
						
						if(angleerr2>360)
							angleerr2=angleerr2-360;
						else if(angleerr2<-360)
							angleerr2=angleerr2+360;
						
						if(angleerr2>180)
							angleerr2=angleerr2-360;
						else if(angleerr2<-180)
							angleerr2=angleerr2+360;
						
						bok1=!(angleerr>90 || angleerr<-90 || (angleerr2>-90 && angleerr2<90));
						
						angleerr=m_GpsData.GetAngle(obpathpoint,GPSpoint[i])-m_GpsData.GetAngle(GPSpoint[i+1],GPSpoint[i]);
						angleerr2=m_GpsData.GetAngle(obpathpoint,GPSpoint[i+1])-m_GpsData.GetAngle(GPSpoint[i+1],GPSpoint[i]);
						
						if(angleerr>360)
							angleerr=angleerr-360;
						else if(angleerr<-360)
							angleerr=angleerr+360;
						
						if(angleerr>180)
							angleerr=angleerr-360;
						else if(angleerr<-180)
							angleerr=angleerr+360;
						
						if(angleerr2>360)
							angleerr2=angleerr2-360;
						else if(angleerr2<-360)
							angleerr2=angleerr2+360;
						
						if(angleerr2>180)
							angleerr2=angleerr2-360;
						else if(angleerr2<-180)
							angleerr2=angleerr2+360;
						bok2=!(angleerr>90 || angleerr<-90 || (angleerr2>-90 && angleerr2<90));

						if(bok1&&(!bok2))
						{
							pianyipoint1=GPSpoint[i-1];
							pianyipoint2=GPSpoint[i];
						}
						else if(bok2&&(!bok1))
						{
							pianyipoint1=GPSpoint[i];
							pianyipoint2=GPSpoint[i+1];
						}
						else
						{
							dis_tmp = m_GpsData.GetDistance(obpathpoint.x,obpathpoint.y,GPSpoint[i+1].x,GPSpoint[i+1].y);
							if(dis_tmp<=(m_GpsData.GetDistance(obpathpoint.x,obpathpoint.y,GPSpoint[i-1].x,GPSpoint[i-1].y)))
							{
								pianyipoint1=GPSpoint[i];
								pianyipoint2=GPSpoint[i+1];
							}
							else
							{
								pianyipoint1=GPSpoint[i-1];
								pianyipoint2=GPSpoint[i];
							}
						}
					}

					angleerr=m_GpsData.GetAngle(obpathpoint,pianyipoint1)-m_GpsData.GetAngle(pianyipoint2,pianyipoint1);
					double lengtherr=m_GpsData.GetDistance(obpathpoint.x,obpathpoint.y,pianyipoint1.x,pianyipoint1.y);

					if(angleerr>360)
						angleerr=angleerr-360;
					else if(angleerr<-360)
						angleerr=angleerr+360;
					if(angleerr>180)
						angleerr=angleerr-360;
					else if(angleerr<-180)
						angleerr=angleerr+360;

					angleerr2=m_GpsData.GetAngle(obpathpoint,pianyipoint2)-m_GpsData.GetAngle(pianyipoint2,pianyipoint1);
					if(angleerr2>360)
						angleerr2=angleerr2-360;
					else if(angleerr2<-360)
						angleerr2=angleerr2+360;
					if(angleerr2>180)
						angleerr2=angleerr2-360;
					else if(angleerr2<-180)
						angleerr2=angleerr2+360;

					tempcost = abs(lengtherr * sin(angleerr * 3.14159265 / 180));

					if(app->GPS_Speed*3.6>12)
					{
						if(angleerr>90 || angleerr<-90 || (angleerr2>-90 && angleerr2<90) || (tempcost>sqrt(double(n*n+m*m))/5))
							tempcost = sqrt(double(n*n+m*m))/5;
						else
							tempcost = abs(lengtherr*sin(angleerr*3.14159265/180));
					}
					else
						tempcost = sqrt(double(n*n+m*m))/5;

					if (cost>tempcost)
					{
						cost = tempcost;
					}
					
				}
			}
		}

	}
	
	if(cost<=1)
		cost=-700+cost+distob;
	else if(cost<=1.2)
		cost=-500+cost;
	if(cost>range)
		cost=range;
	
	return range-cost; 
}

double PathPlan::ObstacleCost_lukou(CvPoint2D64f GPSpoint[],I_Map *map,int ob_n,double range)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();//锁住
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;
	app->critical_section.Unlock();

	int ob_x,ob_y;
	double cost,tempcost;
	cost = range;
	tempcost = range;
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
		//return false;
	}


	int x = 0;
	int y = 0;
	int r = 1.5;//1.5


	int up = 30;
	if(app->range_flag)
	{
		up=30;
		if (app->left_right!=0)
		{
			up=20;
		}
	}
	else if (app->road_ut)
	{
		up=30;
		if (app->left_right!=0)
		{
			up=25;
		}
	}
	else
	{
		up=45;
		if (app->left_right!=0)
		{
			up=30;
		}
	}
	double speedreducedist_;
	if(app->left_right==0)
		speedreducedist_ = app->GPS_Speed*3.6+17;
	else
		speedreducedist_ = (app->GPS_Speed)*(app->GPS_Speed)/8+11;
		//speedreducedist_ = app->GPS_Speed*3.6-5;
	if(speedreducedist_<15)
		speedreducedist_=15;
	else if(speedreducedist_>80)
		speedreducedist_=80;

	if(up>speedreducedist_)
		up=speedreducedist_;

	double distob=-1;

	double lstrt=0;

	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > (411-r*5)||Rndf_MapPoint[i].y < 411-up*5)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		if(i>0)
			lstrt=lstrt+m_GpsData.GetDistance(GPSpoint[i-1].x,GPSpoint[i-1].y,GPSpoint[i].x,GPSpoint[i].y);
		for(int m=-13;m<13;m++)
		{
			for(int n=-13;n<13;n++)
			{	
				if(y+m>412||y+m<0||x+n>511||x+n<0)
					continue;
				if(!(m>-5 && m<5 && n>-5 && n<5))
				{
					if(y+m>411-r*5)
						continue;
				}
				//if(map->MapPoint[y+m][x+n] == 8 || map->MapPoint[y+m][x+n] == 18 || ((map->MapPoint[y+m][x+n] == 28)&&(!(((dynamicmap_v_x->MapPoint[y+m][x+n]>=10/3.6)&&(y+m<=372))||((dynamicmap_v_x->MapPoint[y+m][x+n]>=3/3.6)&&(y+m<=337))))))
				if(map->MapPoint[y+m][x+n] == 8 || map->MapPoint[y+m][x+n] == 18 || map->MapPoint[y+m][x+n] == 28)
				{
					CvPoint2D64f obpathpoint = m_GpsData.MaptoGPS(m_gps,m_gpsdir,cvPoint2D64f(x+n,y+m));
					if(!(m>-5 && m<5 && n>-5 && n<5))
					{
						if(m_GpsData.GetDistance(m_gps.x,m_gps.y,obpathpoint.x,obpathpoint.y)<r)
							continue;
					}
					CvPoint2D64f pianyipoint1;
					CvPoint2D64f pianyipoint2;
					double dis_tmp;
					double angleerr;
					double angleerr2;
					bool bok1;
					bool bok2;
					if(i==199)
					{
						pianyipoint1=GPSpoint[198];
						pianyipoint2=GPSpoint[199];
					}
					else if(i==0)
					{
						pianyipoint1=GPSpoint[0];
						pianyipoint2=GPSpoint[1];
					}
					else
					{
						angleerr=m_GpsData.GetAngle(obpathpoint,GPSpoint[i-1])-m_GpsData.GetAngle(GPSpoint[i],GPSpoint[i-1]);
						angleerr2=m_GpsData.GetAngle(obpathpoint,GPSpoint[i])-m_GpsData.GetAngle(GPSpoint[i],GPSpoint[i-1]);
						if(angleerr>360)
							angleerr=angleerr-360;
						else if(angleerr<-360)
							angleerr=angleerr+360;
						if(angleerr>180)
							angleerr=angleerr-360;
						else if(angleerr<-180)
							angleerr=angleerr+360;
						if(angleerr2>360)
							angleerr2=angleerr2-360;
						else if(angleerr2<-360)
							angleerr2=angleerr2+360;
						if(angleerr2>180)
							angleerr2=angleerr2-360;
						else if(angleerr2<-180)
							angleerr2=angleerr2+360;
						bok1=!(angleerr>90 || angleerr<-90 || (angleerr2>-90 && angleerr2<90));
						angleerr=m_GpsData.GetAngle(obpathpoint,GPSpoint[i])-m_GpsData.GetAngle(GPSpoint[i+1],GPSpoint[i]);
						angleerr2=m_GpsData.GetAngle(obpathpoint,GPSpoint[i+1])-m_GpsData.GetAngle(GPSpoint[i+1],GPSpoint[i]);
						if(angleerr>360)
							angleerr=angleerr-360;
						else if(angleerr<-360)
							angleerr=angleerr+360;
						if(angleerr>180)
							angleerr=angleerr-360;
						else if(angleerr<-180)
							angleerr=angleerr+360;
						if(angleerr2>360)
							angleerr2=angleerr2-360;
						else if(angleerr2<-360)
							angleerr2=angleerr2+360;
						if(angleerr2>180)
							angleerr2=angleerr2-360;
						else if(angleerr2<-180)
							angleerr2=angleerr2+360;
						bok2=!(angleerr>90 || angleerr<-90 || (angleerr2>-90 && angleerr2<90));

						if(bok1&&(!bok2))
						{
							pianyipoint1=GPSpoint[i-1];
							pianyipoint2=GPSpoint[i];
						}
						else if(bok2&&(!bok1))
						{
							pianyipoint1=GPSpoint[i];
							pianyipoint2=GPSpoint[i+1];
						}
						else
						{
							dis_tmp = m_GpsData.GetDistance(obpathpoint.x,obpathpoint.y,GPSpoint[i+1].x,GPSpoint[i+1].y);
							if(dis_tmp<=(m_GpsData.GetDistance(obpathpoint.x,obpathpoint.y,GPSpoint[i-1].x,GPSpoint[i-1].y)))
							{
								pianyipoint1=GPSpoint[i];
								pianyipoint2=GPSpoint[i+1];
							}
							else
							{
								pianyipoint1=GPSpoint[i-1];
								pianyipoint2=GPSpoint[i];
							}
						}
					}

					angleerr=m_GpsData.GetAngle(obpathpoint,pianyipoint1)-m_GpsData.GetAngle(pianyipoint2,pianyipoint1);
					double lengtherr=m_GpsData.GetDistance(obpathpoint.x,obpathpoint.y,pianyipoint1.x,pianyipoint1.y);

					if(angleerr>360)
						angleerr=angleerr-360;
					else if(angleerr<-360)
						angleerr=angleerr+360;
					if(angleerr>180)
						angleerr=angleerr-360;
					else if(angleerr<-180)
						angleerr=angleerr+360;

					angleerr2=m_GpsData.GetAngle(obpathpoint,pianyipoint2)-m_GpsData.GetAngle(pianyipoint2,pianyipoint1);
					if(angleerr2>360)
						angleerr2=angleerr2-360;
					else if(angleerr2<-360)
						angleerr2=angleerr2+360;
					if(angleerr2>180)
						angleerr2=angleerr2-360;
					else if(angleerr2<-180)
						angleerr2=angleerr2+360;

					tempcost = abs(lengtherr*sin(angleerr*3.14159265/180));

					if(app->GPS_Speed*3.6>12)
					{
						if(angleerr>90 || angleerr<-90 || (angleerr2>-90 && angleerr2<90) || (tempcost>sqrt(double(n*n+m*m))/5))
							tempcost = sqrt(double(n*n+m*m))/5;
						else
							tempcost = abs(lengtherr*sin(angleerr*3.14159265/180));
					}
					else
						tempcost = sqrt(double(n*n+m*m))/5;

					if (cost>tempcost)
					{
						cost = tempcost;
					}
					
				}
			}
		}
	}
	//if(cost<=1.6)cost=-500+cost;
	if(cost<=1)cost=-700+cost+distob;
	else if(cost<=1.2)cost=-500+cost;
	if(cost>range)cost=range;
	return range-cost;
}

// ob_n 和 range 的含义？
double PathPlan::ObstacleCost_lukouwan(CvPoint2D64f GPSpoint[],I_Map *map,int ob_n,double range) 
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	
	// 获取当前GPS坐标
	app->critical_section.Lock();//锁住
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;
	app->critical_section.Unlock();
	
	int ob_x,ob_y;
	int ob_num = 0;

	double cost,tempcost;
	cost = range;
	tempcost = range;

	CvPoint2D64f Rndf_MapPoint[200]={0};
	for(int i=0;i<200;i++)
	{
		// 将GPS 坐标转换为栅格地图，然后赋值到 Rndf_MapPoint[200]
		Rndf_MapPoint[i] = m_GpsData.APiontConverD(m_gps,GPSpoint[i],m_gpsdir);

		if(m_gps.x==GPSpoint[i].x&&m_gps.y==GPSpoint[i].y)
		{
			Rndf_MapPoint[i].x = 256;
			Rndf_MapPoint[i].y = 411;
		}
	}

	if(Rndf_MapPoint[199].x == 256 && Rndf_MapPoint[199].y == 411)
	{
		//return false;
	}

	int x = 0;
	int y = 0;
	int r = 1.5;
	int up = 30;

	// 代码块设置 up ，up是什么？
	if(app->range_flag)
	{
		up=30;
		if (app->left_right!=0)
		{
			up=20;
		}
	}
	else if (app->road_ut)
	{
		up=30;
		if (app->left_right!=0)
		{
			up=25;
		}
	}
	else
	{
		up=45;
		if (app->left_right!=0)
		{
			up=30;
		}
	}

	// 代码块设置 speedreducedist_， speedreducedist_ 具体为什么速度？
	double speedreducedist_;
	
	// 算出来的 speedreducedist_ 一定在 15~80 范围内
	if(app->left_right==0)
		speedreducedist_ = app->GPS_Speed * 3.6+17;
	else
		speedreducedist_ = (app->GPS_Speed)*(app->GPS_Speed)/8+11;
	
	if(speedreducedist_<15)
		speedreducedist_=15;
	else if(speedreducedist_>80)
		speedreducedist_=80;

	if(up > speedreducedist_)
		up = speedreducedist_;

	double distob=-1;
	double lstrt=0;

	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > (411-r*5)||Rndf_MapPoint[i].y < 411- up*5)
			continue;
		x = Rndf_MapPoint[i].x;
		y = Rndf_MapPoint[i].y;
		
		// lstrt 含义？
		if(i>0)
			lstrt = lstrt + m_GpsData.GetDistance(GPSpoint[i-1].x,GPSpoint[i-1].y,GPSpoint[i].x,GPSpoint[i].y);

		for(int m=-13;m<13;m++)
		{
			for(int n=-13;n<13;n++)
			{	
				if(y+m>412||y+m<0||x+n>511||x+n<0)
					continue;
				if(!(m>-5 && m<5 && n>-5 && n<5))
				{
					if(y+m>411-r*5)
						continue;
				}
				

				if(map->MapPoint[y+m][x+n] == 8 || map->MapPoint[y+m][x+n] == 18 || map->MapPoint[y+m][x+n] == 28)
				{
					CvPoint2D64f obpathpoint = m_GpsData.MaptoGPS(m_gps,m_gpsdir,cvPoint2D64f(x+n , y+m));
					
					if(!(m>-5 && m<5 && n>-5 && n<5))
					{
						// obpathpoint 含义？ r 的含义？
						if(m_GpsData.GetDistance(m_gps.x,m_gps.y, obpathpoint.x, obpathpoint.y) < r)
							continue;
					}

					CvPoint2D64f pianyipoint1;
					CvPoint2D64f pianyipoint2;
					double dis_tmp;
					double angleerr;
					double angleerr2;
					bool bok1;
					bool bok2;

					if(i==199)
					{
						pianyipoint1=GPSpoint[198];
						pianyipoint2=GPSpoint[199];
					}
					else if(i==0)
					{
						pianyipoint1=GPSpoint[0];
						pianyipoint2=GPSpoint[1];
					}

					else
					{
						angleerr= m_GpsData.GetAngle(obpathpoint,GPSpoint[i-1]) - m_GpsData.GetAngle(GPSpoint[i], GPSpoint[i-1]);
						angleerr2=m_GpsData.GetAngle(obpathpoint,GPSpoint[i]) - m_GpsData.GetAngle(GPSpoint[i], GPSpoint[i-1]);
						
						if(angleerr>360)
							angleerr=angleerr-360;
						else if(angleerr<-360)
							angleerr=angleerr+360;
						
						if(angleerr>180)
							angleerr=angleerr-360;
						else if(angleerr<-180)
							angleerr=angleerr+360;
						
						if(angleerr2>360)
							angleerr2=angleerr2-360;
						else if(angleerr2<-360)
							angleerr2=angleerr2+360;
						if(angleerr2>180)
							angleerr2=angleerr2-360;
						else if(angleerr2<-180)
							angleerr2=angleerr2+360;
						
						bok1= !(angleerr>90 || angleerr<-90 || (angleerr2>-90 && angleerr2<90));

						angleerr = m_GpsData.GetAngle(obpathpoint,GPSpoint[i]) - m_GpsData.GetAngle(GPSpoint[i+1],GPSpoint[i]);
						angleerr2 = m_GpsData.GetAngle(obpathpoint,GPSpoint[i+1])- m_GpsData.GetAngle(GPSpoint[i+1],GPSpoint[i]);
						
						if(angleerr>360)
							angleerr=angleerr-360;
						else if(angleerr<-360)
							angleerr=angleerr+360;
						if(angleerr>180)
							angleerr=angleerr-360;
						else if(angleerr<-180)
							angleerr=angleerr+360;
						
						if(angleerr2>360)
							angleerr2=angleerr2-360;
						else if(angleerr2<-360)
							angleerr2=angleerr2+360;
						if(angleerr2>180)
							angleerr2=angleerr2-360;
						else if(angleerr2<-180)
							angleerr2=angleerr2+360;
						bok2=!(angleerr>90 || angleerr<-90 || (angleerr2>-90 && angleerr2<90));

						if(bok1&&(!bok2))
						{
							pianyipoint1=GPSpoint[i-1];
							pianyipoint2=GPSpoint[i];
						}
						else if(bok2&&(!bok1))
						{
							pianyipoint1=GPSpoint[i];
							pianyipoint2=GPSpoint[i+1];
						}
						else
						{
							dis_tmp = m_GpsData.GetDistance(obpathpoint.x,obpathpoint.y,GPSpoint[i+1].x,GPSpoint[i+1].y);
							if(dis_tmp<=(m_GpsData.GetDistance(obpathpoint.x,obpathpoint.y,GPSpoint[i-1].x,GPSpoint[i-1].y)))
							{
								pianyipoint1=GPSpoint[i];
								pianyipoint2=GPSpoint[i+1];
							}
							else
							{
								pianyipoint1=GPSpoint[i-1];
								pianyipoint2=GPSpoint[i];
							}
						}
					}

					angleerr = m_GpsData.GetAngle(obpathpoint, pianyipoint1) - m_GpsData.GetAngle(pianyipoint2,pianyipoint1);
					double lengtherr = m_GpsData.GetDistance(obpathpoint.x,obpathpoint.y,pianyipoint1.x,pianyipoint1.y);

					if(angleerr>360)
						angleerr=angleerr-360;
					else if(angleerr<-360)
						angleerr=angleerr+360;
					if(angleerr>180)
						angleerr=angleerr-360;
					else if(angleerr<-180)
						angleerr=angleerr+360;

					angleerr2=m_GpsData.GetAngle(obpathpoint,pianyipoint2)-m_GpsData.GetAngle(pianyipoint2,pianyipoint1);
					if(angleerr2>360)
						angleerr2=angleerr2-360;
					else if(angleerr2<-360)
						angleerr2=angleerr2+360;
					if(angleerr2>180)
						angleerr2=angleerr2-360;
					else if(angleerr2<-180)
						angleerr2=angleerr2+360;

					tempcost = abs(lengtherr * sin(angleerr*3.14159265/180));

					if(angleerr>90 || angleerr<-90 || (angleerr2>-90 && angleerr2<90) || (tempcost>sqrt(double(n*n+m*m))/5))
						tempcost = sqrt(double(n*n+m*m))/5;
					else
						tempcost = abs(lengtherr*sin(angleerr*3.14159265/180));

					tempcost = sqrt(double(n*n+m*m))/5;

					if (cost>tempcost)
					{
						cost = tempcost;
					}

				}
			}
		}
	}
	
	// 如果 cost(距离)小于1，则 cost = -1200，为什么用负值？
	if(cost<=1)
		cost= -1200 + cost + distob;
	else if (cost<=1.2)
		cost= -1000 + cost;
	else if (cost<=1.6)
		cost= -500 + cost;

	// 如果 cost > range(设定好的距离) ,则返回0
	if(cost > range)
		cost=range;

	return range-cost;
}

double PathPlan::ObstacleCost2(CvPoint2D64f GPSpoint[],I_Map *map,int ob_n,double range)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();//锁住
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;

	app->critical_section.Unlock();
	int ob_x,ob_y;
	double cost,tempcost;
	cost = range;
	tempcost = range;
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
		//return false;
	}


	int x = 0;
	int y = 0;
	int r = 1.5;


	int up;
	int width = 15;
	up = 60;
	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > (411-r*5)||Rndf_MapPoint[i].y < 411-up*5)
			continue;
		x = Rndf_MapPoint[i].x ;//2012.7.18xiugai -10
		y = Rndf_MapPoint[i].y;
		if(y>411-80)
			width = 7;
		else
			width =7;
		for(int m=-width;m<width;m++)
			for(int n=-width;n<width;n++)
			{	
				if(y+m>411-r*5||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == 8 || map->MapPoint[y+m][x+n] == 18 || map->MapPoint[y+m][x+n] == 28)
				{
					CvPoint2D64f obpathpoint = m_GpsData.MaptoGPS(m_gps,m_gpsdir,cvPoint2D64f(x+n,y+m));
					if(m_GpsData.GetDistance(m_gps.x,m_gps.y,obpathpoint.x,obpathpoint.y)<r)
						continue;
					CvPoint2D64f pianyipoint1;
					CvPoint2D64f pianyipoint2;
					double dis_tmp;
					double angleerr;
					double angleerr2;
					bool bok1;
					bool bok2;
					if(i==199)
					{
						pianyipoint1=GPSpoint[198];
						pianyipoint2=GPSpoint[199];
					}
					else if(i==0)
					{
						pianyipoint1=GPSpoint[0];
						pianyipoint2=GPSpoint[1];
					}
					else
					{
						angleerr=m_GpsData.GetAngle(obpathpoint,GPSpoint[i-1])-m_GpsData.GetAngle(GPSpoint[i],GPSpoint[i-1]);
						angleerr2=m_GpsData.GetAngle(obpathpoint,GPSpoint[i])-m_GpsData.GetAngle(GPSpoint[i],GPSpoint[i-1]);
						if(angleerr>180)
							angleerr=angleerr-360;
						else if(angleerr<-180)
							angleerr=angleerr+360;
						if(angleerr2>180)
							angleerr2=angleerr2-360;
						else if(angleerr2<-180)
							angleerr2=angleerr2+360;
						bok1=!(angleerr>90 || angleerr<-90 || (angleerr2>-90 && angleerr2<90));
						angleerr=m_GpsData.GetAngle(obpathpoint,GPSpoint[i])-m_GpsData.GetAngle(GPSpoint[i+1],GPSpoint[i]);
						angleerr2=m_GpsData.GetAngle(obpathpoint,GPSpoint[i+1])-m_GpsData.GetAngle(GPSpoint[i+1],GPSpoint[i]);
						if(angleerr>180)
							angleerr=angleerr-360;
						else if(angleerr<-180)
							angleerr=angleerr+360;
						if(angleerr2>180)
							angleerr2=angleerr2-360;
						else if(angleerr2<-180)
							angleerr2=angleerr2+360;
						bok2=!(angleerr>90 || angleerr<-90 || (angleerr2>-90 && angleerr2<90));

						if(bok1&&(!bok2))
						{
							pianyipoint1=GPSpoint[i-1];
							pianyipoint2=GPSpoint[i];
						}
						else if(bok2&&(!bok1))
						{
							pianyipoint1=GPSpoint[i];
							pianyipoint2=GPSpoint[i+1];
						}
						else
						{
							dis_tmp = m_GpsData.GetDistance(obpathpoint.x,obpathpoint.y,GPSpoint[i+1].x,GPSpoint[i+1].y);
							if(dis_tmp<=(m_GpsData.GetDistance(obpathpoint.x,obpathpoint.y,GPSpoint[i-1].x,GPSpoint[i-1].y)))
							{
								pianyipoint1=GPSpoint[i];
								pianyipoint2=GPSpoint[i+1];
							}
							else
							{
								pianyipoint1=GPSpoint[i-1];
								pianyipoint2=GPSpoint[i];
							}
						}
					}

					angleerr=m_GpsData.GetAngle(obpathpoint,pianyipoint1)-m_GpsData.GetAngle(pianyipoint2,pianyipoint1);
					double lengtherr=m_GpsData.GetDistance(obpathpoint.x,obpathpoint.y,pianyipoint1.x,pianyipoint1.y);

					if(angleerr>180)
						angleerr=angleerr-360;
					else if(angleerr<-180)
						angleerr=angleerr+360;

					angleerr2=m_GpsData.GetAngle(obpathpoint,pianyipoint2)-m_GpsData.GetAngle(pianyipoint2,pianyipoint1);
					if(angleerr2>180)
						angleerr2=angleerr2-360;
					else if(angleerr2<-180)
						angleerr2=angleerr2+360;

					if(app->GPS_Speed*3.6>12)
					{
						if(angleerr>90 || angleerr<-90 || (angleerr2>-90 && angleerr2<90))
							tempcost = sqrt(double(n*n+m*m))/5;
						else
							tempcost = abs(lengtherr*sin(angleerr*3.14159265/180));
					}
					else
						tempcost = sqrt(double(n*n+m*m))/5;

					if (cost>tempcost)
					{
						cost = tempcost;
					}

				}
			}
	}
	if(cost<1)cost=-500;
	if(cost>range)cost = range;
	return range-cost;

}
double PathPlan::ObstacleCost3(CvPoint2D64f GPSpoint[],I_Map *map,int ob_n,double range)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();//锁住
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;

	app->critical_section.Unlock();
	int ob_x,ob_y;
	double cost,tempcost;
	cost = range;
	tempcost = range;
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
		//return false;
	}


	int x = 0;
	int y = 0;
	int r = 1;//2


	int up = 30;
	if(app->range_flag)
	{
		up=30;
		if (app->left_right!=0)
		{
			up=20;
		}
	}
	else if (app->road_ut)
	{
		up=30;
		if (app->left_right!=0)
		{
			up=25;
		}
	}
	else
	{
		up=45;
		if (app->left_right!=0)
		{
			up=30;
		}
	}
	double speedreducedist_;
	if(app->left_right==0)
		speedreducedist_ = app->GPS_Speed*3.6+17;
	else
		speedreducedist_ = (app->GPS_Speed)*(app->GPS_Speed)/8+11;
		//speedreducedist_ = app->GPS_Speed*3.6-5;
	if(speedreducedist_<15)
		speedreducedist_=15;
	else if(speedreducedist_>80)
		speedreducedist_=80;

	if(up>speedreducedist_)
		up=speedreducedist_;
		
	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > 400||Rndf_MapPoint[i].y < 411-up*5)
			continue;
		x = Rndf_MapPoint[i].x ;//2012.7.18xiugai -10
		y = Rndf_MapPoint[i].y;
		for(int m=-13;m<6;m++)//-25
			for(int n=-13;n<13/*10*/;n++)
			{	
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == 8 || map->MapPoint[y+m][x+n] == 18 || map->MapPoint[y+m][x+n] == 28)
				{
					ob_x = x+n;
					ob_y = y+m;


					tempcost = sqrt(double(n*n+m*m))/5; 

					if (cost>tempcost)
					{
						cost = tempcost;
					}

				}
			}
	}
	if(cost < 1.2)cost=-500;
	if(cost > range)cost=range;
	return range-cost;

}

int PathPlan::Cross1(CvPoint2D64f m_gps,double m_dir,CvPoint2D64f ap_gps,double ap_dir,CvPoint2D64f (&MPoint)[200])//路口
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	Map_Point ap_map = m_GpsData.APiontConver(m_gps,ap_gps,m_dir);
	CvPoint2D64f tp_map[23];//计算待选点
	CvPoint2D64f tp_gps[23];
	CvPoint2D64f hpoint[23][200];
	CvPoint2D64f ypoint[23][200];
	int res = 1;

	double apcost[23];
	double obcost[23];
	double hiscost[23];

	if(SearchFrontOb(vel_Map,5))
		return -1;

	////////

	
	/////动态障碍物停车

	for (int i = 0; i<23;i++)
	{
		tp_map[i].x = ap_map.x + (-30 + 2.5*i)*cos((ap_dir-m_dir)*3.14/180);
		tp_map[i].y = ap_map.y + (-30 + 2.5*i)*sin((ap_dir-m_dir)*3.14/180);
		apcost[i] = abs(30 - 2.5*i);
		tp_gps[i] = m_GpsData.MaptoGPS(m_gps,m_dir,tp_map[i]);

		if(app->range_flag)
			Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],2,5);
		else
			Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],2,5);//Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],5,5);
		//YanChang(m_gps,m_dir,hpoint[i],ap_dir,ypoint[i]);
		//obcost[i] = ObstacleCost2(ypoint[i],vel_Map,8,3);
		obcost[i] = ObstacleCost2(hpoint[i],vel_Map,8,3);
		if(his_ap.x == 0)
			hiscost[i]=0;
		else
			hiscost[i] = m_GpsData.GetDistance(hpoint[i][199].x,hpoint[i][199].y,his_ap.x,his_ap.y);
		hiscost[i] = 0.3*hiscost[i]+0.7*his_hiscost[i];
		his_hiscost[i] = hiscost[i];
	}//计算车道方向，按照垂直方向算？

	cvClearSeq( plan_road );
	for(int j = 0; j<23; j++)
	{
		for(int i = 0; i<200; i++)
		{
			cvSeqPush(plan_road, &hpoint[j][i] );
		}
	}

	int num = 24;
	double cost,tempcost;
	cost = 1000;
	tempcost = 1000;
	for(int i = 0;i<14;i++)
	{
		tempcost = 0.18*apcost[i]*apcost[i]+20*obcost[i]*obcost[i]+18*hiscost[i]*hiscost[i];
		if (cost>tempcost && (!(((app->obb_left)&&(i<10))||((app->obb_right)&&(i>14)))))
		{
			cost = tempcost;
			num = i;
		}
		if (abs(num-12)<4)
			res = 1;
		else
			res = 3;
	}
	if(num == 24)
	{
		for(int i = 14;i<23;i++)
		{
			tempcost = 0.18*apcost[i]*apcost[i]+20*obcost[i]*obcost[i]+18*hiscost[i]*hiscost[i];
			if (cost>tempcost && (!(((app->obb_left)&&(i<10))||((app->obb_right)&&(i>14)))))
			{
				cost = tempcost;
				num = i;
			}
			if (abs(num-12)<4)
				res = 1;
			else
				res = 3;
		}
	}
	if (num==24)
	{
		his_ap = ap_gps;
		return 0;
	}
	
	for(int i = 0;i<200;i++)
	{
		MPoint[i]=hpoint[num][i];
	}
	his_ap = hpoint[num][199];

	return res;
}

int PathPlan::Cross2(CvPoint2D64f m_gps,double m_dir,CvPoint2D64f ap_gps,double ap_dir,CvPoint2D64f (&MPoint)[200])//路口
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	Map_Point ap_map = m_GpsData.APiontConver(m_gps,ap_gps,m_dir);
	CvPoint2D64f tp_map[23];//计算待选点
	CvPoint2D64f tp_gps[23];
	CvPoint2D64f hpoint[23][200];

	double apcost[23];
	double obcost[23];
	double hiscost[23];

	if(SearchFrontOb(vel_Map,5))
		return -1;

	for (int i = 0; i<23;i++)
	{
		tp_map[i].x = ap_map.x + (-30 + 2.5*i)*cos((ap_dir-m_dir)*3.14/180);
		tp_map[i].y = ap_map.y + (-30 + 2.5*i)*sin((ap_dir-m_dir)*3.14/180);
		apcost[i] = abs(30 - 2.5*i);
		tp_gps[i] = m_GpsData.MaptoGPS(m_gps,m_dir,tp_map[i]);
	
		Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],2,3);
		//obcost[i] = ObstacleCost(hpoint[i],vel_Map,8,4);
		obcost[i] = ObstacleCost(hpoint[i],dy_map,8,4);
		if(his_ap.x == 0)
			hiscost[i]=0;
		else
			hiscost[i] = m_GpsData.GetDistance(hpoint[i][199].x,hpoint[i][199].y,his_ap.x,his_ap.y);
	}//计算车道方向，按照垂直方向算？

	cvClearSeq( plan_road );
	for(int j = 0; j<23; j++)
	{
		for(int i = 0; i<200; i++)
		{
			cvSeqPush(plan_road, &hpoint[j][i] );
		}
	}

	int num=24;
	double cost,tempcost;
	cost = 1000;
	tempcost = 1000;
	for(int i = 0;i<23;i++)
	{
		tempcost = 0.2*apcost[i]*apcost[i]+3*obcost[i]*obcost[i]+2*hiscost[i]*hiscost[i];
		if (cost>tempcost && (!(((app->obb_left)&&(i<10))||((app->obb_right)&&(i>14)))))
		{
			cost = tempcost;
			num = i;
		}
	}
	
	if (num==24)
	{
		his_ap = ap_gps;
		return false;
	}
	
	for(int i = 0;i<200;i++)
	{
		MPoint[i]=hpoint[num][i];
	}
	his_ap = hpoint[num][199];

	return true;
}

int PathPlan::ExceptionPath(CvPoint2D64f m_gps,double m_dir,CvPoint2D64f ap_gps,double ap_dir,CvPoint2D64f (&MPoint)[200])//异常轨迹
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();	
	Map_Point ap_map = m_GpsData.APiontConver(m_gps,ap_gps,m_dir);
	CvPoint2D64f tp_map[23];//计算待选点
	CvPoint2D64f tp_gps[23];
	CvPoint2D64f hpoint[23][200];

	double obdist[23];


	//////计算轨迹端点，轨迹代价
	for (int i = 0; i<23;i++)
	{
		tp_map[i].x = ap_map.x + (-48 + 4*i)*cos((ap_dir-m_dir)*3.14/180);
		tp_map[i].y = ap_map.y + (-48 + 4*i)*sin((ap_dir-m_dir)*3.14/180);
	
		tp_gps[i] = m_GpsData.MaptoGPS(m_gps,m_dir,tp_map[i]);
		if(m_GpsData.GetDistance(m_gps.x,m_gps.y,ap_gps.x,ap_gps.y)<25)
		{
			if(app->GPS_Speed>22/3.6)
				Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],0.2,0.5);
			else
				Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],0.2,4);
		}
		else
		{
			
			if(app->GPS_Speed>22/3.6)
				Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],0.2,0.5);
			else
				Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],0.2,6);
		}
		obdist[i] = ObstacleDist(hpoint[i],vel_Map,8);
	
	}

	int num=24;
	double dist,temdist;
	dist = 0;
	temdist = 0;
	for(int i = 0;i<12;i++)
	{
		temdist = obdist[i+11];
		if (dist<temdist && (!(((app->obb_left||app->GPS_Speed>22/3.6)&&(11+i<10))||(app->obb_left&&app->left_right!=0&&app->GPS_Speed>22/3.6&&11+i<11)||(app->obb_right&&app->left_right!=0&&app->GPS_Speed>22/3.6&&11+i>13)||((app->obb_right||app->GPS_Speed>22/3.6)&&(11+i>14)))))
		{
			dist = temdist;
			num = i+11;
		}
		temdist = obdist[11-i];
		if (dist<temdist && (!(((app->obb_left||app->GPS_Speed>22/3.6)&&(11-i<10))||(app->obb_left&&app->left_right!=0&&app->GPS_Speed>22/3.6&&11-i<11)||(app->obb_right&&app->left_right!=0&&app->GPS_Speed>22/3.6&&11-i>13)||((app->obb_right||app->GPS_Speed>22/3.6)&&(11-i>14)))))
		{
			dist = temdist;
			num = 11-i;
		}

	}
	if (dist<5)
	return 0;
	
	app->critical_pathplanroad.Lock();
	cvClearSeq( plan_road );
	for(int j = 0; j<23; j++)
	{
		for(int i = 0; i<200; i++)
		{
			cvSeqPush(plan_road, &hpoint[j][i] );
		}
	}	
	app->critical_pathplanroad.Unlock();
	/////平滑路径
	if (num<24)
	{

		for(int i = 0;i<200;i++)
		{
			MPoint[i]=hpoint[num][i];
		}
		if (num<9)
		{
			changedis = (num-12)*0.5;
			return 2;
		}
		if (num>15)
		{
			changedis = (num-12)*0.5;
			return 2;
		}
		//Sleep(1000);
		return 1;
	}

	else
		return 0;

}

double PathPlan::ObstacleDist(CvPoint2D64f GPSpoint[],I_Map *map,int ob_n)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	app->critical_section.Lock();//锁住
	CvPoint2D64f m_gps = app->GPS_Point;
	double m_gpsdir = app->GPS_Direction;

	app->critical_section.Unlock();
	int ob_x,ob_y;
	double dist,tempdist;
	dist = 100;
	tempdist = 100;
	
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


	int x = 0;
	int y = 0;
	for(int i = 0;i<200;i++)
	{
		if(Rndf_MapPoint[i].x < 0||Rndf_MapPoint[i].x > 511||Rndf_MapPoint[i].y > 400||Rndf_MapPoint[i].y < 0)
			continue;
		x = Rndf_MapPoint[i].x ;//2012.7.18xiugai -10
		y = Rndf_MapPoint[i].y;
		for(int m=-4;m<5;m++)
			for(int n=-4;n<5;n++)
			{	
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == 8 || map->MapPoint[y+m][x+n] == 18 || map->MapPoint[y+m][x+n] == 28)
				{
					ob_x = x+n;
					ob_y = y+m;
					
					//tempdist = sqrt(double((x+n-256)*(x+n-256)+(y+m-412)*(y+m-412)))/5; 
					tempdist = abs(y+m-412)/5/*-abs(x+n-256)/50*/; 
					if ( tempdist<0 ) tempdist = 0;
					
					if (dist>tempdist)
					{
						dist = tempdist;
					}

				}
			}
	}

	return dist;
	
}

// 
int PathPlan::YanChang(CvPoint2D64f m_gps, double m_dir,CvPoint2D64f *op, double dir,CvPoint2D64f *yanp)
{
	
	for (int i = 0;i<100;i++)
	{
		yanp[i]=op[2*i];
	}
	for(int i=100;i<200;i++)
	{
		yanp[i]=m_GpsData.MaptoGPS(op[199],dir,cvPoint2D64f(256,(int)(412-(i-100)*0.025)));
	}

	return 1;
}

// 正前方range米内有无障碍
double PathPlan::SearchFrontOb(I_Map *map,double range)
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();
	int x = 256;
	int y = 411;
	int path_width = 4;
	if(app->range_flag)
	{
		path_width = 4;
	}
	for(int m=-1;m>-range*5;m--)
		for(int n=-path_width;n<=path_width;n++)
		{	
			if(y+m>511||y+m<0||x+n>511||x+n<0)
				continue;
			if(map->MapPoint[y+m][x+n] == 8 || map->MapPoint[y+m][x+n] == 18 || map->MapPoint[y+m][x+n] == 28)
			{
				return abs(m/5.0);
				
			}
		}

	return 0;
	}

/*
double PathPlan::SearchFrontOb(I_Map *map,double range)//正前方range米内有无障碍
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();

	double steertmp[51]={10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,210,220,230,240,250,260,270,280,290,300,310,320,330,340,350,360,370,380,390,400,410,420,430,440,450,460,470,480,490,500,510};
	double radiustmp[51]={262054.200000000,104574.200000000,68769.8000000000,52496.1000000000,42692.5000000000,36855.7000000000,32581.6000000000,29515.1000000000,26924.3000000000,24854.9000000000,22848.8000000000,21031.2000000000,19718.2000000000,18158.2000000000,17651.1000000000,16162.7000000000,15073,13892,13175.9000000000,11987.6000000000,11094,10821,10363.2000000000,9797.70000000000,9463.90000000000,9164.80000000000,8889.60000000000,8629.20000000000,8208.90000000000,7965.10000000000,7718.30000000000,7465.20000000000,7204.70000000000,6933.20000000000,6694.30000000000,6416,6172.50000000000,5992.30000000000,5760.50000000000,5594.20000000000,5447.90000000000,5325.60000000000,5207.70000000000,5098.30000000000,4992.60000000000,4890.50000000000,4788.40000000000,4683.90000000000,4575.10000000000,4460.20000000000,4337.80000000000};

	int x = 256;
	int y = 411;
	int path_width = 4;
	if(app->range_flag)
	{
		path_width = 4;
	}
	for(int m=-1;m>-4*5;m--)
	{
		for(int n=-2;n<=2;n++)
		{	
			if(y+m>511||y+m<0||x+n>511||x+n<0)
				continue;
			if(map->MapPoint[y+m][x+n] == 8 || map->MapPoint[y+m][x+n] == 18 || map->MapPoint[y+m][x+n] == 28)
			{
				return abs(m/5.0);
			}
		}
	}

	x = 256;
	y = 396;

	double steerabsval=abs(app->realsteerangle);
	double radius;
	if(steerabsval<=steertmp[0])
		radius=radiustmp[0];
	else if(steerabsval>=steertmp[50])
		radius=radiustmp[50];
	else
	{
		for(int i=1;i<=50;i++)
		{
			if(steerabsval==steertmp[i])
			{
				radius=radiustmp[i];
				break;
			}
			else if(steerabsval<steertmp[i])
			{
				radius=radiustmp[i-1]+(radiustmp[i]-radiustmp[i-1])*(steerabsval-steertmp[i-1])/(steertmp[i]-steertmp[i-1]);
				break;
			}
		}
	}
	radius=radius*0.001;

	for(int m=-1;m>-(range-3)*5;m--)
	{
		for(int n=-path_width;n<=path_width;n++)
		{	
			double radiustmp=radius;
			if(app->realsteerangle>0)
				radiustmp=radiustmp+n*0.2;
			else
				radiustmp=radiustmp-n*0.2;
			double angle=abs(m*0.2)/radiustmp;
			int xtmp=x+n+radiustmp*(1-cos(angle))*5;
			if(app->realsteerangle>0)
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
*/

int PathPlan::KeepLane2(CvPoint2D64f m_gps,double m_dir,CvPoint2D64f ap_gps,double ap_dir,CvPoint2D64f (&MPoint)[200])//保持车道
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();	
	Map_Point ap_map = m_GpsData.APiontConver(m_gps,ap_gps,m_dir);
	CvPoint2D64f tp_map[23];//计算待选点
	CvPoint2D64f tp_gps[23];
	CvPoint2D64f hpoint[23][200];
	CvPoint2D64f ypoint[23][200];

	double apcost[23];
	double obcost[23];
	double hiscost[23];//代价

	int res;

	int range;
	if(app->GPS_Speed*3.6<3)range = /*2*/8;
	else if(app->GPS_Speed*3.6<5)range = /*5*/8;
	else if(app->GPS_Speed*3.6<10)range = /*8*/8;
	else range = 8/*25*/;

	if(SearchFrontOb(vel_Map,6))
		return -1;

	if(app->bzheng)
	{
		if(SearchFrontOb(vel_Map,range))
			return 0;
	}

	//////计算轨迹端点，轨迹代价
	for (int i = 0; i<23;i++)
	{
		tp_map[i].x = ap_map.x + (-30 + 2.5*i)*cos((ap_dir-m_dir)*3.14/180);
		tp_map[i].y = ap_map.y + (-30 + 2.5*i)*sin((ap_dir-m_dir)*3.14/180);
		apcost[i] = abs(30 - 2.5*i);
		tp_gps[i] = m_GpsData.MaptoGPS(m_gps,m_dir,tp_map[i]);

		double ln = app->GPS_Speed*2;
		if(ln>=10)
			Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],2,5);
		if(ln<10)
			Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],2,10);
		YanChang(m_gps,m_dir,hpoint[i],ap_dir,ypoint[i]);
		obcost[i] = ObstacleCost(ypoint[i],vel_Map,8,3);
		if(his_ap.x==0)
			hiscost[i]=0;
		else
			hiscost[i] = m_GpsData.GetDistance(hpoint[i][199].x,hpoint[i][199].y,his_ap.x,his_ap.y);
	}

	int num=24;
	double cost,tempcost;
	cost = 40000;
	tempcost = 40000;
	for(int i = 10;i<15;i++)
	{
		tempcost = 0.2*apcost[i]*apcost[i]+2*obcost[i]*obcost[i]+hiscost[i]*hiscost[i];
		if (cost>tempcost)
		{
			cost = tempcost;
			num = i;
		}
	}
	app->critical_pathplanroad.Lock();
	cvClearSeq( plan_road );
	for(int j = 0; j<23; j++)
	{
		for(int i = 0; i<200; i++)
		{
			cvSeqPush(plan_road, &ypoint[j][i] );
		}
	}	
	app->critical_pathplanroad.Unlock();
	/////平滑路径
	if (num<24)
	{

		for(int i = 0;i<200;i++)
		{
			MPoint[i]=ypoint[num][i];
		}

		if (abs(num-12)<4)
			res = 1;
		else
			res = 2;
		his_ap = hpoint[num][199];

		return res;
	}

	////尖锐路径
	if (num==24)
	{	
		return 0;
	}


}

double PathPlan::SearchObCircle(I_Map *map,double range)//正前方range米内有无障碍
{

	int x = 256;
	int y = 411;
	int dd = 0;

	for(int m=0;m>-range*5;m--)
	{
		dd = sqrt((range*5)*(range*5)-m*m);
		for(int n=-dd;n<=dd;n++)
		{	
			if(y+m>511||y+m<0||x+n>511||x+n<0)
				continue;
			if(map->MapPoint[y+m][x+n] == 38)
			{
				return 1;

			}
		}
	}
	return 0;
	

}

double PathPlan::SearchFrontObCross(I_Map *map,double range,int n)//正前方range米内有无障碍
{

	int x = 256;
	int y = 411;
	if(n==2)
	{
		for(int m=0;m>-range*5;m--)
			for(int n=-10;n<=10;n++)
			{	
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == 28||map->MapPoint[y+m][x+n] == 38)
				{
					return 1;

				}
			}

		return 0;
	}
	if (n==1)
	{
		for(int m=0;m>-range*5;m--)
			for(int n=-6;n<=6;n++)
			{	
				if(y+m>511||y+m<0||x+n>511||x+n<0)
					continue;
				if(map->MapPoint[y+m][x+n] == 38)
				{
					return 1;

				}
			}

		return 0;
	}
}
int PathPlan::ExceptionPath2(CvPoint2D64f m_gps,double m_dir,CvPoint2D64f ap_gps,double ap_dir,CvPoint2D64f (&MPoint)[200])//异常轨迹
{
	CIVDecisionApp *app=( CIVDecisionApp*)AfxGetApp();	
	Map_Point ap_map = m_GpsData.APiontConver(m_gps,ap_gps,m_dir);
	CvPoint2D64f tp_map[23];//计算待选点
	CvPoint2D64f tp_gps[23];
	CvPoint2D64f hpoint[23][200];

	double obdist[23];


	//////计算轨迹端点，轨迹代价
	for (int i = 0; i<23;i++)
	{
		tp_map[i].x = ap_map.x + (-30 + 2.5*i)*cos((ap_dir-m_dir)*3.14/180);
		tp_map[i].y = ap_map.y + (-30 + 2.5*i)*sin((ap_dir-m_dir)*3.14/180);

		tp_gps[i] = m_GpsData.MaptoGPS(m_gps,m_dir,tp_map[i]);

		Hermite(m_gps,m_dir,tp_gps[i],ap_dir,hpoint[i],2,3);
		obdist[i] = ObstacleDist(hpoint[i],vel_Map,8);

	}

	int num=24;
	double dist,temdist;
	dist = 0;
	temdist = 0;
	for(int i = 0;i<23;i++)
	{
		temdist = obdist[i];
		if (dist<temdist && (!(((app->obb_left)&&(i<10))||((app->obb_right)&&(i>14)))))
		{
			dist = temdist;
			num = i;
		}
	}
	if (dist<5)
		return 0;

	app->critical_pathplanroad.Lock();
	cvClearSeq( plan_road );
	for(int j = 0; j<23; j++)
	{
		for(int i = 0; i<200; i++)
		{
			cvSeqPush(plan_road, &hpoint[j][i] );
		}
	}	
	app->critical_pathplanroad.Unlock();
	/////平滑路径
	if (num<24)
	{

		for(int i = 0;i<200;i++)
		{
			MPoint[i]=hpoint[num][i];
		}
		his_ap = hpoint[num][199];
		return 1;
	}

	else
		return 0;

}
