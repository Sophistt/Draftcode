#include "StdAfx.h"
#include "TSmap.h"
#include "My_Timer.h"
//My_Timer mytime1;
//My_Timer mytime2;
//My_Timer mytime3;
//My_Timer mytime4;
TSmap::TSmap(void)
{
}

TSmap::~TSmap(void)
{
}
void TSmap::Modifymap(I_Map* ditu,NewMap* xinditu)
{
	CIVDecisionApp *app=(CIVDecisionApp*)AfxGetApp();
	//app->critical_section.Lock();
	CvPoint2D64f m_gps=dyob_list.xinhuang_pos;
	double dir=dyob_list.xinhuang_dir;
	//app->critical_section.Unlock();  
	
	//mytime4.Start();
	////CreateTimeTable(); //新算法不再需要时间查找表了
	//mytime4.End();
	//mytime4.costTime;

	int direction_error = 10;//设置方向误差为10度

	//NewMap* dynamic_map=(struct NewMap*)calloc(1,sizeof( struct NewMap )); 
	//NewMap* dynamic_map=new NewMap; 

	//mytime1.Start();
	for (int i=0;i<512;i++)
	{
		for(int j=0;j<512;j++)
		{

			//ASSERT(i>=0&&i<512&&j>=0&&j<512);
			xinditu->newmap[j][i].contant = ditu->MapPoint[j][i];
			xinditu->newmap[j][i].collision = 0;
			xinditu->newmap[j][i].danger = -1;
			xinditu->newmap[j][i].v = -1;
			xinditu->newmap[j][i].dir = -1;
			xinditu->newmap[j][i].ID = -1;
			xinditu->newmap[j][i].ID1 = -1;
			xinditu->newmap[j][i].ID2 = -1;
		}
	}
	//mytime1.End();
	//mytime1.costTime;

	//mytime2.Start();
	POSITION pos=dyob_list.fusionlist.GetHeadPosition();
	while(pos!=NULL)
	{
		DyObstacle newobstacle=dyob_list.fusionlist.GetNext(pos);
		/*if (newobstacle.GetMovingConfidence()==0)
		{
			continue;
		}*/
		CvPoint2D64f newpos[2000];
		CvPoint2D64f p,q,r;
		int x,y,dx,dy;
		int collision_num;
		newobstacle.GetPos(newpos);

		q=newobstacle.GetIbeoCenter();
		p = m_GpsData.APiontConverD(m_gps,q,dir);
		p.x=int(p.x+0.5);
		p.y=int(p.y+0.5);
		//xinditu->newmap[(int)p.y][(int)p.x].ID=newobstacle.GetObNum();
		/*if(newobstacle.GetObstacleKind() != 3 )
		{
			collision_num = collision_analysis(newobstacle,q);
		}*/
		collision_num = collision_analysis(newobstacle,q);
		for(int k=0; k<newobstacle.GetArea(); k++)
		{
			r = m_GpsData.APiontConverD(m_gps,newpos[k],dir);
			r.x=int(r.x+0.5);
			r.y=int(r.y+0.5);
			if (newobstacle.GetObstacleKind()==3)
			{
				xinditu->newmap[(int)r.y][(int)r.x].ID = -2;
			}
			else
			{
				xinditu->newmap[(int)r.y][(int)r.x].ID=newobstacle.GetObNum();
			}
			
			xinditu->newmap[(int)r.y][(int)r.x].v = newobstacle.GetV();
			xinditu->newmap[(int)r.y][(int)r.x].dir = newobstacle.GetMovingDir();
			xinditu->newmap[(int)r.y][(int)r.x].danger = newobstacle.GetDanger();
		}
		if (collision_num == 1)
		{
			x = (int)collision_position1.x;
			y = (int)collision_position1.y;
			if ((x>=0)&&(y>=0)&&(x<512)&&(y<512))
			{
				dx = x-p.x;
				dy = y-p.y;
				for(int k=0; k<newobstacle.GetArea(); k++)
				{
					r = m_GpsData.APiontConverD(m_gps,newpos[k],dir);
					r.x=int(r.x+0.5);
					r.y=int(r.y+0.5);
					//xinditu->newmap[(int)r.y][(int)r.x].ID=newobstacle.GetObNum();
					r.x = r.x + dx;
					r.y = r.y + dy;
					if ((r.x>=0)&&(r.y>=0)&&(r.x<512)&&(r.y<512)&&(newobstacle.GetObstacleKind()!=3))
					{
						xinditu->newmap[(int)r.y][(int)r.x].collision = 1;
					}
					else if ((r.x>=0)&&(r.y>=0)&&(r.x<512)&&(r.y<512)&&(newobstacle.GetObstacleKind()==3))
					{
						xinditu->newmap[(int)r.y][(int)r.x].collision = 2;
					}
				}
				
			}
			/*else
			{
				for(int k=0; k<newobstacle.GetArea(); k++)
				{
					r = m_GpsData.APiontConverD(m_gps,newpos[k],dir);
					r.x=int(r.x+0.5);
					r.y=int(r.y+0.5);
					xinditu->newmap[(int)r.y][(int)r.x].ID=newobstacle.GetObNum();
				}
			}*/
				
		}
		else if (collision_num == 2)
		{
			x = (int)collision_position1.x;
			y = (int)collision_position1.y;
			if ((x>=0)&&(y>=0)&&(x<512)&&(y<512))
			{
				dx = x-p.x;
				dy = y-p.y;
				for(int k=0; k<newobstacle.GetArea(); k++)
				{
					r = m_GpsData.APiontConverD(m_gps,newpos[k],dir);
					r.x=int(r.x+0.5);
					r.y=int(r.y+0.5);
					//xinditu->newmap[(int)r.y][(int)r.x].ID=newobstacle.GetObNum();
					r.x = r.x + dx;
					r.y = r.y + dy;
					if ((r.x>=0)&&(r.y>=0)&&(r.x<512)&&(r.y<512)&&(newobstacle.GetObstacleKind()!=3))
					{
						xinditu->newmap[(int)r.y][(int)r.x].collision = 1;
					}
					else if ((r.x>=0)&&(r.y>=0)&&(r.x<512)&&(r.y<512)&&(newobstacle.GetObstacleKind()==3))
					{
						xinditu->newmap[(int)r.y][(int)r.x].collision = 2;
					}
				}
			}
			/*else
			{
				for(int k=0; k<newobstacle.GetArea(); k++)
				{
					r = m_GpsData.APiontConverD(m_gps,newpos[k],dir);
					r.x=int(r.x+0.5);
					r.y=int(r.y+0.5);
					xinditu->newmap[(int)r.y][(int)r.x].ID=newobstacle.GetObNum();
				}
			}*/
				
			x = (int)collision_position2.x;
			y = (int)collision_position2.y;
			if ((x>=0)&&(y>=0)&&(x<512)&&(y<512))
			{
				dx = x-p.x;
				dy = y-p.y;
				for(int k=0; k<newobstacle.GetArea(); k++)
				{
					r = m_GpsData.APiontConverD(m_gps,newpos[k],dir);
					r.x=int(r.x+0.5);
					r.y=int(r.y+0.5);
					//xinditu->newmap[(int)r.y][(int)r.x].ID=newobstacle.GetObNum();
					r.x = r.x + dx;
					r.y = r.y + dy;
					if ((r.x>=0)&&(r.y>=0)&&(r.x<512)&&(r.y<512)&&(newobstacle.GetObstacleKind()!=3))
					{
						xinditu->newmap[(int)r.y][(int)r.x].collision = 1;
					}
					else if ((r.x>=0)&&(r.y>=0)&&(r.x<512)&&(r.y<512)&&(newobstacle.GetObstacleKind()==3))
					{
						xinditu->newmap[(int)r.y][(int)r.x].collision = 2;
					}
				}
			}
			/*else
			{
				for(int k=0; k<newobstacle.GetArea(); k++)
				{
					r = m_GpsData.APiontConverD(m_gps,newpos[k],dir);
					r.x=int(r.x+0.5);
					r.y=int(r.y+0.5);
					xinditu->newmap[(int)r.y][(int)r.x].ID=newobstacle.GetObNum();
				}
			}*/
		}


		/*for (int k=0; k<newobstacle.GetArea(); k++)
		{
			p = m_GpsData.APiontConverD(m_gps,newpos[k],dir);
			p.x=int(p.x+0.5);
			p.y=int(p.y+0.5);
			xinditu->newmap[(int)p.y][(int)p.x].ID=newobstacle.GetObNum();
			collision_num = collision_analysis(newobstacle,newpos[k]);
			if (collision_num == 1)
			{
				x = (int)collision_position1.x;
				y = (int)collision_position1.y;
				if ((x>=0)&&(y>=0)&&(x<512)&&(y<512))
				{
					xinditu->newmap[y][x].collision = 1;
					if (xinditu->newmap[y][x].ID1== -1)
					{
						xinditu->newmap[y][x].ID1 = newobstacle.GetObNum();
					}
					else
					{
						xinditu->newmap[y][x].ID2 = newobstacle.GetObNum();
					}
				}
				
			}
			else if (collision_num == 2)
			{
				x = (int)collision_position1.x;
				y = (int)collision_position1.y;
				if ((x>=0)&&(y>=0)&&(x<512)&&(y<512))
				{
					xinditu->newmap[y][x].collision = 1;
					if (xinditu->newmap[y][x].ID1== -1)
					{
						xinditu->newmap[y][x].ID1 = newobstacle.GetObNum();
					}
					else
					{
						xinditu->newmap[y][x].ID2 = newobstacle.GetObNum();
					}
				}
				
				x = (int)collision_position2.x;
				y = (int)collision_position2.y;
				if ((x>=0)&&(y>=0)&&(x<512)&&(y<512))
				{
					xinditu->newmap[y][x].collision = 1;
					if (xinditu->newmap[y][x].ID1== -1)
					{
						xinditu->newmap[y][x].ID1 = newobstacle.GetObNum();
					}
					else
					{
						xinditu->newmap[y][x].ID2 = newobstacle.GetObNum();
					}
				}
			}
		}*/
	}
	
		
		
	//mytime2.End();
	//mytime2.costTime;
	/*NewMap *mymap=dynamic_map;
	delete dynamic_map;
	return mymap;*/
	//return dynamic_map;
	//pos=dyob_list.ibeolist.GetHeadPosition();
	//while(pos!=NULL)
	//{
	//	DyObstacle newobstacle=dyob_list.ibeolist.GetNext(pos);
	//	CvPoint2D64f box_point[4]={0};
	//	newobstacle.GetBoxPoints(box_point);

	//	CvPoint2D64f box_center;
	//	box_center=newobstacle.GetIbeoCenter();

	//	int length_x=newobstacle.GetLength_X();
	//	int length_y=newobstacle.GetLength_Y();

 //       box_center=m_GpsData.APiontConverD(m_gps,box_center,dir);
	//	for (int index=0; index<4; index++)
	//	{
	//		box_point[index]=m_GpsData.APiontConverD(m_gps,box_point[index],dir);
	//	}
 //       int radius=length_x+length_y;
	//	for (int index_x=-radius; index_x<radius; index_x++)
	//	{
	//		for (int index_y=-radius; index_y<radius; index_y++)
	//		{
	//			int pos_x=box_center.x+index_x;
	//			int pos_y=box_center.y+index_y;
	//			CvPoint2D64f inner_point;
	//			inner_point.x=pos_x;
	//			inner_point.y=pos_y;
	//			if ((pos_x<512&&pos_x>=0&&pos_y<512&&pos_y>=0)&&((xinditu->newmap[pos_y][pos_x].contant==8)||(xinditu->newmap[pos_y][pos_x].contant==18))&&(xinditu->newmap[pos_y][pos_x].ID==-1))
	//			{
	//				if (dyob_list.IbeoExpansion2(box_point,inner_point,length_x,length_y))
	//				{
	//					//xinditu->newmap[pos_y][pos_x].ID=1;
	//				}	
	//			}
	//		}
	//	}
	//}

}



int TSmap::collision_analysis(DyObstacle newobstacle,CvPoint2D64f p)//进行碰撞分析，返回预测到的碰撞位置个数，因为在平面内解析几何，有时候会产生两个碰撞位置的
{
	CIVDecisionApp *app=(CIVDecisionApp*)AfxGetApp();
	/*app->critical_section.Lock();
	CvPoint2D64f m_gps=app->GPS_Point;
	double dir=app->GPS_Direction;
	app->critical_section.Unlock();*/ 
	CvPoint2D64f m_gps=dyob_list.xinhuang_pos;
	double dir=dyob_list.xinhuang_dir;
	
	bool verify_result,verify_result1,verify_result2;
	double x,y;//求解出的预测碰撞位置
	double direction;
	double angle;//障碍物与无人车连线在地图坐标系中的方向
	double angle_difference;
	double side_length1,side_length2,side_length2_1,side_length2_2,side_length3;
	double a,b,c,delta;
	double vv,vo;
	vv = m_GpsData.GetSpeed();//无人车行驶速度
	if(vv ==0)
	{
		//vv = 9; //实验室仿真专用
		vv = 0.001;
	}
	vo = newobstacle.GetV();//障碍物运动速度
	direction = newobstacle.GetMovingDir();//障碍物运动方向，下面转换为地图坐标系中的方向
	direction = 270 + direction - dir;
	if(direction < 0)
	{
		direction = direction + 360;
	}
	else if(direction > 360)
	{
		direction = direction - 360;
	}
	p=m_GpsData.APiontConverD(m_gps,p,dir);

	angle = m_GpsData.GetAngle(m_gps.x,m_gps.y,newobstacle.GetCenter().x,newobstacle.GetCenter().y);
	angle = 270 + angle - dir;
	if(angle < 0)
	{
		angle = angle + 360;
	}
	else if(angle > 360)
	{
		angle = angle - 360;
	}

	angle_difference = abs(angle - direction);
	if (angle_difference == 0)
	{
		angle_difference = 1;
	}
	if (angle_difference == 180)
	{
		angle_difference = 179;
	}

		if((360 - angle_difference)<180)
		{
			angle_difference = 360 - angle_difference;
		}
		side_length1 = sqrt(pow((double)(p.x-256),2)+pow((double)(p.y-412),2));
		a = pow((vo/vv),2)-1;
		if(a == 0)
		{
			a = 0.0001;
		}
		b = -2*(vo/vv)*side_length1*cos(rad(angle_difference));
		c = pow(side_length1,2);
		delta = pow(b,2)-4*a*c;
		if (delta<0)
		{
			return 0;
		}
		if (delta==0)
		{
			delta = 0.0001;
		}
		side_length2_1 = ((-b)+(sqrt(delta)))/(2*a);
		side_length2_2 = ((-b)-(sqrt(delta)))/(2*a);
		if ((side_length2_1<=0)&&(side_length2_2<=0))
		{
			return 0;
		}
		else if((side_length2_1>0)&&(side_length2_2<=0))
		{
			collision_position1.x = p.x + side_length2_1*cos(rad(direction))*vo/vv;
			collision_position1.y = p.y + side_length2_1*sin(rad(direction))*vo/vv;
			verify_result = determine_solution(collision_position1,p,newobstacle.GetMovingDir());
			if (verify_result)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}
		else if((side_length2_2>0)&&(side_length2_1<=0))
		{
			collision_position1.x = p.x + side_length2_2*cos(rad(direction))*vo/vv;
			collision_position1.y = p.y + side_length2_2*sin(rad(direction))*vo/vv;
			verify_result = determine_solution(collision_position1,p,newobstacle.GetMovingDir());
			if (verify_result)
			{
				return 1;
			}
			else
			{
				return 0;
			}
		}
		else if((side_length2_1>0)&&(side_length2_2>0))
		{
			collision_position1.x = p.x + side_length2_1*cos(rad(direction))*vo/vv;
			collision_position1.y = p.y + side_length2_1*sin(rad(direction))*vo/vv;
			collision_position2.x = p.x + side_length2_2*cos(rad(direction))*vo/vv;
			collision_position2.y = p.y + side_length2_2*sin(rad(direction))*vo/vv;
			verify_result1 = determine_solution(collision_position1,p,newobstacle.GetMovingDir());
			verify_result2 = determine_solution(collision_position2,p,newobstacle.GetMovingDir());
			if((verify_result1==1)&&(verify_result2==1))
			{
				return 2;
			}
			else if((verify_result1 == 0)&&(verify_result2 == 0))
			{
				return 0;
			}
			else if((verify_result1 == 0)&&(verify_result2 == 1))
			{
				collision_position1.x = collision_position2.x;
				collision_position1.y = collision_position2.y;
				return 1;
			}
			else if((verify_result1 == 1)&&(verify_result2 == 0))
			{
				return 1;
			}
		}
	
}

bool TSmap::determine_solution(CvPoint2D64f collision_position,CvPoint2D64f ob_position,double travel_direction)//对求得的解进行验证，可能某些求得的解是无用解
{
	CIVDecisionApp *app=(CIVDecisionApp*)AfxGetApp();
	/*app->critical_section.Lock();
	CvPoint2D64f m_gps=app->GPS_Point;
	double dir=app->GPS_Direction;
	app->critical_section.Unlock();*/ 
	CvPoint2D64f m_gps=dyob_list.xinhuang_pos;
	double dir=dyob_list.xinhuang_dir;

	double verification_direction,direction_error;
	collision_position = m_GpsData.MaptoGPS(m_gps,dir,collision_position);
	ob_position = m_GpsData.MaptoGPS(m_gps,dir,ob_position);
	verification_direction = m_GpsData.GetAngle(collision_position.x,collision_position.y,ob_position.x,ob_position.y);
	direction_error = abs(verification_direction - travel_direction);
	if(direction_error>=360)
	{
		direction_error = direction_error-360;
	}
	if ((direction_error<20)||(360-direction_error)<20)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

void TSmap::Sequence(int x1,int y1,int x2,int y2,int x3,int y3,int x4,int y4)
{
	int x;
	x = x1;
	if (x<=x2)
	{
	}
	else
	{
		x=x2;
	}
	if (x<=x3)
	{
	}
	else
	{
		x=x3;
	}
	if (x<=x4)
	{
	}
	else
	{
		x=x4;
	}
	min_x = x; 
	x = x1;
	if (x>=x2)
	{
	}
	else
	{
		x=x2;
	}
	if (x>=x3)
	{
	}
	else
	{
		x=x3;
	}
	if (x>=x4)
	{
	}
	else
	{
		x=x4;
	}
	max_x = x;
	x = y1;
	if (x<=y2)
	{
	}
	else
	{
		x=y2;
	}
	if (x<=y3)
	{
	}
	else
	{
		x=y3;
	}
	if (x<=y4)
	{
	}
	else
	{
		x=y4;
	}
	min_y = x;
	x = y1;
	if (x>=y2)
	{
	}
	else
	{
		x=y2;
	}
	if (x>=y3)
	{
	}
	else
	{
		x=y3;
	}
	if (x>=y4)
	{
	}
	else
	{
		x=y4;
	}
	max_y = x;
}
