#include "StdAfx.h"
#include "DyObstacle.h"

DyObstacle::DyObstacle(void)
{
	/*this->guauss=5;
	this->pre_uncertan=1;*/
	this->optimal_d=1;
	this->optimal_angle=0.3;
	/*this->see_error=3;*/
	for (int i=0; i<10; i++)                 //对位置赋初值-1
	{
		for (int j=0; j<SIZE; j++)
		{
			pos[i][j].x=-1;
			pos[i][j].y=-1;
		}
	}
	for (int i=0; i<10; i++)                //对加速度赋初值0
	{
		this->m_accel[i]=0;
	}
	for (int i=0; i<10; i++)               //对面积大小赋初值0
	{
		this->m_area[i]=0;
	}
	for (int i=0; i<10; i++)               //对速度赋初值0
	{
		this->m_v[i]=0;
	}
	for (int i=0; i<10; i++)               //对运动标志赋初值0
	{
		this->m_movingflag[i]=0;
	}
	for (int i=0; i<10; i++)               //对运动方向赋初值0
	{
		this->m_movingdir[i]=0;
	}
	for (int i=0; i<10; i++)               //对中心点赋初值
	{
		center[i].x=0;
		center[i].y=0;
	}
	
	this->m_count=1;                       //对已经存储的帧数赋初值0
	this->m_existconfidence=0;            //对存在置信度赋初值10
	this->m_movingconfidence=0;           //对运动置信度赋初值10
	this->m_ob_num=-1;                     //对障碍物编号赋初值-1
	this->m_dirchange=0;
	this->m_radius=0;
	this->m_ibeodir = 0;
	this->m_ibeoarea = 0;

	Length_X=0;
	Length_Y=0;
	for(int i=0; i<4; i++)
	{
		box_points[i].x=0;
		box_points[i].y=0;
	}
	IsIbeo=0;
	relative_speed=0;
	relative_dir = 0;
	m_danger = 0;
	m_oriX = 0;
	m_oriY = 0;
	m_vchange = 0;
	m_obstacle_kind = -1;
	visted = -1;
	IsMan = 0;
	man_count = 0;
	notman_count = 0;

}

DyObstacle::~DyObstacle(void)
{
}



void DyObstacle::SetPos(CvPoint2D64f *obpos)
{
	/*第obnum*/
	
		for (int k=0; k<SIZE; k++)
		{
			this->pos[0][k].x=obpos[k].x;
			this->pos[0][k].y=obpos[k].y;
		}
		return;
	
}



void DyObstacle::SetMapPos( CvPoint2D64f *obpos )
{
	for (int k=0; k<SIZE; k++)
	{
		this->mappos[0][k].x=obpos[k].x;
		this->mappos[0][k].y=obpos[k].y;
	}
	return;

}


void DyObstacle::SetV(double v)
{
	
		this->m_v[0]=v;
	
	
}

void DyObstacle::SetAccel(double a)
{
	if (this->m_count<10)
	{	
	     this->m_accel[m_count]=a;	
	}
	else{
		for (int j=0; j<9; j++)
		{
			this->m_accel[j]=this->m_accel[j+1];	
		}
		this->m_accel[9]=a;		
	}
}

void DyObstacle::SetMovingDir(double dir)
{
	
		this->m_movingdir[0]=dir;	
	
}
void DyObstacle::SetExistConfidence(int e)
{
	this->m_existconfidence=e;
}
void DyObstacle::SetMovingConfidence(int m)
{
	/*this->m_movingconfidence=this->m_movingconfidence+m;*/
	this->m_movingconfidence=m;
}

void DyObstacle::SetMovingFlag(bool flag)
{
	
	if (m_count<10)
	{
	     this->m_movingflag[m_count]=flag;
	}
	else
	{
		for (int j=0; j<9; j++)
		{
			this->m_movingflag[j]=this->m_movingflag[j+1];
		}
		this->m_movingflag[9]=flag;
	}
}

void DyObstacle::SetArea(unsigned int aer)
{
	
		this->m_area[0]=aer;	

}

void DyObstacle::SetObNum(int n)
{
	this->m_ob_num=n;
}
void DyObstacle::SetObCount(int m)
{
	this->m_count=this->m_count+m;
}

void DyObstacle::SetCenter(CvPoint2D64f newcenter)
{

		this->center[0].x=newcenter.x;
		this->center[0].y=newcenter.y;

}

void DyObstacle::Setdirchange(int dc)
{
	m_dirchange=m_dirchange+dc;
}

void DyObstacle::SetRadius(double radius)
{
	m_radius=radius;
}
void DyObstacle::SetTime(int time)
{
	if (m_count<10)
	{
		this->Time[m_count]=time;
		
	}
	else
	{
		for (int i=0; i<9; i++)
		{
			Time[i]=Time[i+1];
		}
		Time[9]=time;
	}

}

void DyObstacle::SetDanger(double danger)
{
	m_danger = danger;
}
//void DyObstacle::SetRightPoint(CvPoint2D64f point)
//{
//	rightup_Boxpoint=point;
//}
//void DyObstacle::SetLeftPoint(CvPoint2D64f point)
//{
//	leftdown_Boxpoint=point;
//}
void DyObstacle::SetLength_X(double x)
{
	Length_X=x;
}
void DyObstacle::SetLength_Y(double y)
{
	Length_Y=y;
}
void DyObstacle::SetBoxPoints(CvPoint2D64f *boxpoints)
{
	for (int i=0; i!=4; i++)
	{
		box_points[i]=boxpoints[i];
	}
}

void DyObstacle::SetIsIbeo(bool isIbeo)
{
	IsIbeo=isIbeo;
}

void DyObstacle::SetRelativeSpeed(double rl_speed)
{
	relative_speed=rl_speed;
}
void DyObstacle::SetRelativaDir(double rl_dir)
{
	relative_dir=rl_dir;
}
void DyObstacle::SetIbeoCenter(CvPoint2D64f center)
{
	ibeo_center=center;
}

void DyObstacle::SetOriX(double x)
{
	m_oriX = x;
}

void DyObstacle::SetOriY(double y)
{
	m_oriY = y;
}

void DyObstacle::SetVchange(int vchange)
{
	m_dirchange = vchange;
}

void DyObstacle::SetObstacleKind(int kind)
{
	m_obstacle_kind = kind;
}

void DyObstacle::SetIbeoarea(int i)
{
	m_ibeoarea = i;
}

void DyObstacle::SetIbeodir(double idir)
{
	m_ibeodir = idir;
}
//CvPoint2D64f DyObstacle::GetCenter(CvPoint2D64f *ob1)
//{
//	CvPoint2D64f pt1;
//	CvPoint2D64f center1;
//	int i=0;
//	while(!(ob1[i].x==0&&ob1[i].y==0))
//	{
//    pt1.x+=ob1[i].x;
//	pt1.y+=ob1[i].y;
//	i++;
//	}
//   center1.x=pt1.x/i;
//   center1.y=pt1.y/i;
//   return center1;
//}

void DyObstacle::GetPos(int i,CvPoint2D64f (&newpos)[SIZE])
{
	for (int k=0; k<SIZE; k++)
	{
		newpos[k].x=pos[0][k].x;
		newpos[k].y=pos[0][k].y;
	}
}
void DyObstacle::GetPos(CvPoint2D64f (&newpos)[SIZE])
{
	
		for (int i=0; i<SIZE; i++)
		{
			newpos[i].x=pos[0][i].x;
			newpos[i].y=pos[0][i].y;
		}
}



void DyObstacle::GetMapPos( CvPoint2D64f (&newpos)[SIZE] )
{
	for (int i=0; i<SIZE; i++)
	{
		newpos[i].x=mappos[0][i].x;
		newpos[i].y=mappos[0][i].y;
	}

}


double DyObstacle::GetV(int i)            //返回前i个速度，当i=0，返回当前的速度
{
	
	return this->m_v[0];
}
double DyObstacle::GetV()
{
	
	return this->m_v[0];
	
}

double DyObstacle::GetAccel(int i)
{
	
	return this->m_accel[9-i];
}
double DyObstacle::GetAccel()
{
	if (m_count>10)
	{
		return this->m_accel[9];
	}
	else
		return this->m_accel[m_count-1];
}

double DyObstacle::GetMovingDir(int i)
{
	
	return this->m_movingdir[9-i];
}

double DyObstacle::GetMovingDir()
{

		return this->m_movingdir[0];

}

int DyObstacle::GetExistConfidence()
{
	
	return this->m_existconfidence;
}

int  DyObstacle::GetMovingConfidence()
{
	
	return this->m_movingconfidence;
}

bool DyObstacle::GetMovingFlag(int i)
{
		return this->m_movingflag[9-i];
}

bool DyObstacle::GetMovingFlag()
{
	if (m_count>10)
	{
		return this->m_movingflag[9];
	}
	else
		return this->m_movingflag[m_count-1];
}

int DyObstacle::GetObNum()
{
	return this->m_ob_num;
}

int DyObstacle::GetArea(int i)
{
	return this->m_area[0];
}

int DyObstacle::GetArea()
{
	if (m_count>10)
	{
		return this->m_area[9];
	}
	else 
		return this->m_area[m_count-1];
}
int DyObstacle::GetObCount()
{
	return this->m_count;
}

int DyObstacle::Getdirchange()
{
	return m_dirchange;
}
double DyObstacle::GetRadius()
{
	return this->m_radius;
}

CvPoint2D64f DyObstacle::GetCenter(int k)
{
	
		return this->center[9-k];
	
}

CvPoint2D64f DyObstacle::GetCenter()
{

		return this->center[0];
	
}
int DyObstacle::GetTime()
{
	if (m_count>10)
	{
		return this->Time[9];
	}
	else{
		return this->Time[m_count-1];
	}
}
//void DyObstacle::GetRightPoint(CvPoint2D64f &point)
//{
//	point=rightup_Boxpoint;
//}
//void DyObstacle::GetLeftPoint(CvPoint2D64f &point)
//{
//	point=leftdown_Boxpoint;
//}

double DyObstacle::GetLength_X()
{
	return Length_X;
}
double DyObstacle::GetLength_Y()
{
	return Length_Y;
}
void DyObstacle::GetBoxPoints(CvPoint2D64f (&boxpoints)[4])
{
	for (int i=0; i!=4; i++)
	{
		boxpoints[i]=box_points[i];
	}
}

bool DyObstacle::GetIsIbeo()
{
	return IsIbeo;
}

double DyObstacle::GetRLSpeed()
{
	return relative_speed;
}
double DyObstacle::GetRLDir()
{
	return relative_dir;
}
CvPoint2D64f DyObstacle::GetIbeoCenter()
{
	return ibeo_center;
}

double DyObstacle::GetDanger()
{
	return m_danger;
}

double DyObstacle::GetOriX()
{
	return m_oriX;
}

double DyObstacle::GetOriY()
{
	return m_oriY;
}
int DyObstacle::GetVchange()
{
	return m_vchange;
}

int DyObstacle::GetObstacleKind()
{
	return m_obstacle_kind;
}

int DyObstacle::GetIbeoarea()
{
	return m_ibeoarea;
}
double DyObstacle::GetIbeodir()
{
	return m_ibeodir;
}