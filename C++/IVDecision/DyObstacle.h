#pragma once

#include "ComuLcm.h"
#include "KalmanFilter.h"
#define  SIZE 2000              
struct ObPos
{
	unsigned long x;
	unsigned long y;
};
class DyObstacle
{
private:
	CvPoint2D64f pos[10][SIZE];     //位置
	CvPoint2D64f mappos[10][SIZE];
	double m_v[10];                              //速度
	double m_accel[10];                          //加速度
	double m_movingdir[10];                      //行驶方向
	int m_existconfidence;                       //存在置信度 
	int m_movingconfidence;                      //运动置信度
	bool m_movingflag[10];                       //运动状态
	int  m_ob_num;                               //障碍物编号
	int m_area[10];                              //栅格数量
	unsigned int m_count;                        //已经存储的帧数
	CvPoint2D64f center[10];                     //障碍物中心
	int Time[10];
	int m_dirchange;                             //方向改变次数，用于过滤道路边沿
	int m_vchange;                               //速度大小变化次数
	double m_radius;                                //障碍物最半径
	double sum_v;
	int sum_vnum;
	
	/*CvPoint2D64f rightup_Boxpoint;
	CvPoint2D64f leftdown_Boxpoint;*/

	double Length_X;                             //长度
	double Length_Y;                             //宽度
	CvPoint2D64f box_points[4];                  //盒子顶角
	
	double relative_speed;                       //相对速度
	double relative_dir;                         //相对角度
	CvPoint2D64f ibeo_center;                     //障碍物中心坐标

	double m_danger;
	double m_oriX;
	double m_oriY;
	int m_obstacle_kind;

	int m_ibeoarea;
	int m_ibeodir;

	
	

	
	

	
public:
	DyObstacle(void);
	void SetPos(CvPoint2D64f *obpos);                     //设定障碍物所含有的点的位置，传入参数为障碍物点
	void SetMapPos(CvPoint2D64f *obpos);
	void SetV(double v);                                  //设定障碍物速度
	void SetAccel(double a);                              //设定加速度
	void SetMovingDir(double dir);                        //设定运动方向
	void SetExistConfidence(int e);                       //设定存在置信度,e为-1或者+1
	void SetMovingConfidence(int m);                      //设定运动置信度
	void SetMovingFlag(bool flag);                        //运动设定为1，静止设定为0；
	void SetArea(unsigned int aer);                                //设定栅格数量
	void SetObNum(int n);                                 //设定障碍物序号
	void SetObCount(int m);                               //设定已经观测的帧数
    void SetCenter(CvPoint2D64f newcenter);                                     //设定障碍物中心坐标
	void Setdirchange(int dc);
	void SetRadius(double radius); 
	void SetTime(int time);
	void SetDanger(double danger);
	/*void SetRightPoint(CvPoint2D64f point);
	void SetLeftPoint(CvPoint2D64f point);*/

	void SetLength_X(double x);//长度
	void SetLength_Y(double y);//宽度
	void SetBoxPoints(CvPoint2D64f *boxpoints);//设定方框定点
	void SetIsIbeo(bool isIbeo);//是否是ibeo障碍物
	void SetRelativeSpeed(double rl_speed);//设定相对速度
	void SetRelativaDir(double rl_dir);//设定相对速度
	void SetIbeoCenter(CvPoint2D64f center);
	void SetOriX(double x);
	void SetOriY(double y);
	void SetVchange(int vchange);
	void SetObstacleKind(int kind);
	void SetIbeoarea(int i);
	void SetIbeodir(double idir);

	void GetPos(int i,CvPoint2D64f (&newpos)[SIZE]);           //获取前i帧的位置
	void GetMapPos(CvPoint2D64f (&newpos)[SIZE]);
	void GetPos(CvPoint2D64f (&newpos)[SIZE]);
	double GetV(int i);                                        //获取前i帧的障碍物速度
	double GetV();
	double GetAccel(int i);                                    //获取障碍物加速度
	double GetAccel();
	double GetMovingDir(int i);                                //获取运动方向
	double GetMovingDir();
	int GetExistConfidence();                                  //获取障碍物存在置信度
	int GetMovingConfidence();                                 //获取运动置信度
	bool GetMovingFlag(int i);                                 //获取运动状态标志
	bool GetMovingFlag();
	int GetObNum();                                            //获取障碍物编号
	int GetArea(int i);                                        //获取障碍物栅格数量
	int GetArea();
	int GetObCount();                                          //获取已经存储的帧数
	int Getdirchange(); 
	double GetRadius();
	CvPoint2D64f GetCenter(int k);                             //获取动态障碍物第k个中心坐标                             
	CvPoint2D64f GetCenter();                                  //获取障碍物中心点坐标
	int GetTime();
	/*void GetRightPoint(CvPoint2D64f &point);
	void GetLeftPoint(CvPoint2D64f &point);*/
  
	double GetLength_X();
	double GetLength_Y();
	void GetBoxPoints(CvPoint2D64f (&boxpoints)[4]);
	bool GetIsIbeo();
	double GetRLSpeed();
	double GetRLDir();
	CvPoint2D64f GetIbeoCenter();
	double GetDanger();

	double GetOriX();
	double GetOriY();
	int GetVchange();
	int GetObstacleKind();
	int GetIbeoarea();
	double GetIbeodir();

	/************************************************************************/
	/* 卡尔曼滤波所需要的数据                                                                     */
	/************************************************************************/
	double optimal_d;                                           //速度最优偏差值
	double optimal_angle;                                       //角度最优值
	int visted;
	int IsMan;
	double man_count;
	double notman_count;

	int xmin;
	int xmax;
	int ymin;
	int ymax;
	int disapear;
	int showp;
	double dir_change;
	double v_change;

	

	std::queue<CvPoint2D64f> centerlist;
	//KalmanFilter fileter;
	bool IsIbeo;         



	DyObstacle &operator =(const DyObstacle &D)                //重载等号
	{
		this->optimal_d=D.optimal_d;
	
		for (int i=0; i<10; i++)
		{
			this->m_v[i]=D.m_v[i];
		}
		for (int i=0; i<10; i++)
		{
			this->m_accel[i]=D.m_accel[i];
		}

		for (int i=0; i<10; i++)
		{
			this->center[i].x=D.center[i].x;
			this->center[i].y=D.center[i].y;
		}
		for (int i=0; i<10; i++)
		{
			this->m_movingdir[i]=D.m_movingdir[i];
		}

		this->m_existconfidence=D.m_existconfidence;

		this->m_movingconfidence=D.m_movingconfidence;

		for (int i=0; i<10; i++)
		{
		this->m_movingflag[i]=D.m_movingflag[i];
		}

		for (int i=0; i<10; i++)
		{
			this->m_area[i]=D.m_area[i];
		}

		for (int i=0; i<10; i++)
		{
			for (int j=0; j<SIZE; j++)
			{
				this->pos[i][j].x=D.pos[i][j].x;
				this->pos[i][j].y=D.pos[i][j].y;
			}
		}

		this->m_ob_num=D.m_ob_num;
		this->m_count=D.m_count;
		this->m_dirchange=D.m_dirchange;
		this->m_radius=D.m_radius;
		this->centerlist=D.centerlist;

		/*this->rightup_Boxpoint=D.rightup_Boxpoint;
		this->leftdown_Boxpoint=D.leftdown_Boxpoint;*/
		this->Length_X=D.Length_X;
		this->Length_Y=D.Length_Y;
		this->relative_speed=D.relative_speed;
		this->relative_dir=D.relative_dir;
		this->ibeo_center=D.ibeo_center;
		this->m_danger = D.m_danger;
		this->m_oriX = D.m_oriX;
		this->m_oriY = D.m_oriY;
		this->m_vchange = D.m_vchange;
		this->m_obstacle_kind = D.m_obstacle_kind;
		this->visted = D.visted;
		this->IsMan = D.IsMan;
		this->m_ibeoarea = D.m_ibeoarea;
		this->m_ibeodir = D.m_ibeodir;

		this->xmin=D.xmin;
		this->xmax=D.xmax;
		this->ymin=D.ymin;
		this->ymax=D.ymax;
		this->dir_change=D.dir_change;
		this->disapear=D.disapear;
		this->v_change=D.v_change;
		//this->fileter=D.fileter;
		this->IsIbeo=D.IsIbeo;
		this->sum_v=D.sum_v;
		this->sum_vnum=D.sum_vnum;
		this->showp=D.showp;
	
		for (int i=0; i!=4; i++)
		{
			this->box_points[i]=D.box_points[i];
		}
		return *this;
	}

	//bool operator ==(const DyObstacle &D)                                            //重载相等
	//{
	//
	//	for (int i=0; i<10; i++)
	//	{
	//		for (int j=0; j<SIZE; j++)
	//		{
	//			if (this->pos[i][j].x!=D.pos[i][j].x||this->pos[i][j].y!=D.pos[i][j].y)
	//			{
	//				return 0;
	//			}
	//		}
	//	}
	//	for (int i=0; i<10; i++)
	//	{
	//		if (this->center[i].x!=D.center[i].x||this->center[i].y!=D.center[i].y)
	//		{
	//			return 0;
	//		}
	//	}
	//	for (int i=0; i<10; i++)
	//	{
	//		if (this->m_v[i]!=D.m_v[i])
	//		{
	//			return 0;
	//		}
	//	}
	//	for (int i=0; i<10; i++)
	//	{
	//		if (this->m_accel[i]!=D.m_accel[i])
	//		{
	//			return 0;
	//		}
	//	}
	//	for (int i=0; i<10; i++)
	//	{
	//		if (this->m_movingdir[i]!=D.m_movingdir[i])
	//		{
	//			return 0;
	//		}
	//	}
	//	for (int i=0; i<10; i++)
	//	{
	//		if (this->m_area[i]!=D.m_area[i])
	//		{
	//			return 0;
	//		}
	//	}
	//		if (this->m_existconfidence!=D.m_existconfidence)
	//		{
	//			return 0;
	//		}
	//	
	//	    if (this->m_movingconfidence!=D.m_movingconfidence)
	//	    {
	//			return 0;
	//	    }
	//		for (int i=0; i<10; i++)
	//		{
	//			if (this->m_movingflag[i]!=D.m_movingflag[i])
	//			{
	//				return 0;
	//			}
	//		}
	//		
	//		if (this->m_ob_num!=D.m_ob_num)
	//		{
	//			return 0;
	//		}
	//		if (this->m_count!=D.m_count)
	//		{
	//			return 0;
	//		}
	//		return 1;
	//}
public:
	~DyObstacle(void);
};

