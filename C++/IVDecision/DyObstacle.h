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
	CvPoint2D64f pos[10][SIZE];     //λ��
	CvPoint2D64f mappos[10][SIZE];
	double m_v[10];                              //�ٶ�
	double m_accel[10];                          //���ٶ�
	double m_movingdir[10];                      //��ʻ����
	int m_existconfidence;                       //�������Ŷ� 
	int m_movingconfidence;                      //�˶����Ŷ�
	bool m_movingflag[10];                       //�˶�״̬
	int  m_ob_num;                               //�ϰ�����
	int m_area[10];                              //դ������
	unsigned int m_count;                        //�Ѿ��洢��֡��
	CvPoint2D64f center[10];                     //�ϰ�������
	int Time[10];
	int m_dirchange;                             //����ı���������ڹ��˵�·����
	int m_vchange;                               //�ٶȴ�С�仯����
	double m_radius;                                //�ϰ�����뾶
	double sum_v;
	int sum_vnum;
	
	/*CvPoint2D64f rightup_Boxpoint;
	CvPoint2D64f leftdown_Boxpoint;*/

	double Length_X;                             //����
	double Length_Y;                             //���
	CvPoint2D64f box_points[4];                  //���Ӷ���
	
	double relative_speed;                       //����ٶ�
	double relative_dir;                         //��ԽǶ�
	CvPoint2D64f ibeo_center;                     //�ϰ�����������

	double m_danger;
	double m_oriX;
	double m_oriY;
	int m_obstacle_kind;

	int m_ibeoarea;
	int m_ibeodir;

	
	

	
	

	
public:
	DyObstacle(void);
	void SetPos(CvPoint2D64f *obpos);                     //�趨�ϰ��������еĵ��λ�ã��������Ϊ�ϰ����
	void SetMapPos(CvPoint2D64f *obpos);
	void SetV(double v);                                  //�趨�ϰ����ٶ�
	void SetAccel(double a);                              //�趨���ٶ�
	void SetMovingDir(double dir);                        //�趨�˶�����
	void SetExistConfidence(int e);                       //�趨�������Ŷ�,eΪ-1����+1
	void SetMovingConfidence(int m);                      //�趨�˶����Ŷ�
	void SetMovingFlag(bool flag);                        //�˶��趨Ϊ1����ֹ�趨Ϊ0��
	void SetArea(unsigned int aer);                                //�趨դ������
	void SetObNum(int n);                                 //�趨�ϰ������
	void SetObCount(int m);                               //�趨�Ѿ��۲��֡��
    void SetCenter(CvPoint2D64f newcenter);                                     //�趨�ϰ�����������
	void Setdirchange(int dc);
	void SetRadius(double radius); 
	void SetTime(int time);
	void SetDanger(double danger);
	/*void SetRightPoint(CvPoint2D64f point);
	void SetLeftPoint(CvPoint2D64f point);*/

	void SetLength_X(double x);//����
	void SetLength_Y(double y);//���
	void SetBoxPoints(CvPoint2D64f *boxpoints);//�趨���򶨵�
	void SetIsIbeo(bool isIbeo);//�Ƿ���ibeo�ϰ���
	void SetRelativeSpeed(double rl_speed);//�趨����ٶ�
	void SetRelativaDir(double rl_dir);//�趨����ٶ�
	void SetIbeoCenter(CvPoint2D64f center);
	void SetOriX(double x);
	void SetOriY(double y);
	void SetVchange(int vchange);
	void SetObstacleKind(int kind);
	void SetIbeoarea(int i);
	void SetIbeodir(double idir);

	void GetPos(int i,CvPoint2D64f (&newpos)[SIZE]);           //��ȡǰi֡��λ��
	void GetMapPos(CvPoint2D64f (&newpos)[SIZE]);
	void GetPos(CvPoint2D64f (&newpos)[SIZE]);
	double GetV(int i);                                        //��ȡǰi֡���ϰ����ٶ�
	double GetV();
	double GetAccel(int i);                                    //��ȡ�ϰ�����ٶ�
	double GetAccel();
	double GetMovingDir(int i);                                //��ȡ�˶�����
	double GetMovingDir();
	int GetExistConfidence();                                  //��ȡ�ϰ���������Ŷ�
	int GetMovingConfidence();                                 //��ȡ�˶����Ŷ�
	bool GetMovingFlag(int i);                                 //��ȡ�˶�״̬��־
	bool GetMovingFlag();
	int GetObNum();                                            //��ȡ�ϰ�����
	int GetArea(int i);                                        //��ȡ�ϰ���դ������
	int GetArea();
	int GetObCount();                                          //��ȡ�Ѿ��洢��֡��
	int Getdirchange(); 
	double GetRadius();
	CvPoint2D64f GetCenter(int k);                             //��ȡ��̬�ϰ����k����������                             
	CvPoint2D64f GetCenter();                                  //��ȡ�ϰ������ĵ�����
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
	/* �������˲�����Ҫ������                                                                     */
	/************************************************************************/
	double optimal_d;                                           //�ٶ�����ƫ��ֵ
	double optimal_angle;                                       //�Ƕ�����ֵ
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



	DyObstacle &operator =(const DyObstacle &D)                //���صȺ�
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

	//bool operator ==(const DyObstacle &D)                                            //�������
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

