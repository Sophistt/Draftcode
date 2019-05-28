#pragma once
#include "vehicle.h"
#include "DyObstacle.h"
#include "GrowCluster.h"
#include "My_Timer.h"

#include <vector>

//struct NDiff
//{
//	double d;
//	bool visited;
//};
class Vlist :
	public CVehicle
{	
private:
	int  m_rIsBelievable;
	int  m_lIsBelievable;
	double m_rk;
	double m_rb;
	double m_lk;
	double m_lb;
	
	I_Map *m_imap;

public:
	Vlist(void);
	void Update(I_Map *map);
	int dy_obstacle_num;
	/*vector<DyObstacle>  oblist;*/
	double ComparaeOb(DyObstacle listob,DyObstacle newob);
	/*NewMap* UpdateMap(NewMap *map);*/
	GrowCluster cluster;
    int s;
	CList<DyObstacle,DyObstacle> newlist;//velodyn动态障碍物列表
	CList<DyObstacle,DyObstacle> ibeolist;//Ibeo动态障碍物列表
	CList<DyObstacle,DyObstacle>  fusionlist;//融合得到的列表
	CList<DyObstacle,DyObstacle> temp_list;       //存储上一帧中的障碍物列表
	double l_variancex;
	double l_variancey;
    double r_variancex;
	double r_variancey;
	double l_meanx;
	double l_meany;
	double r_meanx;
	double r_meany;

	double aaa[6];

	CvPoint2D64f l_roadedge[512];
	CvPoint2D64f r_roadedge[512];

	double xinhuang_dir;             //存储实时方向
	CvPoint2D64f xinhuang_pos;       //存储实时gps坐标

;
	/*DyObstacle FindDyobstacle(CvPoint2D64f point);*/
    void Transfer(CvPoint2D64f &input_point);
    void Update_IbeoList(ObjectWT *ObjectWT1 ,int num);         //读入ibeo数据
	void SetDtRange(int before,int behind, int left, int right);
	void SetGrowRange(int width,int length);
	void SearchRLine(CvPoint2D64f (&line)[512],int &l_num);
	void SearchLLine(CvPoint2D64f (&line)[512],int &l_num);
	void CountLine(CvPoint2D64f *line,int l_num,double &k,double &b);
	void ObInRoad();
	void ObInPath();
	void Fuse(I_Map *map);
	void Fuse2(I_Map *map);
	bool IbeoExpansion(CvPoint2D64f box_points[4],CvPoint2D64f point);
	bool IbeoExpansion2(CvPoint2D64f box_points[4],CvPoint2D64f point,int length,int width);
	void  ObInRoad2();
	void Intersection();
	void SetAAA(double a[6]);
	void CountRoadEdge();
	double GetObDistace(DyObstacle ob1, DyObstacle ob2);
	double ClearOb();

public:
	~Vlist(void);
};
