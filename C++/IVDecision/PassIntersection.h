#pragma once
#include "intersectionstate.h"
#include "UTurn.h"
class CPassIntersection :
	public CVehicle
{
public:
	CPassIntersection(void);
	void Intersection_Driver();
    void PassIntersectionDirve();

	CvPoint2D64f endpoint,next_intersection_point,currentpoint,cpoint_xiugai;
	int GetCross_newtest();
	CvPoint2D64f WayPoint1[5];
	int GetCross_new(CvSeq *gp,CvPoint2D64f *Point,CvPoint2D64f (&WayPoint)[7],CvPoint2D64f (&IntersectionPath)[200]);
	int GetCross_lrguihua(CvPoint2D64f (&upoint)[4],CvPoint2D64f (&IntersectionPath)[200]);
	double ae,ac;//终止点的方向角，当前点的航向角
	double possible_ae;
	double difference_ae1,difference_ae2;
	bool next_road_edge0;
	bool next_road_edge;
	void solvepoints(CvPoint2D64f P1,CvPoint2D64f P2,double a2,CvPoint2D64f &P3,CvPoint2D64f &P4,CvPoint2D64f &P5);
	void solvepoints1(CvPoint2D64f P1,CvPoint2D64f P2,double a2,CvPoint2D64f &P3,CvPoint2D64f &P4,CvPoint2D64f &P5);
	void solvepoints2(CvPoint2D64f P1,CvPoint2D64f P2,CvPoint2D64f &P3,CvPoint2D64f &P4,CvPoint2D64f &P5);
	void correct_endpoint_direction(CvPoint2D64f ppp,double &lll);
	double correct_endpoint_direction_1(I_Map *map,CvPoint2D64f ggg,int luduan1,double jjj);
	double correct_endpoint_direction_2(I_Map *map,CvPoint2D64f ggggg,int luduan2,double jjjjj);
	void planpath(CvPoint2D64f ccc,double ddd);
	CvPoint2D64f correct_endpoint_location(CvPoint2D64f ppp,CvPoint2D64f &qqq,double lll);
	bool Search_edge(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down,int &edge_x,int &edge_y);
	bool SearchObstacle(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down);
	void qiudian(CvPoint2D64f *Point,int luduan);
	void qiudian1(CvPoint2D64f *Point,int luduan,double gps_fangxiang);
	int GetCross_newexe(CvPoint2D64f (&IntersectionPath)[200]);
	double next_direction[10];
	double ae_map;
	int zuohuandao;
	double ae_history[2];
	CvPoint2D64f endpoint_map;
	I_Map *temp_map;

	void PassIntersectionDirvecjj();
	void PassIntersectionDirvexy();
	CvPoint2D64f findInterAimPoint(CvPoint2D64f m_gps,double dir,CvPoint2D64f (&CrossPath)[200],double s,double &goal_dir);
	CvPoint2D64f MidGpsPoint_1[200],MidGpsPoint_2[202],MidGpsPoint_3[201],MidGpsPoint_4[200],MidGpsPoint_5[200],MidGpsPoint_6[200],MidGpsPoint_7[200];
	int panduan_1,panduan_2,panduan_11,panduan_22;
	bool fangxianggengxin1,fangxianggengxin2;
	CvPoint2D64f GetCrossPoint(CvPoint2D64f *Point,CvPoint2D64f (&WayPoint)[7]);
	CvPoint2D64f findUturnAimPoint(CvPoint2D64f m_gps,double dir,CvPoint2D64f (&CrossPath)[200],double s,double &goal_dir);
	CvPoint2D64f findUturnAimPointlukou(CvPoint2D64f m_gps,double dir,CvPoint2D64f (&CrossPath)[200],double s,double &goal_dir);
	CUTurn uturn;
public:
	~CPassIntersection(void);
};
