#pragma once
#include "vehicle.h"
#define TT 3 //换道时间
#define VEL 8
class CChangeLane :
	public CVehicle
{
public:
	CChangeLane(void);
	bool ChangeLeft_RightLane(CvPoint2D64f cur_point[],int left_right);//1表示左边换道，-1表示右边换道l
	void JudgeLeftOrRight();
	void CChangeLane::ChangeLaneDirve();
	bool ChangeLeftLanecos(I_Map *map,CvPoint2D64f rposition,double rdirection,int ob_x,int ob_y,int left_right);//1表示左边换道，-1表示右边换道
	int  GetCurLinenum(I_Map *map);
	bool SearchObstacleMoveWay(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down);
	bool SearchObstacleObstacle(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down);
	bool SearchObstacleObstacle(CvPoint2D64f GPSpoint[],I_Map *map,int up,int down,int &ob_x,int ob_y);
	double CChangeLane::SearchChangeLaneDist(int left_right,int ob_x,int ob_y);

	int FindXfromPath(CvPoint2D64f *map_path, int path_num,int y, int &id);
	int Findedge(I_Map *map, int ob_x, int ob_y, int step, CvPoint2D64f *map_pointzone, int &n);//step上下扩展行数,n边缘障碍个数
	double DistancePointToPath(CvPoint2D64f point, CvPoint2D64f *path, int path_num);
	double DistancePointToPoint(CvPoint2D64f point1, CvPoint2D64f point2);
	int FindMinPointID(double *min_dist, int k,int &id, double &mindist);//从一组距离内找到最小的距离和它是第几个
	void FindObDisAngle(CvPoint2D64f *midgps,CvPoint2D64f m_gps,double &obdis,double &ob_ang);
	//int MoveLeft(CvPoint2D64f WayPoint[],CvPoint2D64f LeftWayPoint[],double length);
	//int MoveRight(CvPoint2D64f WayPoint[],CvPoint2D64f RightWayPoint[],double length);
private:
	CvSeq* left_road;
	CvPoint2D64f *left_temp;
	
	int cl_time;
	int num;
public:
	~CChangeLane(void);
};
