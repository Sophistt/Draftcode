#pragma once
#include "structhead.h"
#include <vector>
#include "ObPoint.h"
#include "ObCluster.h"
typedef struct ObstaclePoint
{
	int isob;                             //是否是障碍物
	int isreached;                        //是否分类
	int clusterId;                        //所属障碍物类别
};
typedef struct Pos
{
	int x;
	int y;
};
typedef struct Relate
{
	int x;
	int y;
};

typedef struct ObMap
{
	ObstaclePoint obstacle[512][512];     //自定义障碍物地图，可以方便的操作
};
class GrowCluster
{
private:
	int m_bf;
	int m_bh;
	int m_lt;
	int m_rt;
	int m_width,m_lengh;     
public:
	GrowCluster(void);
	void Init(I_Map *map);
	void Init(I_Map *map,ObjectWT *ibcpWT, int ib_num);
	void Init_history(I_Map *map,ObjectWT *ibcpWT, int ib_num);
    int ob_count;                        //障碍物点个数
	int cluster_num;
	ObMap *obmap;
	int xmin;
	int xmax;
	int ymin;
	int ymax;

	float v_x_clusterId[5000];
	float v_y_clusterId[5000];

	/*vector <ObPoint> ObPoints1;  */
    vector <ObPoint> ObPoints; 
	vector <ObCluster> obclusters;
	
	void Setdetect(double before,double behind, double left, double right);
	void Setgrow(double width, double length);
	void Clear();
public:
	~GrowCluster(void);
};
