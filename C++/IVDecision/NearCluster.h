#pragma once
#include "structhead.h "
#include <vector>
#include "ObPoint.h"

typedef struct ObstaclePoint
{
	int isob;                             //是否是障碍物
	int isreached;                        //是否分类
	int clusterId;                        //所属障碍物类别

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

class NearCluster
{
public:
	NearCluster(void);
	vector<ObPoint> ObPoints;            //存储分类好的障碍物点
	void Ini(I_Map *map);                //初始化函数，读入障碍物地图并处理
public:
	~NearCluster(void);
};
