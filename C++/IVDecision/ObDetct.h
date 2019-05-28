#pragma once
#include "structhead.h"
#include <vector>
#include "ObPoint.h"
extern const int ob_flag=18;
/************************************************************************/
/* 定义检测范围                                                                     */
/************************************************************************/
extern const int o_left=160;
extern const int o_right=276;
extern const int o_front=200;
extern const int o_rear=450;
/************************************************************************/
/* 定义单个点的生长范围                                                                     */
/************************************************************************/
extern const int o_width=-3;
extern const int o_lenghth=4;
/************************************************************************/
/* 重新定义一个地图，每个点含有多个性质                                                                     */
/************************************************************************/
typedef struct ObstaclePoint
{
	int isob;
	int clusterId;
};
typedef struct ObMap
{
	ObstaclePoint obstacle[512][512];
};
/************************************************************************/
/* 定义用来存储坐标数据的结构体                                                                     */
/************************************************************************/
typedef struct Pos
{
	int x;
	int y;
};
class ObDetct
{
public:
	ObDetct(void);
	vector <ObPoint> ObPoints;
	int point_count;
	int ob_count;
	void Init(I_Map *map);
public:
	~ObDetct(void);
};
