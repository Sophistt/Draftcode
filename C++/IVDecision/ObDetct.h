#pragma once
#include "structhead.h"
#include <vector>
#include "ObPoint.h"
extern const int ob_flag=18;
/************************************************************************/
/* �����ⷶΧ                                                                     */
/************************************************************************/
extern const int o_left=160;
extern const int o_right=276;
extern const int o_front=200;
extern const int o_rear=450;
/************************************************************************/
/* ���嵥�����������Χ                                                                     */
/************************************************************************/
extern const int o_width=-3;
extern const int o_lenghth=4;
/************************************************************************/
/* ���¶���һ����ͼ��ÿ���㺬�ж������                                                                     */
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
/* ���������洢�������ݵĽṹ��                                                                     */
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
