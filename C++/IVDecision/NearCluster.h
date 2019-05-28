#pragma once
#include "structhead.h "
#include <vector>
#include "ObPoint.h"

typedef struct ObstaclePoint
{
	int isob;                             //�Ƿ����ϰ���
	int isreached;                        //�Ƿ����
	int clusterId;                        //�����ϰ������

};
typedef struct Relate
{
	int x;
	int y;
};

typedef struct ObMap
{
	ObstaclePoint obstacle[512][512];     //�Զ����ϰ����ͼ�����Է���Ĳ���
};

class NearCluster
{
public:
	NearCluster(void);
	vector<ObPoint> ObPoints;            //�洢����õ��ϰ����
	void Ini(I_Map *map);                //��ʼ�������������ϰ����ͼ������
public:
	~NearCluster(void);
};
