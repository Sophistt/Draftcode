#pragma once
#include "ObstacleNode.h"
#include <vector>
#include "structhead.h"
#include "ObPoint.h"
#include "GrowCluster.h"
#include <math.h>
#include "vehicle.h"
#include "GetGPSData.h"
#include "ComuLcm.h"

struct Diff
{
	double d;
	bool visited;
};
class DyList:public CVehicle
{
private:
	ObstacleNode *head;
	ObstacleNode *tail;
	int num;
public:
	DyList(void);
	bool isEmpty();                                        //�ж������Ƿ�Ϊ��
	ObstacleNode *GetHead();                               //��ȡ����ͷ
	ObstacleNode *GetTail();                               //��ȡ����β
	int GetNum();
	ObstacleNode *ItemAt(int position);                    //��ȡ��i������Ԫ�صĵ�ַ
    ObstacleNode *Search(DyObstacle dyobstaclenode);       //��������Ԫ��
	void RemoveHead();                                     //�Ƴ���ͷ
	void RemoveTail();                                     //�Ƴ���β
	void Remove(int x);                                    //�Ƴ������е�x���ڵ㣬num>x>=0
	void InsertHead(ObstacleNode obstaclenode);            //��������ͷ
	void InsertTail(ObstacleNode obstaclenode);            //��������β
	void Insert(ObstacleNode *p, int x);                   //���ض��ڵ���������е�ָ��λ��֮��
	void UpDate(I_Map *map);                               //�����ϰ����б�
	void UpDate2(I_Map *map);                              
	double ComPare(DyObstacle ob1,DyObstacle ob2);         //�Ƚ������ϰ���
	double V_Dir(DyObstacle ob1, DyObstacle ob2);          //���ٶȷ���
	double CountV(DyObstacle ob1, DyObstacle ob2);         //�����ٶ�
	double CountDir(DyObstacle ob1,DyObstacle ob2);
	int mapcount;                                          //�������к���������֡��
	int dy_obstacle_num;                                   //��ʷ���Ѿ����ֵ��ϰ�����Ŀ

public:
	~DyList(void);
};
