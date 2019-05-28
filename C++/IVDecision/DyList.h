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
	bool isEmpty();                                        //判断链表是否为空
	ObstacleNode *GetHead();                               //获取链表头
	ObstacleNode *GetTail();                               //获取链表尾
	int GetNum();
	ObstacleNode *ItemAt(int position);                    //获取第i个链表元素的地址
    ObstacleNode *Search(DyObstacle dyobstaclenode);       //查找链表元素
	void RemoveHead();                                     //移除表头
	void RemoveTail();                                     //移除表尾
	void Remove(int x);                                    //移除链表中第x个节点，num>x>=0
	void InsertHead(ObstacleNode obstaclenode);            //插入链表头
	void InsertTail(ObstacleNode obstaclenode);            //插入链表尾
	void Insert(ObstacleNode *p, int x);                   //将特定节点插入链表中的指定位置之后
	void UpDate(I_Map *map);                               //更新障碍物列表
	void UpDate2(I_Map *map);                              
	double ComPare(DyObstacle ob1,DyObstacle ob2);         //比较两个障碍物
	double V_Dir(DyObstacle ob1, DyObstacle ob2);          //求速度方向
	double CountV(DyObstacle ob1, DyObstacle ob2);         //计算速度
	double CountDir(DyObstacle ob1,DyObstacle ob2);
	int mapcount;                                          //程序运行后经历过的总帧数
	int dy_obstacle_num;                                   //历史上已经出现的障碍物数目

public:
	~DyList(void);
};
