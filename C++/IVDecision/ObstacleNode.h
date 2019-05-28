#pragma once
#include "DyObstacle.h"
class ObstacleNode
{
private:
	DyObstacle obstacle;
	ObstacleNode *next;
	bool updateflag;
public:
	ObstacleNode(void);                         //��next��ʼ��Ϊnull��updateflag��ʼ��Ϊ1
	DyObstacle GetData();
	ObstacleNode *GetNext();
	bool GetUpdateFlag();
	void SetData(DyObstacle ob);
	void SetNext(ObstacleNode *ptr);
	void SetUpdateFlag(bool update);
	ObstacleNode &operator =(const ObstacleNode &D)
		{
			this->obstacle=D.obstacle;
			this->next=D.next;
			return *this;
		}	
public:
	~ObstacleNode(void);
};
