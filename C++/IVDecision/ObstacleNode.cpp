#include "StdAfx.h"
#include "ObstacleNode.h"

ObstacleNode::ObstacleNode(void)
{
	this->next=NULL;
	this->updateflag=1;
}

ObstacleNode::~ObstacleNode(void)
{
}
DyObstacle ObstacleNode::GetData()
{
	return this->obstacle; 
}
ObstacleNode *ObstacleNode::GetNext()
{
	return this->next;
}
bool ObstacleNode::GetUpdateFlag()
{
	return this->updateflag;
}

void ObstacleNode::SetData(DyObstacle ob)
{
	this->obstacle=ob;
}
void ObstacleNode::SetNext(ObstacleNode *ptr)
{
	this->next=ptr;
}
void ObstacleNode::SetUpdateFlag(bool update)
{
	this->updateflag=update;
}
