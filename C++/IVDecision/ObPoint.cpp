#include "StdAfx.h"
#include "ObPoint.h"

ObPoint::ObPoint(void)
{
}

ObPoint::~ObPoint(void)
{
}

int ObPoint::Getx()
{
	return this->x;
}

int ObPoint::Gety()
{
	return this->y;
}

int ObPoint::GetClusterId()
{
	return this->clusterId;
}
void ObPoint::InPut(int x1,int y1, int clusterId1)
{
	x=x1;
	y=y1;
	clusterId=clusterId1;
}
