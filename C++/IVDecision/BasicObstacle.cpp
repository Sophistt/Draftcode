#include "StdAfx.h"
#include "BasicObstacle.h"

BasicObstacle::BasicObstacle(void)
{
}

BasicObstacle::~BasicObstacle(void)
{
}

void BasicObstacle::SetIndiCluster(CvPoint2D64f *pos)
{
	for (int i = 0; i < 1000; i++)
	{
		indiCluster[i] = pos[i];
	}
}

void BasicObstacle::SetIndiCenter(CvPoint2D64f center)
{
	indiCenter = center;
}

void BasicObstacle::SetIndiSize(int size)
{
	indiSize = size;
}

void BasicObstacle::GetIndiCluster(CvPoint2D64f (&newpos)[1000])
{
	for (int i = 0; i<1000; i++)
	{
		newpos[i] = indiCluster[i];
	}
}