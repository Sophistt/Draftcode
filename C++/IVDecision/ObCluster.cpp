#include "StdAfx.h"
#include "ObCluster.h"

ObCluster::ObCluster(void)
{
	m_flag = 0;
	for (int i = 0; i<2000; i++)
	{
		m_point[i].x = 0;
		m_point[i].y = 0;
	}
	m_size = 0;
	m_center.x = 0;
	m_center.y = 0;
}

ObCluster::~ObCluster(void)
{
}

void ObCluster::SetPoint(CvPoint2D64f *points)
{
	for (int i = 0; i < 2000; i++)
	{
		ASSERT(i <= 2000&&i >= 0);
		m_point[i] = points[i];
	}
}

void ObCluster::SetPoint( vector <ObPoint> ObPoints )
{
	m_size=ObPoints.size();
	int x_avg=0;
	int y_avg=0;
	for (int i=0; i<m_size; i++)
	{
		m_point[i].x=ObPoints[i].Getx();
		m_point[i].y=ObPoints[i].Gety();
		x_avg+=ObPoints[i].Getx();
		y_avg+=ObPoints[i].Gety();
	}
	m_center.x=x_avg/m_size;
	m_center.y=y_avg/m_size;

}

void ObCluster::SetCenter(CvPoint2D64f center)
{
	m_center = center;
}

void ObCluster::SetSize(int size)
{
	m_size = size;
}

void ObCluster::SetFlag(int flag)
{
	m_flag = flag;
}

void ObCluster::GetPoint(CvPoint2D64f (&point)[2000])
{
	for (int i = 0; i < 2000; i++)
	{
		ASSERT(i >= 0&&i <=2000);
		point[i] = m_point[i];
	}
}

CvPoint2D64f ObCluster::GetCenter()
{
	return m_center;
}

int ObCluster::GetSize()
{
	return m_size;
}

int ObCluster::GetFlag()
{
	return m_flag;
}
