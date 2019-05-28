#pragma once
#include "structhead.h"
#include <vector>
#include "ObPoint.h"
class ObCluster
{
private:
	CvPoint2D64f m_point[2000];
	CvPoint2D64f m_center;
	int m_size;
	int m_flag;   //=0表示大小符合要求，=1表示大于阈值
public:
	void SetPoint(CvPoint2D64f *points);
	void SetPoint(vector <ObPoint> ObPoints);
	void SetCenter(CvPoint2D64f center);
	void SetSize(int size);
	void SetFlag(int flag);
	int xmin;
	int xmax;
	int ymin;
	int ymax;

	void GetPoint(CvPoint2D64f (&point)[2000]);
	CvPoint2D64f GetCenter();
	int GetSize();
	int GetFlag();
	double velocity;
	double direction;
	ObCluster(void);
	~ObCluster(void);
};
