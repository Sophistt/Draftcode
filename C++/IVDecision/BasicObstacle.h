#pragma once

class BasicObstacle
{
private:
	
	CvPoint2D64f indiCluster[1000];
	CvPoint2D64f indiCenter;
	int indiSize;

public:
	void SetIndiCluster(CvPoint2D64f *pos);
	void SetIndiCenter(CvPoint2D64f center);
	void SetIndiSize(int size);

	void GetIndiCluster(CvPoint2D64f (&newpos)[1000]);
	CvPoint2D64f GetIndiCenter();
	int GetIndiSize();
	BasicObstacle(void);
public:
	~BasicObstacle(void);
};
