#pragma once

class ObPoint
{
private:
	int x;
	int y;
	int clusterId;
public:
	ObPoint(void);
	void InPut(int x1,int y1, int clusterId1);
	int Getx();
	int Gety();
	int GetClusterId();
public:
	~ObPoint(void);
};
