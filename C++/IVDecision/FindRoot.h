#pragma once 
#include <math.h>
//template <class T>
class FindRoot
{
private:
	double a,b,c;
public:
	double dt,x1,x2,rp,ip;
public:
	FindRoot(double x,double y,double z);
	void Find();
	//void Display();

};

FindRoot::FindRoot(double x,double y,double z)
{
	a = x;
	b = y;
	c = z;
	dt = b*b-4*a*c;
}

void FindRoot::Find()
{
	if(dt > 0)
	{
		x1 = (-b + sqrt(dt))/(2*a);
		x2 = (-b - sqrt(dt))/(2*a);
	}
	else if(0==dt)
	{
		x1 = x2 =(-b)/(2*a);
	}
	else 
	{
		 rp = (-b)/(2*a);
		 ip = sqrt(-dt)/(2*a);

	}
}
