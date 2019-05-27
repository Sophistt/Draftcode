#ifndef MYRECTANGLE_H
#define MYRECTANGLE_H

#include <iostream>
#include "Screen.h"
#include "MyShape.h"

class MyRectangle: public MyShape
{
public:
	MyRectangle();
	MyRectangle(int x1, int y1, int x2, int y2, Screen* screen);
	void setCoordinations(int x1, int y1, int x2, int y2);
	void Draw();
private:
	int x1, x2, y1, y2;
};

inline MyRectangle::MyRectangle():x1(10), y1(10), x2(100), y2(100), MyShape("myrectangle") { }

inline MyRectangle::MyRectangle(int x1, int y1, int x2, int y2, Screen* screen):
					x1(x1), y1(y1), x2(x2), y2(y2), MyShape(screen, "myrectangle") { }

inline void MyRectangle::setCoordinations(int x1, int y1, int x2, int y2) {
	this->x1 = x1;
	this->x2 = x2;
	this->y1 = y1;
	this->y2 = y2;
}

#endif 