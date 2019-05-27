#ifndef MYRECTANGLE_H
#define MYRECTANGLE_H

#include <iostream>
#include "Screen.h"

class MyRectangle
{
public:
	MyRectangle();
	MyRectangle(int x1, int y1, int x2, int y2, Screen* screen);

	void setCoordinations(int x1, int y1, int x2, int y2);
	void setScreen(Screen& screen);
	void Draw();
private:
	int x1, x2, y1, y2;
	Screen* screen_;
};

inline MyRectangle::MyRectangle():x1(0), y1(0), x2(0), y2(0) {
	std::cout << "myrectangle" << std::endl;
}

inline MyRectangle::MyRectangle(int x1, int y1, int x2, int y2, Screen* screen):
					x1(x1), y1(y1), x2(x2), y2(y2), screen_(screen) {
	std::cout << "myrectangle" << std::endl;	
}

inline void MyRectangle::setCoordinations(int x1, int y1, int x2, int y2) {
	this->x1 = x1;
	this->x2 = x2;
	this->y1 = y1;
	this->y2 = y2;
}

inline void MyRectangle::setScreen(Screen& screen) {
	screen_ = &screen;
}


#endif 