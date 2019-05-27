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
	void setColor(int R, int G, int B);
	void Draw();
	void showScreen();
private:
	int x1, x2, y1, y2;
	int red, green, blue;
	Screen* screen_;
};

inline MyRectangle::MyRectangle():x1(10), y1(10), x2(100), y2(100), red(255), green(255), blue(255) {
	std::cout << "myrectangle" << std::endl;
}

inline MyRectangle::MyRectangle(int x1, int y1, int x2, int y2, Screen* screen):
					x1(x1), y1(y1), x2(x2), y2(y2), screen_(screen), red(255), green(255), blue(255) {
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

inline void MyRectangle::setColor(int R, int G, int B) {
	red = R;
	green = G;
	blue = B;
}

#endif 