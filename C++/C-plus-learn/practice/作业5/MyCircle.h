#ifndef MYCIRCLE_H
#define MYCIRCLE_H

#include <iostream>
#include "Screen.h"

class MyCircle
{
public:
	MyCircle();
	MyCircle(int x, int y, int radius, Screen* screen);
    MyCircle(const MyCircle& mycircle);

    void setScreen(Screen& screen);
	void setColor(int R, int G, int B);
    void setRadius(int r);
    void setCenter(int x, int y);
    void Draw();
    void showScreen();
private:
	int x, y, radius;
	int red, green, blue;
	Screen* screen_;	
};

inline MyCircle::MyCircle(): x(200), y(200), radius(100), red(255), green(255), blue(255) {
    std::cout << "mycircle" << std::endl;
}

inline MyCircle::MyCircle(int x, int y, int radius, Screen* screen):
						  x(x), y(y), radius(radius), screen_(screen), red(255), green(255), blue(255) {
    std::cout << "mycircle" << std::endl;
}

inline void MyCircle::setScreen(Screen& screen) {
    screen_ = &screen;
}

#endif