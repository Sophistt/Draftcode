#ifndef MYCIRCLE_H
#define MYCIRCLE_H

#include <iostream>
#include "Screen.h"
#include "MyShape.h"

class MyCircle: public MyShape
{
public:
	MyCircle();
	MyCircle(int x, int y, int radius, Screen* screen);
    // MyCircle(MyCircle& mycircle);

    void setRadius(int r);
    void setCenter(int x, int y);
    void Draw();
private:
	int x, y, radius;
};

inline MyCircle::MyCircle(): x(200), y(200), radius(100), MyShape("mycircle") { }

inline MyCircle::MyCircle(int x, int y, int radius, Screen* screen):
						  x(x), y(y), radius(radius), MyShape(screen, "mycircle") { }

#endif
