#include "MyCircle.h"

/*
MyCircle::MyCircle(MyCircle& mycircle):MyShape(mycircle.getScreen(), "mycircle") {
    x = mycircle.x;
    y = mycircle.y;
    radius = mycircle.radius;
}

void MyCircle::setRadius(int r) {
    radius = r;
}
*/

void MyCircle::setCenter(int x, int y) {
    this->x = x;
    this->y = y;
}

void MyCircle::Draw() {
    showShape();
    std::cout << x << ' ' << y << ' ' << radius << ' ' << std::endl;
}
