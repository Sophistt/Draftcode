#include "MyCircle.h"

MyCircle::MyCircle(const MyCircle& mycircle) {
    x = mycircle.x;
    y = mycircle.y;
    radius = mycircle.radius;
    red = mycircle.red;
    green = mycircle.green;
    blue = mycircle.blue;
    screen_ = mycircle.screen_;

    std::cout << "copy mycircle" << std::endl;
}

void MyCircle::setColor(int R, int G, int B) {
    red = R;
    green = G;
    blue = B;
}

void MyCircle::setRadius(int r) {
    radius = r;
}

void MyCircle::setCenter(int x, int y) {
    this->x = x;
    this->y = y;
}

void MyCircle::Draw() {
    std::cout << x << ' ' << y << ' ' << radius << ' ' << std::endl;
    std::cout << red << ' ' << green << ' ' <<  blue << std::endl;
}

void MyCircle::showScreen() {
    std::cout << screen_->getWidth() << ' ' << screen_->getHeight() << std::endl;
}