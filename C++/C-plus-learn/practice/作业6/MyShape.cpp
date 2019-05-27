#include "MyShape.h"

using std::string;

MyShape::MyShape(string t): red(255), green(255), blue(255), type_(t) { };

MyShape::MyShape(Screen* screen, string t):
                 red(255), green(255), blue(255), screen_(screen), type_(t) { };


void MyShape::showShape() {
    std::cout << '[' << screen_->getWidth() << 'X' << screen_->getHeight() << ']' <<
                type_ << '(' << red << ',' << green << ',' << blue << ')' << std::endl;
}
