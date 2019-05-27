#ifndef MYSHAPE_H
#define MYSHAPE_H

#include <iostream>
#include <string>
#include "Screen.h"

using std::string;

class MyShape
{
public:
    MyShape(string t);
    MyShape(Screen* screen, string t);
    
    void setColor(int R, int G, int B);
    void setScreen(Screen& screen);
    Screen* getScreen();
    int getRed();
    int getGreen();
    int getBlue();
    void showShape();
    
    virtual void Draw() = 0;

private:
    int red, green, blue;
    string type_;
    Screen* screen_;
};


inline void MyShape::setColor(int R, int G, int B) {
    red = R;
    green = G;
    blue = B;
}

inline void MyShape::setScreen(Screen& screen) {
    screen_ = &screen;
}

inline Screen* MyShape::getScreen() { return screen_; }


inline int MyShape::getRed() { return red; }
inline int MyShape::getGreen() { return green; }
inline int MyShape::getBlue() { return blue; }


#endif
