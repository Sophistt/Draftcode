#ifndef SCREEN_H
#define SCREEN_H

#include <iostream>

class Screen
{
public:
	Screen();
	Screen(int w, int h);
	
	int getWidth();
	int getHeight();
	int setWidth(int width);
	int setHeight(int height);
private:
	int width;
	int height;

	void exitWhenInvalidScreen(int width, int height);
	
};

inline Screen::Screen():width(640), height(480) {
	std::cout << "screen" << std::endl;
}

inline Screen::Screen(int w, int h):width(w), height(h) {
	exitWhenInvalidScreen(w, h);
	std::cout << "screen" << std::endl;
}

inline Screen::getWidth(){
	return width;
}

inline Screen::getHeight() {
	return height;
}


#endif