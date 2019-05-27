#ifndef SCREEN_H
#define SCREEN_H

#include <iostream>
#include <string>

using std::string;

class Screen
{
public:
	int getWidth();
	int getHeight();

	static Screen* getInstance(int width = 640, int height = 480);
	static void deleteInstance();
private:
	int width;
	int height;
	string enter;
	string leave;

	static Screen* instance;
private:
	Screen(int w, int h);
	~Screen();

	void exitWhenInvalidScreen(int width, int height);
	
};

inline Screen::Screen(int w, int h):width(w), height(h), enter("enter screen"), leave("leave screen") {
	std::cout << enter << std::endl;
	exitWhenInvalidScreen(w, h);
}

inline Screen::~Screen() {
	std::cout << leave << std::endl;
}

inline Screen::getWidth(){
	return width;
}

inline Screen::getHeight() {
	return height;
}


#endif