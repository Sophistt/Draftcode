#include "Screen.h"
#include <cstdlib>

Screen* Screen::instance = 0;

void Screen::exitWhenInvalidScreen(int width, int height){
	if( width <= 0 || width > 1000 || height <=0 || height > 1000 ){
		std::cout << "invalid screen size";
		exit(0);
	}
}

Screen* Screen::getInstance(int width, int height){
	if (Screen::instance == 0){
		Screen* p = new Screen(width, height);
		Screen::instance = p;
		
		return Screen::instance;
	}
	else{
		return Screen::instance;
	}
}

void Screen::deleteInstance() {
	if (Screen::instance != 0) {
		delete Screen::instance;
		Screen::instance = 0;
	}
}