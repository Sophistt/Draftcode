#include "Screen.h"
#include <cstdlib>

Screen::setWidth(int width){
	this->width = width;
	return width;
}

Screen::setHeight(int height){
	this->height = height;
	return height;
}

void Screen::exitWhenInvalidScreen(int width, int height){
	if( width <= 0 || width > 1000 || height <=0 || height > 1000 ){
		std::cout << "invalid screen size";
		exit(0);
	}
}