#include "MyRectangle.h"

void MyRectangle::Draw() {
	int w_temp, h_temp;

	w_temp = x2 - x1;
	h_temp = y2 - y1;

	std::cout << x1 << ' ' << y1 << ' ' << w_temp << ' ' << h_temp << std::endl;
	std::cout << red << ' '  << green << ' ' <<  blue << std::endl;					

}

void MyRectangle::showScreen() {
	std::cout << screen_->getWidth() << ' ' << screen_->getHeight() << std::endl;
}