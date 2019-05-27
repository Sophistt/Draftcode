#include "MyRectangle.h"

void MyRectangle::Draw() {
	int w_temp, h_temp;

	w_temp = x2 - x1;
	h_temp = y2 - y1;

    showShape();
	std::cout << x1 << ' ' << y1 << ' ' << w_temp << ' ' << h_temp << std::endl;			

}
