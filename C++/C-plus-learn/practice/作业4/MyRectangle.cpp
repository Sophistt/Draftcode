#include "MyRectangle.h"

void MyRectangle::Draw() {
	int w_temp, h_temp;

	w_temp = x2 - x1;
	h_temp = y2 - y1;

	if (x1 == 0 && y1 == 0){
		if (x2 > screen_->getWidth() || y2 > screen_->getHeight() || (x2 == screen_->getWidth() && y2 == screen_->getHeight()) ||
		   (x2 == 0 || y2 == 0 )){
			std::cout << "invalid myrectangle" << std::endl;
		}
		else{
			std::cout << x1 << ' ' << y1 << ' ' << w_temp << ' ' << h_temp << std::endl;
		}
	}
	else{
		if (x1 >= screen_->getWidth() || y1 >= screen_->getHeight()){
			std::cout << "invalid myrectangle" << std::endl;
		}
		else{
			int rest_width = screen_->getWidth() - x1;
			int rest_height = screen_->getHeight() - y1;

			if (w_temp > rest_width || h_temp > rest_height){
				std::cout << "invalid myrectangle" << std::endl;		
			}
			else{
				std::cout << x1 << ' ' << y1 << ' ' << w_temp << ' ' << h_temp << std::endl;					
			}
		}
	}
}