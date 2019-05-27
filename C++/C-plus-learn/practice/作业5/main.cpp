#include "Screen.h"
#include "MyRectangle.h"
#include "MyCircle.h"

int main(int argc, char const *argv[])
{
	/* code */
	int width, height;
	int leftX, leftY, rightX, rightY;

	std::cin >> width >> height;
	std::cin >> leftX >> leftY >> rightX >> rightY;

	int CenterX, CenterY, radius;
	std::cin >> CenterX >> CenterY >> radius;

	Screen* screen = Screen::getInstance(width, height);

	MyRectangle myrectangle(leftX, leftY, rightX, rightY, screen);
	myrectangle.setColor(0, 0, 0xff);
	myrectangle.Draw();

	MyCircle myCircles[2] = { MyCircle(CenterX, CenterY, radius, screen) };

	(myCircles + 1)->setCenter(CenterX + 10, CenterY + 20);
	myCircles[1].setRadius(radius + 30);
	(*(myCircles +1)).setColor(0x00, 0x00, 0x00);
	myCircles[1].setScreen(*screen);
	
	for(int i = 0; i <= 1; i++) {
		myCircles[i].showScreen();
		(myCircles + i)->Draw();
	}

	MyCircle yourCircle(myCircles[1]);
	yourCircle.showScreen();
	(&yourCircle)->Draw();

	screen->deleteInstance();

	#ifdef DEBUG
		std::cin.get();
	#endif

	return 0;
}
