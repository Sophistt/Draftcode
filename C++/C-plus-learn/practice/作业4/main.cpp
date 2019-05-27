#include "Screen.h"
#include "MyRectangle.h"

int main(int argc, char const *argv[])
{
	/* code */
	int width, height;
	std::cin >> width >> height;

	Screen screen(width, height);

	int leftX, leftY, rightX, rightY;
	std::cin >> leftX >>  leftY >>  rightX >> rightY;

	MyRectangle myRectangle1(leftX, leftY, rightX, rightY, &screen);
	MyRectangle* myRectangles = new MyRectangle[2];
	
	myRectangles[1].setCoordinations(10, 300, 700, 500);
	myRectangles[1].setScreen(screen);

	myRectangle1.Draw();
	for (int i = 0; i < 2; i++) {
		myRectangles[i].setScreen(screen);
		(myRectangles + i)->Draw();
	}

	delete[] myRectangles;

	#ifdef DEBUG
		std::cin.get();
	#endif

	return 0;
}
