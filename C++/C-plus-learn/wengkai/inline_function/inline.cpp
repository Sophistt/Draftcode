#include "inline.h"
#include <iostream>

using namespace std;

void Rectangle::print_perimeter()
{
    cout << "perimeter is " << (width + height) * 2 << endl;
}

void Rectangle::print_area()
{
    cout << "area is " << width * height << endl;
}