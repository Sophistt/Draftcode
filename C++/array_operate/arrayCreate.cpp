#include <iostream>

using namespace std;

typedef struct CvPoint2D64f {
    double x;
    double y;
} Point;


int main(int argc, char** argv) {
    // static array (Created in stack)
    int array[10] = {0};

    cout << sizeof(array) << endl;

    // dynamic array (Created in heap)
    int * array_ptr = new int [20];
    
    cout << sizeof(array_ptr) << endl;  // The length of pointer is 8.

    delete[] array_ptr;
}
