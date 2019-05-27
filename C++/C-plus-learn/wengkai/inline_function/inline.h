/* 普通的函数应该是在 .h 文件里面放原型，在 .cpp 文件里面放 body
   但是 inline function(内联函数)不同，在函数前面加了 inline之后，
   将 body 也一起放到头文件里面去，因此不需要存在 .cpp 文件 */
#include <iostream>

using namespace std;

inline void inline_function(int i , int j)
{
    cout << i << " " << j << endl; 
}

class Rectangle
{
private:
    int x_min, y_min, width, height;
public:
    Rectangle(int x, int y , int w, int h);
    ~Rectangle();
    void set_localization(int x, int y);
    void set_line_length(int w, int h);
    void print_perimeter();
    void print_area();
};

inline Rectangle::Rectangle(int x, int y, int w, int h):x_min(x), y_min(y), width(w), height(h)
{
    std::cout << "Rectangle construct function..." << std::endl;
}

inline void Rectangle::set_localization(int x, int y)
{
    x_min = x;
    y_min = y;
}

inline void Rectangle::set_line_length(int w, int h)
{
    width = w;
    height = h;
}

inline Rectangle::~Rectangle(){
    std::cout << "Rectangle deconstruct function..." << std::endl;
}

