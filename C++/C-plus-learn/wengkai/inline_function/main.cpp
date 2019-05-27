/* Include 了内联函数的 .h 文件之后，直接调用内联函数即可，
   调用内联函数与普通函数没有区别，区别只在定义内联函数时只需要在 .h 文件里面声明*/
#include "inline.h"

int main(int argc, char const *argv[])
{
    // inline_function(10, 10);
    Rectangle rect1(5, 10, 12, 18);
    rect1.print_area();
    
    return 0;
}