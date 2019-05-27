/*
创建对指针的引用
*/
#include <iostream>

using namespace std;

typedef char const* PTR_CHAR;      // 对指针引用的语言比较混乱，用 typeset 可以更清楚

void initialize(char const **ptrPtr)    // 声明了一个 指向指针的指针 作为形参
{   
    *ptrPtr = "C-style pointer-to-pointer";    // 意义为；将字符串内容给到指针（对指针的指针取值即为指针）
}

void initialize(char const *&refPtr)    // 声明了一个针对指针的引用作为形参
{
    refPtr = "C++ reference-to-pointer";    // 将字符串内容给到指针
}

void initialize_typedef(PTR_CHAR &refPtr)
{
    refPtr = "C++ reference-to-pointer";
}

int main(int argc, char const *argv[])
{
    char const *ptr;
    PTR_CHAR ptr_typedef;
    initialize(&ptr);   // C sytle pointer-to-pointer
    cout << ptr << endl;
    initialize(ptr);
    initialize_typedef(ptr_typedef);
    cout << ptr << endl; // C++ reference-to-pointer
    cout << ptr_typedef << endl;
    return 0;
}
