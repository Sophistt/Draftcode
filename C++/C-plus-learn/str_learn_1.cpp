/*
创建并存储字符串的时候有两种方法，分别是字符串数组与字符串常量
两者的不同之处在于，字符串数组是读写模式，字符串常量是只读模式
*/
#include <iostream>
#include "string.h"

using namespace std;

int main(int argc, char const *argv[])
{
    char const *str = "this is used for testing";
    char str1[] =  "this is used for testing";
    int len = strlen(str);
    
    for(int i=0;i<len;i++)
    {
        *(str1 + i) = '1';  
        // *(str + i) = '1';   可以修改 str1 ，但是修改 str 的时候就会报错： assignment in read-only character
        cout << *(str + i ) << endl;
    }
    cout << str1 << endl;
    return 0;
}