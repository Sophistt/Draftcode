/*
创建字符串数组 str1 与 str2，将 str1 复制到 str2
*/
#include <iostream>

using namespace std;

int main( )
{
    char str1[]="I love CHINA!",str2[20],*p1,*p2;
    p1=str1;    // p1 指向 str1的头地址
    cout<<"p1 is: "<<p1<<endl;  
    cout<<"*p1 is: "<<*p1<<endl;
    p2=str2;    // p2 指向 str2的头地址

    for(;*p1!='\0';p1++,p2++)   // 当 *p1 != '\0', 便一直循环，理解为一直循环直到 p1 == &str1[-1]
    {   
        cout << *p1 << endl;
        *p2=*p1;    // 将 p1 指向的内存的值赋予 p2 指向的内存
    }
    
    *p2='\0';   // 最后在 p2 指向的内存的地址加入 '\0'
    /*由于此时 p1 = &str1[-1], p2 = &str2[-1], 重新将 p1 指向 str1, p2 指向 str2*/
    p1 = str1;
    p2 = str2;

    cout<<"str1 is: "<<p1<<endl;
    cout<<"str2 is: "<<p2<<endl;
    /*
    最后输出：
    str1 is: I love CHINA!
    str2 is: I love CHINA!
    可以看到 str2 已经从未初始化的字符串数组变为 str1 的内容
    */
    return 0;
}