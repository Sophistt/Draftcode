#include <iostream>
#include <cstring>

 using namespace std;

class B
{
private:
    string name;
public:
    // 在析构函数后面采用 :成员变量名(值) 的方式来初始化变量，而不是在析构函数里赋值
    B(string s):name(s) 
    {
        cout << name << endl;
        cout << "construct funciton..." << endl;
    }
};

int main()
{
    B b("initialize");

    return 0;
}