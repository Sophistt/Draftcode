/* 通过 composition 的方式将两个或者多个 对象 合并成为一个新的 对象，
初始化 成员变量 的时候，通过 initialize list 来初始化而不是在构造函数里面赋值 */
#include <iostream>
#include <cstring>

 using namespace std;

/* class 1，该 class 内有两个常量字符指针成员变量和构造函数，
通过 initialize list 来初始化成员变量 */
 class Person  
 {
 private:
    const char *_name;
    const char *_address;
 public:
     Person(const char *name, const char *address):_name(name), _address(address)
     {
        cout << _name << endl;
        cout << _address << endl;
        cout << "Person constructor" << endl;
     }
 };


/* class 2，该 class 内有一个整型成员变量和构造函数，
通过 initialize list 来初始化成员变量 */
 class Currency
 {
 private:
    int _cent;
 public:
     Currency(int cent):_cent(cent)
     {
        cout << _cent << endl;
        cout << "Currency constructor" << endl;
     }
 };

/* class 3，该 class 包含了 class1 和 class2 作为成员变量，
通过 initialize list 来初始化成员变量，初始化的过程中其实是分别调用了 class1 和 class2 的构造函数*/
 class SavingAccount
 {
 private:
    Person m_saver;
    Currency m_balance;
 public:
     SavingAccount(const char *name, const char *address, int cents):
     m_saver(name , address), m_balance(cents)
     {
        cout << "SavingAccount constructor..." << endl;
     }

     void print()
     {
        cout << "Print function..." << endl;
     }
     
 };

int main(int argc, char const *argv[])
{
    // 创建了一个 s_account 的 object，该 object 里面由 m_saver, m_balance 两个对象
    SavingAccount s_account("wangcanye", "huazhou", 100);

    return 0;
}