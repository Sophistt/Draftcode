
#include <iostream>

using namespace std;

/* 创建父类 class A，包含了：
   public 类型的： 构造与析构函数， print 函数， addself 函数作为 interface
   protected 类型的： set 函数供自己及子类调用
   private 类型的： 成员变量 i 只允许自己调用*/
 class A
 {
 public:
     A():i(0) { cout << "A::A()" << endl; }
     ~A() { cout << "A::~A()" << endl; }
     /* 定义了不带参数的 print 函数和带参数的 print 函数，调用的时候通过
     是否带参数来自动调用不同的函数 */
     void print() { cout << "A::print()" << endl; }
     // 该函数里又调用了不带参数的 print 函数，这种用法被称为函数重载
     void print(int ii) 
     { 
        cout << ii << endl;
        print();
    }
 private:
     int i;
 };

/* 创建 class A 的子类 class B，定义了：
   public 类型的： change_value 函数，该函数调用了 class A 的 protected 函数
作为 interface 供外界调用
*/
 class B : public A 
 {
 public:
    B() { cout << "B::B()" << endl; }
    ~B() { cout << "B::~B()" << endl; }
    /* 在子类 class B 里面定义了与 class A 一摸一样的 print 函数，因此，
    这时的 cLass A 的 print 和 print(int ii) 函数都会被 hide name，默认调用 class B 
    的 print 函数*/
    void print() { cout << "B::print()" << endl; };
 };

 int main(int argc, char const *argv[])
 {
     B b;
     b.print();
     // 想调用 class A 的 print 函数，需要用下面这种方式调用
     b.A::print(200);
     
     return 0;
 }