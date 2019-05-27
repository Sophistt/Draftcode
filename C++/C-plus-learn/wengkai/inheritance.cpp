/* 继承：通过将父类的所有 成员变量/函数 全部获取，再加入自己
的 成员变量/函数 ，扩展为一个新的类
   与 composition 面向对象不同， inheritance 是面向类的
   子类一定包含了父类的所有 成员变量/函数，并且可以访问父类的 protected 
属性的 成员变量/函数。
*/

#include <iostream>

using namespace std;

/* 创建父类 class A，包含了：
   public 类型的： 构造与析构函数， print 函数， addself 函数作为 interface
   protected 类型的： set 函数供自己及子类调用
   private 类型的： 成员变量 i 只允许自己调用*/
 class A
 {
 public:
    // A 初始化的时候需要传入参数
     A(int init_i):i(init_i) { cout << "A::A()" << endl; }
     ~A() { cout << "A::~A()" << endl; }
     void print() { cout << "Print Function -- i = " << i << endl; }
     void addself() { i += 1; }
 protected:
     void set(int ii) { i = ii; }
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
    // 由于父类 class A 初始化需要参数，所以在 class B 的 initialize list 里要初始化 A
    B(int init_i) : A(init_i) { cout << "B::B()" << endl; }
    ~B() { cout << "B::~B()" << endl; }
    void change_value(int value) 
    { 
        if(value <= 10) set(value);
        else  cout << "vaule is out of 10, change value failed" << endl; 
    }
 };

 int main(int argc, char const *argv[])
 {
     B b(15);
     b.addself();
     b.print();
     b.change_value(9);
     b.print();
     
     return 0;
 }