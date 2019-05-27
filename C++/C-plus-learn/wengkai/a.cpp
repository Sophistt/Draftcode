#include <iostream>

using namespace std;

class A
{
private:
    /* private 是针对类的，在同一个类里面的对象可以互相访问各自的 private 内的成员*/
    int i;
    int *ptr;
public:
    A() 
    { 
        ptr = 0;    // 指针 = 0，注意不是 *ptr = 0
        cout << "A::A()" << endl; 
    }
    
    ~A() 
    {   
        delete ptr;   // 删除一个空指针是是合理的操作，但是删除非空指针则会报错
        cout << "A::~A()" << i << endl;
    }
    
    void set(int ii) { i = ii;}
    void g(A* q) { cout << q->i << endl; }

};

int main(int argc, char const *argv[])
{
    /* 分清楚变量的作用域：通过申请动态内存得到的变量，
    除非调用 delete，否则作用域是整个程序运行期间。只有当程序退出后，内存才会被回收，所以不调用 delete的话，
    看不到析构函数的执行 */
    A *p = new A[10];  
    A b;    // 该变量不是通过申请动态内存得到的，因此在退出 main 函数之前会自动调用析构函数

    b.set(100);
    p[0].g(&b);    // 调用对象 p[0]的函数去访问 对象 b 的 private 成员是合法的，因为他们在同一个类里

    for(int i=0; i<10; i++)
    {
        p[i].set(i);
    }
    
    delete[] p;
    return 0;
}