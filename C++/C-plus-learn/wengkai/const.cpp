/* 理解 const 的含义，定义了 const 之后，即相当于告诉编译器这个东西不能再修改
*/
#include <iostream>

using namespace std;

class A{
	int i;
public:
	A():i(10) {};
	f() { cout << "f()" << endl; }; 
};

void f1(const int *x){
	/* 函数 f1 里面有个参数是 const *p 型的，就是证明在 f1 里面，不会对 *p 进行修改
	   但是在传参数进来的时候，无论是 &(int) 还是 &(const int)都可以传进来
	*/

	/* 这样做的好处，当定义了一个很大的对象的时候，如果直接将 对象 传到函数里面，
	   会存在很多开销(变量值的复制)，因此通过传指针可以节省开销，但是如果传了指针，
	   就容易出现对象的变量被修改，因此在传参的时候加入 const 可以防止这类事情发生
	*/
}

int main(int argc, char const *argv[])
{
	/* char * const 的命名方式代表了这个指针是 const，也就是说不能修改这个指针
	所指向的内存的地址，但是可以改变该内存地址的值
	*/
	char * const p1 = "abc";
	*p1 = 'c';  // ok

	/* const char * 的命名方式代表了通过该指针指向的内存地址是 const，
	也就是说不能通过该指针去修改内存里面的内容，但是指针本身可以修改，内存也可以通过别的指针修改
	*/
	const char *p2 = "abcd";
	p2++;  // ok

	int i;
	const int const_i = 3;
	int *ip;
	const int *const_ip;

	ip = &i; // ok
	const_ip = &i; // ok

	i = 50; // ok,because i is int
	*ip = 52 ;// ok
	// *const_ip = 50  error, because *const_ip is const

	// ip = &const_i  error，const_i 是一个 const 变量，不能将其地址赋给普通的变量
	const_ip = &const_i;

	/* 当定义了一个 const 对象的时候，该对象的所有变量都应该用 initialize_list 进行初始化
	   这是因为由于对象是 const 那么里面所有成员都是只读的，如果一开始没有赋值，那么后面
	   将无法再次赋值
	*/
	const A const_a;   

	return 0;
}

