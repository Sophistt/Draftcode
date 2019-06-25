#include <iostream>
#include "parent.h"

using namespace std;

int main(int argc, char** argv)
{
    Parent parent;

    std::cout << parent.a << std::endl;

    parent.son_ptr->hello();

    parent.a = 20;
    
    parent.son_ptr->hello();
    
    return 0;
}
