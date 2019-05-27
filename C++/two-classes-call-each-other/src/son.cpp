#include <iostream>
#include "parent.h"

Son::Son(Parent* ptr) : parent_ptr(ptr)
{}

void Son::hello()
{
    std::cout << "I am son class and I use parent variables. cout parent.a: " << parent_ptr->a << std::endl;
}

Son::~Son()
{

}