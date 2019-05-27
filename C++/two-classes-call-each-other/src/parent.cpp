#include "parent.h"

Parent::Parent() : a(10), b(11), son_ptr(new Son(this))
{}

Parent::~Parent()
{
    delete son_ptr;
}