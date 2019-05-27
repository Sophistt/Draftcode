#ifndef _PARENT_H_
#define _PARENT_H_

#include "son.h"

class Parent
{
public:
    Parent();
    ~Parent();

public:
    int a, b;
    Son *son_ptr;
};


#endif