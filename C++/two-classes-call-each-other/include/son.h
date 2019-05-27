#ifndef _SON_H_
#define _SON_H_ 

class Parent;

class Son
{
public:
    Son(Parent* ptr);
    ~Son();

    void hello();

public:
    Parent *parent_ptr;
};

#endif