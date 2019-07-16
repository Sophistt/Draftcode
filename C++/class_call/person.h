#ifndef PERSON_H
#define PERSON_H

#include <iostream>
#include "string.h"
#include "car.h"

using std::cout;
using std::endl;
using std::string;


class Person {

public:
    Person();
    Person(string name, int age);
    ~Person();

private:
    string name;
    int age;
    Car * car;

public:
    void setCar(Car * car);
    Car * getCar();

    void getInfo();
};

#endif
