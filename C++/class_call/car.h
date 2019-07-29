#ifndef CAR_H
#define CAR_H

#include <iostream>
#include "string.h"

using std::cout;
using std::endl;
using std::string;

class Person;

class Car {

public:
    Car();
    Car(string name, double prize);
    ~Car();

private:
    string name;
    double prize;
    Person * owner;

public:
    void setOwner(Person * owner);
    Person * getOwner();
    
    void getInfo();
    
};

#endif
