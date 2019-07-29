
#include "person.h"

Person::Person() {
    this->name = "";
    this->age = 0;
};

Person::Person(string name, int age) {
    this->name = name;
    this->age = age;
}

Person::~Person() {}

void Person::setCar(Car * car) {
    this->car = car;
}

Car * Person::getCar() {
    return this->car;
}

void Person::getInfo() {
    cout << "Name: " << this->name << " Age: " << this->age << endl;
}
