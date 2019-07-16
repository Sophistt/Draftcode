
#include "person.h"

Car::Car() {
    this->name = "";
    this->prize = 0;
};

Car::Car(string name, double prize) {
    this->name = name;
    this->prize = prize;
}

Car::~Car() {}

void Car::setOwner(Person * owner) {
    this->owner = owner;
}

Person * Car::getOwner() {
    return this->owner;
}

void Car::getInfo() {
    cout << "Name: " << this->name << " Prize: " << this->prize << endl;
}
