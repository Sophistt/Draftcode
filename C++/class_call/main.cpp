
#include "car.h"
#include "person.h"

int main(int argc, char** argv) {
    
    Car * car = new Car("Benz", 350000.00);
    car->getInfo();
    Person * person = new Person("Jack", 25);
    person->getInfo();

    car->setOwner(person);
    person->setCar(car);

    person->getCar()->getInfo();

    delete person;
    delete car;

}
