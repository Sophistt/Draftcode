
#include <iostream>
#include "string.h"

using namespace std;

class Person {

public:
	// Only available after c++ 11
	Person() : Person("None", 0) {};

	Person(string name, int age) {
		this->setName(name);
		this->setAge(age);
	}
	
	~Person() {};

private:
	static string country;
	string name;
	int age;

public:
	// Setter
	void setName(string name) {
		this->name = name;
	}

	void setAge(int age) {
		this->age = age;
	}

	// Getter
	string getName() {
		return this->name;
	}

	int getAge() {
		return this->age;
	}

	static void setCountry(string c) {
		country = c;
	}

	static string getCountry() {
		return country;
	}
};

// static member initialization 
string Person::country = "CN";

int main(int argv, char ** args) {
	
	Person *per = new Person();
	
	cout << per->getCountry() << endl;

	cout << per->getName() << endl;

	delete per;
	return 0;
}
