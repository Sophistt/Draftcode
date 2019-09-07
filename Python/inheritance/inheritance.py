#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
* @Copyright (c) all right reserved 
* 
* @File    : inheritance.py
* @Date    : 2019-09-07 21:40:46
* @Author  : Sophistt
* @Desc    : Python file
"""

import os


class Person(object):
    """
    Person Class

    :param name: (String) 
    :param age:  (int)
    """

    def __init__(self, name, age):
        super(Person, self).__init__()  # Make codes compatible with python 2 and 3
        self.name = name
        self.age = age

    def getInfo(self):
        return self.name, self.age


class Student(Person):
    """
    Student class extends Person

    :param school: (String)
    """

    def __init__(self, name, age, school):
        super().__init__(name, age)
        self.school = school

    def getInfo(self):
        return self.name, self.age, self.school


def main():
    stu = Student('Jack', 18, 'MIT')
    print(stu.getInfo())


if __name__ == '__main__':
    main()
