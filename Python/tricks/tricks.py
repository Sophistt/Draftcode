#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
* @Copyright (c) all right reserved 
* 
* @File    : tricks.py
* @Date    : 2019-09-27 22:07:04
* @Author  : Sophistt
* @Desc    : Python file
"""

import os
import numpy as np


def swap(a, b):
    '''
    Swap a and b
    '''
    return b, a


def transpose(lst):
    """
    Rearrange the dimension of the list 
    """
    return list(zip(*lst))


def is_unique(lst):
    """
    Determine wheter each element in list is unique

    :return: bool
    """
    assert type(lst) == list 
    return len(lst) == len(set(lst))


def element_add_num(lst, num):
    """
    Add the same number to each element in a list.

    :return: list
    """
    return [lst + num for l in lst]


def main():
    #l1 = [1, 2, 3, 4, 5, 5]
    l1 = [[1.0, 2.0], [3.0, 4.0], [1.0, 2.0]]
    print(element_add_num(l1, 5))

if __name__ == '__main__':
    main()
