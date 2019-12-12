#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
* @Copyright (c) all right reserved 
* 
* @File    : np_tricks.py
* @Date    : 2019-09-28 11:47:02
* @Author  : Sophistt
* @Desc    : Python file
"""

import numpy as np


def delete(array, index, axis):
    """
    Delete the specified row or column in ndarray.

    :return: ndarray
    """
    assert len(array.shape) == 2
    return np.delete(array, index, axis)


def is_unique(array, axis):
    """
    Determine whether each element is unique in ndarray.

    :return: bool
    """
    assert type(array) == np.ndarray
    return array.shape[axis] == np.unique(array, axis=axis).shape[axis]


def main():
    array = np.array([[0., 1.],
                      [1., 1.],
                      [2., 1.],
                      [3., 1.],
                      [4., 1.],
                      [5., 1.],
                      [6., 1.],
                      [7., 1.],
                      [8., 1.],
                      [5., 1.],
                      [9., 1.]])

    print(is_unique(array, 1))
    print(delete(array, 1, axis=1))


if __name__ == '__main__':
    main()
