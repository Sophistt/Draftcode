#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
* @Copyright (c) all right reserved 
* 
* @File    : main.py
* @Date    : 2019-08-30 20:33:23
* @Author  : Sophistt
* @Desc    : Python file
"""

from common import variables
import func


def main():
    print(variables.acceleration_x)
    func.change_acceleration()
    print(variables.acceleration_x)
    func.output_params()
    return


if __name__ == '__main__':
    main()
