#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
* @Copyright (c) all right reserved 
* 
* @File    : func.py
* @Date    : 2019-08-30 20:41:20
* @Author  : Sophistt
* @Desc    : Python file
"""

from common import variables
from common.params import *


def change_acceleration():
    variables.acceleration_x = 10.0
    variables.acceleration_y = 5.0
    variables.acceleration_z = 0.0


def output_params():
    print(GRIVITY)
