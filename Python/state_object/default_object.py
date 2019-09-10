#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
* @Copyright (c) all right reserved 
* 
* @File    : default_object.py
* @Date    : 2019-09-10 22:08:59
* @Author  : Sophistt
* @Desc    : Python file
"""

class Connection:
    """
    Defualt Connection Class

    Advantage: Simple codes while small program
    Disadvantage: low efficiency (too many if--else), hard to deal with multiple states (4 or more)
    """
    def __init__(self):
        self._state = 'CLOSED'

    def read(self):
        if self._state != 'OPEN':
            raise RuntimeError('Not open')
        print('reading')

    def write(self):
        if self._state != 'OPEN':
            raise RuntimeError('Not open')
        print('writing')

    def close(self):
        if self._state == 'CLOSED':
            raise RuntimeError('Already closed')
        self._state = 'CLOSED'

    def open(self):
        if self._state == 'OPEN':
            raise RuntimeError('Already open')
        self._state = 'OPEN'
            
        