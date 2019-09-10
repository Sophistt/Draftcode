#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
* @Copyright (c) all right reserved 
* 
* @File    : state_object.py
* @Date    : 2019-09-10 22:20:13
* @Author  : Sophistt
* @Desc    : Python file
"""

class Connection:
    """
    Connection Class in state object programming

    """
    def __init__(self):
        self.new_state(ClosedConnectionState)
        self._data = 0

    def new_state(self, newstate):
        self._state = newstate

    def read(self):
        return self._state.read(self)

    def write(self, data):
        return self._state.write(self, data)

    def close(self):
        return self._state.close(self)

    def open(self):
        return self._state.open(self)
    
    def getData(self):
        return self._data

    def setData(self, data):
        self._data = data

class ConnectionState:
    """docstring for ClassName"""
    @staticmethod
    def read(conn):
        raise NotImplementedError()

    @staticmethod
    def write(conn, data):
        raise NotImplementedError()

    @staticmethod
    def close(conn):
        raise NotImplementedError()

    @staticmethod
    def open(conn):
        raise NotImplementedError()


class ClosedConnectionState(ConnectionState):
    @staticmethod
    def read(conn):
        raise RuntimeError('Not open')

    @staticmethod
    def write(conn, data):
        raise RuntimeError('Not open')

    @staticmethod
    def close(conn):
        raise RuntimeError('Already closed')

    @staticmethod
    def open(conn):
        conn.new_state(OpenConnectionState)


class OpenConnectionState(ConnectionState):
    @staticmethod
    def read(conn):
        print(conn.getData())

    @staticmethod
    def write(conn, data):
        conn.setData(data)

    @staticmethod
    def close(conn):
        conn.new_state(ClosedConnectionState)

    @staticmethod
    def open(conn):
        raise RuntimeError('Already open')


def main():
    c = Connection()
    c.open()
    c.write('192.168.x.x')
    c.read()
    c.close()
    c.read()    

if __name__ == '__main__':
    main()
                


        
