#!/home/wcy/software/miniconda3/envs/py3.6/bin/python3
# -*- coding: utf-8 -*-

"""
* @Copyright (c)  all right reserved
* 
* @file:character.py
* @author: Sophistt
* @date:2019-09-12 14:33
* @description: Python file
"""

class Character:
    """
    Main Class of Character. All member variables should be stored here.
    """
    def __init__(self):
        self.new_state(HealthState)

    def new_state(self, state):
        self._state = state

    def work(self):
        return self._state.work(self)

    def sleep(self):
        return self._state.sleep(self)

    def eat(self):
        return self._state.eat(self)


class CharacterState:
    """
    Class of CharacterState
    """
    @staticmethod
    def work(cha):
        raise NotImplementedError()

    @staticmethod
    def sleep(cha):
        raise NotImplementedError()

    @staticmethod
    def eat(cha):
        raise NotImplementedError()


class HealthState(CharacterState):
    @staticmethod
    def work(cha):
        print('Character finishs work in 5 hours and get tired.')
        cha.new_state(TiredState)

    @staticmethod
    def sleep(cha):
        print('Character does not need to sleep!')

    def eat(cha):
        print('Character eats food and get excited.')
        cha.new_state(ExcitedState)


class TiredState(CharacterState):
    @staticmethod
    def work(cha):
        print('Character feels too tired to work.')

    @staticmethod
    def sleep(cha):
        cha.new_state(HealthState)
        print('After sleep, character become healthy again.')
    
    @staticmethod
    def eat(cha):
        print('Character eats food and get excited.')
        cha.new_state(ExcitedState)


class ExcitedState(CharacterState):
    @staticmethod
    def work(cha):
        print('Character finishs work in 1 hours and get tired.')
        cha.new_state(TiredState)

    @staticmethod
    def sleep(cha):
        print('Character feels too excited to sleep.')
    
    @staticmethod
    def eat(cha):
        print('Character can not eat any more.')


def main():
    cha = Character()

    cha.work()
    cha.eat()
    cha.eat()
    cha.sleep()
    cha.work()
    cha.sleep()


if __name__ == '__main__':
    main()
