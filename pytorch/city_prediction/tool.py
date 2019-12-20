#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Date    : 2019-12-20 19:56:36
# @Author  : Your Name (you@example.org)
# @Link    : http://example.org
# @Version : $Id$

import os
import matplotlib.pyplot as plt

class Visualization:
    "Notice that matplotlib wastes numerous computation resources, hence tensorboard is recommended."
    def __init__(self):
        plt.ion()
        plt.figure(1, dpi=80)

        self.epoch_list = []
        self.loss_list = []

    def data_update(self, epoch, loss):
        self.epoch_list.append(epoch)
        self.loss_list.append(loss)

    def render(self):
        plt.clf()
        plt.plot(self.epoch_list, self.loss_list, 'g')
        plt.pause(0.01)

    def terminate(self):
        plt.ioff()
        plt.show()