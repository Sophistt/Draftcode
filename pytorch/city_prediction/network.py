#!/home/wcy/software/miniconda3/envs/py3.6/bin/python3
# -*- coding: utf-8 -*-

"""
* @Copyright (c)  all right reserved
* 
* @file:network.py
* @author: Sophistt
* @date:2019-12-13 09:19
* @description: Python file
"""

import torch.nn as nn
import torch.nn.functional as F


class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        # Define neural network
        # self.conv1 = nn.Conv2d(1, 32, 3, 1)  For example
        self.fc1 = nn.Linear(5, 10).double()
        self.fc2 = nn.Linear(10, 1).double()


    # Define the computation performed at every call
    # Should be overridden by all subclasses
    def forward(self, x):
        
        x = self.fc1(x)
        x = F.relu(x)
        output = self.fc2(x)
        
        return output 
