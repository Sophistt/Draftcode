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


    # Define the computation performed at every call
    # Should be overridden by all subclasses
    def forward(self, x):
        
        output = F.log_softmax(x, dim=1) 
        
        return output 
