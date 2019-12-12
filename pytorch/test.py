#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Date    : 2019-12-11 21:50:22
# @Author  : Your Name (you@example.org)
# @Link    : http://example.org
# @Version : $Id$

from __future__ import print_function
import argparse
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import numpy as np
from torchvision import datasets, transforms
from torch.optim.lr_scheduler import StepLR


class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.conv1 = nn.Conv2d(1, 32, 3, 1)
        self.conv2 = nn.Conv2d(32, 64, 3, 1)
        self.dropout1 = nn.Dropout2d(0.25)
        self.dropout2 = nn.Dropout2d(0.5)
        self.fc1 = nn.Linear(9216, 128)
        self.fc2 = nn.Linear(128, 10)

    # Deifne the computation performed at every call. 
    # Should be overridden by all subclasses.
    def forward(self, x):
        x = self.conv1(x)
        x = F.relu(x)
        x = self.conv2(x)
        x = F.max_pool2d(x, 2)
        x = self.dropout1(x)
        x = torch.flatten(x, 1)
        x = self.fc1(x)
        x = F.relu(x)
        x = self.dropout2(x)
        x = self.fc2(x)
        output = F.log_softmax(x, dim=1)
        return output

def main():

    device = torch.device("cuda")

    model = Net()

    # Print parameters of the NN model
    for param in model.parameters():
        print(type(param.data), param.size())
    
    # Define Input
    input = torch.randn(1, 1, 28, 28)
    
    output = model(input)

    # Define target
    target = torch.randn(10)
    target = target.view(1, -1)  # Use view() to reshape tensor

    loss = F.mse_loss(output, target)

    # propagate backward the gradients
    loss.backward()

    # Update weights
    optimizer = optim.SGD(model.parameters(), lr=0.01)

    print("output: ", output)
    output.backward(torch.randn(1, 10))
    print(input.grad)
    
    # Gradients will be calculated and recorded automatically
    x = torch.ones(2, 2, requires_grad=True)
    print(x)
    y = x + 2
    print(y)
    z = y * y * 3
    out = z.mean()
    print(z, out)

    out.backward()
    print(x.grad)

    
    return

if __name__ == '__main__':
    main()
