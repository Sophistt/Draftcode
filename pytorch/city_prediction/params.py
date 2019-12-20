#!/home/wcy/software/miniconda3/envs/py3.6/bin/python3
# -*- coding: utf-8 -*-

"""
* @Copyright (c)  all right reserved
* 
* @file:params.py
* @author: Sophistt
* @date:2019-12-13 10:58
* @description: Python file
"""

import argparse

parser = argparse.ArgumentParser(description='Parameters of neural network')

parser.add_argument('--batch-size', type=int, default=12, metavar='N',
                    help='input batch size for training (default: 64)')

parser.add_argument('--eval-batch-size', type=int, default=24, metavar='N',
                    help='input batch size for evaluating (default: 1000)')

parser.add_argument('--epochs', type=int, default=800, metavar='N',
                    help='number of epochs to train (default: 14)')

parser.add_argument('--lr', type=float, default=0.1, metavar='LR',
                    help='learning rate (default: 1.0)')

parser.add_argument('--gamma', type=float, default=0.9995, metavar='M',
                    help='Learning rate step gamma (default: 0.7)')

parser.add_argument('--no-cuda', action='store_true', default=True,
                    help='disables CUDA traing')

parser.add_argument('--seed', type=int, default=1, metavar='S',
                    help='random seed (default: 1)')

parser.add_argument('--log-interval', type=int, default=1, metavar='N',
                    help='how many batches to wait before logging training status')

parser.add_argument('--save-model', action='store_true', default=False,
                    help='For saving the current mdel')

parser.add_argument('--visualization', action='store_true', default=True,
                    help='For visualize the traning process.')
