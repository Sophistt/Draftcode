#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Date    : 2019-12-20 23:51:38
# @Author  : Sophistt 
# @Link    : https://Sophistt.github.io
# @Version : 1.0

import os
import numpy as np

from torch.utils.data import Dataset

class CustomDataset(Dataset):
    def __init__(self):
        # TODO Initialize file path or list of file names
        # self.label_list = []
        # self.data_list = []

    def __getitem__(self, index):
        # TODO  Overwrite this method to return a pair of sample (data and label).
        # 1. Read data from file
        # 2. Process data into numpy array (e.g. torchvision.Transform)
        # 3. return a data pair
        return None

    def __len__(self):
        # TODO  This method should return the length of the dataset.
        return 0