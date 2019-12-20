#!/home/wcy/software/miniconda3/envs/py3.6/bin/python3
# -*- coding: utf-8 -*-

"""
* @Copyright (c)  all right reserved
* 
* @file:data.py
* @author: Sophistt
* @date:2019-12-20 10:13
* @description: Python file
"""

import os
import numpy as np
import torch.nn as nn

from torch.utils.data import Dataset, DataLoader


class CustomDataset(Dataset):
    def __init__(self, txt):
        self.data_list = self.read_file(txt)
        self.len = len(self.data_list)

    
    def __getitem__(self, index):
        data, label = self.data_list[index]

        data = np.array(data, dtype=float)
        label = np.array(label, dtype=float)

        return data, label


    def __len__(self):
        return len(self.data_list)

    
    def read_file(self, filename):
        
        data_list = []

        with open(filename, 'r') as fh:
            lines = fh.readlines()
            for line in lines:
                contents= line.rstrip().split()
                
                data = [float(i) for i in contents[:-1]] 
                label = float(contents[-1])

                data_list.append((data, label))
    
        return data_list


def main():
    
    train_dataset = CustomDataset('./train.txt')
    eval_dataset = CustomDataset('./eval.txt')

    train_loader = DataLoader(dataset=train_dataset, batch_size=12, shuffle=False)

    for step, (batch_data, batch_label) in enumerate(train_loader):
        print("step:{}, batch_data.shape:{}, batch_label:{}".format(step, batch_data.shape,
            batch_label))


if __name__ == '__main__':
    main()
