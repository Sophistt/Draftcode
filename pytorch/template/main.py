#!/home/wcy/software/miniconda3/envs/py3.6/bin/python3
# -*- coding: utf-8 -*-

"""
* @Copyright (c)  all right reserved
* 
* @file:main.py
* @author: Sophistt
* @date:2019-12-13 09:12
* @description: Python file
"""

import torch
import torch.nn.functional as F
import torch.optim as optim

from torch.optim.lr_scheduler import StepLR
from params import parser
from network import Net
from data import CustomDataset


def train(args, model, device, train_loader, optimizer, epoch):

    model.train()  # Set the model in train mode.

    for batch_idx, (data, target) in enumerate(train_loader):
        data, target = data.to(device), target.to(device)
        optimizer.zero_grad()
        output = model(data)
        loss = F.nll_loss(output, target.view_as(output))
        loss.backward()
        optimizer.step()

        # Print info
        # if batch_idx % args.log_interval == 0:
        #     print('Train Epoch: {} [{}/{} ({:.0f}%)]\tLoss: {:.6f}'.format(
        #         epoch, batch_idx * len(data), len(train_loader.dataset),
        #         100. * batch_idx / len(train_loader), loss.item()))


def evaluate(args, model, device, eval_loader):

    model.eval()  # Set the model in evaluate mode.
    eval_loss = 0
    correct = 0

    with torch.no_grad():  # In this block, gradients will not be calculated
        for data, target in eval_loader:
            data, target = data.to(device), target.to(device)
            output = model(data)
            # Sum up batch loss
            eval_loss += F.nll_loss(output, target.view_as(output), reduction='sum').item()
            # Get the index of the max log-probability (Just for classification)
            pred = output.argmax(dim=1, keepdim=True)
            correct += pred.eq(target.view_as(pred)).sum().item()

    return eval_loss
    # Print info
    # eval_loss /= len(eval_loader.dataset)
    # print('\nEval set: Average loss: {:.4f}, Accuracy: {}/{} ({:.0f}%)\n'.format(
    #    eval_loss, correct, len(eval_loader.dataset),
    #    100. * correct / len(eval_loader.dataset)))


def main():

    # Training Setting
    args = parser.parse_args()

    use_cuda = not args.no_cuda and torch.cuda.is_available()

    device = torch.device('cuda' if use_cuda else 'cpu')

    kwargs = {'num_workers': 1, 'pin_memory': True} if use_cuda else {}

    if args.visualization:
        from tool import Visualization
        visualization = Visualization()
    
    # TODO  Define your own train and evaluate dataset
    train_dataset = CustomDataset()
    eval_dataset = CustomDataset()

    # Define train and evaluation dataloader
    train_loader = torch.utils.data.DataLoader(train_dataset, 
        batch_size=args.batch_size,shuffle=True, **kwargs)
    eval_loader = torch.utils.data.DataLoader(eval_dataset, 
        batch_size=args.eval_batch_size, shuffle=False, **kwargs) 

    # Define nerual network model and Optimizer 
    model = Net().to(device)
    optimizer = optim.Adadelta(model.parameters(), lr=args.lr)
    
    # Scheduler of decreasing learning rate each epoch
    scheduler = StepLR(optimizer, step_size=1, gamma=args.gamma)

    # Train neural network
    for epoch in range(1, args.epoch + 1):
        train(args, model, device, train_loader, optimizer, epoch)
        eval_loss = evaluate(args, model, device, eval_loader)
        scheduler.step()

        # Picture updated
        if args.visualization:
            visualization.data_update(epoch, eval_loss)

    # Save model
    if args.save_model:
        torch.save(model.state_dict(), "model_record.pt")

    # Keep showing picture after training model
    if args.visualization:
        visualization.terminate()


if __name__ == '__main__':
    main()
