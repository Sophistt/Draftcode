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
from network import Net
from params import parser
from data import CustomDataset
from tool import Visualization

def train(args, model, device, train_loader, optimizer, epoch):

    model.train()  # Set the model in train mode.

    for batch_idx, (data, target) in enumerate(train_loader):
        data, target = data.to(device), target.to(device)
        optimizer.zero_grad()
        output = model(data)
        loss = F.mse_loss(output, target.view_as(output))
        loss.backward()
        optimizer.step()

        # Print info
        if batch_idx % args.log_interval == 0:
            print('Train Epoch: {} [{}/{} ({:.0f}%)]\tLoss: {:.6f}'.format(
                epoch, batch_idx * len(data), len(train_loader.dataset),
                100. * batch_idx / len(train_loader), loss.item()))


def evaluate(args, model, device, eval_loader):

    model.eval()  # Set the model in evaluate mode.
    eval_loss = 0
    correct = 0

    with torch.no_grad():  # In this block, gradients will not be calculated
        for data, target in eval_loader:
            data, target = data.to(device), target.to(device)
            output = model(data)
            # Sum up batch loss
            eval_loss += F.mse_loss(output, target.view_as(output), reduction='sum').item()

    # Print info
    eval_loss /= len(eval_loader.dataset)

    return eval_loss


def calculate_R(args, model, device, eval_loader):
    model.eval()

    err = 0
    orr = 0

    with torch.no_grad():
        for data, target in eval_loader:
            data, target = data.to(device), target.to(device)
            output = model(data)
            
            pred = output.cpu().numpy()
            truth = target.cpu().numpy()

        for i in range(len(pred)):
            err += (pred[i][0] - truth[i]) * (pred[i][0] - truth[i])
            orr += truth[i] * truth[i]

        return 1 - (err / orr)

             
def main():

    # Training Setting
    args = parser.parse_args()

    use_cuda = not args.no_cuda and torch.cuda.is_available()

    device = torch.device('cuda' if use_cuda else 'cpu')

    kwargs = {'num_workers': 1, 'pin_memory': True} if use_cuda else {}

    # Visualize the value of loss function in the train process
    visual = Visualization() if args.visualization else None
    
    # Define dataset
    train_dataset = CustomDataset('./train.txt')
    eval_dataset = CustomDataset('./eval.txt')
    
    # Define dataloader
    train_loader = torch.utils.data.DataLoader(dataset=train_dataset,
                                               batch_size=args.batch_size,
                                               shuffle=True,
                                               **kwargs)
    
    eval_loader = torch.utils.data.DataLoader(dataset=eval_dataset,
                                              batch_size=args.eval_batch_size,
                                              shuffle=False,
                                              **kwargs) 

    # Define nerual network model
    model = Net().to(device)

    # Set Optimizier to calculate gradients
    optimizer = optim.Adadelta(model.parameters(), lr=args.lr)
    
    # Scheduler of decreasing learning rate each epoch
    scheduler = StepLR(optimizer, step_size=1, gamma=args.gamma)

    # Train neural network
    for epoch in range(1, args.epochs + 1):
        train(args, model, device, train_loader, optimizer, epoch)
        eval_loss = evaluate(args, model, device, eval_loader)
        scheduler.step()
        
        print("R2: ", calculate_R(args, model, device, eval_loader))

        # Update image
        if args.visualization:
            visual.data_update(epoch, eval_loss)
            visual.render()

    # Save model
    if args.save_model:
        torch.save(model.state_dict(), "model_record.pt")

    # Keep showing picture after training model
    if args.visualization:
        visual.terminate()


if __name__ == '__main__':
    main()
