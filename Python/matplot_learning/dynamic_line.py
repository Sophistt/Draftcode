#!/home/wcy/software/miniconda3/envs/py3.6/bin/python3
# -*- coding: utf-8 -*-

""" 
* @Copyright (c)  all right reserved 
* 
* @file:dynamic_line.py 
* @author: Sophistt 
* @date:2019-07-29 15:55 
* @description: Python file 
"""

import numpy as np

import matplotlib.pyplot as plt


def main():
    # Activate interacitve mode
    plt.ion()

    # Set only one figure
    plt.figure(1, dpi=80)

    for index in range(100):
        # Clear axes object
        plt.cla()
        # Set title of the picture
        plt.title("Animation Curve")
        # Show grid
        plt.grid(True)

        # Create data
        x = np.linspace(-np.pi + 0.1 * index, np.pi +
                        0.1 * index, 256, endpoint=True)
        y_cos, y_sin = np.cos(x), np.sin(x)
        
        # Set label of X axis
        plt.xlabel("X-axis")
        # Set minimum and maximum value of X axis
        plt.xlim(-4 + 0.1 * index, 4 + 0.1 * index)
        # Set scale interval of X axis
        plt.xticks(np.linspace(-4 + 0.1*index, 4+0.1*index, 9, endpoint=True))
        
        # Set lable of Y axis
        plt.ylabel("Y-axis")
        # Set minumum and maximum of Y axis
        plt.ylim(-1.0, 1.0)
        # Set scale interval of Y axis 
        plt.yticks(np.linspace(-1, 1, 9, endpoint=True))
        
        # Display cos and sin lines
        plt.plot(x, y_cos, "b--", linewidth=2.0, label="cosLine")
        plt.plot(x, y_sin, "g-", linewidth=2.0, label="sinLine")
        
        # Set location of the floating window 
        plt.legend(loc="upper left", shadow=True)
        
        # Render rate
        plt.pause(0.01)

    # Deactivate interactive mode
    plt.ioff()

    # Keep showing picture after updating data
    plt.show()


if __name__ == "__main__":
    main()
