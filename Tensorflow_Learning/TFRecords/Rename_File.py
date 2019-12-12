
"""
批量重命名文件
os.listdir列出文件夹下所有目录与文件名
os.rename 将文件重命名为所给名字，所给名字必须为string格式，带后缀名
"""

import os


#文件的上级路径
image_path_up = 'C:/Users/Administrator/Downloads/notMNIST_small/notMNIST_small/A/'

addrs = os.listdir(image_path_up)

#用for循环将文件夹里面所有文件重命名
for index in range(len(addrs)):
    item = addrs[index]
    item = image_path_up + item
    new_name = image_path_up + str(index + 1) + '.png'
    os.rename(item, new_name)