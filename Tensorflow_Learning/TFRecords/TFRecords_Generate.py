"""
多文件夹与图片生成TFRecords文件
存储格式：总文件夹下的子文件夹按图片分类命名，子文件夹下是分类好的图片
生成方法：通过os模块获取文件路径名，通过cv2.imread函数读取图片，通过tf函数转换格式并储存
"""

from random import shuffle
import numpy as np
import glob
import tensorflow as tf
import cv2
import sys
import os

# 是否打乱顺序（True打乱，False不打乱）
shuffle_image = True
# 定义图片的最上级目录路径名
image_path_up = 'C:/Users/Administrator/Downloads/notMNIST_small/notMNIST_small/'
# 定义TFRecords生成路径名
train_filename = 'D:/test1/train.tfrecords'


# 定义读取图片的函数
def load_image(addr):
    img = cv2.imread(addr)
    return img


# 定义打乱数据的函数
def shuffle_data(img_addr, label):
    collection = list(zip(img_addr, label))
    shuffle(collection)
    img_addr, label = zip(*collection)

    return img_addr, label


# 将数据转换成对应的属性，在后面需要用到
def _int64_feature(value):
    return tf.train.Feature(int64_list=tf.train.Int64List(value=[value]))


def _bytes_feature(value):
    return tf.train.Feature(bytes_list=tf.train.BytesList(value=[value]))


def _float_feature(value):
    return tf.train.Feature(float_list=tf.train.FloatList(value=[value]))


# 创建一个writer来写 TFRecords 文件
writer = tf.python_io.TFRecordWriter(train_filename)
# 获取主目录下各子文件夹名称
up_addrs = os.listdir(image_path_up)

# 用大循环遍历主目录下所有文件夹
for i in range(len(up_addrs)):
    # 更改子文件夹路径名
    image_path = image_path_up + up_addrs[i] + '/'
    image_addrs = os.listdir(image_path)
    # 进行路径名补全和定义标签名
    labels = []
    img_addrs = []
    for label in range(len(image_addrs)):
        img_addrs.append(image_path + image_addrs[label])
        labels.append(i + 1)
    # 打乱数据
    if shuffle_image:
        img_addrs, labels = shuffle_data(img_addrs, labels)
    # 大循环内嵌套小循环将image与label写入
    for i in range(len(img_addrs)):
        # 这是写入操作可视化处理
        if not i % 1000:
            print('Train data: {}/{}'.format(i, len(img_addrs)))
            sys.stdout.flush()

        # 加载图片
        img = load_image(img_addrs[i])
        # opencv在python3下面存在bug，可能会读取图片的时候返回None，因此这里做小处理
        if img is None:
            continue
        label = labels[i]

        # 用字典创建一个属性（feature）
        # img.tostring将int转换为string，然后通过tf.compat.as_bytes进行utf-8编码
        feature = {'train/label': _int64_feature(label),
                   'train/image': _bytes_feature(tf.compat.as_bytes(img.tostring()))}

        # 创建一个 example protocol buffer
        example = tf.train.Example(features=tf.train.Features(feature=feature))

        # 将上面的example protocol buffer写入文件
        writer.write(example.SerializeToString())

# TFRecords写完后，关闭writer
writer.close()
sys.stdout.flush()
