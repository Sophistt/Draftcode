import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
import os
import cv2

# 定义读取TFRecords的路径
data_path = 'D:/test1/train.tfrecords'


# 定义读取和解码TFRecords文件的函数
def read_and_decode(filename):
    # 根据文件名生成一个队列,这里的shuffle是读取多个file的时候，对file进行乱序处理
    filename_queue = tf.train.string_input_producer([filename], shuffle=True)
    # 定义一个TFRecords文件读取器
    reader = tf.TFRecordReader()
    # reader.read传入参数为刚刚生成的队列，返回参数为key和value的元组（即label和image）
    _, serialized_example = reader.read(filename_queue)  # 返回文件名和文件

    # tf.parse_single_example函数一个一个地解析刚刚的一系列的文件（定义的features要和存的feature保持一致）
    features = tf.parse_single_example(serialized_example,
                                       features={
                                           'train/label': tf.FixedLenFeature([], tf.int64),
                                           'train/image': tf.FixedLenFeature([], tf.string),
                                       })
    # 将读到的image解码为uint8格式（注意uint8和int8的区别）
    img = tf.decode_raw(features['train/image'], tf.uint8)
    img = tf.reshape(img, [28, 28, 3])
    # img = tf.cast(img, tf.float32) * (1. / 255) - 0.5

    # tf.cast是转换tensor数据格式的函数，将int64转换为int32
    label = tf.cast(features['train/label'], tf.int32)

    return img, label


img, label = read_and_decode(data_path)

# 读取出来的img与label先不直接进入网络，进入缓存中用batch的形式随机排序再进入网络
# img_batch_shape = [batch_size, 28, 28, 3]
img_batch, label_batch = tf.train.shuffle_batch([img, label],
                                                batch_size=30, capacity=15000,
                                                min_after_dequeue=1000)
tf.summary.image('input', img_batch, 10)
merged = tf.summary.merge_all()

init = tf.initialize_all_variables()

with tf.Session() as sess:
    writer = tf.summary.FileWriter('logs', sess.graph)

    sess.run(init)
    threads = tf.train.start_queue_runners(sess=sess)
    for i in range(1500):
        summary, val, l = sess.run([merged, img, label])
        # val, l = sess.run([img, label])
        # 我们也可以根据需要对val， l进行处理
        # l = to_categorical(l, 12)
        writer.add_summary(summary, i)

        print(val, l)
writer.close()
