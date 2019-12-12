# -*- coding: utf-8 -*-
"""
Created on Mon Jun 12 14:53:38 2017

@author: 123
"""

import tensorflow as tf

#定义函数get_weight()
"""
获取一层神经网络边上的权重，并将这个权重的L2正则化损失加入名称为"losses"的集合中
"""
def get_weight(shape,lamda):
    #生成一个变量
    var = tf.Variable(tf.random_normal(shape),dtype=tf.float32)
    #add_to_collection函数将这个变量的L2正则化损失项加入集合
    tf.add_to_collection(
            'losses',tf.contrib.layers.l2_regularizer(lamda)(var))
    #返回生成的变量
    return var
x = tf.placeholder(tf.float32,shape=(None,2))
y_ = tf.placeholder(tf.float32,shape=(None,1))

batch_size = 8
#定义每一层网络中节点的个数,这里调用了一个列表
layer_dimension = [2,10,10,10,1]
#神经网络的层数,这里的长度是是5，即5层（包括输入输出层）
n_layers = len(layer_dimension)

#这个变量为前向传播时最深层的节点，开始的时候就是输入层
cur_layer = x
#当前层的节点个数
in_dimension = layer_dimension[0]

#通过一个循环来生成5层全连接的神经网络结构
for i in range(1,n_layers):
    #layer_dimension[i]为下一层的节点个数
    out_dimension = layer_dimension[i]
    #生成当前层中权重的变量，并将这个变量的L2正则化损失加入计算图上的集合
    weight = get_weight([in_dimension,out_dimension],0.001)
    bias = tf.Variable(tf.constant(0.1,shape=[out_dimension]))
    #使用ReLU激活函数
    cur_layer = tf.nn.relu(tf.matmul(cur_layer,weight) + bias)
    #进入下一层之前将下一层的节点个数更新为当前层的节点个数
    in_dimension = layer_dimension[i]
    
#在定义神经网络前线长波的同时已经将所有的L2正则化损失加入了图上的集合，这里只需要计算
#刻画模型在训练数据上表现的损失函数
mse_loss = tf.reduce_mean(tf.square(y_ - cur_layer))

#将均方差函数加入损失集合
tf.add_to_collection('losses',mse_loss)

#get_collection返回一个列表，这个列表是所有这个集合中的元素。
#这个样例中，元素就是损失函数的不同部分，全加起来就是损失函数
loss = tf.add_n(tf.get_collection('losses'))