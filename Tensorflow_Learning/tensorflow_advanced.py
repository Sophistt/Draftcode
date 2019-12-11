# -*- coding: utf-8 -*-
"""
Created on Wed Jun  7 17:55:41 2017

@author: 123
"""
import tensorflow as tf

#学习率设置
"""
为了更好地提高神经网络的训练速度，可以在训练刚开始的时候采用大的学习率，随着训练的进行
学习率衰减。
Tensorflow提供了tf.train.exponential_decay函数实现指数衰减的学习率，实现以下功能：
decayed_learning_rate = learning_rate * decay_rate ^ (global_step / decay_step)
decayed_learning_rate:当前学习率
learning_rate : 初始学习率
decay_rate : 衰减系数
global_step : 全局步数
decay_step ： 衰减速度，一般设置为总样本数除以batch中的样本数
"""
global_step = tf.Variable()

#通过exponential_decay函数生成学习率
learning_rate = tf.train.exponential_decay(
        0.1, global_step, 100, 0.96 ,staircase=True)

#使用指数帅衰减的学习率，在minimize函数中传入global_step将自动更新
#global_step参数，从而使学习率也得到相应更新
learning_step = tf.train.GradientDescentOptimizer(learning_rate).minimize(
       loss_function, global_step = global_step)


#过拟合问题
"""
为了避免过拟合的问题，一个常用的方法是正则化，损失函数中加入刻画模型复杂程度的指标
一般来说，模型的复杂程度只由权重w来决定
常用的刻画复杂度的函数R(w)有两种
L1：R(w) = Σ|wi|
L2: R(w) = Σ|wi^2|
Tensorflow可以优化带正则化的损失函数
"""
w = tf.Variable(tf.random_normal([2,1],stddev=1,seed=1))
y = tf.matmul(x,w)

#tf.contrib.layers.l2_regularizer()函数可以计算一个给定参数的L2正则化项的值，省去自己
#写正则化计算公式的步骤，相当于λR(w)
loss = tf.reduce_mean(tf.square(y_-y)) + tf.contrib.layers.l2_regularizer(lambda)(w)

#维度不一致的问题
"""
Tensorflow采用流图进行运算以提高计算速度，因此产生的数据在Tensorflow中都是以tensor的方式
流通。
Tensorflow中容易出现维度不一致的情况，可以采用tf.reshpae函数进行维度修正。
x_tans ==> [[1,2,3],
            [4,5,6],
            [7,8,9]]
"""
x = tf.constant([1,2,3,4,5,6,7,8,9])
x_trans = tf.reshape(x,[3,3])


