# -*- coding: utf-8 -*-
"""
Created on Wed May 24 15:53:10 2017

出自《Tensorflow 实战Google深度学习框架》 
侵权望告知，删除相关代码
"""

import  tensorflow as tf

from numpy.random import RandomState

#定义训练数据batch的大小
batch_size = 8

#定义神经网络的参数
w1 = tf.Variable(tf.random_normal([2,3],stddev=1,seed=1))
w2 = tf.Variable(tf.random_normal([3,1],stddev=1,seed=1))

#定义placeholder
"""
在shape的一个维度上使用None可以方便使用不大的batch。在训练时需要把数据分成较小的batch，
但是在测试时，可以一次性使用全部的数据。当数据集比较小时这样比较方便测试，但数据集比较大时，
将大量数据放入一个batch可能会导致内存溢出。
"""
x = tf.placeholder(tf.float32,shape=(None,2),name='x-input')
y_ = tf.placeholder(tf.float32,shape=(None,1),name='y_input')

#定义神经网络的计算过程
a = tf.matmul(x,w1)
y = tf.matmul(a,w2)

#定义损失函数和反向传播算法。
cross_entropy = -tf.reduce_mean(
        y_ * tf.log(tf.clip_by_value(y,1e-10,1.0)))
"""
tf.reduce_mean()函数是求平均的意思。
tf.clip_by_value()函数将一个张量的数值限制在一定范围之内，上面的就是把y的值限制在了1e-10与1.0之间
*指的是点乘，如果要矩阵乘法要使用tf.matmul()函数来完成。

通过上面的组合就可以得出交叉熵的计算公式
因为交叉熵一般会与softmax回归一起使用，所以Tensorflow对这两个功能进行了封装
提供了tf.nn.softmax_cross_entropy_with_logits()函数
例如调用tf.nn.softmax_cross_entropy_with_logits(y,y_)即可。
"""
train_step = tf.train.AdamOptimizer(0.001).minimize(cross_entropy)

#通过随机数生成一个模拟数据集
rdm = RandomState(1)
dataset_size = 128
X = rdm.rand(dataset_size,2)

#定义规则来给出样本的标签
"""
所有x1+x2<1的样例都被认为是正样本，而其他为负样本
"""
Y = [[int(x1+x2<1)] for (x1,x2) in X]

#创建一个会话来运行Tensorflow程序。
with tf.Session() as sess:
    init_op = tf.global_variables_initializer()
    #初始化变量
    sess.run(init_op)
    print(sess.run(w1))
    print(sess.run(w2))
    
    #设定训练的轮数
    STEPS = 5000
    for i in range(STEPS):
        #每次选取batch_size个样本进行训练。
        start = (i * batch_size) % dataset_size
        end = min(start+batch_size,dataset_size)
        
        #通过选取的样本训练神经网络并更新参数
        sess.run(train_step,
                 feed_dict={x:X[start:end],y_:Y[start:end]})
        if i % 1000 == 0:
            total_cross_entropy = sess.run(
                    cross_entropy,feed_dict={x:X,y_:Y})
            print("After %d training step(s),cross entropy on all data is %g" %(i,total_cross_entropy))
            
    print(sess.run(w1))
    print(sess.run(w2))

    writer = tf.summary.FileWriter("E:\\test",tf.get_default_graph())
    writer.close()
        