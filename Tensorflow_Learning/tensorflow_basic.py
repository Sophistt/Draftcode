# -*- coding: utf-8 -*-
"""
Created on Mon May 15 22:12:44 2017

@author: Wang
"""

import tensorflow as tf

#理解张量（tensor），张量可以被简单理解为多维数组，其中零阶张量表示标量（scalar）
#一阶张量为向量（vector），n阶张量为n阶数组

#张量在tensorflow中的保存不是直接采用数组的形式，而是对tensorflow运算结果的运用

#下面的a是一个张量，包括了三个主要属性：（名字，维度和类型）
#注意：定义的a，b矩阵是2x1的，即是一个列向量
a = tf.constant([1.0,2.0],name="a")
b = tf.constant([2.0,3.0],name="b")
result = tf.add(a,b,name="add")
#两个张量相加，得到的结果也是一个张量
print(result)
#如果要表现为数组的形式，需要创建一个会话（Session）然后将张量表示出来
print(tf.Session().run(result))

#———————————————————————————————————————————————————————————————————————————————

#创建变量并且运行会话
"""
变量是特殊的张量，也拥有张量的三个主要属性
"""

#创建一个张量，是一个2x3的正态分布的随机矩阵，标准差=1，均值=0
w1 = tf.Variable(tf.random_normal([2,3],stddev=1,mean=0,seed=1))
w2 = tf.Variable(tf.random_normal([3,1],stddev=1,mean=0,seed=1))

#定义placeholder作为存放输入数据的地方
x = tf.placeholder(tf.float32,shape=(1,2),name="input")
#如果是要定义输入数据，用以下表达式：(定义了x1矩阵，维度为1x2)
x1 = tf.constant([[0.7,0.9]])

#定义运算关系
a1 = tf.matmul(x,w1)
y1 = tf.matmul(a1,w2)



#创建会话
sess = tf.Session()
#按照上面定义的方法初始化所有变量
init_op = tf.global_variables_initializer()
sess.run(init_op)
#注意到x是placeholder，需要提供一个feed_dict来指定x的取值
"""
feed_dict是一个字典，在字典中需要给出每个用到的placeholder的取值，如果某个需要的placeholder
没有被指定取值，那么程序在运行时会报错
"""
print(sess.run(y1,feed_dict={x: [[0.7,0.9]]}))


#下面是一个3x2的palceholder
x3 = tf.placeholder(tf.float32,shape=(3,2),name="input")


a3 = tf.matmul(x3,w1)
y3 = tf.matmul(a3,w2)

print(sess.run(y3,feed_dict={x3:[[0.7,0.9],[0.1,0.4],[0.5,0.8]]}))

#———————————————————————————————————————————————————————————————————————————————

#使用非线性激活函数解决非线性分类问题
"""
Tensorflow提供了7种不同的非线性激活函数:
tf.nn.relu,tf.sigmoid,tf.tanh是比较常用的几种
"""

a_acti = tf.nn.relu(tf.matmul(x,w1))

#———————————————————————————————————————————————————————————————————————————————

#两个tensorflow函数
"""
tf.greater():输入两个张量，比较张量每个元素的大小，返回比较结果
tf.select():三个参数，第一个参数为True/False,为True时，返回第二个参数，为False返回第三个参数
"""

v1 = tf.constant([1.0,2.0,3.0,4.0])
v2 = tf.constant([4.0,3.0,2.0,1.0])

sess = tf.InteractiveSession()
print(tf.greater(v1,v2).eval())

print(tf.where(tf.greater(v1,v2),v1,v2).eval())

sess.close()
