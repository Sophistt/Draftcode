# -*- coding: utf-8 -*-


import tensorflow as tf

"""
通过tf.get_variable的方式创建过滤器的权重变量和偏置项变量。卷积层的参数个数只和过滤器的
尺寸，深度以及当前结点矩阵的深度有关，所以这里声明的参数变量是一个四维矩阵，前面两个维度
代表了过滤器吃ucn，第三个维度表示当前层的深度，第四个维度表示过滤器的深度。
"""
filter_weight = tf.get_variable('weights',
                        shape=[5,5,3,16],
                        initializer=tf.truncated_normal_initializer(stddev=0.1))
"""
和卷积层类似，当前层矩阵上不同位置的偏置项也是共享的，所以总共有下一层深度个不同的偏置
项，上面过滤器的深度为16，则偏置项的维度应该为16。
"""
#偏置项的维度和下一层的深度相同，但并不代表偏置项只有16个，因为每一个输出节点都会加上偏置项
biases = tf.get_variable('biases',
                    shape=[16],
                    initializer=tf.constant_initializer(0.1))

#tf.nn.conv2d是实现卷积层前向传播的函数。
"""
tf.nn.conv2d第一个参数是当前层的节点矩阵。节点矩阵为4维矩阵，后面三个维度是一个节点矩阵，第一个维度是
输入batch。第二个参数是卷积层的权重，第三个参数为不同维度上的步长，其维度与输入节点矩阵对应，所以第一与
第四个维度的值必须为1。最后一个参数是全0填充，'SAME'使用全0填充，'VALID'代表不使用全0填充。
"""
conv = tf.nn.conv2d(input,filter_weight,strides=[1,1,1,1],padding='SAME')

"""
tf.nn.bias_add函数用于给输出节点添加偏置项，这里不能简单的使用+法，因为biases是1x16的，但输出矩阵是
?x?x16的
"""
bias = tf.nn.bias_add(conv,biases)

#使用ReLU函数去线性化
actived_conv = tf.nn.relu(bias)

#使用池化层来减小矩阵的长和宽
pool = tf.nn.max_pool(actived_conv,ksize=[1,3,3,1],
                    strides=[1,2,2,1],padding='SAME')

