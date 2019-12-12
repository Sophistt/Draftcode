# -*- coding: utf-8 -*-

import tensorflow as tf
import tensorflow.contrib.slim as slim

#配置神经网络的参数
INPUT_NODE = 784
OUTPUT_NODE = 10

IMAGE_SIZE = 28
NUM_CHANNELS = 1
NUM_LABELS = 10

#第一层卷积层的尺寸和深度
CONV1_DEEP = 32
CONV1_SIZE = 5
#第二层卷积层的尺寸和深度
CONV2_DEEP = 64
CONV2_SIZE = 5

#全连接层的节点个数
FC_SIZE = 512

#定义卷积神经网络的前向传播过程。
"""
这里添加了一个新的参数train，用于区分训练过程和测试过程。在这个程序中将用到dropout方法，
dropout方法可以进一步提升模型可靠性并防止过拟合，dropout过程只在训练时使用。
"""
def inference(input_tensor,train,regularizer):
    #声明第一层卷积层的变量并实现前向传播过程。通过使用不同的命名空间来隔离不同层的变量，可以让
    #每一层的变量命名只需要考虑在当前层的作用，而不需要担心重名的问题。
    with tf.variable_scope('layer1-conv1'):
        #使用边长为5，深度为32的Filter
        conv1_weights = tf.get_variable(
            "weight",[CONV1_SIZE,CONV1_SIZE,NUM_CHANNELS,CONV1_DEEP],
            initializer=tf.truncated_normal_initializer(stddev=0.1))
        conv1_biaes = tf.get_variable("bias",[CONV1_DEEP],initializer=tf.constant_initializer(0.0))

        #过滤器移动的步长为1，并且使用全0填充，使用全0填充后 OUTlength = INlength / stride
        conv1 = tf.nn.conv2d(
            input_tensor,filter=conv1_weights,strides=[1,1,1,1],padding="SAME")
        relu1 = tf.nn.relu(tf.nn.bias_add(conv1,conv1_biaes))
        
    #实现池化层的前向传播过程，这里选用最大池化层，池化层过滤器的边长为2，使用全0填充，
    #移动步长为2。
    with tf.name_scope('layer2-pool1'):
        pool1 = tf.nn.max_pool(relu1,ksize=[1,2,2,1],strides=[1,2,2,1],padding='SAME')

    #声明第三层卷积层的变量并实现前向传播过程，输入为14x14x32的矩阵，输出为14x14x64的矩阵
    with tf.variable_scope('layer3-conv2'):
        conv2_weights = tf.get_variable(
            "weight",[CONV2_SIZE,CONV2_SIZE,CONV1_DEEP,CONV2_DEEP],
            initializer=tf.truncated_normal_initializer(stddev=0.1))
        conv2_biases = tf.get_variable(
            "bias",[CONV2_DEEP],initializer=tf.constant_initializer(0.1))

        #过滤器边长为5，步长为1
        conv2 = tf.nn.conv2d(
            pool1,filter=conv2_weights,strides=[1,1,1,1],padding='SAME')
        relu2 = tf.nn.relu(tf.nn.bias_add(conv1,conv1_biaes))
        
    #实现第四层池化层前向传播的过程，输入14x14x64的矩阵，输出7x7x64的矩阵
    with tf.name_scope('layer4-pool2'):
        pool2 = tf.nn.max_pool(relu2,ksize=[1,2,2,1],strides=[1,2,2,1],padding='SAME')

    #获得经过池化层第二层后的数据流图的维度
    pool_shape = pool2.get_shape().as_list()

    #将三维的数据流图转一维 
    nodes = pool_shape[1] * pool_shape[2] * pool_shape[3]
    reshaped = tf.reshape(pool2,[pool_shape[0],nodes])

    #声明全连接层的变量并实现前向传播的过程。
    """
    该层输入时上层池化层拉直后的向量，向量长度为7x7x64=3136，输出一组512的向量。
    与前面的全连接层的训练方法基本一致，区别就是引入了dropout的概念，dropout在训练时
    会随机将部分节点的输出改为0。dropout方法可以有效避免过拟合问题，从而使模型在测试数据
    上表现得更好。——dropout方法一般只在全连接层使用。
    """
    with tf.variable_scope('layer5-fc1'):
        fc1_weights = tf.get_variable(
            "weight",[nodes,FC_SIZE],
        initializer=tf.truncated_normal_initializer(stddev=0.1))
        #全连接层的权重加入正则化
        if regularizer is not None:
            tf.add_to_collection('losses',regularizer(fc1_weights))
        fc1_biases = tf.get_variable(
            "bias",[FC_SIZE],initializer=tf.constant_initializer(0.1))

        fc1 = tf.nn.relu(tf.matmul(reshaped,fc1_weights) + fc1_biases)
        if train: fc1 = tf.nn.dropout(fc1,0.5)

    #声明第六层全连接层的变量并实现前向传播的过程。
    #该层输入512维的向量，输出10维的向量，输出通过softmax层后得到最后的分类结果。
    with tf.variable_scope('layer6-fc2'):
        fc2_weight = tf.get_variable(
            "weight",[FC_SIZE,NUM_LABELS],
            initializer=tf.truncated_normal_initializer(stddev=0.1))
        if regularizer is not None:
            tf.add_to_collection('losses',regularizer(fc2_weight))
        fc2_biases = tf.get_variable(
            "bias",[NUM_LABELS],initializer=tf.constant_initializer(0.1))
        logit = tf.matmul(fc1,fc2_weight) + fc2_biases

    #返回第6层的输出
    return logit


#使用slim方法快速建立神经网络模型
#定义卷积神经网络的前向传播过程。
"""
这里添加了一个新的参数train，用于区分训练过程和测试过程。在这个程序中将用到dropout方法，
dropout方法可以进一步提升模型可靠性并防止过拟合，dropout过程只在训练时使用。
"""
def inference_slim(input_tensor,train):
    #声明第一层卷积层的变量并实现前向传播过程
    with tf.variable_scope('layer1-conv1'):
        net_1 = slim.conv2d(
            input_tensor,num_outputs=CONV1_DEEP,kernel_size=[CONV1_SIZE,CONV1_SIZE],
            weights_initializer=tf.truncated_normal_initializer(stddev=0.1))
    #声明池化层
    with tf.name_scope('layer2-pool1'):
        pool_1 = slim.max_pool2d(net_1,kernel_size=[2,2],stride=[2,2],padding='SAME')
    #声明第三层卷积层的变量并实现前向传播过程
    with tf.variable_scope('layer3-conv2'):
        net_2 = slim.conv2d(
            pool_1,num_outputs=CONV2_DEEP,kernel_size=[CONV2_SIZE,CONV2_SIZE],
            weights_initializer=tf.truncated_normal_initializer(stddev=0.1))
    #声明池化层
    with tf.name_scope('layer4-pool2'):
        pool_2 = slim.max_pool2d(net_2,kernel_size=[2,2],stride=[2,2],padding='SAME')

    #获得经过池化层第二层后的数据流图的维度
    pool_shape = pool_2.get_shape().as_list()

    #将三维的数据流图转一维 
    nodes = pool_shape[1] * pool_shape[2] * pool_shape[3]
    reshaped = tf.reshape(pool_2,[pool_shape[0],nodes])

    #声明全连接层的变量并实现前向传播的过程。
    with tf.variable_scope('layer5-fc1'):
        fc1 = slim.fully_connected(
            inputs=reshaped,num_outputs=FC_SIZE,
            weights_initializer=tf.truncated_normal_initializer(stddev=0.1),
            biases_initializer=tf.constant_initializer(0.1),
            """
            定义了weight_regularizer后，slim函数会自动将计算好的正则化项
            放到tf.GraphKeys.REGULARIZATION_LOSSES中，在后面计算loss函数
            的时候，使用tf.get_collection将正则化项提取出来与权重相加
            """
            weights_regularizer=tf.contrib.layers.l2_regularizer(0.0001))
        if train: fc1 = tf.nn.dropout(fc1,0.5)
    
    #声明第六层全连接层的变量并实现前向传播的过程。
    #该层输入512维的向量，输出10维的向量，输出通过softmax层后得到最后的分类结果。
    with tf.variable_scope('layer6-fc2'):
        logit = slim.fully_connected(
            inputs=fc1,num_outputs=NUM_LABELS,
            weights_initializer=tf.truncated_normal_initializer(stddev=0.1),
            biases_initializer=tf.constant_initializer(0.1),
            activation_fn=None,
            weights_regularizer=tf.contrib.layers.l2_regularizer(0.0001)
        )
    
    #返回第6层的输出
    return logit


