# -*- coding: utf-8 -*-

import tensorflow as tf
from tensorflow.examples.tutorials.mnist import input_data
import LeNet5_Model_Tensorflow
import os
import numpy as np

#定义神经网络的相关参数
BATCH_SIZE = 100
LEARNING_RATE_BASE = 0.01
LEARNING_RATE_DECAY = 0.99
REGULARIZATION_RATE = 0.0001
TRAINING_STEPS = 6000
MOVING_AVERAGE_DECAY = 0.99

#定义训练过程
def train(mnist):
    #定义四维矩阵的placeholder作为输入
    x = tf.placeholder(
        tf.float32,
        [BATCH_SIZE,
        LeNet5_Model_Tensorflow.IMAGE_SIZE,
        LeNet5_Model_Tensorflow.IMAGE_SIZE,
        LeNet5_Model_Tensorflow.NUM_CHANNELS],
        name='x-input')
    #定义标签的placeholder输入
    y_ = tf.placeholder(tf.float32,[None,LeNet5_Model_Tensorflow.OUTPUT_NODE],name='y-input')

    "如果要使用非slim建立的模型，则调用以下两行代码"
#    regularizer = tf.contrib.layers.l2_regularizer(REGULARIZATION_RATE)
#    y = LeNet5_Model_Tensorflow.inference(x,False,regularizer)

    #基于slim快速建立模型调用的inference_slim函数
    y = LeNet5_Model_Tensorflow.inference_slim(x, False)

    # 定义四维矩阵的placeholder作为测试输入
    """
    此处取1000张test数据集的图片作为测试的输入
    将测试集的输出logit定义为y_test
    """
    x_test = tf.placeholder(
        tf.float32,
        [1000,
        LeNet5_Model_Tensorflow.IMAGE_SIZE,
        LeNet5_Model_Tensorflow.IMAGE_SIZE,
        LeNet5_Model_Tensorflow.NUM_CHANNELS],
        name='x-test')
    y_test = LeNet5_Model_Tensorflow.inference_slim(x_test, False)

    #定义训练步数
    global_step = tf.Variable(0,trainable=False)

    #定义滑动平均操作
    variable_averages = tf.train.ExponentialMovingAverage(MOVING_AVERAGE_DECAY,global_step)
    variable_averages_op = variable_averages.apply(tf.trainable_variables())
    #定义损失函数
    cross_entropy = tf.nn.sparse_softmax_cross_entropy_with_logits(logits=y,labels=tf.arg_max(y_,1))
    cross_entropy_mean = tf.reduce_mean(cross_entropy)
#    loss = cross_entropy_mean + tf.add_n(tf.get_collection('losses'))

    #slim快速建立模型的正则化计算
    regularization_losses = tf.get_collection(tf.GraphKeys.REGULARIZATION_LOSSES)
    loss = cross_entropy_mean + tf.add_n(regularization_losses, name='total_loss')

    #定义学习率
    learning_rate = tf.train.exponential_decay(
        LEARNING_RATE_BASE,
        global_step,
        mnist.train.num_examples / BATCH_SIZE,
        LEARNING_RATE_DECAY,
        staircase=True)
    
    #定义训练过程
    train_step = tf.train.GradientDescentOptimizer(learning_rate).minimize(loss,global_step=global_step)
    with tf.control_dependencies([train_step,variable_averages_op]):
        train_op = tf.no_op(name='train')

    #定义预测准确率的计算
    correct_prediction = tf.equal(tf.arg_max(y_test, 1), tf.arg_max(y_, 1))
    accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32))
    
    #初始化tensorflow持久化类
    saver = tf.train.Saver()
    with tf.Session() as sess:
        tf.global_variables_initializer().run()

        #载入测试数据集
        """
        从mnist.test中载入1000张图片，然后将10000x784解维成10000x28x28x1
        再分别对test_xs,test_ys建立字典，方便后面placeholder的填入
        """
        test_xs = mnist.test.images[0:1000]
        test_ys = mnist.test.labels[0:1000]

        reshape_test_xs = np.reshape(test_xs, (
            1000,
            LeNet5_Model_Tensorflow.IMAGE_SIZE,
            LeNet5_Model_Tensorflow.IMAGE_SIZE,
            LeNet5_Model_Tensorflow.NUM_CHANNELS))

        test_feed = {x_test:reshape_test_xs, y_:test_ys}

        for i in range(TRAINING_STEPS):
            xs,ys = mnist.train.next_batch(BATCH_SIZE)

            reshape_xs = np.reshape(xs,(
                BATCH_SIZE,
                LeNet5_Model_Tensorflow.IMAGE_SIZE,
                LeNet5_Model_Tensorflow.IMAGE_SIZE,
                LeNet5_Model_Tensorflow.NUM_CHANNELS))

            _, loss_value, step = sess.run([train_op, loss, global_step], feed_dict={x: reshape_xs, y_: ys})

            if i % 1000 == 0:
                print("After %d training step(s), loss on training batch is %g." % (step,loss_value))

        #训练完毕后，进行test数据集的预测，验证预测准确率
        test_acc = sess.run(accuracy, feed_dict=test_feed)
        print("After training, test accuracy is %g" % (test_acc))

    writer = tf.summary.FileWriter("E:\\Tensorboard",tf.get_default_graph())
    writer.close()

def main(argv = None):
    mnist = input_data.read_data_sets("D:/MNIST_data", one_hot=True)
    train(mnist)

if __name__ == "__main__":
    main()



