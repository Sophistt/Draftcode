import tensorflow as tf
import numpy as np
from tensorflow.examples.tutorials.mnist import input_data
from RNN_Learn import LSTM
import os

os.environ["CUDA_VISIBLE_DEVICES"] = "0"

mnist = input_data.read_data_sets('MNIST_data', one_hot=True)

KEEP_PROB = 0.75

Lstm = LSTM()

with tf.Session() as sess:
    cost = Lstm.compute_loss()
    train_op = Lstm.train_op(cost)
    accuracy = Lstm.correct_pred()

    init_op = tf.global_variables_initializer()

    sess.run(init_op)

    for i in range(10000):

        batch_xs, batch_ys = mnist.train.next_batch(256)


        sess.run(train_op, feed_dict={
            Lstm.X: batch_xs,
            Lstm.y: batch_ys,
            Lstm.keep_prob: KEEP_PROB
        })

        if i % 1000 == 0:
            test_xs, test_ys = mnist.test.next_batch(256)

            print(sess.run([cost, accuracy], feed_dict={
                Lstm.X: test_xs,
                Lstm.y: test_ys,
                Lstm.keep_prob: 1.0
            }))
