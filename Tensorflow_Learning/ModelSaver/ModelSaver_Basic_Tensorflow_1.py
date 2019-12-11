
import tensorflow as tf

"""
以下代码展示了最简单的调用已经保存好的模型的方法
"""

#使用和保存模型一样的方式来声明变量
v1 = tf.Variable(tf.constant(3.0,shape=[1]),name="v1")
v2 = tf.Variable(tf.constant(4.0,shape=[1]),name="v2")

result = v1 - v2

saver = tf.train.Saver()

with tf.Session() as sess:
    #加载已经保存的模型，并通过已经保存的模型中的变量的值来计算上面定义的结构，上面的定义的变量的值不会被初始化
    saver.restore(sess,"E:\\Model_Tensorflow\\model.ckpt")
    print(sess.run(result))




