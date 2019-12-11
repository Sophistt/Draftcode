
import tensorflow as tf

"""
通过字典将模型保存时的变量名和需要加载的变量联系起来，这样做的主要目的之一是方便使用变量的滑动平均值。
以下代码定义了一个滑动平均模型并且将其保存起来。
"""
v = tf.Variable(0, dtype=tf.float32,name="v")

#输出所有变量的名称
for variables in tf.all_variables():
    print(variables.name)

#声明滑动平均模型
ema = tf.train.ExponentialMovingAverage(0.99)
maintain_averages_op = ema.apply(tf.all_variables())

for variables in tf.global_variables():
    print(variables.name)

saver = tf.train.Saver()
with tf.Session() as sess:
    init_op = tf.global_variables_initializer()
    sess.run(init_op)

    #改变v的值
    sess.run(tf.assign(v,10))
    sess.run(maintain_averages_op)
    #输出v及其影子变量
    saver.save(sess,"E:\\Model_Tensorflow\\model_ave.ckpt")
    print(sess.run([v,ema.average(v)]))