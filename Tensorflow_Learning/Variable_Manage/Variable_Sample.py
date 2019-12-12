"""
1.  用命名空间进行变量创建和管理
    主要用到tf.variable_scope和tf.get_variable函数
2.  用tf.summary函数实现tensorboard可视化变量
3.  训练一段时间后，暂停训练，用测试集验证准确率
"""
import tensorflow as tf
from tensorflow.examples.tutorials.mnist import input_data

mnist = input_data.read_data_sets("D:/MNIST_data", one_hot=True)

with tf.variable_scope('input'):
    s = tf.placeholder(dtype=tf.float32, shape=[None, 784], name='s_input')
    target = tf.placeholder(dtype=tf.float32, shape=[None, 10], name='target')

with tf.variable_scope('input_reshape'):
    image_reshape_input = tf.reshape(s, [-1, 28, 28, 1])
    tf.summary.image('input', image_reshape_input, 10)

with tf.variable_scope('eval_net'):
    # Collection name
    c_name = ['eval_net_param', tf.GraphKeys.GLOBAL_VARIABLES]
    w_initializer = tf.truncated_normal_initializer()
    b_initializer = tf.constant_initializer(0.1)

    with tf.variable_scope('l1'):
        w1 = tf.get_variable(name='weight1', shape=[784, 500], dtype=tf.float32,
                             initializer=w_initializer, collections=c_name)
        b1 = tf.get_variable(name='bias1', shape=[500], dtype=tf.float32,
                             initializer=b_initializer, collections=c_name)
        l1 = tf.nn.relu(tf.matmul(s, w1) + b1)
        tf.summary.histogram('l1', l1)

    with tf.variable_scope('l2'):
        w2 = tf.get_variable(name='weight1', shape=[500, 10], dtype=tf.float32,
                             initializer=w_initializer, collections=c_name)
        b2 = tf.get_variable(name='bias1', shape=[10], dtype=tf.float32,
                             initializer=b_initializer, collections=c_name)
        l2 = tf.matmul(l1, w2) + b2
        tf.summary.histogram('l2', l2)

    with tf.variable_scope('loss'):
        cross_entropy = tf.nn.sparse_softmax_cross_entropy_with_logits(logits=l2, labels=tf.argmax(target, 1))
        cross_entropy_mean = tf.reduce_mean(cross_entropy)
        tf.summary.scalar('cross_entropy', cross_entropy_mean)

    with tf.variable_scope('optimization'):
        train_step = tf.train.RMSPropOptimizer(0.01).minimize(cross_entropy_mean)

    # 测试正确率
    with tf.variable_scope('test_accuracy'):
        correct_prediction = tf.equal(tf.argmax(l2, 1), tf.argmax(target, 1))
        accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32))
        tf.summary.scalar('accuracy', accuracy)

    e_params = tf.get_collection('eval_net_params')

    merged = tf.summary.merge_all()

    init_op = tf.global_variables_initializer()

with tf.Session() as sess:
    writer = tf.summary.FileWriter('logs', sess.graph)

    # 初始化所有参数
    sess.run(init_op)
    # 开始训练
    for epoch in range(10):
        for batch_step in range(600):
            # 每次循环进来首先更新数据
            batch_xs, batch_ys = mnist.train.next_batch(100)
            # 用sess.run()函数开始训练
            summary, _ = sess.run([merged, train_step], feed_dict={s: batch_xs, target: batch_ys})
            writer.add_summary(summary, batch_step)
            if batch_step % 500 == 0:
                summary, mean_cross_entropy = sess.run([merged, cross_entropy_mean],
                                                       feed_dict={s: batch_xs, target: batch_ys})
                # 将loss写入writer
                writer.add_summary(summary, batch_step)
                print('After train %d epoch, the mean cross entropy is ' % (epoch + 1), mean_cross_entropy)

        print('Finish train %d epoch' % (epoch + 1))
        # 运行测试数据集
        summary, accuracy_test = sess.run([merged, accuracy],
                                          feed_dict={s: mnist.test.images, target: mnist.test.labels})
        # 将accuracy写入writer
        writer.add_summary(summary, epoch)
        # 打印测试集的测试准确率
        print('After trained %d epoch, the accuracy is %s' % (epoch + 1, accuracy_test))

writer.close()
