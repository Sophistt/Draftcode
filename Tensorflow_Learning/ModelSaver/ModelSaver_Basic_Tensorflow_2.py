
import tensorflow as tf

"""
像ModelSaver_Basic_Tensorflow_1中的加载模型是很麻烦的，必须重复定义图上的运算。
因此，可以直接加载已经持久化的图。
"""
#tf.train.import_meta_graph函数载入计算图的结构
saver = tf.train.import_meta_graph("E:\\Model_Tensorflow\\model.ckpt.meta")

with tf.Session() as sess:
    #saver.restort()载入图的变量的取值
    saver.restore(sess,"E:\\Model_Tensorflow\\model.ckpt")
    #由于没有重新定义模型，所以只能通过tensor的名称来或者tensor
    print(sess.run(tf.get_default_graph().get_tensor_by_name("add:0")))
    print(sess.run(tf.get_default_graph().get_tensor_by_name("v2:0")))