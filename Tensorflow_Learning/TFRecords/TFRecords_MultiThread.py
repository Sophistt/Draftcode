

import tensorflow as tf


x_inputs_data = tf.random_normal([128, 1024], mean=0, stddev=1)
y_inputs_data = tf.cast(tf.reduce_sum(x_inputs_data, axis=1, keep_dims=True) > 0, tf.int32)

with tf.Session() as sess:
    init_op = tf.initialize_all_variables()
    sess.run(init_op)

    x, y = sess.run([x_inputs_data, y_inputs_data])
    print(x, y)
