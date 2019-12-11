
import tensorflow as tf
import numpy as np


a = np.ones((32, 1024))
b = np.ones((2, 3))

print(a.shape)
a = tf.convert_to_tensor(a)
b = tf.convert_to_tensor(b)

a = tf.reshape(a, [-1, 16, 1024])

test = tf.unstack(a, 16, 1)
test1 = tf.unstack(b, axis=0)

print(np.shape(test))



with tf.Session() as sess:
    vector = sess.run(test)
    vector1 = sess.run(test1)
    for vec in vector:
        print(vec.shape)