
import tensorflow as tf

import tensorflow.contrib.layers as layers
from tensorflow.contrib import rnn



SEQ_LENGTH = 16
INPUT_LENGTH = 512
N_CLASSES = 10
BATCH_SIZE = 16
HIDDEN_SIZE = 200

LEARNINGRATE = 0.01

lstm_global_step = tf.Variable(0, dtype=tf.int32, trainable=False, name='lstm_global_step')


class LSTM(object):
    def __init__(self):
        self.X = tf.placeholder(dtype=tf.float32, shape=[None, 784], name='images_input')
        self.y = tf.placeholder(dtype=tf.float32, shape=[None, N_CLASSES])
        self.keep_prob = tf.placeholder(dtype=tf.float32, shape=[], name='keep_prob')

        # Create lstm network
        self._build_network()

    def _build_network(self):
        with tf.variable_scope('Network'):
            net = layers.fully_connected(self.X, 512)

            self.features = tf.reshape(net, [-1, SEQ_LENGTH, INPUT_LENGTH])

        with tf.variable_scope('lstm_cell'):
            self.lstm = rnn.BasicLSTMCell(HIDDEN_SIZE, forget_bias=1.0, state_is_tuple=True)

        with tf.variable_scope('lstm_run'):
            self.lstm = rnn.DropoutWrapper(self.lstm, output_keep_prob=self.keep_prob)
            # inputs = tf.unstack(self.features, SEQ_LENGTH, axis=1)

            init_state = self.lstm.zero_state(batch_size=16, dtype=tf.float32)
            # outputs is a list, which in every element is a tensor with shape [batch_size, input_size]
            # outputs, states = rnn.static_rnn(self.lstm, inputs, dtype=tf.float32)
            outputs, states = tf.nn.dynamic_rnn(self.lstm, self.features, initial_state=init_state, time_major=False)

            output = tf.reshape(outputs, [-1, HIDDEN_SIZE])

            self.logits = layers.fully_connected(output, 10, activation_fn=None)

    def compute_loss(self):
        with tf.name_scope('lstm_cost'):
            entropy = tf.nn.softmax_cross_entropy_with_logits(labels=self.y, logits=self.logits)
            cost = tf.reduce_mean(entropy)
            self.sum_cost = tf.summary.scalar('cost', cost)

            return cost

    def train_op(self, cost):
        with tf.name_scope('lstM_train_op'):
            train_op = tf.train.AdamOptimizer(LEARNINGRATE).minimize(cost, global_step=lstm_global_step)

        return train_op

    def correct_pred(self):
        correct_prediction = tf.equal(tf.argmax(self.logits, 1), tf.argmax(self.y, 1))

        return tf.reduce_mean(tf.cast(correct_prediction, tf.float32))
