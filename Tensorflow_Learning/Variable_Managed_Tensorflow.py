
import tensorflow as tf

"""
    Tensorflow提供了通过变量名称来创建或获取一个变量的机制，通过该机制，在不同的函数中可以直接通过变量
的名字来使用变量，而不需要将变量通过参数的形式到处传递。
"""

#创建变量除了可以使用tf.Variable()函数，还可以使用tf.get_variable()函数，在创建变量上两者等价
v = tf.get_variable("v",shape=[1],
                    initializer=tf.constant_initializer(1.0))
v = tf.Variable(tf.constant(1.0,shape=[1]),name="v")

"""
    tf.get_variable()函数中，变量名称为必填参数，该函数会试图去创建一个给定变量名称的变量，但如果该变量
已经存在，则tf.get_variable()函数会报错。
    如果想用tf.get_variable()函数会获取一个已经存在的变量，需要通过tf.variable_scope()函数来生成一个
上下文管理器，并在管理器中明确说明。
"""

#在名称为foo的命名空间内创建名字为v0的变量。
with tf.variable_scope("foo"):
    v0 = tf.get_variable(
        "v",[1],initializer=tf.constant_initializer(1.0))

#在生成上下文管理器时，将参数reuse设置为True。这样tf.get_variable()函数会获取已经存在的变量
with tf.variable_scope("foo",reuse=True):
    v1 = tf.get_variable("v",[1])
    print(v0==v1)

"""
    命名空间可以嵌套，嵌套里面的命名空间如果没有定义reuse参数，则会继承上一层命名空间的参数，同时通过
命名空间可以管理变量名称。
"""
#命名空间嵌套
with tf.variable_scope("layer1"):
    print("layer1",tf.get_variable_scope().reuse)
    with tf.variable_scope("layer2",reuse=True):
        print("layer2",tf.get_variable_scope().reuse)
        with tf.variable_scope("layer3"):
            print("layer3",tf.get_variable_scope().reuse)

#命名空间管理变量名称
s1 = tf.get_variable("s",[1])
print(s1.name)

with tf.variable_scope("foo"):
    s2 = tf.get_variable("s",[1])
    print(s2.name)
    with tf.variable_scope("bar"):
        s3 = tf.get_variable("s",[1])
        print(s3.name)

#另一种获取变量的方法；首先创建一个空的变量空间
with tf.variable_scope("",reuse=True):
    s4 = tf.get_variable("foo/bar/s",[1])
    print("s4==s3?",s4==s3)