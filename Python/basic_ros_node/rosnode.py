#!/home/wcy/software/miniconda3/envs/py3.6/bin/python3
# -*- coding: utf-8 -*-

""" 
* @Copyright (c)  all right reserved 
* 
* @file:rosnode.py 
* @author: Sophistt 
* @date:2019-07-29 15:03 
* @description: Python file 
"""

# Basic Ros library required
import rospy

# Standard messages of Ros
from std_msgs.msg import String, Float32


def basic_msg_callback(msg):

    rospy.loginfo("I heard basic_msg: %s", msg.data)


def main():

    # Ros node initialization
    rospy.init_node("basic_ros_node", anonymous=True)

    # Create a Ros Subscriber to subcribe messge from "/basic_msg" topic
    basic_sub = rospy.Subscriber(
        "/basic_msg", String, basic_msg_callback, queue_size=1)

    # Create a Ros Publisher to publish message to "/basic_msg" topic
    basic_pub = rospy.Publisher("/basic_msg", String, queue_size=1)

    # Set running rate of Ros node
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub_msg = "Publish Message"

        basic_pub.publish(pub_msg)

        rospy.loginfo(pub_msg)

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
