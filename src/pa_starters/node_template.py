#!/usr/bin/env python3

# This is an example ROS node with best practices.

import rospy

# import message types and anything else you need

# Subscriber callback, this runs every time your subscriber receives a new message
def cb_name(msg):
    # This uses a global keywork, consider using classes to avoid several global variables
    global value
    value = msg.data

# This is a simple function example
def get_new_pub_value(value):
    return value * 2

# Init node
rospy.init_node('my_node')

# Make a publisher
my_pub = rospy.Publisher('topic_name', MessageType, queue_size = 1)

# Make a subscriber
my_sub = rospy.Subscriber('topic_name', MessageType, cb_name)

# Rate object
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    # Try not to have complex logic in here, simply call a couple high level functions or have a simple logic test.
    # Make sure this while loop can run all the way through 10 times per second. Any computations in this loop should be very fast.
    new_pub_value = get_new_pub_value(value)
    my_pub.publish(new_pub_value)
    rate.sleep()