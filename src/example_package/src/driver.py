#!/usr/bin/env python

#This node drives the robot based on information from the other nodes.

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32

LINEAR_SPEED = 0.2
ANGULAR_SPEED = 0.3

#Makes the twist object sent from PID global
def cb_twist(msg):
    global t_pid
    t_pid = msg

#Init node
rospy.init_node('driver')

#Make publisher for cmd_vel
pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

#Make all subscribers
sub_pid_twist = rospy.Subscriber('pid_twist', Twist, cb_twist)

#Rate object
rate = rospy.Rate(10)

#Create two twist variable, one is modified here, one is copied from the PID messages
t_pub = Twist()
t_pid = Twist()

should_follow_pid = False

while not rospy.is_shutdown():
    if (should_follow_pid):
        t_pub = t_pid
    else:
        t_pub.linear.x = LINEAR_SPEED
        t_pub.angular.z = 0
    pub_vel.publish(t_pub)
    rate.sleep()
