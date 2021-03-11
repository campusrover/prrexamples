#!/usr/bin/env python


#This node drives the robot based on information from the other nodes.

import rospy
from state_definitions import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32

#Makes the state message global
def cb_state(msg):
    global state
    state = msg.data

#Makes the twist object sent from PID global
def cb_twist(msg):
    global t_pid
    t_pid = msg

#Init node
rospy.init_node('driver')

#Make publisher for cmd_vel
pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

#Make all subscribers
sub_state = rospy.Subscriber('state', Int16, cb_state)
sub_pid_twist = rospy.Subscriber('pid_twist', Twist, cb_twist)

#Rate object
rate = rospy.Rate(10)

state = #Some starting state

#Create two twist variable, one is modified here, one is copied from the PID messages
t_pub = Twist()
t_pid = Twist()

print("STARTING")

while not rospy.is_shutdown():
    print("STATE: ", state)
    if (state ==        ):
        #Calculate and set appropriate t_pub values
    elif (state ==       ):
        #Calculate and set appropriate t_pub values
    elif (state ==       ):
        #Calculate and set appropriate t_pub values
    else:
        print("STATE NOT FOUND")
    pub_vel.publish(t_pub)
    rate.sleep()
